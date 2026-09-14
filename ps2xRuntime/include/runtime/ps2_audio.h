#ifndef PS2_AUDIO_H
#define PS2_AUDIO_H

#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

class PS2AudioBackend
{
public:
    PS2AudioBackend();
    ~PS2AudioBackend();

    void onVagTransfer(const uint8_t *rdram, uint32_t srcAddr, uint32_t sizeBytes);
    void onVagTransferFromBuffer(const uint8_t *data, uint32_t sizeBytes, uint32_t keyAddr);

    // BOOT 162: .hack PSS movie PCM bridge.
    // Mirrors the movie player's EE->IOP PCM transfer into the
    // host audio backend without altering guest DMA semantics.
    void onMoviePcmTransfer(const uint8_t *rdram,
                            uint32_t srcAddr,
                            uint32_t sizeBytes);

    // BOOT 170:
    // Feed a complete host-resident PSS PCM payload through the
    // proven Boot 162-166 parser without temporary guest memory.
    void onMoviePcmTransferFromBuffer(
        const uint8_t *data,
        uint32_t sizeBytes);

    // BOOT 165: periodically refill the host PSS AudioStream after
    // the initial MPEG demux burst has filled the PCM queue.
    void pumpMovieAudio();

    // BOOT 172:
    // Begin a prefilled PSS host stream when the guest actually
    // enables movie presentation via startDisplay().
    void startMovieAudioForDisplay();

    // BOOT 175:
    // Real host-time clock beginning at the exact transition that
    // starts physical PSS audio playback. Returns -1 until active.
    double movieAudioElapsedSeconds();

    void onSoundCommand(uint32_t sid, uint32_t rpcNum,
                        const uint8_t *sendBuf, uint32_t sendSize,
                        uint8_t *recvBuf, uint32_t recvSize);

    void play(uint32_t sampleAddr, float pitch = 1.0f, float volume = 1.0f,
              uint32_t voiceIndex = 0xFFFFFFFFu);
    void stop(uint32_t voiceId);
    void stopAll();
    void setAudioReady(bool ready) { m_audioReady = ready; }

private:
    struct DecodedSample
    {
        std::vector<int16_t> pcm;
        uint32_t sampleRate = 44100;
    };

    struct Impl;
    std::unique_ptr<Impl> m_impl;
    bool m_audioReady = false;
    uint32_t m_mostRecentSampleKey = 0;
    std::vector<DecodedSample> m_loadOrderSamples;
    std::vector<uint32_t> m_loadOrderSampleKeys;
    std::unordered_map<uint32_t, DecodedSample> m_sampleBank;
    std::mutex m_mutex;

    void playDecodedSample(uint32_t sampleKey, DecodedSample &sample, float pitch, float volume,
                          bool isBgm = false);
    void pruneFinishedSounds();
};

#endif

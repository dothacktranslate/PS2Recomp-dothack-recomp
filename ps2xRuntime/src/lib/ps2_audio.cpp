#include "runtime/ps2_audio.h"
#include "runtime/ps2_memory.h"
#include "ps2_host_backend.h"
#include <cstring>
#include <cstdio>
#include <vector>

namespace
{
    std::vector<uint8_t> buildWavFromPcm(const int16_t *pcm, size_t sampleCount, uint32_t sampleRate)
    {
        const uint32_t dataSize = static_cast<uint32_t>(sampleCount * 2);
        const uint32_t fileSize = 36 + dataSize;
        std::vector<uint8_t> wav(8 + fileSize);

        uint8_t *p = wav.data();
        p[0] = 'R';
        p[1] = 'I';
        p[2] = 'F';
        p[3] = 'F';
        p[4] = static_cast<uint8_t>(fileSize);
        p[5] = static_cast<uint8_t>(fileSize >> 8);
        p[6] = static_cast<uint8_t>(fileSize >> 16);
        p[7] = static_cast<uint8_t>(fileSize >> 24);
        p[8] = 'W';
        p[9] = 'A';
        p[10] = 'V';
        p[11] = 'E';
        p[12] = 'f';
        p[13] = 'm';
        p[14] = 't';
        p[15] = ' ';
        p[16] = 16;
        p[17] = 0;
        p[18] = 0;
        p[19] = 0;
        p[20] = 1;
        p[21] = 0;
        p[22] = 1;
        p[23] = 0;
        p[24] = static_cast<uint8_t>(sampleRate);
        p[25] = static_cast<uint8_t>(sampleRate >> 8);
        p[26] = static_cast<uint8_t>(sampleRate >> 16);
        p[27] = static_cast<uint8_t>(sampleRate >> 24);
        const uint32_t byteRate = sampleRate * 2;
        p[28] = static_cast<uint8_t>(byteRate);
        p[29] = static_cast<uint8_t>(byteRate >> 8);
        p[30] = static_cast<uint8_t>(byteRate >> 16);
        p[31] = static_cast<uint8_t>(byteRate >> 24);
        p[32] = 2;
        p[33] = 0;
        p[34] = 16;
        p[35] = 0;
        p[36] = 'd';
        p[37] = 'a';
        p[38] = 't';
        p[39] = 'a';
        p[40] = static_cast<uint8_t>(dataSize);
        p[41] = static_cast<uint8_t>(dataSize >> 8);
        p[42] = static_cast<uint8_t>(dataSize >> 16);
        p[43] = static_cast<uint8_t>(dataSize >> 24);
        std::memcpy(p + 44, pcm, dataSize);
        return wav;
    }
}

namespace ps2_vag
{
    bool decode(const uint8_t *data, uint32_t sizeBytes,
                std::vector<int16_t> &outPcm, uint32_t &outSampleRate);
}

struct PS2AudioBackend::Impl
{
    struct TrackedSound
    {
        Sound snd;
        uint32_t sampleKey;
    };
    std::vector<TrackedSound> activeSounds;

    // BOOT 162: streaming state for .hack's PSS PCM soundtrack.
    //
    // OPENING.PSS:
    //   SShd type       = 1
    //   sample rate     = 48000 Hz
    //   channels        = 2
    //   sample format   = signed 16-bit little-endian
    //   channel block   = 0x200 bytes
    //
    // The SSbd body is stored as:
    //
    //   0x200 bytes left
    //   0x200 bytes right
    //   0x200 bytes left
    //   0x200 bytes right
    //   ...
    //
    // Raylib expects ordinary interleaved stereo frames, so the bridge
    // converts each channel-block pair before submitting it.
    std::vector<uint8_t> boot162MovieInput;
    std::vector<int16_t> boot162MoviePcm;

    uint32_t boot162MovieSampleRate = 48000u;
    uint32_t boot162MovieChannels = 2u;
    uint32_t boot162MovieInterleave = 0x200u;
    uint32_t boot162MovieBodyRemaining = 0u;

    bool boot162MovieHeaderParsed = false;
    bool boot162MovieBodyStarted = false;
    bool boot162MovieUnsupported = false;

    AudioStream boot162MovieStream{};
    bool boot162MovieStreamLoaded = false;
    bool boot162MovieStreamStarted = false;

    uint64_t boot162MovieBytesReceived = 0u;
    uint64_t boot162MovieFramesSubmitted = 0u;
};

PS2AudioBackend::PS2AudioBackend() : m_impl(std::make_unique<Impl>())
{
}

PS2AudioBackend::~PS2AudioBackend()
{
    if (m_impl)
        stopAll();
}

void PS2AudioBackend::onVagTransfer(const uint8_t *rdram, uint32_t srcAddr, uint32_t sizeBytes)
{
    if (!rdram || sizeBytes < 48)
        return;

    const uint32_t physAddr = srcAddr & PS2_RAM_MASK;
    if (physAddr + sizeBytes > PS2_RAM_SIZE)
        return;

    std::vector<int16_t> pcm;
    uint32_t sampleRate = 44100;
    if (!ps2_vag::decode(rdram + physAddr, sizeBytes, pcm, sampleRate))
        return;

    std::lock_guard<std::mutex> lock(m_mutex);
    DecodedSample sample;
    sample.pcm = std::move(pcm);
    sample.sampleRate = sampleRate;
    m_sampleBank[physAddr] = std::move(sample);
    m_mostRecentSampleKey = physAddr;
}

void PS2AudioBackend::onVagTransferFromBuffer(const uint8_t *data, uint32_t sizeBytes, uint32_t keyAddr)
{
    if (!data || sizeBytes < 48)
        return;

    std::vector<int16_t> pcm;
    uint32_t sampleRate = 44100;
    if (!ps2_vag::decode(data, sizeBytes, pcm, sampleRate))
        return;

    const uint32_t physAddr = keyAddr & PS2_RAM_MASK;
    std::lock_guard<std::mutex> lock(m_mutex);
    DecodedSample sample;
    sample.pcm = std::move(pcm);
    sample.sampleRate = sampleRate;
    m_sampleBank[physAddr] = sample;
    m_mostRecentSampleKey = physAddr;
    m_loadOrderSamples.push_back(std::move(sample));
    m_loadOrderSampleKeys.push_back(physAddr);
    constexpr size_t kMaxLoadOrderSamples = 32;
    if (m_loadOrderSamples.size() > kMaxLoadOrderSamples)
    {
        m_loadOrderSamples.erase(m_loadOrderSamples.begin());
        m_loadOrderSampleKeys.erase(m_loadOrderSampleKeys.begin());
    }
}

namespace
{
    constexpr uint32_t LIBSD_CMD_SET_VOICE = 0x8010u;
}


void PS2AudioBackend::onMoviePcmTransfer(const uint8_t *rdram,
                                         uint32_t srcAddr,
                                         uint32_t sizeBytes)
{
#if defined(PLATFORM_VITA)
    (void)rdram;
    (void)srcAddr;
    (void)sizeBytes;
    return;
#else
    if (!m_audioReady || !m_impl || !rdram || sizeBytes == 0u)
        return;

    const uint32_t physAddr = srcAddr & PS2_RAM_MASK;

    if (physAddr >= PS2_RAM_SIZE ||
        sizeBytes > (PS2_RAM_SIZE - physAddr))
    {
        return;
    }

    std::lock_guard<std::mutex> lock(m_mutex);

    auto &st = *m_impl;

    if (st.boot162MovieUnsupported)
        return;

    st.boot162MovieBytesReceived += sizeBytes;

    st.boot162MovieInput.insert(
        st.boot162MovieInput.end(),
        rdram + physAddr,
        rdram + physAddr + sizeBytes);

    auto readLe32 = [](const uint8_t *p) -> uint32_t
    {
        return
            static_cast<uint32_t>(p[0]) |
            (static_cast<uint32_t>(p[1]) << 8u) |
            (static_cast<uint32_t>(p[2]) << 16u) |
            (static_cast<uint32_t>(p[3]) << 24u);
    };

    auto findSignature =
        [](const std::vector<uint8_t> &buf,
           const char *sig,
           size_t start) -> size_t
    {
        if (!sig || buf.size() < 4u || start >= buf.size())
            return static_cast<size_t>(-1);

        for (size_t i = start; i + 4u <= buf.size(); ++i)
        {
            if (buf[i + 0u] == static_cast<uint8_t>(sig[0]) &&
                buf[i + 1u] == static_cast<uint8_t>(sig[1]) &&
                buf[i + 2u] == static_cast<uint8_t>(sig[2]) &&
                buf[i + 3u] == static_cast<uint8_t>(sig[3]))
            {
                return i;
            }
        }

        return static_cast<size_t>(-1);
    };

    if (!st.boot162MovieBodyStarted)
    {
        const size_t sh =
            findSignature(st.boot162MovieInput, "SShd", 0u);

        if (sh == static_cast<size_t>(-1))
        {
            // Preserve a few trailing bytes in case a signature straddles
            // two guest DMA transfers.
            if (st.boot162MovieInput.size() > 4096u)
            {
                st.boot162MovieInput.erase(
                    st.boot162MovieInput.begin(),
                    st.boot162MovieInput.end() - 3);
            }

            return;
        }

        if (st.boot162MovieInput.size() < sh + 32u)
            return;

        const uint32_t structSize =
            readLe32(st.boot162MovieInput.data() + sh + 4u);

        if (structSize < 24u)
        {
            std::fprintf(
                stderr,
                "[dothack:boot162-movie-audio] "
                "stage=unsupported-header structSize=%u\n",
                structSize);

            st.boot162MovieUnsupported = true;
            return;
        }

        const uint32_t type =
            readLe32(st.boot162MovieInput.data() + sh + 8u);

        const uint32_t sampleRate =
            readLe32(st.boot162MovieInput.data() + sh + 12u);

        const uint32_t channels =
            readLe32(st.boot162MovieInput.data() + sh + 16u);

        const uint32_t interleave =
            readLe32(st.boot162MovieInput.data() + sh + 20u);

        const size_t ssbdSearchStart =
            sh + 8u + static_cast<size_t>(structSize);

        const size_t ssbd =
            findSignature(
                st.boot162MovieInput,
                "SSbd",
                ssbdSearchStart);

        if (ssbd == static_cast<size_t>(-1) ||
            st.boot162MovieInput.size() < ssbd + 8u)
        {
            return;
        }

        const uint32_t bodySize =
            readLe32(st.boot162MovieInput.data() + ssbd + 4u);

        st.boot162MovieHeaderParsed = true;

        if (type != 1u ||
            sampleRate == 0u ||
            channels != 2u ||
            interleave < 2u ||
            (interleave & 1u) != 0u)
        {
            std::fprintf(
                stderr,
                "[dothack:boot162-movie-audio] "
                "stage=unsupported-format "
                "type=%u rate=%u channels=%u interleave=%u "
                "body=%u\n",
                type,
                sampleRate,
                channels,
                interleave,
                bodySize);

            st.boot162MovieUnsupported = true;
            return;
        }

        st.boot162MovieSampleRate = sampleRate;
        st.boot162MovieChannels = channels;
        st.boot162MovieInterleave = interleave;
        st.boot162MovieBodyRemaining = bodySize;
        st.boot162MovieBodyStarted = true;

        st.boot162MovieInput.erase(
            st.boot162MovieInput.begin(),
            st.boot162MovieInput.begin() +
                static_cast<std::ptrdiff_t>(ssbd + 8u));

        std::fprintf(
            stderr,
            "[dothack:boot162-movie-audio] "
            "stage=header "
            "type=%u rate=%u channels=%u "
            "interleave=%u body=%u\n",
            type,
            sampleRate,
            channels,
            interleave,
            bodySize);
    }

    const size_t channelBytes =
        static_cast<size_t>(st.boot162MovieInterleave);

    const size_t pairBytes =
        channelBytes * 2u;

    if (pairBytes == 0u)
        return;

    while (st.boot162MovieBodyRemaining >= pairBytes &&
           st.boot162MovieInput.size() >= pairBytes)
    {
        const uint8_t *left =
            st.boot162MovieInput.data();

        const uint8_t *right =
            st.boot162MovieInput.data() + channelBytes;

        const size_t samplesPerChannel =
            channelBytes / sizeof(int16_t);

        st.boot162MoviePcm.reserve(
            st.boot162MoviePcm.size() +
            samplesPerChannel * 2u);

        for (size_t i = 0u; i < channelBytes; i += 2u)
        {
            const uint16_t lu =
                static_cast<uint16_t>(left[i + 0u]) |
                static_cast<uint16_t>(
                    static_cast<uint16_t>(left[i + 1u]) << 8u);

            const uint16_t ru =
                static_cast<uint16_t>(right[i + 0u]) |
                static_cast<uint16_t>(
                    static_cast<uint16_t>(right[i + 1u]) << 8u);

            st.boot162MoviePcm.push_back(
                static_cast<int16_t>(lu));

            st.boot162MoviePcm.push_back(
                static_cast<int16_t>(ru));
        }

        st.boot162MovieInput.erase(
            st.boot162MovieInput.begin(),
            st.boot162MovieInput.begin() +
                static_cast<std::ptrdiff_t>(pairBytes));

        st.boot162MovieBodyRemaining -=
            static_cast<uint32_t>(pairBytes);
    }

    // BOOT 166: exact Raylib stream-buffer sizing.
    //
    // Raylib treats UpdateAudioStream() as a complete half-buffer
    // replacement and zero-fills unused frames.  Keep our submitted
    // frame count identical to the explicitly requested stream
    // half-buffer size so no artificial silence is inserted.
    constexpr size_t kFramesPerUpdate = 4096u;
    constexpr size_t kSamplesPerFrame = 2u;
    constexpr size_t kSamplesPerUpdate =
        kFramesPerUpdate * kSamplesPerFrame;

    if (!st.boot162MovieStreamLoaded &&
        st.boot162MoviePcm.size() >= kSamplesPerUpdate)
    {
        // BOOT 166:
        // Force Raylib's streaming half-buffer to the same number of
        // frames supplied by both movie-audio update paths.
        //
        // The current host reports a total device period footprint of
        // 3600 frames.  A requested 4096-frame half-buffer is therefore
        // above the individual period-size floor on this host.
        constexpr int kBoot166MovieBufferFrames = 4096;

        SetAudioStreamBufferSizeDefault(
            kBoot166MovieBufferFrames);

        st.boot162MovieStream =
            LoadAudioStream(
                st.boot162MovieSampleRate,
                16,
                2);

        // Restore Raylib's automatic default for any later streams.
        SetAudioStreamBufferSizeDefault(0);

        st.boot162MovieStreamLoaded = true;

        std::fprintf(
            stderr,
            "[dothack:boot166-movie-audio] "
            "stage=stream-buffer "
            "frames=%d rate=%u channels=2 "
            "bytesPerFrame=4\n",
            kBoot166MovieBufferFrames,
            st.boot162MovieSampleRate);

        SetAudioStreamVolume(
            st.boot162MovieStream,
            1.0f);

        std::fprintf(
            stderr,
            "[dothack:boot162-movie-audio] "
            "stage=stream-created rate=%u channels=2\n",
            st.boot162MovieSampleRate);
    }

    if (!st.boot162MovieStreamLoaded)
        return;

    static uint32_t s_boot162SubmitLogCount = 0u;

    while (st.boot162MoviePcm.size() >= kSamplesPerUpdate &&
           IsAudioStreamProcessed(st.boot162MovieStream))
    {
        UpdateAudioStream(
            st.boot162MovieStream,
            st.boot162MoviePcm.data(),
            static_cast<int>(kFramesPerUpdate));

        st.boot162MoviePcm.erase(
            st.boot162MoviePcm.begin(),
            st.boot162MoviePcm.begin() +
                static_cast<std::ptrdiff_t>(kSamplesPerUpdate));

        st.boot162MovieFramesSubmitted +=
            kFramesPerUpdate;

        if (!st.boot162MovieStreamStarted)
        {
            PlayAudioStream(
                st.boot162MovieStream);

            st.boot162MovieStreamStarted = true;

            std::fprintf(
                stderr,
                "[dothack:boot162-movie-audio] "
                "stage=stream-start\n");
        }

        if (s_boot162SubmitLogCount < 64u)
        {
            std::fprintf(
                stderr,
                "[dothack:boot162-movie-audio] "
                "stage=submit n=%u frames=%zu "
                "totalFrames=%llu pendingSamples=%zu "
                "bodyRemaining=%u\n",
                s_boot162SubmitLogCount++,
                kFramesPerUpdate,
                static_cast<unsigned long long>(
                    st.boot162MovieFramesSubmitted),
                st.boot162MoviePcm.size(),
                st.boot162MovieBodyRemaining);
        }
    }

    if (st.boot162MovieStreamStarted &&
        !IsAudioStreamPlaying(st.boot162MovieStream))
    {
        PlayAudioStream(st.boot162MovieStream);
    }
#endif
}

// BOOT 165: periodic host movie-audio refill.
//
// onMoviePcmTransfer() can receive a large burst of PSS payloads much
// faster than the physical audio device consumes them.  Once both
// Raylib stream buffers become busy, decoded PCM remains queued in
// boot162MoviePcm.  The original Boot 162 implementation had no later
// opportunity to submit those samples.
//
// sceMpegGetPicture() now calls this once per movie presentation cycle.
void PS2AudioBackend::pumpMovieAudio()
{
#if defined(PLATFORM_VITA)
    return;
#else
    if (!m_audioReady || !m_impl)
        return;

    std::lock_guard<std::mutex> lock(m_mutex);

    auto &st = *m_impl;

    if (!st.boot162MovieStreamLoaded)
        return;

    // BOOT 166: exact Raylib stream-buffer sizing.
    //
    // Raylib treats UpdateAudioStream() as a complete half-buffer
    // replacement and zero-fills unused frames.  Keep our submitted
    // frame count identical to the explicitly requested stream
    // half-buffer size so no artificial silence is inserted.
    constexpr size_t kFramesPerUpdate = 4096u;
    constexpr size_t kSamplesPerFrame = 2u;
    constexpr size_t kSamplesPerUpdate =
        kFramesPerUpdate * kSamplesPerFrame;

    static uint32_t s_boot165PumpLogCount = 0u;
    static uint32_t s_boot165SubmitLogCount = 0u;
    static uint32_t s_boot165RestartLogCount = 0u;

    const bool processedBefore =
        IsAudioStreamProcessed(st.boot162MovieStream);

    const bool playingBefore =
        IsAudioStreamPlaying(st.boot162MovieStream);

    if (s_boot165PumpLogCount < 180u)
    {
        std::fprintf(
            stderr,
            "[dothack:boot165-audio-pump] "
            "stage=pump n=%u "
            "pendingSamples=%zu "
            "processed=%d playing=%d "
            "totalFrames=%llu\n",
            s_boot165PumpLogCount++,
            st.boot162MoviePcm.size(),
            processedBefore ? 1 : 0,
            playingBefore ? 1 : 0,
            static_cast<unsigned long long>(
                st.boot162MovieFramesSubmitted));
    }

    while (st.boot162MoviePcm.size() >= kSamplesPerUpdate &&
           IsAudioStreamProcessed(st.boot162MovieStream))
    {
        int32_t peak = 0;
        size_t nonZero = 0u;

        for (size_t i = 0u; i < kSamplesPerUpdate; ++i)
        {
            const int32_t sample =
                static_cast<int32_t>(
                    st.boot162MoviePcm[i]);

            const int32_t magnitude =
                sample < 0 ? -sample : sample;

            if (magnitude > peak)
                peak = magnitude;

            if (sample != 0)
                ++nonZero;
        }

        UpdateAudioStream(
            st.boot162MovieStream,
            st.boot162MoviePcm.data(),
            static_cast<int>(kFramesPerUpdate));

        st.boot162MoviePcm.erase(
            st.boot162MoviePcm.begin(),
            st.boot162MoviePcm.begin() +
                static_cast<std::ptrdiff_t>(
                    kSamplesPerUpdate));

        st.boot162MovieFramesSubmitted +=
            kFramesPerUpdate;

        if (s_boot165SubmitLogCount < 160u)
        {
            std::fprintf(
                stderr,
                "[dothack:boot165-audio-pump] "
                "stage=submit n=%u "
                "frames=%zu "
                "peak=%d nonZero=%zu/%zu "
                "pendingSamples=%zu "
                "totalFrames=%llu\n",
                s_boot165SubmitLogCount++,
                kFramesPerUpdate,
                peak,
                nonZero,
                kSamplesPerUpdate,
                st.boot162MoviePcm.size(),
                static_cast<unsigned long long>(
                    st.boot162MovieFramesSubmitted));
        }
    }

    // An underrun can cause the host stream to stop.  Refill first,
    // then restart it once usable data has been supplied.
    if (st.boot162MovieStreamStarted &&
        !IsAudioStreamPlaying(st.boot162MovieStream))
    {
        PlayAudioStream(st.boot162MovieStream);

        if (s_boot165RestartLogCount < 32u)
        {
            std::fprintf(
                stderr,
                "[dothack:boot165-audio-pump] "
                "stage=restart n=%u "
                "pendingSamples=%zu "
                "totalFrames=%llu\n",
                s_boot165RestartLogCount++,
                st.boot162MoviePcm.size(),
                static_cast<unsigned long long>(
                    st.boot162MovieFramesSubmitted));
        }
    }
#endif
}

void PS2AudioBackend::onSoundCommand(uint32_t sid, uint32_t rpcNum,
                                     const uint8_t *sendBuf, uint32_t sendSize,
                                     uint8_t *recvBuf, uint32_t recvSize)
{
    if (sid != 0x80000701u)
        return;

    if ((rpcNum == LIBSD_CMD_SET_VOICE || (rpcNum & 0xFF00u) == 0x8100u) &&
        sendBuf && sendSize >= 20)
    {
        uint32_t sampleAddr = 0;
        uint32_t voiceIndex = 0xFFFFFFFFu;
        for (int vo = 4; vo >= 0 && voiceIndex == 0xFFFFFFFFu; vo -= 4)
        {
            if (vo < static_cast<int>(sendSize))
            {
                uint32_t v = 0;
                std::memcpy(&v, sendBuf + vo, sizeof(v));
                if (v < 24u)
                    voiceIndex = v;
            }
        }

        constexpr uint32_t kMinPlausibleAddr = 0x1000u;
        for (int off = 12; off <= 24 && sampleAddr == 0; off += 4)
        {
            if (sendSize >= static_cast<uint32_t>(off + 4))
            {
                uint32_t cand = 0;
                std::memcpy(&cand, sendBuf + off, sizeof(cand));
                if (cand >= kMinPlausibleAddr && (cand <= PS2_RAM_MASK || (cand & ~PS2_RAM_MASK) == 0))
                    sampleAddr = cand;
            }
        }
        if (sampleAddr == 0)
            sampleAddr = m_mostRecentSampleKey;

        float pitch = 1.0f;
        if (sendSize >= 12)
        {
            uint16_t pitchHalf = 0;
            std::memcpy(&pitchHalf, sendBuf + 8, sizeof(pitchHalf));
            if (pitchHalf != 0)
                pitch = 4096.0f / static_cast<float>(pitchHalf);
        }
        play(sampleAddr, pitch, 1.0f, voiceIndex);
    }
}

void PS2AudioBackend::play(uint32_t sampleAddr, float pitch, float volume, uint32_t voiceIndex)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    DecodedSample *sampleToPlay = nullptr;
    uint32_t sampleKey = 0;

    auto it = m_sampleBank.find(sampleAddr & PS2_RAM_MASK);
    if (it != m_sampleBank.end())
    {
        sampleToPlay = &it->second;
        sampleKey = it->first;
    }
    else if (voiceIndex != 0xFFFFFFFFu &&
             voiceIndex < m_loadOrderSamples.size() &&
             voiceIndex < m_loadOrderSampleKeys.size())
    {
        sampleToPlay = &m_loadOrderSamples[voiceIndex];
        sampleKey = m_loadOrderSampleKeys[voiceIndex];
    }
    else
    {
        it = m_sampleBank.find(m_mostRecentSampleKey);
        if (it == m_sampleBank.end())
            return;
        sampleToPlay = &it->second;
        sampleKey = it->first;
    }
    if (!sampleToPlay || sampleToPlay->pcm.empty())
        return;

    const bool isBgm = (sampleToPlay->pcm.size() > static_cast<size_t>(sampleToPlay->sampleRate * 5));
    playDecodedSample(sampleKey, *sampleToPlay, pitch, volume, isBgm);
}

void PS2AudioBackend::pruneFinishedSounds()
{
#if defined(PLATFORM_VITA)
    return;
#else
    auto &sounds = m_impl->activeSounds;
    auto it = sounds.begin();
    while (it != sounds.end())
    {
        if (!IsSoundPlaying(it->snd))
        {
            UnloadSound(it->snd);
            it = sounds.erase(it);
        }
        else
        {
            ++it;
        }
    }
#endif
}

void PS2AudioBackend::playDecodedSample(uint32_t sampleKey, DecodedSample &sample, float pitch, float volume,
                                        bool isBgm)
{
#if defined(PLATFORM_VITA)
    (void)sampleKey;
    (void)sample;
    (void)pitch;
    (void)volume;
    (void)isBgm;
    return;
#else
    if (!m_audioReady || sample.pcm.empty())
        return;

    pruneFinishedSounds();

    for (const auto &t : m_impl->activeSounds)
    {
        if (t.sampleKey == sampleKey && IsSoundPlaying(t.snd))
            return;
    }

    auto &sounds = m_impl->activeSounds;
    if (isBgm)
    {
        for (auto it = sounds.begin(); it != sounds.end();)
        {
            if (IsSoundPlaying(it->snd))
            {
                StopSound(it->snd);
                UnloadSound(it->snd);
                it = sounds.erase(it);
            }
            else
                ++it;
        }
    }

    constexpr int kMaxConcurrentSounds = 4;
    while (static_cast<int>(sounds.size()) >= kMaxConcurrentSounds)
    {
        StopSound(sounds.front().snd);
        UnloadSound(sounds.front().snd);
        sounds.erase(sounds.begin());
    }

    std::vector<uint8_t> wav = buildWavFromPcm(sample.pcm.data(), sample.pcm.size(), sample.sampleRate);
    Wave wave = LoadWaveFromMemory(".wav", wav.data(), static_cast<int>(wav.size()));
    if (wave.frameCount <= 0)
        return;
    Sound snd = LoadSoundFromWave(wave);
    UnloadWave(wave);
    SetSoundPitch(snd, pitch);
    SetSoundVolume(snd, volume);
    m_impl->activeSounds.push_back({snd, sampleKey});
    PlaySound(snd);
#endif
}

void PS2AudioBackend::stop(uint32_t voiceId)
{
    (void)voiceId;
}

void PS2AudioBackend::stopAll()
{
    std::lock_guard<std::mutex> lock(m_mutex);
#if defined(PLATFORM_VITA)
    return;
#else
    // BOOT 162 movie stream cleanup.
    if (m_impl->boot162MovieStreamLoaded)
    {
        StopAudioStream(m_impl->boot162MovieStream);
        UnloadAudioStream(m_impl->boot162MovieStream);

        m_impl->boot162MovieStreamLoaded = false;
        m_impl->boot162MovieStreamStarted = false;
    }

    m_impl->boot162MovieInput.clear();
    m_impl->boot162MoviePcm.clear();

    for (auto &t : m_impl->activeSounds)
    {
        StopSound(t.snd);
        UnloadSound(t.snd);
    }
    m_impl->activeSounds.clear();
#endif
}

#include "Common.h"
#include "MPEG.h"
#include "runtime/ee_scheduler.h"

#if !defined(PS2X_HAS_FFMPEG)
#define PS2X_HAS_FFMPEG 1
#endif

#if PS2X_HAS_FFMPEG
extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavutil/error.h>
#include <libavutil/log.h>
#include <libswscale/swscale.h>
}
#endif

#include <deque>
#include <memory>

#include "Syscalls/Helpers/State.h"

namespace ps2_stubs
{

    // BOOT 167: CD-side bounded host PSS refill helper.
    size_t refillDothackHostPssStream(
        PS2Runtime *runtime,
        size_t maxBytes);

    namespace
    {
        struct MpegDecodedFrame
        {
            int width = 0;
            int height = 0;
            int repeatPict = 0;
            int64_t pts90k = -1;
            std::vector<uint8_t> rgba;
        };

#if PS2X_HAS_FFMPEG
        std::string ffmpegErrorString(int err)
        {
            std::array<char, AV_ERROR_MAX_STRING_SIZE> buffer{};
            if (av_strerror(err, buffer.data(), buffer.size()) < 0)
            {
                return "unknown FFmpeg error";
            }
            return std::string(buffer.data());
        }

        void configureFfmpegLogLevel()
        {
            static std::once_flag s_once;
            std::call_once(s_once, []
                           {
#if AGRESSIVE_LOGS
                               av_log_set_level(AV_LOG_WARNING);
#else
                               av_log_set_level(AV_LOG_ERROR);
#endif
                           });
        }

        class MpegFfmpegDecoder
        {
        public:
            MpegFfmpegDecoder() = default;

            ~MpegFfmpegDecoder()
            {
                reset();
            }

            MpegFfmpegDecoder(const MpegFfmpegDecoder &) = delete;
            MpegFfmpegDecoder &operator=(const MpegFfmpegDecoder &) = delete;

            bool feed(const uint8_t *data, size_t size, std::deque<MpegDecodedFrame> &frames, int64_t pts90k = -1, int64_t dts90k = -1)
            {
                if (!data || size == 0)
                {
                    return true;
                }

                if (!ensureInitialized())
                {
                    return false;
                }

                static uint32_t s_feedLogCount = 0u;
                const bool shouldLog = (s_feedLogCount < 32u);
                if (shouldLog)
                    ++s_feedLogCount;

                size_t totalParsed = 0u;
                size_t totalPacketsSent = 0u;
                size_t framesBefore = frames.size();

                const uint8_t *cursor = data;
                size_t remaining = size;
                int64_t parserPts = pts90k >= 0 ? pts90k : AV_NOPTS_VALUE;
                int64_t parserDts = dts90k >= 0 ? dts90k : AV_NOPTS_VALUE;
                while (remaining > 0)
                {
                    uint8_t *packetData = nullptr;
                    int packetSize = 0;
                    const int chunk = static_cast<int>(std::min<size_t>(
                        remaining, static_cast<size_t>(std::numeric_limits<int>::max())));
                    const int used = av_parser_parse2(
                        m_parser,
                        m_codecCtx,
                        &packetData,
                        &packetSize,
                        cursor,
                        chunk,
                        parserPts,
                        parserDts,
                        0);
                    if (used < 0)
                    {
                        std::cerr << "[MPEG] parser failed: " << ffmpegErrorString(used) << std::endl;
                        return false;
                    }
                    if (used == 0 && packetSize == 0)
                    {
                        break;
                    }

                    totalParsed += static_cast<size_t>(used);
                    cursor += used;
                    remaining -= static_cast<size_t>(used);

                    if (used > 0)
                    {
                        parserPts = AV_NOPTS_VALUE;
                        parserDts = AV_NOPTS_VALUE;
                    }

                    if (packetSize > 0)
                    {
                        ++totalPacketsSent;
                        if (!sendPacket(packetData, static_cast<size_t>(packetSize), frames, m_parser->pts, m_parser->dts))
                        {
                            return false;
                        }
                    }
                }

                if (shouldLog)
                {
                    PS2_IF_AGRESSIVE_LOGS({
                        std::cerr << "[MPEG:feed] inSize=" << size
                                  << " parsed=" << totalParsed
                                  << " packets=" << totalPacketsSent
                                  << " newFrames=" << (frames.size() - framesBefore)
                                  << " totalFrames=" << frames.size()
                                  << std::endl;
                    });
                }

                return true;
            }

            bool flush(std::deque<MpegDecodedFrame> &frames)
            {
                if (!m_initialized || m_drained)
                {
                    return true;
                }

                if (m_parser)
                {
                    uint8_t *packetData = nullptr;
                    int packetSize = 0;
                    const int used = av_parser_parse2(
                        m_parser,
                        m_codecCtx,
                        &packetData,
                        &packetSize,
                        nullptr,
                        0,
                        AV_NOPTS_VALUE,
                        AV_NOPTS_VALUE,
                        0);
                    (void)used;
                    if (packetSize > 0 && !sendPacket(packetData, static_cast<size_t>(packetSize), frames, m_parser->pts, m_parser->dts))
                    {
                        return false;
                    }
                }

                const int sendRet = avcodec_send_packet(m_codecCtx, nullptr);
                if (sendRet < 0 && sendRet != AVERROR_EOF)
                {
                    std::cerr << "[MPEG] decoder flush failed: " << ffmpegErrorString(sendRet) << std::endl;
                    return false;
                }

                const bool ok = receiveFrames(frames);
                m_drained = true;
                return ok;
            }

            void reset()
            {
                if (m_swsCtx)
                {
                    sws_freeContext(m_swsCtx);
                    m_swsCtx = nullptr;
                }
                if (m_frame)
                {
                    av_frame_free(&m_frame);
                }
                if (m_packet)
                {
                    av_packet_free(&m_packet);
                }
                if (m_codecCtx)
                {
                    avcodec_free_context(&m_codecCtx);
                }
                if (m_parser)
                {
                    av_parser_close(m_parser);
                    m_parser = nullptr;
                }

                m_swsWidth = 0;
                m_swsHeight = 0;
                m_swsFormat = AV_PIX_FMT_NONE;
                m_initialized = false;
                m_drained = false;
            }

        private:
            bool ensureInitialized()
            {
                if (m_initialized)
                {
                    return true;
                }

                configureFfmpegLogLevel();

                const AVCodec *codec = avcodec_find_decoder(AV_CODEC_ID_MPEG2VIDEO);
                if (!codec)
                {
                    std::cerr << "[MPEG] FFmpeg MPEG-2 decoder not found." << std::endl;
                    return false;
                }

                m_parser = av_parser_init(AV_CODEC_ID_MPEG2VIDEO);
                if (!m_parser)
                {
                    std::cerr << "[MPEG] FFmpeg MPEG-video parser not found." << std::endl;
                    return false;
                }

                m_codecCtx = avcodec_alloc_context3(codec);
                m_frame = av_frame_alloc();
                m_packet = av_packet_alloc();
                if (!m_codecCtx || !m_frame || !m_packet)
                {
                    std::cerr << "[MPEG] failed to allocate FFmpeg decoder state." << std::endl;
                    reset();
                    return false;
                }

                m_codecCtx->thread_count = 1;
                m_codecCtx->pkt_timebase = AVRational{1, 90000};
                // feedElementaryStream() does not create the decoder until a valid
                // MPEG sequence header has been found.  Dropping non-key pictures
                // here therefore throws away real presentation frames and makes
                // movies finish early once the EE is fast enough to drain them.
                m_codecCtx->skip_frame = AVDISCARD_DEFAULT;
                m_codecCtx->err_recognition = 0;
                const int ret = avcodec_open2(m_codecCtx, codec, nullptr);
                if (ret < 0)
                {
                    std::cerr << "[MPEG] failed to open MPEG decoder: " << ffmpegErrorString(ret) << std::endl;
                    reset();
                    return false;
                }

                m_initialized = true;
                m_drained = false;
                return true;
            }

            bool sendPacket(const uint8_t *data,
                            size_t size,
                            std::deque<MpegDecodedFrame> &frames,
                            int64_t pts = AV_NOPTS_VALUE,
                            int64_t dts = AV_NOPTS_VALUE)
            {
                if (!data || size == 0)
                {
                    return true;
                }

                av_packet_unref(m_packet);
                const int allocRet = av_new_packet(m_packet, static_cast<int>(size));
                if (allocRet < 0)
                {
                    std::cerr << "[MPEG] failed to allocate packet: " << ffmpegErrorString(allocRet) << std::endl;
                    return false;
                }
                std::memcpy(m_packet->data, data, size);
                m_packet->pts = pts;
                m_packet->dts = dts;

                int ret = avcodec_send_packet(m_codecCtx, m_packet);
                if (ret == AVERROR(EAGAIN))
                {
                    if (!receiveFrames(frames))
                    {
                        av_packet_unref(m_packet);
                        return false;
                    }
                    ret = avcodec_send_packet(m_codecCtx, m_packet);
                }
                av_packet_unref(m_packet);
                if (ret < 0 && ret != AVERROR(EAGAIN))
                {
                    static uint32_t s_rejectedPacketLogCount = 0u;
                    if (s_rejectedPacketLogCount < 32u)
                    {
                        std::cerr << "[MPEG] decoder rejected packet, dropping: "
                                  << ffmpegErrorString(ret) << std::endl;
                        ++s_rejectedPacketLogCount;
                    }
                    return true;
                }

                return receiveFrames(frames);
            }

            bool receiveFrames(std::deque<MpegDecodedFrame> &frames)
            {
                while (true)
                {
                    const int ret = avcodec_receive_frame(m_codecCtx, m_frame);
                    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                    {
                        return true;
                    }
                    if (ret < 0)
                    {
                        static uint32_t s_receiveErrorLogCount = 0u;
                        if (s_receiveErrorLogCount < 32u)
                        {
                            std::cerr << "[MPEG] decoder receive failed, dropping: "
                                      << ffmpegErrorString(ret) << std::endl;
                            ++s_receiveErrorLogCount;
                        }
                        return true;
                    }

                    if (!convertFrame(frames))
                    {
                        av_frame_unref(m_frame);
                        return false;
                    }
                    av_frame_unref(m_frame);
                }
            }

            bool convertFrame(std::deque<MpegDecodedFrame> &frames)
            {
                const int width = m_frame->width;
                const int height = m_frame->height;
                const AVPixelFormat srcFormat = static_cast<AVPixelFormat>(m_frame->format);
                if (width <= 0 || height <= 0 || srcFormat == AV_PIX_FMT_NONE)
                {
                    return false;
                }

                if (!m_swsCtx ||
                    m_swsWidth != width ||
                    m_swsHeight != height ||
                    m_swsFormat != srcFormat)
                {
                    if (m_swsCtx)
                    {
                        sws_freeContext(m_swsCtx);
                        m_swsCtx = nullptr;
                    }
                    m_swsCtx = sws_getContext(
                        width,
                        height,
                        srcFormat,
                        width,
                        height,
                        AV_PIX_FMT_RGBA,
                        SWS_BILINEAR,
                        nullptr,
                        nullptr,
                        nullptr);
                    if (!m_swsCtx)
                    {
                        std::cerr << "[MPEG] failed to create FFmpeg scaler." << std::endl;
                        return false;
                    }
                    m_swsWidth = width;
                    m_swsHeight = height;
                    m_swsFormat = srcFormat;
                }

                MpegDecodedFrame decoded;
                decoded.width = width;
                decoded.height = height;
                decoded.repeatPict = std::max(0, m_frame->repeat_pict);
                decoded.pts90k = m_frame->best_effort_timestamp != AV_NOPTS_VALUE
                                     ? m_frame->best_effort_timestamp
                                     : -1;
                decoded.rgba.resize(static_cast<size_t>(width) * static_cast<size_t>(height) * 4u);

                uint8_t *dstData[4] = {decoded.rgba.data(), nullptr, nullptr, nullptr};
                int dstLinesize[4] = {width * 4, 0, 0, 0};
                const int scaledRows = sws_scale(
                    m_swsCtx,
                    m_frame->data,
                    m_frame->linesize,
                    0,
                    height,
                    dstData,
                    dstLinesize);
                if (scaledRows <= 0)
                {
                    std::cerr << "[MPEG] FFmpeg scaler produced no rows." << std::endl;
                    return false;
                }

                frames.push_back(std::move(decoded));
                return true;
            }

            AVCodecParserContext *m_parser = nullptr;
            AVCodecContext *m_codecCtx = nullptr;
            AVFrame *m_frame = nullptr;
            AVPacket *m_packet = nullptr;
            SwsContext *m_swsCtx = nullptr;
            int m_swsWidth = 0;
            int m_swsHeight = 0;
            AVPixelFormat m_swsFormat = AV_PIX_FMT_NONE;
            bool m_initialized = false;
            bool m_drained = false;
        };
#else
        // TODO
        class MpegFfmpegDecoder
        {
        public:
            bool feed(const uint8_t *, size_t, std::deque<MpegDecodedFrame> &, int64_t = -1, int64_t = -1)
            {
                static bool s_warnedNoFfmpeg = false;
                if (!s_warnedNoFfmpeg)
                {
                    std::cerr << "[MPEG] runtime built without FFmpeg; MPEG video decode is disabled." << std::endl;
                    s_warnedNoFfmpeg = true;
                }
                return false;
            }

            bool flush(std::deque<MpegDecodedFrame> &)
            {
                return true;
            }

            void reset() {}
        };
#endif

        struct MpegRegisteredCallback
        {
            uint32_t type = 0u;
            uint32_t streamId = 0u;
            uint32_t func = 0u;
            uint32_t data = 0u;
            uint32_t handle = 0u;
            bool stream = false;
        };

        constexpr uint64_t kPictureClockOne = 1ull << 32u;
        // NTSC-style fields at ~59.94 Hz to keep MPEG timing yet (29.97 fps).
        constexpr uint64_t kDefaultPictureIntervalQ32 = 2ull * kPictureClockOne;
        constexpr size_t kMpegTimingScanLimit = 4096u;
        constexpr size_t kMaxDecodedPicturesAhead = 8u;

        struct MpegPlaybackState
        {
            uint32_t picturesServed = 0u;
            uint32_t width = 320u;
            uint32_t height = 240u;
            uint32_t decodeMode = 0u;
            uint32_t imageBufferAddr = 0u;
            bool sawInput = false;
            bool sawSequenceEnd = false;
            bool streamEnded = false;
            bool decoderFailed = false;
            uint64_t cdStreamGeneration = 0u;
            bool waitingForVideoSequenceHeader = true;
            std::vector<uint8_t> videoSequenceSyncBuffer;
            std::vector<uint8_t> pssBuffer;
            std::vector<uint32_t> pssGuestAddrs;
            std::deque<MpegDecodedFrame> decodedFrames;
            std::unique_ptr<MpegFfmpegDecoder> decoder;
            uint8_t frameRateCode = 0u;
            uint8_t frameRateExtensionN = 0u;
            uint8_t frameRateExtensionD = 0u;
            bool hasFrameRateExtension = false;
            std::vector<uint8_t> videoTimingScanBuffer;
            uint64_t pictureIntervalQ32 = kDefaultPictureIntervalQ32;
            uint64_t nextPictureTickQ32 = std::numeric_limits<uint64_t>::max();
            uint64_t presentationEndTickQ32 = std::numeric_limits<uint64_t>::max();
            int64_t firstPresentedPts90k = -1;
            uint64_t ptsPresentationBaseTickQ32 = 0u;

            // BOOT 175:
            // Host-audio master-clock synchronization state.
            //
            // picturesServed counts pictures handed to the guest.
            // boot175DroppedFrames counts source pictures deliberately
            // discarded to keep source time aligned with real audio time.
            bool boot175HostClockArmed = false;
            double boot175HostClockBaseSeconds = 0.0;
            uint64_t boot175HostClockBaseSourceProgress = 0u;
            uint64_t boot175DroppedFrames = 0u;
            bool boot175FinalSyncLogged = false;
        };

        struct MpegStreamCallbackEvent
        {
            uint32_t mpegAddr = 0u;
            uint32_t streamType = 0u;
            uint32_t dataAddr = 0u;
            uint32_t len = 0u;
            uint64_t pts = 0xFFFFFFFFFFFFFFFFull;
            uint64_t dts = 0xFFFFFFFFFFFFFFFFull;
            std::vector<MpegRegisteredCallback> callbacks;

            // Host-fed PSS bytes do not have a guest address. Retain the
            // original payload here until it can be materialized in guest RAM
            // immediately before dispatching the registered guest callback.
            std::vector<uint8_t> hostPayload;
        };

        struct MpegStubState
        {
            bool initialized = false;
            uint32_t nextCallbackHandle = 1u;
            uint64_t cdStreamGeneration = 0u;
            uint64_t cdStreamBytesProduced = 0u;
            uint64_t cdStreamBytesDemuxed = 0u;
            bool cdStreamEofPending = false;
            bool currentCdStreamEofSeen = false;
            std::vector<uint8_t> cdStreamStagedBytes;       // host-fed ES held before a decoder exists
            uint64_t cdStreamStageGeneration = 0u;          // generation the stage belongs to
            bool cdStreamStageOverflowed = false;           // cap hit -> stage abandoned this generation

            // Complete host-fed stream packets whose callbacks need a real
            // guest payload address before they can be delivered.
            std::vector<MpegStreamCallbackEvent> pendingHostCallbackEvents;
            bool hostCallbackInFlight = false;

            uint32_t feedEsTraceCount = 0u;
            uint32_t demuxPssTraceCount = 0u;
            uint32_t demuxRingTraceCount = 0u;
            uint32_t getPictureWaitTraceCount = 0u;
            uint32_t pictureTraceCount = 0u;
            uint32_t isEndTraceCount = 0u;
            std::unordered_map<uint32_t, std::vector<MpegRegisteredCallback>> callbacksByMpeg;
            std::unordered_map<uint32_t, MpegPlaybackState> playbackByMpeg;
        };

        std::mutex g_mpeg_stub_mutex;
        constexpr uint32_t kMpegPictureWaitType = 1u;
        MpegStubState g_mpeg_stub_state;

        // TODO this resolution should follow runtime resolution
        constexpr uint32_t kStubMovieWidth = 320u;
        constexpr uint32_t kStubMovieHeight = 240u;
        constexpr uint32_t kMpegStrM2V = 0u;
        constexpr uint32_t kMpegStrPCM = 1u;
        constexpr uint32_t kMpegStrADPCM = 2u;
        constexpr uint8_t kMpegPackHeader = 0xBAu;
        constexpr uint8_t kMpegSystemHeader = 0xBBu;
        constexpr uint8_t kMpegProgramEnd = 0xB9u;
        constexpr uint8_t kMpegSequenceEnd = 0xB7u;
        constexpr uint8_t kMpegPrivateStream1 = 0xBDu;
        constexpr size_t kStartCodeNotFound = std::numeric_limits<size_t>::max();
        constexpr uint32_t kMpegCallbackDataSize = 0x20u;

        uint64_t mpegPictureIntervalQ32(uint8_t frameRateCode, uint8_t frameRateExtensionN = 0u, uint8_t frameRateExtensionD = 0u)
        {
            uint64_t frameRateNumerator = 0u;
            uint64_t frameRateDenominator = 1u;
            switch (frameRateCode)
            {
            case 1u:
                frameRateNumerator = 24000u;
                frameRateDenominator = 1001u;
                break;
            case 2u:
                frameRateNumerator = 24u;
                break;
            case 3u:
                frameRateNumerator = 25u;
                break;
            case 4u:
                frameRateNumerator = 30000u;
                frameRateDenominator = 1001u;
                break;
            case 5u:
                frameRateNumerator = 30u;
                break;
            case 6u:
                frameRateNumerator = 50u;
                break;
            case 7u:
                frameRateNumerator = 60000u;
                frameRateDenominator = 1001u;
                break;
            case 8u:
                frameRateNumerator = 60u;
                break;
            default:
                return 0u;
            }

            const uint64_t extensionNumerator = static_cast<uint64_t>(frameRateExtensionN) + 1u;
            const uint64_t extensionDenominator = static_cast<uint64_t>(frameRateExtensionD) + 1u;
            const uint64_t denominator = 1001u * frameRateNumerator * extensionNumerator;
            const uint64_t numerator = (60000u * frameRateDenominator * extensionDenominator) << 32u;
            return std::max(kPictureClockOne, (numerator + denominator / 2u) / denominator);
        }

        uint32_t readMpegBits(const uint8_t *data, size_t bitOffset, uint32_t bitCount)
        {
            uint32_t value = 0u;
            for (uint32_t bit = 0u; bit < bitCount; ++bit)
            {
                const size_t absoluteBit = bitOffset + bit;
                const uint8_t source = data[absoluteBit >> 3u];
                value = (value << 1u) | ((source >> (7u - static_cast<uint32_t>(absoluteBit & 7u))) & 1u);
            }
            return value;
        }

        void updateMpegPictureTiming(MpegPlaybackState &playback, const uint8_t *data, size_t size)
        {
            if (!data || size == 0u)
            {
                return;
            }

            playback.videoTimingScanBuffer.insert(playback.videoTimingScanBuffer.end(), data, data + size);
            if (playback.videoTimingScanBuffer.size() > kMpegTimingScanLimit)
            {
                const size_t discard = playback.videoTimingScanBuffer.size() - kMpegTimingScanLimit;
                playback.videoTimingScanBuffer.erase(playback.videoTimingScanBuffer.begin(), playback.videoTimingScanBuffer.begin() + static_cast<std::ptrdiff_t>(discard));
            }

            const std::vector<uint8_t> &buffer = playback.videoTimingScanBuffer;
            size_t lastSequenceHeader = kStartCodeNotFound;
            for (size_t i = 0u; i + 7u < buffer.size(); ++i)
            {
                if (buffer[i + 0u] == 0x00u &&
                    buffer[i + 1u] == 0x00u &&
                    buffer[i + 2u] == 0x01u &&
                    buffer[i + 3u] == 0xB3u)
                {
                    const uint8_t frameRateCode = buffer[i + 7u] & 0x0Fu;
                    if (mpegPictureIntervalQ32(frameRateCode) != 0u)
                    {
                        lastSequenceHeader = i;
                    }
                }
            }

            if (lastSequenceHeader == kStartCodeNotFound)
            {
                return;
            }

            playback.frameRateCode = buffer[lastSequenceHeader + 7u] & 0x0Fu;
            playback.frameRateExtensionN = 0u;
            playback.frameRateExtensionD = 0u;
            playback.hasFrameRateExtension = false;
            playback.pictureIntervalQ32 = mpegPictureIntervalQ32(playback.frameRateCode);

            for (size_t i = lastSequenceHeader + 8u; i + 9u < buffer.size(); ++i)
            {
                if (buffer[i + 0u] != 0x00u ||
                    buffer[i + 1u] != 0x00u ||
                    buffer[i + 2u] != 0x01u)
                {
                    continue;
                }

                if (buffer[i + 3u] == 0xB3u)
                {
                    break;
                }
                if (buffer[i + 3u] != 0xB5u)
                {
                    continue;
                }

                const uint8_t *extension = buffer.data() + i + 4u;
                if (readMpegBits(extension, 0u, 4u) != 1u)
                {
                    continue;
                }

                playback.frameRateExtensionN = static_cast<uint8_t>(readMpegBits(extension, 41u, 2u));
                playback.frameRateExtensionD = static_cast<uint8_t>(readMpegBits(extension, 43u, 5u));
                playback.hasFrameRateExtension = true;
                playback.pictureIntervalQ32 = mpegPictureIntervalQ32(
                    playback.frameRateCode,
                    playback.frameRateExtensionN,
                    playback.frameRateExtensionD);
                break;
            }

            if (playback.pictureIntervalQ32 == 0u)
            {
                playback.pictureIntervalQ32 = kDefaultPictureIntervalQ32;
            }
        }

        uint64_t decodedFrameIntervalQ32(const MpegPlaybackState &playback,
                                         const MpegDecodedFrame &frame)
        {
            const uint64_t base = playback.pictureIntervalQ32 != 0u
                                      ? playback.pictureIntervalQ32
                                      : kDefaultPictureIntervalQ32;
            const uint64_t fields = static_cast<uint64_t>(2 + std::max(0, frame.repeatPict));
            return std::max(kPictureClockOne, (base * fields + 1u) / 2u);
        }

        constexpr uint64_t kMpegPtsWrap = 1ull << 33u;
        constexpr uint64_t kMpegPtsHalfWrap = 1ull << 32u;

        int64_t mpegPtsDelta90k(int64_t fromPts, int64_t toPts)
        {
            if (fromPts < 0 || toPts < 0)
            {
                return 0;
            }

            uint64_t from = static_cast<uint64_t>(fromPts) & (kMpegPtsWrap - 1u);
            uint64_t to = static_cast<uint64_t>(toPts) & (kMpegPtsWrap - 1u);
            uint64_t delta = (to - from) & (kMpegPtsWrap - 1u);
            if (delta >= kMpegPtsHalfWrap)
            {
                return -static_cast<int64_t>(kMpegPtsWrap - delta);
            }
            return static_cast<int64_t>(delta);
        }

        uint64_t mpegPtsDeltaToVSyncQ32(uint64_t delta90k)
        {
            // 90 kHz MPEG clock -> NTSC field clock (60000/1001 Hz):
            // fields = pts * 60000 / (90000 * 1001) = pts * 2 / 3003.
            constexpr uint64_t kPtsDivisor = 3003u;
            const uint64_t whole = delta90k / kPtsDivisor;
            const uint64_t remainder = delta90k % kPtsDivisor;
            const uint64_t wholeQ32 = whole * 2u * kPictureClockOne;
            const uint64_t remainderQ32 = ((remainder * 2u * kPictureClockOne) + kPtsDivisor / 2u) / kPtsDivisor;
            return wholeQ32 + remainderQ32;
        }

        uint64_t presentationTickForFrame(MpegPlaybackState &playback, const MpegDecodedFrame &frame, uint64_t currentTickQ32)
        {
            if (frame.pts90k < 0)
            {
                if (playback.nextPictureTickQ32 == std::numeric_limits<uint64_t>::max())
                {
                    playback.nextPictureTickQ32 = currentTickQ32;
                }
                return playback.nextPictureTickQ32;
            }

            if (playback.firstPresentedPts90k < 0)
            {
                playback.firstPresentedPts90k = frame.pts90k;
                playback.ptsPresentationBaseTickQ32 = currentTickQ32;
                return currentTickQ32;
            }

            const int64_t delta = mpegPtsDelta90k(playback.firstPresentedPts90k, frame.pts90k);
            if (delta >= 0)
            {
                return playback.ptsPresentationBaseTickQ32 + mpegPtsDeltaToVSyncQ32(static_cast<uint64_t>(delta));
            }

            const uint64_t backwards = mpegPtsDeltaToVSyncQ32(static_cast<uint64_t>(-delta));
            return playback.ptsPresentationBaseTickQ32 > backwards
                       ? playback.ptsPresentationBaseTickQ32 - backwards
                       : 0u;
        }

        uint32_t align16(uint32_t value)
        {
            return (value + 15u) & ~15u;
        }

        uint32_t readStackArg(uint8_t *rdram, R5900Context *ctx, uint32_t offset)
        {
            if (!rdram || !ctx)
            {
                return 0u;
            }
            return FAST_READ32(getRegU32(ctx, 29) + offset);
        }

        uint32_t readAbiArg4(uint8_t *rdram, R5900Context *ctx)
        {
            const uint32_t regArg = getRegU32(ctx, 8);
            if (regArg != 0u)
            {
                return regArg;
            }
            return readStackArg(rdram, ctx, 0x10u);
        }

        MpegPlaybackState &getPlaybackState(uint32_t mpegAddr)
        {
            return g_mpeg_stub_state.playbackByMpeg[mpegAddr];
        }

        MpegPlaybackState makeFreshPlaybackState()
        {
            MpegPlaybackState playback{};
            playback.cdStreamGeneration = g_mpeg_stub_state.cdStreamGeneration;
            return playback;
        }

        MpegPlaybackState makeFreshPlaybackStatePreservingConfig(const MpegPlaybackState &oldPlayback)
        {
            MpegPlaybackState playback = makeFreshPlaybackState();
            playback.decodeMode = oldPlayback.decodeMode;
            playback.imageBufferAddr = oldPlayback.imageBufferAddr;
            playback.width = oldPlayback.width;
            playback.height = oldPlayback.height;
            return playback;
        }

        uint16_t readBe16(const uint8_t *p)
        {
            return static_cast<uint16_t>((static_cast<uint16_t>(p[0]) << 8u) | static_cast<uint16_t>(p[1]));
        }

        bool isVideoStreamId(uint8_t streamId)
        {
            return streamId >= 0xE0u && streamId <= 0xEFu;
        }

        bool isAudioStreamId(uint8_t streamId)
        {
            return streamId == kMpegPrivateStream1 || (streamId >= 0xC0u && streamId <= 0xDFu);
        }

        bool isLengthPrefixedHeader(uint8_t streamId)
        {
            switch (streamId)
            {
            case kMpegSystemHeader:
            case 0xBCu: // program_stream_map
            case 0xBEu: // padding_stream
            case 0xBFu: // private_stream_2
            case 0xF0u: // ECM
            case 0xF1u: // EMM
            case 0xF2u: // DSMCC
            case 0xF8u: // ITU-T H.222.1 type E
            case 0xFFu: // program_stream_directory
                return true;
            default:
                return false;
            }
        }

        size_t findStartCode(const std::vector<uint8_t> &buffer, size_t from)
        {
            if (buffer.size() < 4 || from >= buffer.size() - 3u)
            {
                return kStartCodeNotFound;
            }

            for (size_t i = from; i + 3u < buffer.size(); ++i)
            {
                if (buffer[i] == 0x00u && buffer[i + 1u] == 0x00u && buffer[i + 2u] == 0x01u)
                {
                    return i;
                }
            }
            return kStartCodeNotFound;
        }

        bool containsMpegSequenceEnd(const uint8_t *data, size_t size)
        {
            if (!data || size < 4)
            {
                return false;
            }

            for (size_t i = 0; i + 3u < size; ++i)
            {
                if (data[i] == 0x00u &&
                    data[i + 1u] == 0x00u &&
                    data[i + 2u] == 0x01u &&
                    data[i + 3u] == kMpegSequenceEnd)
                {
                    return true;
                }
            }
            return false;
        }

        size_t findMpegSequenceHeader(const uint8_t *data, size_t size)
        {
            if (!data || size < 8u)
            {
                return kStartCodeNotFound;
            }

            for (size_t i = 0; i + 7u < size; ++i)
            {
                if (data[i] == 0x00u &&
                    data[i + 1u] == 0x00u &&
                    data[i + 2u] == 0x01u &&
                    data[i + 3u] == 0xB3u)
                {
                    const uint32_t width = (static_cast<uint32_t>(data[i + 4u]) << 4u) |
                                           (static_cast<uint32_t>(data[i + 5u]) >> 4u);
                    const uint32_t height = ((static_cast<uint32_t>(data[i + 5u]) & 0x0Fu) << 8u) |
                                            static_cast<uint32_t>(data[i + 6u]);
                    const uint8_t frameRateCode = data[i + 7u] & 0x0Fu;
                    if (width != 0u && height != 0u && width <= 4096u && height <= 4096u &&
                        mpegPictureIntervalQ32(frameRateCode) != 0u)
                    {
                        return i;
                    }
                }
            }
            return kStartCodeNotFound;
        }

        struct MpegPesHeader
        {
            size_t payloadOffset = 0u;
            int64_t pts90k = -1;
            int64_t dts90k = -1;
        };

        int64_t decodePesTimestamp90k(const uint8_t *p, size_t remaining)
        {
            if (!p || remaining < 5u)
            {
                return -1;
            }

            const uint64_t value =
                (static_cast<uint64_t>((p[0] >> 1u) & 0x07u) << 30u) |
                (static_cast<uint64_t>(p[1]) << 22u) |
                (static_cast<uint64_t>((p[2] >> 1u) & 0x7Fu) << 15u) |
                (static_cast<uint64_t>(p[3]) << 7u) |
                static_cast<uint64_t>((p[4] >> 1u) & 0x7Fu);
            return static_cast<int64_t>(value & (kMpegPtsWrap - 1u));
        }

        MpegPesHeader parsePesHeader(const uint8_t *packet, size_t packetSize)
        {
            MpegPesHeader result{};
            result.payloadOffset = packetSize;
            if (!packet || packetSize <= 6u)
            {
                return result;
            }

            size_t pos = 6u;
            if (packetSize >= 9u && (packet[pos] & 0xC0u) == 0x80u)
            {
                const uint8_t ptsDtsFlags = packet[pos + 1u] & 0xC0u;
                const size_t headerDataLength = static_cast<size_t>(packet[pos + 2u]);
                const size_t optionalStart = 9u;
                const size_t optionalEnd = std::min(packetSize, optionalStart + headerDataLength);
                if ((ptsDtsFlags == 0x80u || ptsDtsFlags == 0xC0u) && optionalEnd >= optionalStart + 5u)
                {
                    result.pts90k = decodePesTimestamp90k(packet + optionalStart, optionalEnd - optionalStart);
                }
                if (ptsDtsFlags == 0xC0u && optionalEnd >= optionalStart + 10u)
                {
                    result.dts90k = decodePesTimestamp90k(packet + optionalStart + 5u, optionalEnd - optionalStart - 5u);
                }
                result.payloadOffset = optionalEnd;
                return result;
            }

            // MPEG-1 PES.
            while (pos < packetSize && packet[pos] == 0xFFu)
            {
                ++pos;
            }
            if (pos + 1u < packetSize && (packet[pos] & 0xC0u) == 0x40u)
            {
                pos += 2u;
            }
            if (pos >= packetSize)
            {
                return result;
            }

            const uint8_t marker = packet[pos] & 0xF0u;
            if (marker == 0x20u && pos + 5u <= packetSize)
            {
                result.pts90k = decodePesTimestamp90k(packet + pos, packetSize - pos);
                pos += 5u;
            }
            else if (marker == 0x30u && pos + 10u <= packetSize)
            {
                result.pts90k = decodePesTimestamp90k(packet + pos, packetSize - pos);
                result.dts90k = decodePesTimestamp90k(packet + pos + 5u, packetSize - pos - 5u);
                pos += 10u;
            }
            else if (packet[pos] == 0x0Fu)
            {
                ++pos;
            }

            result.payloadOffset = std::min(packetSize, pos);
            return result;
        }

        void flushDecoderIfEnded(MpegPlaybackState &playback)
        {
            if (playback.streamEnded && playback.decoder)
            {
                playback.decoder->flush(playback.decodedFrames);
            }
        }

        void feedElementaryStream(MpegPlaybackState &playback, const uint8_t *data, size_t size, int64_t pts90k = -1, int64_t dts90k = -1)
        {
            if (!data || size == 0)
            {
                return;
            }

            const uint32_t feedEsIdx = g_mpeg_stub_state.feedEsTraceCount++;
            if (feedEsIdx < 32u)
            {
                PS2_IF_AGRESSIVE_LOGS({
                    char hexBuf[16] = {};
                    for (size_t i = 0; i < std::min<size_t>(4u, size); ++i)
                    {
                        ::snprintf(hexBuf + i * 2, 3, "%02x", data[i]);
                    }
                    std::cerr << "[MPEG:feedES] #" << feedEsIdx
                              << " size=" << size
                              << " first4=" << hexBuf
                              << " decoderFailed=" << playback.decoderFailed
                              << " waitSeq=" << playback.waitingForVideoSequenceHeader
                              << std::endl;
                });
            }

            playback.sawInput = true;
            updateMpegPictureTiming(playback, data, size);
            if (playback.waitingForVideoSequenceHeader)
            {
                playback.videoSequenceSyncBuffer.insert(
                    playback.videoSequenceSyncBuffer.end(),
                    data,
                    data + size);

                constexpr size_t kMaxVideoSequenceSyncBytes = 2u * 1024u * 1024u;
                if (playback.videoSequenceSyncBuffer.size() > kMaxVideoSequenceSyncBytes)
                {
                    const size_t keepFrom = playback.videoSequenceSyncBuffer.size() - 3u;
                    playback.videoSequenceSyncBuffer.erase(
                        playback.videoSequenceSyncBuffer.begin(),
                        playback.videoSequenceSyncBuffer.begin() + static_cast<std::ptrdiff_t>(keepFrom));
                }

                const size_t sequenceHeader = findMpegSequenceHeader(
                    playback.videoSequenceSyncBuffer.data(),
                    playback.videoSequenceSyncBuffer.size());
                if (sequenceHeader == kStartCodeNotFound)
                {
                    return;
                }

                if (sequenceHeader != 0u)
                {
                    playback.videoSequenceSyncBuffer.erase(
                        playback.videoSequenceSyncBuffer.begin(),
                        playback.videoSequenceSyncBuffer.begin() + static_cast<std::ptrdiff_t>(sequenceHeader));
                }

                data = playback.videoSequenceSyncBuffer.data();
                size = playback.videoSequenceSyncBuffer.size();
                if (playback.pictureIntervalQ32 == 0u)
                {
                    playback.pictureIntervalQ32 = kDefaultPictureIntervalQ32;
                }
                playback.nextPictureTickQ32 = std::numeric_limits<uint64_t>::max();
                playback.presentationEndTickQ32 = std::numeric_limits<uint64_t>::max();
                playback.firstPresentedPts90k = -1;
                playback.ptsPresentationBaseTickQ32 = 0u;
                playback.waitingForVideoSequenceHeader = false;
                playback.decoderFailed = false;
                playback.decoder.reset();
                playback.decodedFrames.clear();
            }

            if (containsMpegSequenceEnd(data, size))
            {
                playback.sawSequenceEnd = true;
                playback.cdStreamGeneration = g_mpeg_stub_state.cdStreamGeneration;
            }

            if (!playback.decoder)
            {
                playback.decoder = std::make_unique<MpegFfmpegDecoder>();
            }

            if (!playback.decoder->feed(data, size, playback.decodedFrames, pts90k, dts90k))
            {
                playback.decoder.reset();
                playback.waitingForVideoSequenceHeader = true;
                playback.videoSequenceSyncBuffer.clear();
                playback.decoderFailed = false;
                return;
            }

            playback.videoSequenceSyncBuffer.clear();
            flushDecoderIfEnded(playback);
        }

        void erasePssPrefix(MpegPlaybackState &playback, size_t count)
        {
            std::vector<uint8_t> &buffer = playback.pssBuffer;
            std::vector<uint32_t> &guestAddrs = playback.pssGuestAddrs;
            const size_t clamped = std::min(count, buffer.size());
            if (clamped == 0u)
            {
                return;
            }

            buffer.erase(buffer.begin(), buffer.begin() + static_cast<std::ptrdiff_t>(clamped));
            if (guestAddrs.size() >= clamped)
            {
                guestAddrs.erase(guestAddrs.begin(), guestAddrs.begin() + static_cast<std::ptrdiff_t>(clamped));
            }
            else
            {
                guestAddrs.clear();
            }
        }

        std::vector<MpegRegisteredCallback> matchingStreamCallbacks(uint32_t mpegAddr, uint32_t streamType)
        {
            std::vector<MpegRegisteredCallback> out;
            auto it = g_mpeg_stub_state.callbacksByMpeg.find(mpegAddr);
            if (it == g_mpeg_stub_state.callbacksByMpeg.end())
            {
                return out;
            }

            for (const MpegRegisteredCallback &callback : it->second)
            {
                if (callback.stream && callback.type == streamType)
                {
                    out.push_back(callback);
                }
            }
            return out;
        }

        void queueStreamCallbackEvent(uint32_t mpegAddr,
                                      uint32_t streamType,
                                      uint32_t dataAddr,
                                      uint32_t len,
                                      std::vector<MpegStreamCallbackEvent> &callbackEvents,
                                      int64_t pts90k = -1,
                                      int64_t dts90k = -1)
        {
            MpegStreamCallbackEvent event{};
            event.mpegAddr = mpegAddr;
            event.streamType = streamType;
            event.dataAddr = dataAddr;
            event.len = len;
            event.pts = pts90k >= 0 ? static_cast<uint64_t>(pts90k) : 0xFFFFFFFFFFFFFFFFull;
            event.dts = dts90k >= 0 ? static_cast<uint64_t>(dts90k) : 0xFFFFFFFFFFFFFFFFull;
            event.callbacks = matchingStreamCallbacks(mpegAddr, streamType);
            if (!event.callbacks.empty())
            {
                callbackEvents.push_back(std::move(event));
            }
        }

        void queueHostStreamCallbackEvent(
            uint32_t mpegAddr,
            uint32_t streamType,
            const uint8_t *data,
            size_t len,
            std::vector<MpegStreamCallbackEvent> &callbackEvents,
            int64_t pts90k = -1,
            int64_t dts90k = -1)
        {
            if (!data || len == 0u)
            {
                return;
            }

            MpegStreamCallbackEvent event{};
            event.mpegAddr = mpegAddr;
            event.streamType = streamType;
            event.dataAddr = 0u;
            event.len = static_cast<uint32_t>(len);
            event.pts =
                pts90k >= 0
                    ? static_cast<uint64_t>(pts90k)
                    : 0xFFFFFFFFFFFFFFFFull;
            event.dts =
                dts90k >= 0
                    ? static_cast<uint64_t>(dts90k)
                    : 0xFFFFFFFFFFFFFFFFull;

            event.hostPayload.assign(data, data + len);
            callbackEvents.push_back(std::move(event));
        }

        void drainPendingHostCallbacksUnlocked(
            uint32_t mpegAddr,
            std::vector<MpegStreamCallbackEvent> &callbackEvents,
            size_t maxEvents)
        {
            if (maxEvents == 0u)
            {
                return;
            }

            if (g_mpeg_stub_state.hostCallbackInFlight)
            {
                return;
            }

            auto &pending = g_mpeg_stub_state.pendingHostCallbackEvents;
            size_t emitted = 0u;

            for (auto it = pending.begin();
                 it != pending.end() && emitted < maxEvents;)
            {
                if (it->mpegAddr != mpegAddr)
                {
                    ++it;
                    continue;
                }

                std::vector<MpegRegisteredCallback> callbacks =
                    matchingStreamCallbacks(it->mpegAddr, it->streamType);

                // Keep the event pending if registration has not happened yet.
                if (callbacks.empty())
                {
                    ++it;
                    continue;
                }

                it->callbacks = std::move(callbacks);
                callbackEvents.push_back(std::move(*it));
                it = pending.erase(it);
                ++emitted;
            }
        }

        void processPssBuffer(uint32_t mpegAddr,
                              MpegPlaybackState &playback,
                              std::vector<MpegStreamCallbackEvent> &callbackEvents,
                              bool finalChunk = false)
        {
            std::vector<uint8_t> &buffer = playback.pssBuffer;

            while (true)
            {
                if (playback.streamEnded)
                {
                    erasePssPrefix(playback, buffer.size());
                    return;
                }

                const size_t start = findStartCode(buffer, 0u);
                if (start == kStartCodeNotFound)
                {
                    if (finalChunk)
                    {
                        erasePssPrefix(playback, buffer.size());
                        return;
                    }
                    if (buffer.size() > 3u)
                    {
                        erasePssPrefix(playback, buffer.size() - 3u);
                    }
                    return;
                }

                if (start > 0u)
                {
                    erasePssPrefix(playback, start);
                }

                if (buffer.size() < 4u)
                {
                    return;
                }

                const uint8_t streamId = buffer[3u];

                if (streamId == kMpegProgramEnd)
                {
                    playback.streamEnded = true;
                    playback.cdStreamGeneration = g_mpeg_stub_state.cdStreamGeneration;
                    flushDecoderIfEnded(playback);
                    erasePssPrefix(playback, buffer.size());
                    return;
                }

                if (streamId == kMpegPackHeader)
                {
                    if (buffer.size() < 12u)
                    {
                        if (finalChunk)
                        {
                            erasePssPrefix(playback, buffer.size());
                        }
                        return;
                    }

                    size_t packSize = 12u;
                    if ((buffer[4u] & 0xC0u) == 0x40u)
                    {
                        if (buffer.size() < 14u)
                        {
                            if (finalChunk)
                            {
                                erasePssPrefix(playback, buffer.size());
                            }
                            return;
                        }
                        packSize = 14u + static_cast<size_t>(buffer[13u] & 0x07u);
                    }
                    if (buffer.size() < packSize)
                    {
                        if (finalChunk)
                        {
                            erasePssPrefix(playback, buffer.size());
                        }
                        return;
                    }
                    erasePssPrefix(playback, packSize);
                    continue;
                }

                if (buffer.size() < 6u)
                {
                    if (finalChunk)
                    {
                        erasePssPrefix(playback, buffer.size());
                    }
                    return;
                }

                const uint16_t packetLength = readBe16(buffer.data() + 4u);
                if (isLengthPrefixedHeader(streamId))
                {
                    const size_t packetEnd = 6u + static_cast<size_t>(packetLength);
                    if (buffer.size() < packetEnd)
                    {
                        if (finalChunk)
                        {
                            erasePssPrefix(playback, buffer.size());
                        }
                        return;
                    }
                    erasePssPrefix(playback, packetEnd);
                    continue;
                }

                size_t packetEnd = 0u;
                if (packetLength != 0u)
                {
                    packetEnd = 6u + static_cast<size_t>(packetLength);
                    if (buffer.size() < packetEnd)
                    {
                        if (!finalChunk)
                        {
                            return;
                        }
                        packetEnd = buffer.size();
                    }
                }
                else
                {
                    const size_t next = findStartCode(buffer, 6u);
                    if (next == kStartCodeNotFound)
                    {
                        if (!finalChunk)
                        {
                            return;
                        }
                        packetEnd = buffer.size();
                    }
                    else
                    {
                        packetEnd = next;
                    }
                }

                if (isVideoStreamId(streamId))
                {
                    const MpegPesHeader pes = parsePesHeader(buffer.data(), packetEnd);
                    const size_t payloadStart = pes.payloadOffset;
                    if (payloadStart < packetEnd)
                    {
                        if (payloadStart < playback.pssGuestAddrs.size())
                        {
                            queueStreamCallbackEvent(
                                mpegAddr,
                                kMpegStrM2V,
                                playback.pssGuestAddrs[payloadStart],
                                static_cast<uint32_t>(packetEnd - payloadStart),
                                callbackEvents,
                                pes.pts90k,
                                pes.dts90k);
                        }
                        feedElementaryStream(
                            playback,
                            buffer.data() + payloadStart,
                            packetEnd - payloadStart,
                            pes.pts90k,
                            pes.dts90k);
                    }
                }
                else if (isAudioStreamId(streamId))
                {
                    const MpegPesHeader pes = parsePesHeader(buffer.data(), packetEnd);
                    const size_t payloadStart = pes.payloadOffset;
                    if (payloadStart < packetEnd)
                    {
                        const uint32_t payloadLen =
                            static_cast<uint32_t>(packetEnd - payloadStart);

                        if (payloadStart < playback.pssGuestAddrs.size())
                        {
                            // Normal guest-fed path: retain the exact existing
                            // guest-address callback behavior.
                            queueStreamCallbackEvent(
                                mpegAddr,
                                kMpegStrPCM,
                                playback.pssGuestAddrs[payloadStart],
                                payloadLen,
                                callbackEvents,
                                pes.pts90k,
                                pes.dts90k);
                            queueStreamCallbackEvent(
                                mpegAddr,
                                kMpegStrADPCM,
                                playback.pssGuestAddrs[payloadStart],
                                payloadLen,
                                callbackEvents,
                                pes.pts90k,
                                pes.dts90k);
                        }
                        else
                        {
                            // Host-fed path: preserve the complete original
                            // PES payload until a guest address can be supplied.
                            queueHostStreamCallbackEvent(
                                mpegAddr,
                                kMpegStrPCM,
                                buffer.data() + payloadStart,
                                payloadLen,
                                callbackEvents,
                                pes.pts90k,
                                pes.dts90k);
                            queueHostStreamCallbackEvent(
                                mpegAddr,
                                kMpegStrADPCM,
                                buffer.data() + payloadStart,
                                payloadLen,
                                callbackEvents,
                                pes.pts90k,
                                pes.dts90k);
                        }
                    }
                }

                erasePssPrefix(playback, packetEnd);
            }
        }

        void finishPlaybackStream(uint32_t mpegAddr, MpegPlaybackState &playback)
        {
            std::vector<MpegStreamCallbackEvent> ignoredCallbacks;
            processPssBuffer(mpegAddr, playback, ignoredCallbacks, true);
            playback.streamEnded = true;
            playback.cdStreamGeneration = g_mpeg_stub_state.cdStreamGeneration;
            flushDecoderIfEnded(playback);
        }

        void finalizeCdStreamEofUnlocked(std::vector<uint32_t> &completedMpegIds, bool &changed)
        {
            g_mpeg_stub_state.cdStreamEofPending = false;
            g_mpeg_stub_state.currentCdStreamEofSeen = true;
            for (auto &[mpegAddr, playback] : g_mpeg_stub_state.playbackByMpeg)
            {
                completedMpegIds.push_back(mpegAddr);
                if (!playback.sawInput || playback.streamEnded)
                {
                    continue;
                }

                finishPlaybackStream(mpegAddr, playback);
                changed = true;
            }
        }

        bool mpegDemuxBackpressured(const MpegPlaybackState &playback)
        {
            // Let EOF finalization drain any tail that is already in the guest
            // ring, otherwise bound decode lead to a handful of pictures.
            //
            // Important: do not park sceMpegDemuxPss/Ring here. Code Veronica
            // explicitly wakes its video thread before every demux call and that
            // thread sleeps again after presenting one picture. A single host
            // decoder feed can enqueue more than kMaxDecodedPicturesAhead frames;
            // parking the producer then leaves the consumer asleep after draining
            // just one frame, with nobody left to issue the next WakeupThread.
            // Returning 0 bytes consumed instead leaves the guest ring intact and
            // lets the game's producer loop wake the consumer again. Backpressure
            // still propagates naturally to sceCdStRead because the ring does not
            // advance while this is true.
            return !g_mpeg_stub_state.currentCdStreamEofSeen &&
                   playback.decodedFrames.size() >= kMaxDecodedPicturesAhead;
        }

        void recordCdStreamBytesDemuxedUnlocked(
            size_t consumed,
            std::vector<uint32_t> &completedMpegIds,
            bool &changed)
        {
            g_mpeg_stub_state.cdStreamBytesDemuxed += consumed;
            if (g_mpeg_stub_state.cdStreamEofPending &&
                g_mpeg_stub_state.cdStreamBytesDemuxed >= g_mpeg_stub_state.cdStreamBytesProduced)
            {
                finalizeCdStreamEofUnlocked(completedMpegIds, changed);
            }
        }

        void appendPssBytes(uint32_t mpegAddr,
                            MpegPlaybackState &playback,
                            const uint8_t *data,
                            size_t size,
                            uint32_t guestAddr,
                            std::vector<MpegStreamCallbackEvent> &callbackEvents,
                            bool trackGuestAddrs = true)
        {
            if (!data || size == 0)
            {
                return;
            }

            if (playback.sawInput && playback.cdStreamGeneration != g_mpeg_stub_state.cdStreamGeneration)
            {
                playback = makeFreshPlaybackState();
            }

            if (playback.streamEnded)
            {
                if (playback.cdStreamGeneration == g_mpeg_stub_state.cdStreamGeneration)
                {
                    playback.sawInput = true;
                    return;
                }

                playback = makeFreshPlaybackState();
            }

            if (!playback.sawInput)
            {
                playback.cdStreamGeneration = g_mpeg_stub_state.cdStreamGeneration;
            }
            playback.sawInput = true;

            playback.pssBuffer.insert(playback.pssBuffer.end(), data, data + size);
            if (trackGuestAddrs)
            {
                playback.pssGuestAddrs.reserve(playback.pssGuestAddrs.size() + size);
                for (size_t i = 0; i < size; ++i)
                {
                    playback.pssGuestAddrs.push_back(guestAddr + static_cast<uint32_t>(i));
                }
            }
            processPssBuffer(mpegAddr, playback, callbackEvents);
        }

        size_t appendGuestBytes(uint32_t mpegAddr,
                                MpegPlaybackState &playback,
                                const uint8_t *rdram,
                                uint32_t addr,
                                size_t size,
                                std::vector<MpegStreamCallbackEvent> &callbackEvents)
        {
            size_t copied = 0u;
            while (copied < size)
            {
                const uint32_t curAddr = addr + static_cast<uint32_t>(copied);
                const uint32_t offset = curAddr & PS2_RAM_MASK;
                size_t chunk = std::min<size_t>(size - copied, PS2_RAM_SIZE - offset);
                if (chunk == 0u)
                {
                    break;
                }

                const uint8_t *src = getConstMemPtr(rdram, curAddr);
                if (!src)
                {
                    break;
                }

                appendPssBytes(
                    mpegAddr,
                    playback,
                    src,
                    chunk,
                    curAddr,
                    callbackEvents);
                copied += chunk;
            }
            return copied;
        }

        size_t appendGuestRingBytes(uint32_t mpegAddr,
                                    MpegPlaybackState &playback,
                                    const uint8_t *rdram,
                                    uint32_t dataAddr,
                                    uint32_t byteCount,
                                    uint32_t ringBaseAddr,
                                    uint32_t ringSize,
                                    std::vector<MpegStreamCallbackEvent> &callbackEvents)
        {
            if (byteCount == 0u)
            {
                return 0u;
            }

            const uint32_t base = ringBaseAddr & PS2_RAM_MASK;
            const uint32_t data = dataAddr & PS2_RAM_MASK;
            if (ringBaseAddr != 0u && ringSize != 0u && ringSize <= PS2_RAM_SIZE)
            {
                const uint32_t ringOffset = (data - base) & PS2_RAM_MASK;
                if (ringOffset < ringSize)
                {
                    const uint32_t first = std::min<uint32_t>(byteCount, ringSize - ringOffset);
                    size_t copied = appendGuestBytes(
                        mpegAddr,
                        playback,
                        rdram,
                        dataAddr,
                        first,
                        callbackEvents);
                    if (copied < first)
                    {
                        return copied;
                    }

                    const uint32_t remaining = byteCount - first;
                    if (remaining != 0u)
                    {
                        copied += appendGuestBytes(
                            mpegAddr,
                            playback,
                            rdram,
                            ringBaseAddr,
                            remaining,
                            callbackEvents);
                    }
                    return copied;
                }
            }

            return appendGuestBytes(mpegAddr, playback, rdram, dataAddr, byteCount, callbackEvents);
        }

        bool writeMpegCallbackData(uint8_t *rdram, uint32_t addr, const MpegStreamCallbackEvent &event)
        {
            if (!rdram || addr == 0u)
            {
                return false;
            }

            uint8_t *data = getMemPtr(rdram, addr);
            if (!data)
            {
                return false;
            }

            std::memset(data, 0, kMpegCallbackDataSize);
            *reinterpret_cast<uint32_t *>(data + 0x00u) = event.streamType;
            *reinterpret_cast<uint32_t *>(data + 0x08u) = event.dataAddr;
            *reinterpret_cast<uint32_t *>(data + 0x0Cu) = event.len;
            *reinterpret_cast<uint64_t *>(data + 0x10u) = event.pts;
            *reinterpret_cast<uint64_t *>(data + 0x18u) = event.dts;
            return true;
        }

        void dispatchGuestStreamCallback(uint8_t *rdram,
                                         R5900Context *callerCtx,
                                         PS2Runtime *runtime,
                                         const MpegStreamCallbackEvent &event,
                                         const MpegRegisteredCallback &callback)
        {
            if (!rdram || !callerCtx || !runtime || callback.func == 0u || !runtime->hasFunction(callback.func))
            {
                return;
            }

            // BOOT 164: feed the preserved original host PSS PCM payload
            // directly into the host movie-audio bridge before attempting
            // guest ReadBuf materialization.
            //
            // pcmCallback at 0x40B140 consumes payload+4 / len-4, so mirror
            // exactly those bytes here.
            // BOOT 170: host audio is already mirrored at demux time.
            // Preserve this older diagnostic bridge for reference, but do
            // not execute it. Genuine guest PCM callbacks continue below.
            if (false &&
                callback.func == 0x40B140u &&
                !event.hostPayload.empty() &&
                event.hostPayload.size() > 4u)
            {
                constexpr uint32_t kBoot164ScratchSize = 0x2000u;

                static uint32_t s_boot164ScratchAddr = 0u;
                static uint32_t s_boot164FeedCount = 0u;
                static uint32_t s_boot164FailureCount = 0u;

                const size_t sourceSize =
                    event.hostPayload.size() - 4u;

                const uint8_t *source =
                    event.hostPayload.data() + 4u;

                if (sourceSize <=
                    static_cast<size_t>(kBoot164ScratchSize))
                {
                    if (s_boot164ScratchAddr == 0u)
                    {
                        s_boot164ScratchAddr =
                            runtime->guestMalloc(
                                kBoot164ScratchSize,
                                16u);

                        std::cerr
                            << "[dothack:boot164-host-audio]"
                            << " stage=scratch-alloc"
                            << " addr=0x" << std::hex
                            << s_boot164ScratchAddr
                            << std::dec
                            << " size=" << kBoot164ScratchSize
                            << std::endl;
                    }

                    uint8_t *scratch =
                        s_boot164ScratchAddr != 0u
                            ? getMemPtr(
                                  rdram,
                                  s_boot164ScratchAddr)
                            : nullptr;

                    if (scratch)
                    {
                        std::memcpy(
                            scratch,
                            source,
                            sourceSize);

                        if (s_boot164FeedCount < 96u)
                        {
                            std::cerr
                                << "[dothack:boot164-host-audio]"
                                << " stage=feed"
                                << " n=" << s_boot164FeedCount
                                << " mpeg=0x" << std::hex
                                << event.mpegAddr
                                << " scratch=0x"
                                << s_boot164ScratchAddr
                                << std::dec
                                << " hostLen="
                                << event.hostPayload.size()
                                << " pcmLen="
                                << sourceSize
                                << " first8="
                                << std::hex
                                << static_cast<unsigned>(source[0])
                                << ","
                                << static_cast<unsigned>(source[1])
                                << ","
                                << static_cast<unsigned>(source[2])
                                << ","
                                << static_cast<unsigned>(source[3])
                                << ","
                                << static_cast<unsigned>(source[4])
                                << ","
                                << static_cast<unsigned>(source[5])
                                << ","
                                << static_cast<unsigned>(source[6])
                                << ","
                                << static_cast<unsigned>(source[7])
                                << std::dec
                                << std::endl;
                        }

                        ++s_boot164FeedCount;

                        runtime->audioBackend()
                            .onMoviePcmTransfer(
                                rdram,
                                s_boot164ScratchAddr,
                                static_cast<uint32_t>(
                                    sourceSize));
                    }
                    else if (s_boot164FailureCount < 16u)
                    {
                        std::cerr
                            << "[dothack:boot164-host-audio]"
                            << " stage=scratch-unavailable"
                            << " n="
                            << s_boot164FailureCount++
                            << " addr=0x" << std::hex
                            << s_boot164ScratchAddr
                            << std::dec
                            << std::endl;
                    }
                }
                else if (s_boot164FailureCount < 16u)
                {
                    std::cerr
                        << "[dothack:boot164-host-audio]"
                        << " stage=packet-too-large"
                        << " n="
                        << s_boot164FailureCount++
                        << " sourceSize="
                        << sourceSize
                        << " scratchSize="
                        << kBoot164ScratchSize
                        << std::endl;
                }
            }

            MpegStreamCallbackEvent materializedEvent = event;
            uint32_t callbackUserData = callback.data;

            // BOOT 152:
            //
            // Boot 149 proved that a host-fed PCM packet can be copied into
            // the game's real ReadBuf and successfully delivered through the
            // genuine pcmCallback at 0x40B140.
            //
            // Boot 150 expands that proof only slightly:
            //
            //   * at most EIGHT host PCM callbacks may be materialized;
            //   * each callback must use the game's real ReadBuf;
            //   * the ring must be completely empty (used == 0);
            //   * the payload may wrap naturally at the ring boundary;
            //   * writePos and used are NEVER changed by this experiment;
            //   * subsequent packets are inspected but not delivered once
            //     the eight-callback limit has been reached.
            //
            // This lets us observe the game's audio state after the first
            // packet initialized it, without enabling continuous delivery.
            bool hostPayloadUsesRealReadBuf = false;
            uint32_t hostPayloadDataAddr = 0u;

            // BOOT 152:
            // Verify that the temporary source bytes placed in the free
            // portion of the real ReadBuf survive until the PCM callback
            // has completely returned.
            uint32_t boot152ExpectedSourceHash = 0u;
            uint32_t boot152PayloadLen = 0u;
            uint32_t boot152WritePos = 0u;

            if (!event.hostPayload.empty())
            {
                // BOOT 153:
                // Extend the proven materialization path to eight callbacks.
                // No continuous-delivery or AudioDec back-pressure policy is
                // enabled yet; this remains a bounded diagnostic experiment.
                static uint32_t s_boot152SuccessfulCallbacks = 0u;
                static uint32_t s_boot152LogCount = 0u;

                if (callback.data == 0u)
                {
                    std::cerr
                        << "[dothack:boot153-eight-packet]"
                        << " stage=no-userdata"
                        << " mpeg=0x" << std::hex << event.mpegAddr
                        << " func=0x" << callback.func
                        << std::dec
                        << " len=" << event.hostPayload.size()
                        << std::endl;

                    return;
                }

                // This experiment is specifically for the known .hack PCM
                // callback proven by Boot 149. Do not materialize some other
                // registered stream callback into the ReadBuf by accident.
                if (callback.func != 0x40B140u)
                {
                    if (s_boot152LogCount < 64u)
                    {
                        std::cerr
                            << "[dothack:boot153-eight-packet]"
                            << " stage=non-pcm-skip"
                            << " n=" << s_boot152LogCount++
                            << " mpeg=0x" << std::hex << event.mpegAddr
                            << " func=0x" << callback.func
                            << " type=0x" << event.streamType
                            << std::dec
                            << " len=" << event.hostPayload.size()
                            << std::endl;
                    }

                    return;
                }

                uint8_t *realReadBuf =
                    getMemPtr(rdram, callback.data);

                if (!realReadBuf)
                {
                    std::cerr
                        << "[dothack:boot153-eight-packet]"
                        << " stage=invalid-user-pointer"
                        << " mpeg=0x" << std::hex << event.mpegAddr
                        << " user=0x" << callback.data
                        << std::dec
                        << " len=" << event.hostPayload.size()
                        << std::endl;

                    return;
                }

                const uint32_t writePos =
                    *reinterpret_cast<const uint32_t *>(
                        realReadBuf + 0x50000u);

                const uint32_t used =
                    *reinterpret_cast<const uint32_t *>(
                        realReadBuf + 0x50004u);

                const uint32_t capacity =
                    *reinterpret_cast<const uint32_t *>(
                        realReadBuf + 0x50008u);

                const uint32_t freeBytes =
                    capacity >= used
                        ? capacity - used
                        : 0u;

                if (s_boot152LogCount < 64u)
                {
                    std::cerr
                        << "[dothack:boot153-eight-packet]"
                        << " stage=inspect"
                        << " n=" << s_boot152LogCount++
                        << " successes=" << s_boot152SuccessfulCallbacks
                        << " mpeg=0x" << std::hex << event.mpegAddr
                        << " type=0x" << event.streamType
                        << " func=0x" << callback.func
                        << " user=0x" << callback.data
                        << std::dec
                        << " writePos=" << writePos
                        << " used=" << used
                        << " capacity=" << capacity
                        << " free=" << freeBytes
                        << " hostLen=" << event.hostPayload.size()
                        << std::endl;
                }

                if (capacity != 0x50000u ||
                    writePos >= capacity ||
                    used > capacity)
                {
                    std::cerr
                        << "[dothack:boot153-eight-packet]"
                        << " stage=invalid-ring-state"
                        << " mpeg=0x" << std::hex << event.mpegAddr
                        << " user=0x" << callback.data
                        << std::dec
                        << " writePos=" << writePos
                        << " used=" << used
                        << " capacity=" << capacity
                        << std::endl;

                    return;
                }

                // We inspect later events even after the delivery cap so the
                // report tells us how the real ReadBuf evolves.
                if (s_boot152SuccessfulCallbacks >= 8u)
                {
                    if (s_boot152LogCount < 64u)
                    {
                        std::cerr
                            << "[dothack:boot153-eight-packet]"
                            << " stage=eight-packet-cap"
                            << " n=" << s_boot152LogCount++
                            << " successes=" << s_boot152SuccessfulCallbacks
                            << " mpeg=0x" << std::hex << event.mpegAddr
                            << std::dec
                            << " writePos=" << writePos
                            << " used=" << used
                            << " free=" << freeBytes
                            << " hostLen=" << event.hostPayload.size()
                            << std::endl;
                    }

                    return;
                }

                // BOOT 152:
                //
                // Packets 1 and 2 still arrive while the ring is empty.
                // Packets 3 through 8 are deliberately allowed to borrow the ring's
                // currently FREE region beginning at writePos.
                //
                // We do NOT advance writePos or used. This is temporary
                // callback source storage only. The experiment remains
                // capped at eight callbacks.

                const size_t payloadLen =
                    event.hostPayload.size();

                if (payloadLen < 4u ||
                    payloadLen > static_cast<size_t>(capacity))
                {
                    std::cerr
                        << "[dothack:boot153-eight-packet]"
                        << " stage=invalid-payload-size"
                        << " mpeg=0x" << std::hex << event.mpegAddr
                        << std::dec
                        << " len=" << payloadLen
                        << " capacity=" << capacity
                        << std::endl;

                    return;
                }

                if (payloadLen > static_cast<size_t>(freeBytes))
                {
                    if (s_boot152LogCount < 64u)
                    {
                        std::cerr
                            << "[dothack:boot153-eight-packet]"
                            << " stage=insufficient-free-space"
                            << " n=" << s_boot152LogCount++
                            << " successes=" << s_boot152SuccessfulCallbacks
                            << " mpeg=0x" << std::hex << event.mpegAddr
                            << std::dec
                            << " writePos=" << writePos
                            << " used=" << used
                            << " free=" << freeBytes
                            << " hostLen=" << payloadLen
                            << std::endl;
                    }

                    return;
                }

                const size_t contiguous =
                    static_cast<size_t>(capacity - writePos);

                const size_t firstLen =
                    payloadLen < contiguous
                        ? payloadLen
                        : contiguous;

                std::memcpy(
                    realReadBuf + writePos,
                    event.hostPayload.data(),
                    firstLen);

                if (payloadLen > firstLen)
                {
                    std::memcpy(
                        realReadBuf,
                        event.hostPayload.data() + firstLen,
                        payloadLen - firstLen);
                }

                boot152ExpectedSourceHash = 2166136261u;

                for (const uint8_t byte : event.hostPayload)
                {
                    boot152ExpectedSourceHash ^= byte;
                    boot152ExpectedSourceHash *= 16777619u;
                }

                boot152PayloadLen =
                    static_cast<uint32_t>(payloadLen);

                boot152WritePos =
                    writePos;

                hostPayloadDataAddr =
                    callback.data + writePos;

                materializedEvent.dataAddr =
                    hostPayloadDataAddr;

                materializedEvent.len =
                    static_cast<uint32_t>(payloadLen);

                // Keep the game's original callback userdata. This is the
                // actual ReadBuf whose capacity pcmCallback uses for wrapping.
                callbackUserData = callback.data;

                hostPayloadUsesRealReadBuf = true;

                ++s_boot152SuccessfulCallbacks;

                std::cerr
                    << "[dothack:boot153-eight-packet]"
                    << " stage=materialized"
                    << " sequence=" << s_boot152SuccessfulCallbacks
                    << " mpeg=0x" << std::hex << event.mpegAddr
                    << " func=0x" << callback.func
                    << " user=0x" << callbackUserData
                    << " data=0x" << hostPayloadDataAddr
                    << std::dec
                    << " len=" << payloadLen
                    << " firstLen=" << firstLen
                    << " wrapped=" << (payloadLen > firstLen ? 1 : 0)
                    << " metadataUnchanged=1"
                    << std::endl;
            }

            const uint32_t cbDataAddr =
                runtime->guestMalloc(kMpegCallbackDataSize, 16u);

            if (cbDataAddr == 0u)
            {
                if (hostPayloadUsesRealReadBuf)
                {
                    std::lock_guard<std::mutex> lock(
                        g_mpeg_stub_mutex);

                    g_mpeg_stub_state.hostCallbackInFlight =
                        false;

                    std::cerr
                        << "[dothack:boot153-eight-packet]"
                        << " stage=cbdata-alloc-failed"
                        << " mpeg=0x" << std::hex << event.mpegAddr
                        << std::dec
                        << std::endl;
                }

                return;
            }

            if (!writeMpegCallbackData(
                    rdram,
                    cbDataAddr,
                    materializedEvent))
            {
                runtime->guestFree(cbDataAddr);

                std::cerr
                    << "[dothack:boot153-eight-packet]"
                    << " stage=write-cbdata-failed"
                    << " mpeg=0x" << std::hex << event.mpegAddr
                    << std::dec
                    << std::endl;

                return;
            }

            R5900Context callbackCtx = *callerCtx;
            SET_GPR_U32(&callbackCtx, 4, event.mpegAddr);
            SET_GPR_U32(&callbackCtx, 5, cbDataAddr);
            SET_GPR_U32(&callbackCtx, 6, callbackUserData);
            SET_GPR_U32(&callbackCtx, 7, 0u);
            SET_GPR_U32(&callbackCtx, 29, 0u);
            SET_GPR_U32(&callbackCtx, 31, 0u);
            callbackCtx.pc = callback.func;

            // BOOT 151:
            //
            // Capture the game's AudioDec pointer before dispatch. The PCM
            // callback obtains this from gp-0x6D38. We retain only the address
            // here; the actual state is inspected after the callback returns.
            uint32_t boot151AudioDecAddr = 0u;

            if (hostPayloadUsesRealReadBuf &&
                callback.func == 0x40B140u)
            {
                const uint32_t callbackGp =
                    getRegU32(&callbackCtx, 28);

                const uint32_t audioSlotAddr =
                    callbackGp - 0x6D38u;

                uint8_t *audioSlot =
                    getMemPtr(rdram, audioSlotAddr);

                if (audioSlot)
                {
                    std::memcpy(
                        &boot151AudioDecAddr,
                        audioSlot,
                        sizeof(boot151AudioDecAddr));
                }
            }


            GuestInvocation invocation{};
            invocation.kind = GuestInvocationKind::RpcCallback;
            invocation.context = callbackCtx;

            invocation.onComplete =
                [runtime,
                 rdram,
                 cbDataAddr,
                 hostPayloadUsesRealReadBuf,
                 hostPayloadDataAddr,
                 callbackUserData,
                 boot151AudioDecAddr,
                 boot152ExpectedSourceHash,
                 boot152PayloadLen,
                 boot152WritePos](
                    const R5900Context &,
                    R5900Context &)
            {
                runtime->guestFree(cbDataAddr);

                if (hostPayloadUsesRealReadBuf)
                {
                    static uint32_t s_boot151PostCount = 0u;

                    uint32_t ringWritePos = 0u;
                    uint32_t ringUsed = 0u;
                    uint32_t ringCapacity = 0u;
                    bool ringValid = false;

                    uint8_t *realReadBuf =
                        getMemPtr(rdram, callbackUserData);

                    if (realReadBuf)
                    {
                        std::memcpy(
                            &ringWritePos,
                            realReadBuf + 0x50000u,
                            sizeof(ringWritePos));

                        std::memcpy(
                            &ringUsed,
                            realReadBuf + 0x50004u,
                            sizeof(ringUsed));

                        std::memcpy(
                            &ringCapacity,
                            realReadBuf + 0x50008u,
                            sizeof(ringCapacity));

                        ringValid = true;
                    }

                    int32_t state = -1;
                    int32_t f2c = -1;
                    uint32_t f30 = 0u;
                    int32_t f34 = -1;
                    int32_t f38 = -1;
                    int32_t f3c = -1;
                    uint32_t f44 = 0u;
                    int32_t f48 = -1;
                    int32_t f4c = -1;
                    int32_t f50 = -1;
                    int32_t f54 = -1;
                    int32_t f58 = -1;
                    bool audioValid = false;

                    uint8_t *audioDec = nullptr;

                    if (boot151AudioDecAddr != 0u)
                    {
                        audioDec =
                            getMemPtr(rdram, boot151AudioDecAddr);
                    }

                    if (audioDec)
                    {
                        auto readU32 =
                            [audioDec](size_t offset)
                            {
                                uint32_t value = 0u;

                                std::memcpy(
                                    &value,
                                    audioDec + offset,
                                    sizeof(value));

                                return value;
                            };

                        state =
                            static_cast<int32_t>(readU32(0x00u));

                        f2c =
                            static_cast<int32_t>(readU32(0x2Cu));

                        f30 =
                            readU32(0x30u);

                        f34 =
                            static_cast<int32_t>(readU32(0x34u));

                        f38 =
                            static_cast<int32_t>(readU32(0x38u));

                        f3c =
                            static_cast<int32_t>(readU32(0x3Cu));

                        f44 =
                            readU32(0x44u);

                        f48 =
                            static_cast<int32_t>(readU32(0x48u));

                        f4c =
                            static_cast<int32_t>(readU32(0x4Cu));

                        f50 =
                            static_cast<int32_t>(readU32(0x50u));

                        f54 =
                            static_cast<int32_t>(readU32(0x54u));

                        f58 =
                            static_cast<int32_t>(readU32(0x58u));

                        audioValid = true;
                    }

                    uint32_t boot152ObservedSourceHash = 0u;
                    bool boot152SourceHashValid = false;

                    if (realReadBuf &&
                        ringCapacity == 0x50000u &&
                        boot152PayloadLen > 0u &&
                        boot152PayloadLen <= ringCapacity &&
                        boot152WritePos < ringCapacity)
                    {
                        boot152ObservedSourceHash = 2166136261u;

                        for (uint32_t i = 0u;
                             i < boot152PayloadLen;
                             ++i)
                        {
                            const uint32_t offset =
                                static_cast<uint32_t>(
                                    (static_cast<uint64_t>(
                                         boot152WritePos) +
                                     static_cast<uint64_t>(i)) %
                                    static_cast<uint64_t>(
                                        ringCapacity));

                            boot152ObservedSourceHash ^=
                                realReadBuf[offset];

                            boot152ObservedSourceHash *=
                                16777619u;
                        }

                        boot152SourceHashValid = true;
                    }

                    std::cerr
                        << "[dothack:boot152-source-stability]"
                        << " user=0x" << std::hex
                        << callbackUserData
                        << " data=0x" << hostPayloadDataAddr
                        << " expected=0x"
                        << boot152ExpectedSourceHash
                        << " observed=0x"
                        << boot152ObservedSourceHash
                        << std::dec
                        << " payloadLen="
                        << boot152PayloadLen
                        << " writePos="
                        << boot152WritePos
                        << " hashValid="
                        << (boot152SourceHashValid ? 1 : 0)
                        << " sourceStable="
                        << (
                               boot152SourceHashValid &&
                               boot152ExpectedSourceHash ==
                                   boot152ObservedSourceHash
                               ? 1
                               : 0
                           )
                        << " ringWritePos="
                        << ringWritePos
                        << " ringUsed="
                        << ringUsed
                        << " ringCapacity="
                        << ringCapacity
                        << std::endl;

                    std::cerr
                        << "[dothack:boot151-post-pcm]"
                        << " n=" << s_boot151PostCount++
                        << " user=0x" << std::hex
                        << callbackUserData
                        << " data=0x" << hostPayloadDataAddr
                        << " ad=0x" << boot151AudioDecAddr
                        << std::dec
                        << " audioValid=" << (audioValid ? 1 : 0)
                        << " state=" << state
                        << " f2c=" << f2c
                        << " f30=0x" << std::hex << f30
                        << std::dec
                        << " f34=" << f34
                        << " f38=" << f38
                        << " f3c=" << f3c
                        << " f44=0x" << std::hex << f44
                        << std::dec
                        << " f48=" << f48
                        << " f4c=" << f4c
                        << " f50=" << f50
                        << " f54=" << f54
                        << " f58=" << f58
                        << " ringValid=" << (ringValid ? 1 : 0)
                        << " ringWritePos=" << ringWritePos
                        << " ringUsed=" << ringUsed
                        << " ringCapacity=" << ringCapacity
                        << std::endl;

                    std::lock_guard<std::mutex> lock(
                        g_mpeg_stub_mutex);

                    g_mpeg_stub_state.hostCallbackInFlight =
                        false;

                    std::cerr
                        << "[dothack:boot153-eight-packet]"
                        << " stage=complete"
                        << " data=0x" << std::hex
                        << hostPayloadDataAddr
                        << std::dec
                        << std::endl;
                }
            };

            if (hostPayloadUsesRealReadBuf)
            {
                std::lock_guard<std::mutex> lock(
                    g_mpeg_stub_mutex);

                if (g_mpeg_stub_state.hostCallbackInFlight)
                {
                    runtime->guestFree(cbDataAddr);

                    std::cerr
                        << "[dothack:boot153-eight-packet]"
                        << " stage=inflight-race-skip"
                        << " mpeg=0x" << std::hex
                        << event.mpegAddr
                        << std::dec
                        << std::endl;

                    return;
                }

                g_mpeg_stub_state.hostCallbackInFlight = true;

                std::cerr
                    << "[dothack:boot153-eight-packet]"
                    << " stage=queued"
                    << " mpeg=0x" << std::hex << event.mpegAddr
                    << " func=0x" << callback.func
                    << " user=0x" << callbackUserData
                    << " data=0x" << hostPayloadDataAddr
                    << " cbData=0x" << cbDataAddr
                    << std::dec
                    << std::endl;
            }

            runtime->eeScheduler().queueInvocation(std::move(invocation));
        }

        void dispatchStreamCallbacks(uint8_t *rdram,
                                     R5900Context *ctx,
                                     PS2Runtime *runtime,
                                     const std::vector<MpegStreamCallbackEvent> &events)
        {
            if (events.empty())
            {
                return;
            }

            for (const MpegStreamCallbackEvent &event : events)
            {
                for (const MpegRegisteredCallback &callback : event.callbacks)
                {
                    dispatchGuestStreamCallback(rdram, ctx, runtime, event, callback);
                }
            }
        }

        void dispatchStreamCallbacksUnlocked(uint8_t *rdram,
                                             R5900Context *ctx,
                                             PS2Runtime *runtime,
                                             const std::vector<MpegStreamCallbackEvent> &events)
        {
            if (events.empty())
            {
                return;
            }

            dispatchStreamCallbacks(rdram, ctx, runtime, events);
        }

        void writeBlankMpegFrame(uint8_t *rdram, uint32_t destAddr, uint32_t width, uint32_t height)
        {
            if (!rdram || destAddr == 0u)
            {
                return;
            }

            const uint32_t outWidth = align16(width == 0u ? kStubMovieWidth : width);
            const uint32_t outHeight = align16(height == 0u ? kStubMovieHeight : height);
            const uint32_t macroblockColumns = outWidth / 16u;
            for (uint32_t mbx = 0u; mbx < macroblockColumns; ++mbx)
            {
                const size_t stripOffset =
                    static_cast<size_t>(mbx) * static_cast<size_t>(outHeight) * 16u * 4u;
                for (uint32_t y = 0u; y < outHeight; ++y)
                {
                    uint8_t *dst = getMemPtr(
                        rdram,
                        destAddr + static_cast<uint32_t>(stripOffset + static_cast<size_t>(y) * 16u * 4u));
                    if (!dst)
                    {
                        continue;
                    }
                    for (uint32_t x = 0u; x < 16u; ++x)
                    {
                        dst[x * 4u + 0u] = 0u;
                        dst[x * 4u + 1u] = 0u;
                        dst[x * 4u + 2u] = 0u;
                        dst[x * 4u + 3u] = 0x80u;
                    }
                }
            }
        }

        void writeDecodedFrameToGuest(uint8_t *rdram, uint32_t destAddr, const MpegDecodedFrame &frame)
        {
            if (!rdram || destAddr == 0u || frame.rgba.empty() || frame.width <= 0 || frame.height <= 0)
            {
                return;
            }

            const uint32_t width = static_cast<uint32_t>(frame.width);
            const uint32_t height = static_cast<uint32_t>(frame.height);
            const uint32_t outWidth = align16(width);
            const uint32_t outHeight = align16(height);
            const uint32_t macroblockColumns = outWidth / 16u;

            for (uint32_t mbx = 0u; mbx < macroblockColumns; ++mbx)
            {
                const size_t stripOffset =
                    static_cast<size_t>(mbx) * static_cast<size_t>(outHeight) * 16u * 4u;
                for (uint32_t y = 0u; y < outHeight; ++y)
                {
                    uint8_t *dst = getMemPtr(
                        rdram,
                        destAddr + static_cast<uint32_t>(stripOffset + static_cast<size_t>(y) * 16u * 4u));
                    if (!dst)
                    {
                        continue;
                    }

                    for (uint32_t x = 0u; x < 16u; ++x)
                    {
                        const uint32_t srcX = mbx * 16u + x;
                        const uint8_t *src = nullptr;
                        if (srcX < width && y < height)
                        {
                            src = frame.rgba.data() +
                                  (static_cast<size_t>(y) * static_cast<size_t>(width) + srcX) * 4u;
                        }

                        if (src)
                        {
                            dst[x * 4u + 0u] = src[0u];
                            dst[x * 4u + 1u] = src[1u];
                            dst[x * 4u + 2u] = src[2u];
                            dst[x * 4u + 3u] = 0x80u;
                        }
                        else
                        {
                            dst[x * 4u + 0u] = 0u;
                            dst[x * 4u + 1u] = 0u;
                            dst[x * 4u + 2u] = 0u;
                            dst[x * 4u + 3u] = 0x80u;
                        }
                    }
                }
            }
        }

        void resetMpegStubStateUnlocked()
        {
            g_mpeg_stub_state.initialized = false;
            g_mpeg_stub_state.nextCallbackHandle = 1u;
            g_mpeg_stub_state.cdStreamGeneration = 0u;
            g_mpeg_stub_state.cdStreamBytesProduced = 0u;
            g_mpeg_stub_state.cdStreamBytesDemuxed = 0u;
            g_mpeg_stub_state.cdStreamEofPending = false;
            g_mpeg_stub_state.currentCdStreamEofSeen = false;
            g_mpeg_stub_state.cdStreamStagedBytes.clear();
            g_mpeg_stub_state.pendingHostCallbackEvents.clear();
            g_mpeg_stub_state.hostCallbackInFlight = false;
            g_mpeg_stub_state.cdStreamStageGeneration = 0u;
            g_mpeg_stub_state.cdStreamStageOverflowed = false;
            g_mpeg_stub_state.feedEsTraceCount = 0u;
            g_mpeg_stub_state.demuxPssTraceCount = 0u;
            g_mpeg_stub_state.demuxRingTraceCount = 0u;
            g_mpeg_stub_state.getPictureWaitTraceCount = 0u;
            g_mpeg_stub_state.pictureTraceCount = 0u;
            g_mpeg_stub_state.isEndTraceCount = 0u;
            g_mpeg_stub_state.callbacksByMpeg.clear();
            g_mpeg_stub_state.playbackByMpeg.clear();
        }
    }

    void resetMpegStubState()
    {
        std::lock_guard<std::mutex> lock(g_mpeg_stub_mutex);
        resetMpegStubStateUnlocked();
    }

    void enqueueMpegDecodedFrameForTesting(uint32_t mpegAddr)
    {
        constexpr int kTestFrameWidth = 16;
        constexpr int kTestFrameHeight = 16;

        MpegDecodedFrame frame;
        frame.width = kTestFrameWidth;
        frame.height = kTestFrameHeight;
        frame.rgba.resize(static_cast<size_t>(kTestFrameWidth * kTestFrameHeight * 4), 0x80u);

        std::lock_guard<std::mutex> lock(g_mpeg_stub_mutex);
        MpegPlaybackState &playback = getPlaybackState(mpegAddr);
        playback.sawInput = true;
        playback.decodedFrames.push_back(std::move(frame));
    }

    void notifyMpegCdStreamStart(PS2Runtime *runtime)
    {
        (void)runtime;
        std::lock_guard<std::mutex> lock(g_mpeg_stub_mutex);
        ++g_mpeg_stub_state.cdStreamGeneration;
        g_mpeg_stub_state.cdStreamBytesProduced = 0u;
        g_mpeg_stub_state.cdStreamBytesDemuxed = 0u;
        g_mpeg_stub_state.cdStreamEofPending = false;
        g_mpeg_stub_state.currentCdStreamEofSeen = false;
        g_mpeg_stub_state.cdStreamStagedBytes.clear();
        g_mpeg_stub_state.pendingHostCallbackEvents.clear();
        g_mpeg_stub_state.hostCallbackInFlight = false;
        g_mpeg_stub_state.cdStreamStageOverflowed = false;
        g_mpeg_stub_state.feedEsTraceCount = 0u;
        g_mpeg_stub_state.demuxPssTraceCount = 0u;
        g_mpeg_stub_state.demuxRingTraceCount = 0u;
        g_mpeg_stub_state.getPictureWaitTraceCount = 0u;
        g_mpeg_stub_state.pictureTraceCount = 0u;
        g_mpeg_stub_state.isEndTraceCount = 0u;

        for (auto &[mpegAddr, playback] : g_mpeg_stub_state.playbackByMpeg)
        {
            playback = makeFreshPlaybackStatePreservingConfig(playback);
        }
        PS2_IF_AGRESSIVE_LOGS({
            std::cerr << "[MPEG:CdStreamStart] generation=" << g_mpeg_stub_state.cdStreamGeneration
                      << " reopened=" << g_mpeg_stub_state.playbackByMpeg.size() << std::endl;
        });
    }

        void notifyMpegCdStreamDataProduced(uint32_t byteCount, bool endOfStream)
    {
        std::lock_guard<std::mutex> lock(g_mpeg_stub_mutex);
        g_mpeg_stub_state.cdStreamBytesProduced += byteCount;
        if (endOfStream)
        {
            g_mpeg_stub_state.cdStreamEofPending = true;
        }
    }

    void notifyMpegCdStreamEof(PS2Runtime *runtime)
    {
        std::vector<uint32_t> completedMpegIds;
        bool changed = false;
                {
            std::lock_guard<std::mutex> lock(g_mpeg_stub_mutex);

            // PR #178 host-fed stream state.
            g_mpeg_stub_state.currentCdStreamEofSeen = true;
            g_mpeg_stub_state.cdStreamStagedBytes.clear();
            g_mpeg_stub_state.cdStreamStageOverflowed = false;

            // Preserve the newer runtime-aware EOF handling from our branch.
            finalizeCdStreamEofUnlocked(completedMpegIds, changed);
        }

        if (changed)
        {
            static uint32_t s_eofLogCount = 0u;
            if (s_eofLogCount < 8u)
            {
                PS2_IF_AGRESSIVE_LOGS({
                    std::cerr << "[MPEG:CdStreamEof] finalized active MPEG playback" << std::endl;
                });
                ++s_eofLogCount;
            }
        }
        if (runtime)
        {
            for (const uint32_t mpegAddr : completedMpegIds)
            {
                runtime->eeScheduler().completeExternalWait(kMpegPictureWaitType, mpegAddr, KE_OK);
            }
        }
    }

    // BOOT 170: mirror unique PCM events in demux order.
    //
    // A private PSS audio packet is represented by both PCM and ADPCM
    // callback events for guest compatibility. Host playback must use
    // only the PCM copy. Feeding here preserves exact packet order and
    // prevents the dispatch-time duplicate/rate bottleneck.
    void feedDothackHostMovieAudioEvents(
        PS2Runtime *runtime,
        const std::vector<MpegStreamCallbackEvent> &events,
        const char *stage)
    {
        if (!runtime)
            return;

        size_t packetCount = 0u;
        size_t payloadBytes = 0u;

        for (const MpegStreamCallbackEvent &event : events)
        {
            if (event.streamType != kMpegStrPCM ||
                event.hostPayload.size() <= 4u)
            {
                continue;
            }

            const uint8_t *pcm =
                event.hostPayload.data() + 4u;

            const size_t pcmBytes =
                event.hostPayload.size() - 4u;

            runtime->audioBackend()
                .onMoviePcmTransferFromBuffer(
                    pcm,
                    static_cast<uint32_t>(pcmBytes));

            ++packetCount;
            payloadBytes += pcmBytes;
        }

        if (packetCount != 0u)
        {
            static uint32_t s_boot170LogCount = 0u;

            if (s_boot170LogCount < 256u)
            {
                std::cerr
                    << "[dothack:boot170-demux-audio]"
                    << " n=" << s_boot170LogCount++
                    << " stage="
                    << (stage ? stage : "unknown")
                    << " packets=" << packetCount
                    << " pcmBytes=" << payloadBytes
                    << std::endl;
            }
        }
    }

    size_t feedMpegCdStreamBytes(
        const uint8_t *data,
        size_t size,
        PS2Runtime *runtime)
    {
        if (!data || size == 0u)
        {
            return 0u;
        }

        size_t routedCount = 0u;
        {
            std::lock_guard<std::mutex> lock(g_mpeg_stub_mutex);

            // Active only between notifyMpegCdStreamStart() and notifyMpegCdStreamEof().
            const bool cdStreamActive =
                g_mpeg_stub_state.cdStreamGeneration != 0u &&
                !g_mpeg_stub_state.currentCdStreamEofSeen;
            if (!cdStreamActive)
            {
                return 0u;
            }

            // Host-fed data carries no guest addresses, so no callback event is queued.
            // TODO(host-feed callbacks): queue on MpegPlaybackState for a guest syscall to drain.
            std::vector<MpegStreamCallbackEvent> callbackEvents;
            for (auto &[mpegAddr, playback] : g_mpeg_stub_state.playbackByMpeg)
            {
                if (playback.cdStreamGeneration != g_mpeg_stub_state.cdStreamGeneration)
                {
                    continue;
                }
                appendPssBytes(mpegAddr, playback, data, size, 0u, callbackEvents, false);
                ++routedCount;
            }

            // BOOT 170: feed live host PCM events before guest queuing.
            feedDothackHostMovieAudioEvents(
                runtime,
                callbackEvents,
                "live-refill");

            for (MpegStreamCallbackEvent &event : callbackEvents)
            {
                if (!event.hostPayload.empty())
                {
                    g_mpeg_stub_state.pendingHostCallbackEvents.push_back(
                        std::move(event));
                }
            }

            // No decoder on the current generation took these bytes: stage them until the
            // first sceMpegCreate on this generation; afterwards feed routes live.
            if (routedCount == 0u && !g_mpeg_stub_state.cdStreamStageOverflowed)
            {
                if (g_mpeg_stub_state.cdStreamStageGeneration != g_mpeg_stub_state.cdStreamGeneration)
                {
                    g_mpeg_stub_state.cdStreamStagedBytes.clear();
                    g_mpeg_stub_state.cdStreamStageGeneration = g_mpeg_stub_state.cdStreamGeneration;
                }

                if (g_mpeg_stub_state.cdStreamStagedBytes.size() + size > kMpegHostFeedStageCapBytes)
                {
                    // Dropping the tail would hand a later decoder a gapped stream (staged
                    // prefix, then a hole, then live feed); discard the whole stage instead
                    // and let the next decoder resync cleanly on live feed.
                    g_mpeg_stub_state.cdStreamStagedBytes.clear();
                    g_mpeg_stub_state.cdStreamStageOverflowed = true;
                }
                else
                {
                    g_mpeg_stub_state.cdStreamStagedBytes.insert(
                        g_mpeg_stub_state.cdStreamStagedBytes.end(), data, data + size);
                }
            }
        }

        return size;
    }

    void sceMpegFlush(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        (void)rdram;
        const uint32_t mpegAddr = getRegU32(ctx, 4);
        bool wakePictureWaiter = false;
        {
            std::lock_guard<std::mutex> lock(g_mpeg_stub_mutex);
            MpegPlaybackState &playback = getPlaybackState(mpegAddr);
            const size_t framesBefore = playback.decodedFrames.size();
            if (playback.decoder)
            {
                playback.decoder->flush(playback.decodedFrames);
            }
            wakePictureWaiter = playback.decodedFrames.size() != framesBefore ||
                                playback.streamEnded ||
                                playback.decoderFailed;
        }
        if (wakePictureWaiter)
        {
            runtime->eeScheduler().completeExternalWait(kMpegPictureWaitType, mpegAddr, KE_OK);
        }
        setReturnS32(ctx, 0);
    }

    void sceMpegAddBs(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t mpegAddr = getRegU32(ctx, 4);
        const uint32_t dataAddr = getRegU32(ctx, 5);
        const uint32_t byteCount = getRegU32(ctx, 6);

        size_t copied = 0u;
        bool wakePictureWaiter = false;
        {
            std::lock_guard<std::mutex> lock(g_mpeg_stub_mutex);
            MpegPlaybackState &playback = getPlaybackState(mpegAddr);
            const size_t framesBefore = playback.decodedFrames.size();
            while (copied < byteCount)
            {
                const uint32_t curAddr = dataAddr + static_cast<uint32_t>(copied);
                const uint32_t offset = curAddr & PS2_RAM_MASK;
                const size_t chunk = std::min<size_t>(static_cast<size_t>(byteCount) - copied, PS2_RAM_SIZE - offset);
                const uint8_t *src = getConstMemPtr(rdram, curAddr);
                if (!src || chunk == 0u)
                {
                    break;
                }
                feedElementaryStream(playback, src, chunk);
                copied += chunk;
            }
            wakePictureWaiter = playback.decodedFrames.size() != framesBefore || playback.streamEnded || playback.decoderFailed;
        }

        if (wakePictureWaiter)
        {
            runtime->eeScheduler().completeExternalWait(kMpegPictureWaitType, mpegAddr, KE_OK);
        }
        setReturnS32(ctx, static_cast<int32_t>(copied));
    }

    void sceMpegAddCallback(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        (void)rdram;
        (void)runtime;

        const uint32_t mpegAddr = getRegU32(ctx, 4);
        const uint32_t callbackType = getRegU32(ctx, 5);
        const uint32_t callbackFunc = getRegU32(ctx, 6);
        const uint32_t callbackData = getRegU32(ctx, 7);

        std::lock_guard<std::mutex> lock(g_mpeg_stub_mutex);
        g_mpeg_stub_state.initialized = true;
        (void)getPlaybackState(mpegAddr);

        const uint32_t handle = g_mpeg_stub_state.nextCallbackHandle++;
        g_mpeg_stub_state.callbacksByMpeg[mpegAddr].push_back(
            MpegRegisteredCallback{callbackType, 0u, callbackFunc, callbackData, handle, false});

        setReturnU32(ctx, handle);
    }

    void sceMpegAddStrCallback(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        (void)runtime;
        const uint32_t mpegAddr = getRegU32(ctx, 4);
        const uint32_t streamType = getRegU32(ctx, 5);
        const uint32_t streamId = getRegU32(ctx, 6);
        const uint32_t callbackFunc = getRegU32(ctx, 7);
        const uint32_t callbackData = readAbiArg4(rdram, ctx);

        std::lock_guard<std::mutex> lock(g_mpeg_stub_mutex);
        g_mpeg_stub_state.initialized = true;
        (void)getPlaybackState(mpegAddr);
        const uint32_t handle = g_mpeg_stub_state.nextCallbackHandle++;
        g_mpeg_stub_state.callbacksByMpeg[mpegAddr].push_back(
            MpegRegisteredCallback{streamType, streamId, callbackFunc, callbackData, handle, true});
        setReturnU32(ctx, 0u);
    }

    void sceMpegClearRefBuff(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        (void)ctx;
        (void)runtime;
        static const uint32_t kRefGlobalAddrs[] = {
            0x171800u, 0x17180Cu, 0x171818u, 0x171804u, 0x171810u, 0x17181Cu};
        for (uint32_t addr : kRefGlobalAddrs)
        {
            uint8_t *p = getMemPtr(rdram, addr);
            if (!p)
                continue;
            uint32_t ptr = *reinterpret_cast<uint32_t *>(p);
            if (ptr != 0u)
            {
                uint8_t *q = getMemPtr(rdram, ptr + 0x28u);
                if (q)
                    *reinterpret_cast<uint32_t *>(q) = 0u;
            }
        }
        setReturnU32(ctx, 1u);
    }

    static void mpegGuestWrite32(uint8_t *rdram, uint32_t addr, uint32_t value)
    {
        if (uint8_t *p = getMemPtr(rdram, addr))
            *reinterpret_cast<uint32_t *>(p) = value;
    }
    static void mpegGuestWrite64(uint8_t *rdram, uint32_t addr, uint64_t value)
    {
        if (uint8_t *p = getMemPtr(rdram, addr))
        {
            *reinterpret_cast<uint32_t *>(p) = static_cast<uint32_t>(value);
            *reinterpret_cast<uint32_t *>(p + 4) = static_cast<uint32_t>(value >> 32);
        }
    }

    void sceMpegCreate(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t param_1 = getRegU32(ctx, 4); // a0
        const uint32_t param_2 = getRegU32(ctx, 5); // a1
        const uint32_t param_3 = getRegU32(ctx, 6); // a2

        const uint32_t uVar3 = (param_2 + 3u) & 0xFFFFFFFCu;
        const int32_t iVar2_signed = static_cast<int32_t>(param_3) - static_cast<int32_t>(uVar3 - param_2);

        if (iVar2_signed <= 0x117)
        {
            setReturnU32(ctx, 0u);
            return;
        }

        {
            std::lock_guard<std::mutex> lock(g_mpeg_stub_mutex);
            auto &pb = getPlaybackState(param_1);
            pb = makeFreshPlaybackState();
            if (!g_mpeg_stub_state.cdStreamStageOverflowed &&
                !g_mpeg_stub_state.cdStreamStagedBytes.empty() &&
                g_mpeg_stub_state.cdStreamStageGeneration == pb.cdStreamGeneration)
            {
                std::vector<MpegStreamCallbackEvent> replayEvents; // host feed: none consumed
                appendPssBytes(param_1, pb,
                               g_mpeg_stub_state.cdStreamStagedBytes.data(),
                               g_mpeg_stub_state.cdStreamStagedBytes.size(),
                               0u, replayEvents, /*trackGuestAddrs=*/false);

                // BOOT 170: feed staged startup PCM in demux order.
                feedDothackHostMovieAudioEvents(
                    runtime,
                    replayEvents,
                    "staged-startup");

                for (MpegStreamCallbackEvent &event : replayEvents)
                {
                    if (!event.hostPayload.empty())
                    {
                        g_mpeg_stub_state.pendingHostCallbackEvents.push_back(
                            std::move(event));
                    }
                }

                g_mpeg_stub_state.cdStreamStagedBytes.clear();
            }
        }

        const uint32_t puVar4 = uVar3 + 0x108u;
        const uint32_t innerSize = static_cast<uint32_t>(iVar2_signed) - 0x118u;

        mpegGuestWrite32(rdram, param_1 + 0x40, uVar3);

        const uint32_t a1_init = uVar3 + 0x118u;
        mpegGuestWrite32(rdram, puVar4 + 0x0, a1_init);
        mpegGuestWrite32(rdram, puVar4 + 0x4, innerSize);
        mpegGuestWrite32(rdram, puVar4 + 0x8, a1_init);
        mpegGuestWrite32(rdram, puVar4 + 0xC, a1_init);

        const uint32_t allocResult = runtime ? runtime->guestMalloc(0x600, 8u) : (uVar3 + 0x200u);
        mpegGuestWrite32(rdram, uVar3 + 0x44, allocResult);

        // param_1[0..2] = 0; param_1[4..0xe] = 0xffffffff/0 as per decompilation
        mpegGuestWrite32(rdram, param_1 + 0x00, 0);
        mpegGuestWrite32(rdram, param_1 + 0x04, 0);
        mpegGuestWrite32(rdram, param_1 + 0x08, 0);
        mpegGuestWrite64(rdram, param_1 + 0x10, 0xFFFFFFFFFFFFFFFFULL);
        mpegGuestWrite64(rdram, param_1 + 0x18, 0xFFFFFFFFFFFFFFFFULL);
        mpegGuestWrite64(rdram, param_1 + 0x20, 0);
        mpegGuestWrite64(rdram, param_1 + 0x28, 0xFFFFFFFFFFFFFFFFULL);
        mpegGuestWrite64(rdram, param_1 + 0x30, 0xFFFFFFFFFFFFFFFFULL);
        mpegGuestWrite64(rdram, param_1 + 0x38, 0);

        static const unsigned s_zeroOffsets[] = {
            0xB4, 0xB8, 0xBC, 0xC0, 0xC4, 0xC8, 0xCC, 0xD0, 0xD4, 0xD8, 0xDC, 0xE0, 0xE4, 0xE8, 0xF8,
            0x0C, 0x14, 0x2C, 0x34, 0x3C,
            0x48, 0xFC, 0x100, 0x104, 0x70, 0x90, 0xAC};
        for (unsigned off : s_zeroOffsets)
            mpegGuestWrite32(rdram, uVar3 + off, 0u);
        mpegGuestWrite64(rdram, uVar3 + 0x78, 0);
        mpegGuestWrite64(rdram, uVar3 + 0x88, 0);

        mpegGuestWrite64(rdram, uVar3 + 0xF0, 0xFFFFFFFFFFFFFFFFULL);
        mpegGuestWrite32(rdram, uVar3 + 0x1C, 0x1209F8u);
        mpegGuestWrite32(rdram, uVar3 + 0x24, 0x120A08u);
        mpegGuestWrite32(rdram, uVar3 + 0xB0, 1u);
        mpegGuestWrite32(rdram, uVar3 + 0x9C, 0xFFFFFFFFu);
        mpegGuestWrite32(rdram, uVar3 + 0x80, 0xFFFFFFFFu);
        mpegGuestWrite32(rdram, uVar3 + 0x94, 0xFFFFFFFFu);
        mpegGuestWrite32(rdram, uVar3 + 0x98, 0xFFFFFFFFu);

        mpegGuestWrite32(rdram, 0x1717BCu, param_1);

        static const uint32_t s_refValues[] = {
            0x171A50u, 0x171C58u, 0x171CC0u, 0x171D28u, 0x171D90u,
            0x171AB8u, 0x171B20u, 0x171B88u, 0x171BF0u};
        for (unsigned i = 0; i < 9u; ++i)
            mpegGuestWrite32(rdram, 0x171800u + i * 4u, s_refValues[i]);

        uint32_t setDynamicRet = a1_init;
        if (uint8_t *p = getMemPtr(rdram, puVar4 + 8))
            setDynamicRet = *reinterpret_cast<uint32_t *>(p);
        mpegGuestWrite32(rdram, puVar4 + 12, setDynamicRet);

        setReturnU32(ctx, setDynamicRet);
    }

    void sceMpegDelete(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        (void)rdram;

        const uint32_t mpegAddr = getRegU32(ctx, 4);
        {
            std::lock_guard<std::mutex> lock(g_mpeg_stub_mutex);
            g_mpeg_stub_state.callbacksByMpeg.erase(mpegAddr);
            g_mpeg_stub_state.playbackByMpeg.erase(mpegAddr);
        }
        runtime->eeScheduler().completeExternalWait(kMpegPictureWaitType, mpegAddr, KE_WAIT_DELETE);
        setReturnU32(ctx, 0u);
    }

    void sceMpegDemuxPss(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        const uint32_t mpegAddr = getRegU32(ctx, 4);
        const uint32_t dataAddr = getRegU32(ctx, 5);
        const uint32_t byteCount = getRegU32(ctx, 6);

        std::vector<MpegStreamCallbackEvent> callbackEvents;
        std::vector<uint32_t> completedMpegIds;
        size_t consumed = 0u;
        size_t decodedBefore = 0u;
        size_t decodedCount = 0u;
        uint32_t traceIdx = 0u;
        bool eofChanged = false;
        bool backpressured = false;
        {
            std::lock_guard<std::mutex> lock(g_mpeg_stub_mutex);
            MpegPlaybackState &playback = getPlaybackState(mpegAddr);
            decodedBefore = playback.decodedFrames.size();
            backpressured = mpegDemuxBackpressured(playback);
            if (!backpressured)
            {
                consumed = appendGuestBytes(mpegAddr, playback, rdram, dataAddr, byteCount, callbackEvents);
                recordCdStreamBytesDemuxedUnlocked(consumed, completedMpegIds, eofChanged);
            }
            decodedCount = playback.decodedFrames.size();
            traceIdx = g_mpeg_stub_state.demuxPssTraceCount++;
        }

        if (backpressured)
        {
            if (traceIdx < 32u)
            {
                PS2_IF_AGRESSIVE_LOGS({
                    std::cerr << "[MPEG:DemuxPss:BACKPRESSURE] mpeg=0x" << std::hex << mpegAddr << std::dec << " decoded=" << decodedCount << std::endl;
                });
            }
            setReturnS32(ctx, 0);
            return;
        }
        const bool currentStreamCompleted = std::find(completedMpegIds.begin(), completedMpegIds.end(), mpegAddr) != completedMpegIds.end();
        if (decodedCount != decodedBefore || eofChanged || currentStreamCompleted)
        {
            runtime->eeScheduler().completeExternalWait(kMpegPictureWaitType, mpegAddr, KE_OK);
        }
        for (const uint32_t completedMpegId : completedMpegIds)
        {
            if (completedMpegId != mpegAddr)
            {
                runtime->eeScheduler().completeExternalWait(kMpegPictureWaitType, completedMpegId, KE_OK);
            }
        }

        if (traceIdx < 32u)
        {
            PS2_IF_AGRESSIVE_LOGS({
                std::cerr << "[MPEG:DemuxPss] mpeg=0x" << std::hex << mpegAddr
                          << " data=0x" << dataAddr << std::dec
                          << " bytes=" << byteCount
                          << " consumed=" << consumed
                          << " decoded=" << decodedCount
                          << " callbacks=" << callbackEvents.size()
                          << std::endl;
            });
        }

        dispatchStreamCallbacksUnlocked(rdram, ctx, runtime, callbackEvents);
        setReturnS32(ctx, static_cast<int32_t>(consumed));
    }

    void sceMpegDemuxPssRing(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        static std::atomic<uint32_t> s_demuxRingEntryCount{0u};
        const uint32_t entryIdx = s_demuxRingEntryCount.fetch_add(1u, std::memory_order_relaxed);
        if (entryIdx < 4u)
        {
            PS2_IF_AGRESSIVE_LOGS({
                std::cerr << "[MPEG:DemuxPssRing:ENTER] call #" << entryIdx
                          << " pc=0x" << std::hex << ctx->pc
                          << " ra=0x" << getRegU32(ctx, 31)
                          << std::dec << std::endl;
            });
        }

        const uint32_t mpegAddr = getRegU32(ctx, 4);
        const uint32_t dataAddr = getRegU32(ctx, 5);
        const uint32_t availableBytes = getRegU32(ctx, 6);
        const uint32_t ringBaseAddr = getRegU32(ctx, 7);
        const uint32_t ringSize = readAbiArg4(rdram, ctx);

        // BOOT 157B: pump one serialized host PCM event from ring demux.
        //
        // Boot 155 established that OPENING will not call startDisplay(1)
        // until AudioDec::audioDecIsPreset() becomes true:
        //
        //     AudioDec + 0x54 >= AudioDec + 0x48
        //
        // with OPENING's +0x48 preset target equal to 0x6000 bytes.
        //
        // Boot 156 showed that suppressing GetPicture is not a sustainable
        // preroll pump. The guest eventually stops polling GetPicture while
        // its two-frame VoBuf is full.
        //
        // sceMpegDemuxPssRing continues to participate in the producer loop
        // during that state. At this point the guest producer has already
        // committed its CD bytes to the real ReadBuf, so the existing
        // Boot 152/153 materializer can inspect writePos/used and borrow
        // only the currently FREE region.
        //
        // We drain at most ONE event on each entry. hostCallbackInFlight
        // remains authoritative, so only one real-ReadBuf callback can be
        // outstanding. The existing eight-success diagnostic cap and FNV
        // source-stability verification remain unchanged.
        std::vector<MpegStreamCallbackEvent>
            boot157bHostCallbackEvents;

        size_t boot157bPendingAfterDrain = 0u;
        bool boot157bInFlightAfterDrain = false;

        {
            std::lock_guard<std::mutex> lock(g_mpeg_stub_mutex);

            constexpr size_t
                kDothackHostAudioEventsPerRingDemux = 1u;

            drainPendingHostCallbacksUnlocked(
                mpegAddr,
                boot157bHostCallbackEvents,
                kDothackHostAudioEventsPerRingDemux);

            boot157bPendingAfterDrain =
                g_mpeg_stub_state.pendingHostCallbackEvents.size();

            boot157bInFlightAfterDrain =
                g_mpeg_stub_state.hostCallbackInFlight;
        }

        if (!boot157bHostCallbackEvents.empty())
        {
            static uint32_t s_boot157bPumpCount = 0u;

            if (s_boot157bPumpCount < 96u)
            {
                std::cerr
                    << "[dothack:boot157b-ring-pump]"
                    << " stage=dispatch"
                    << " n=" << s_boot157bPumpCount++
                    << " mpeg=0x" << std::hex << mpegAddr
                    << " data=0x" << dataAddr
                    << " ringBase=0x" << ringBaseAddr
                    << std::dec
                    << " available=" << availableBytes
                    << " ringSize=" << ringSize
                    << " events="
                    << boot157bHostCallbackEvents.size()
                    << " pending="
                    << boot157bPendingAfterDrain
                    << " inFlightBeforeDispatch="
                    << (boot157bInFlightAfterDrain ? 1 : 0)
                    << std::endl;
            }

            dispatchStreamCallbacksUnlocked(
                rdram,
                ctx,
                runtime,
                boot157bHostCallbackEvents);
        }
        else
        {
            static uint32_t s_boot157bIdleCount = 0u;

            if (s_boot157bIdleCount < 32u)
            {
                bool currentInFlight = false;
                size_t currentPending = 0u;

                {
                    std::lock_guard<std::mutex> lock(g_mpeg_stub_mutex);

                    currentInFlight =
                        g_mpeg_stub_state.hostCallbackInFlight;

                    currentPending =
                        g_mpeg_stub_state.pendingHostCallbackEvents.size();
                }

                if (currentPending != 0u || currentInFlight)
                {
                    std::cerr
                        << "[dothack:boot157b-ring-pump]"
                        << " stage=idle"
                        << " n=" << s_boot157bIdleCount++
                        << " mpeg=0x" << std::hex << mpegAddr
                        << std::dec
                        << " pending=" << currentPending
                        << " inFlight="
                        << (currentInFlight ? 1 : 0)
                        << std::endl;
                }
            }
        }

        std::vector<MpegStreamCallbackEvent> callbackEvents;
        std::vector<uint32_t> completedMpegIds;
        size_t consumed = 0u;
        size_t decodedBefore = 0u;
        size_t decodedCount = 0u;
        uint32_t traceIdx = 0u;
        bool eofChanged = false;
        bool backpressured = false;
        {
            std::lock_guard<std::mutex> lock(g_mpeg_stub_mutex);
            MpegPlaybackState &playback = getPlaybackState(mpegAddr);
            decodedBefore = playback.decodedFrames.size();
            backpressured = mpegDemuxBackpressured(playback);
            if (!backpressured)
            {
                consumed = appendGuestRingBytes(
                    mpegAddr,
                    playback,
                    rdram,
                    dataAddr,
                    availableBytes,
                    ringBaseAddr,
                    ringSize,
                    callbackEvents);
                recordCdStreamBytesDemuxedUnlocked(consumed, completedMpegIds, eofChanged);
            }
            decodedCount = playback.decodedFrames.size();
            traceIdx = g_mpeg_stub_state.demuxRingTraceCount++;
        }

        if (backpressured)
        {
            if (traceIdx < 32u)
            {
                PS2_IF_AGRESSIVE_LOGS({
                    std::cerr << "[MPEG:DemuxPssRing:BACKPRESSURE] mpeg=0x" << std::hex << mpegAddr
                              << std::dec << " decoded=" << decodedCount
                              << " avail=" << availableBytes << std::endl;
                });
            }
            setReturnS32(ctx, 0);
            return;
        }
        const bool currentStreamCompleted = std::find(completedMpegIds.begin(), completedMpegIds.end(), mpegAddr) != completedMpegIds.end();
        if (decodedCount != decodedBefore || eofChanged || currentStreamCompleted)
        {
            runtime->eeScheduler().completeExternalWait(kMpegPictureWaitType, mpegAddr, KE_OK);
        }
        for (const uint32_t completedMpegId : completedMpegIds)
        {
            if (completedMpegId != mpegAddr)
            {
                runtime->eeScheduler().completeExternalWait(kMpegPictureWaitType, completedMpegId, KE_OK);
            }
        }

        if (traceIdx < 32u)
        {
            PS2_IF_AGRESSIVE_LOGS({
                std::cerr << "[MPEG:DemuxPssRing] mpeg=0x" << std::hex << mpegAddr
                          << " data=0x" << dataAddr
                          << " ring=0x" << ringBaseAddr << std::dec
                          << " avail=" << availableBytes
                          << " consumed=" << consumed
                          << " decoded=" << decodedCount
                          << " callbacks=" << callbackEvents.size()
                          << std::endl;
            });
        }

        dispatchStreamCallbacksUnlocked(rdram, ctx, runtime, callbackEvents);
        setReturnS32(ctx, static_cast<int32_t>(consumed));
    }

    void sceMpegDispCenterOffX(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        (void)rdram;
        (void)runtime;
        setReturnS32(ctx, 0);
    }

    void sceMpegDispCenterOffY(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        (void)rdram;
        (void)runtime;
        setReturnS32(ctx, 0);
    }

    void sceMpegDispHeight(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        (void)rdram;
        (void)runtime;
        const uint32_t mpegAddr = getRegU32(ctx, 4);
        std::lock_guard<std::mutex> lock(g_mpeg_stub_mutex);
        setReturnU32(ctx, getPlaybackState(mpegAddr).height);
    }

    void sceMpegDispWidth(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        (void)rdram;
        (void)runtime;
        const uint32_t mpegAddr = getRegU32(ctx, 4);
        std::lock_guard<std::mutex> lock(g_mpeg_stub_mutex);
        setReturnU32(ctx, getPlaybackState(mpegAddr).width);
    }

    void sceMpegGetDecodeMode(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        (void)rdram;
        (void)runtime;
        const uint32_t mpegAddr = getRegU32(ctx, 4);
        std::lock_guard<std::mutex> lock(g_mpeg_stub_mutex);
        setReturnU32(ctx, getPlaybackState(mpegAddr).decodeMode);
    }

    void sceMpegGetPicture(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        static uint64_t dothackGetPictureCallCount = 0;
        const uint64_t dothackGetPictureCall = ++dothackGetPictureCallCount;

        if (dothackGetPictureCall <= 64u)
        {
            std::cerr
                << "[dothack:mpeg-getpic-enter]"
                << " call=" << dothackGetPictureCall
                << " mpeg=0x" << std::hex << getRegU32(ctx, 4)
                << " image=0x" << getRegU32(ctx, 5)
                << std::dec
                << " tick="
                << (runtime != nullptr
                        ? runtime->eeScheduler().currentVSyncTick()
                        : 0u)
                << std::endl;
        }

        const uint32_t mpegAddr = getRegU32(ctx, 4);
        const uint32_t imageAddr = getRegU32(ctx, 5);

        // BOOT 165:
        // Revisit the queued host PCM once per movie presentation cycle.
        // The initial PSS demux can outrun Raylib's double-buffered stream;
        // this lets buffers refill after the audio device consumes them.
        if (runtime != nullptr)
        {
            runtime->audioBackend().pumpMovieAudio();
        }

        // BOOT 175:
        // Sample the exact Boot 172 soundtrack clock before entering
        // the MPEG mutex. A negative value means no host PSS soundtrack
        // has started, so silent/logo movies retain the old behavior.
        const double boot175MovieAudioSeconds =
            runtime != nullptr
                ? runtime->audioBackend().movieAudioElapsedSeconds()
                : -1.0;

        // BOOT 167: low/high-water OPENING refill.
        //
        // The 4 MiB startup feed produces roughly 147 decoded OPENING
        // pictures.  Do not keep appending the entire movie.  Instead,
        // when presentation drains the live queue to <= 96 pictures,
        // append one 512 KiB chunk.  A chunk of this size has historically
        // represented roughly 18-20 movie pictures, keeping the queue
        // bounded around a few seconds of video.
        //
        // Critically, inspect the queue under the MPEG lock but perform
        // the CD read/feed only AFTER releasing it: feedMpegCdStreamBytes()
        // acquires the same MPEG mutex internally.
        constexpr size_t kBoot167RefillLowWaterFrames = 112u;
        constexpr size_t kBoot167RefillChunkBytes =
            1024u * 1024u;

        bool boot167ShouldRefill = false;
        size_t boot167QueueBefore = 0u;

        {
            std::lock_guard<std::mutex> lock(
                g_mpeg_stub_mutex);

            const auto playbackIt =
                g_mpeg_stub_state.playbackByMpeg.find(
                    mpegAddr);

            if (playbackIt !=
                g_mpeg_stub_state.playbackByMpeg.end())
            {
                const MpegPlaybackState &playback =
                    playbackIt->second;

                boot167QueueBefore =
                    playback.decodedFrames.size();

                boot167ShouldRefill =
                    playback.cdStreamGeneration ==
                        g_mpeg_stub_state.cdStreamGeneration &&
                    !g_mpeg_stub_state.currentCdStreamEofSeen &&
                    playback.sawInput &&
                    boot167QueueBefore <=
                        kBoot167RefillLowWaterFrames;
            }
        }

        if (runtime != nullptr &&
            boot167ShouldRefill)
        {
            const size_t accepted =
                refillDothackHostPssStream(
                    runtime,
                    kBoot167RefillChunkBytes);

            if (accepted != 0u)
            {
                static uint32_t
                    s_boot167GetPictureRefillCount = 0u;

                if (s_boot167GetPictureRefillCount < 256u)
                {
                    size_t queueAfter = 0u;

                    {
                        std::lock_guard<std::mutex> lock(
                            g_mpeg_stub_mutex);

                        const auto playbackIt =
                            g_mpeg_stub_state.playbackByMpeg.find(
                                mpegAddr);

                        if (playbackIt !=
                            g_mpeg_stub_state.playbackByMpeg.end())
                        {
                            queueAfter =
                                playbackIt->second
                                    .decodedFrames.size();
                        }
                    }

                    std::cerr
                        << "[dothack:boot167-getpic-refill]"
                        << " n="
                        << s_boot167GetPictureRefillCount++
                        << " mpeg=0x"
                        << std::hex << mpegAddr
                        << std::dec
                        << " queueBefore="
                        << boot167QueueBefore
                        << " accepted="
                        << accepted
                        << " queueAfter="
                        << queueAfter
                        << std::endl;
                }
            }
        }

        std::vector<MpegStreamCallbackEvent> hostCallbackEvents;
        size_t boot144PendingAfterDrain = 0u;
        {
            std::lock_guard<std::mutex> lock(g_mpeg_stub_mutex);

            constexpr size_t kDothackHostAudioEventsPerPicture = 1u;
            drainPendingHostCallbacksUnlocked(
                mpegAddr,
                hostCallbackEvents,
                kDothackHostAudioEventsPerPicture);

            boot144PendingAfterDrain =
                g_mpeg_stub_state.pendingHostCallbackEvents.size();
        }

        if (!hostCallbackEvents.empty())
        {
            static uint32_t s_boot144DrainLogCount = 0u;
            if (s_boot144DrainLogCount < 32u)
            {
                std::cerr
                    << "[dothack:boot144-host-audio]"
                    << " stage=drain"
                    << " n=" << s_boot144DrainLogCount++
                    << " mpeg=0x" << std::hex << mpegAddr
                    << std::dec
                    << " events=" << hostCallbackEvents.size()
                    << " pending=" << boot144PendingAfterDrain
                    << std::endl;
            }

            dispatchStreamCallbacksUnlocked(
                rdram,
                ctx,
                runtime,
                hostCallbackEvents);
        }

        uint32_t width = kStubMovieWidth;
        uint32_t height = kStubMovieHeight;
        uint32_t frameCount = 0u;
        bool haveFrame = false;
        MpegDecodedFrame frame;
        {
            std::unique_lock<std::mutex> lock(g_mpeg_stub_mutex);
            MpegPlaybackState &playback = getPlaybackState(mpegAddr);
            if (playback.decodedFrames.empty() &&
                !g_mpeg_stub_state.currentCdStreamEofSeen &&
                !playback.streamEnded &&
                !playback.decoderFailed)
            {
                if (g_mpeg_stub_state.getPictureWaitTraceCount < 32u)
                {
                    PS2_IF_AGRESSIVE_LOGS({
                        std::cerr
    << "[dothack:mpeg-wait]"
    << " mpeg=0x" << std::hex << mpegAddr
    << std::dec
    << " ended=" << playback.streamEnded
    << " failed=" << playback.decoderFailed
    << " sawInput=" << playback.sawInput
    << " queued=" << playback.decodedFrames.size()
    << std::endl;
                    });
                    ++g_mpeg_stub_state.getPictureWaitTraceCount;
                }
                lock.unlock();
                runtime->eeScheduler().waitExternal(
                    EeWaitReason::Mpeg,
                    kMpegPictureWaitType,
                    mpegAddr,
                    [rdram, runtime](R5900Context &resumeContext)
                    {
                        if (static_cast<int32_t>(getRegU32(&resumeContext, 2)) < 0)
                        {
                            return;
                        }
                        if (dothackGetPictureCallCount <= 64u)
                        {
                            std::cerr
                                << "[dothack:mpeg-getpic-resume-1]"
                                << " tick="
                                << (runtime != nullptr
                                        ? runtime->eeScheduler().currentVSyncTick()
                                        : 0u)
                                << " v0=" << static_cast<int32_t>(getRegU32(&resumeContext, 2))
                                << std::endl;
                        }
                        sceMpegGetPicture(rdram, &resumeContext, runtime);
                    });
            }

            if (dothackGetPictureCall <= 64u)
            {
                std::cerr
                    << "[dothack:mpeg-getpic-ready]"
                    << " call=" << dothackGetPictureCall
                    << " queued=" << playback.decodedFrames.size()
                    << " served=" << playback.picturesServed
                    << " streamEnded=" << (playback.streamEnded ? 1 : 0)
                    << " decoderFailed=" << (playback.decoderFailed ? 1 : 0)
                    << " sawInput=" << (playback.sawInput ? 1 : 0)
                    << " tick="
                    << (runtime != nullptr
                            ? runtime->eeScheduler().currentVSyncTick()
                            : 0u)
                    << std::endl;
            }

            if (!playback.decodedFrames.empty())
            {
                const uint64_t currentTick = runtime->eeScheduler().currentVSyncTick();
                const uint64_t currentTickQ32 = currentTick << 32u;
                const MpegDecodedFrame &nextFrame = playback.decodedFrames.front();
                uint64_t frameIntervalQ32 = decodedFrameIntervalQ32(playback, nextFrame);
                uint64_t presentationTargetQ32 = presentationTickForFrame(playback, nextFrame, currentTickQ32);

                if (currentTickQ32 > presentationTargetQ32 && currentTickQ32 - presentationTargetQ32 >= frameIntervalQ32)
                {
                    const uint64_t correction = currentTickQ32 - presentationTargetQ32;
                    presentationTargetQ32 = currentTickQ32;
                    if (nextFrame.pts90k >= 0 && playback.firstPresentedPts90k >= 0)
                    {
                        playback.ptsPresentationBaseTickQ32 += correction;
                    }
                    playback.nextPictureTickQ32 = currentTickQ32;
                }

                if (currentTickQ32 < presentationTargetQ32)
                {
                    const uint64_t eligibleTick = (presentationTargetQ32 + kPictureClockOne - 1u) >> 32u;
                    lock.unlock();
                    if (dothackGetPictureCall <= 64u)
                    {
                        std::cerr
                            << "[dothack:mpeg-getpic-wait-vsync]"
                            << " call=" << dothackGetPictureCall
                            << " current=" << currentTick
                            << " eligible=" << eligibleTick
                            << " queued=" << playback.decodedFrames.size()
                            << std::endl;
                    }

                    runtime->eeScheduler().waitVSync(
                        eligibleTick - 1u,
                        -1,
                        [rdram, runtime](R5900Context &resumeContext)
                        {
                            if (static_cast<int32_t>(getRegU32(&resumeContext, 2)) < 0)
                            {
                                return;
                            }
                            if (dothackGetPictureCallCount <= 64u)
                            {
                                std::cerr
                                    << "[dothack:mpeg-getpic-resume-2]"
                                    << " tick="
                                    << (runtime != nullptr
                                            ? runtime->eeScheduler().currentVSyncTick()
                                            : 0u)
                                    << " v0=" << static_cast<int32_t>(getRegU32(&resumeContext, 2))
                                    << std::endl;
                            }
                            sceMpegGetPicture(rdram, &resumeContext, runtime);
                        });
                }

                // BOOT 175:
                // Keep the source-video timeline aligned with the host
                // soundtrack without changing either audio rate or the
                // guest's VBlank cadence.
                //
                // At each presentation opportunity, determine where the
                // source-video producer should be according to elapsed
                // real audio time. If normal presentation of this frame
                // would still leave us one or more COMPLETE source frames
                // behind, discard only those late source pictures.
                //
                // This is standard late-frame recovery: presentation
                // cadence stays continuous, while source content catches
                // up to the audio master clock.
                if (playback.boot175HostClockArmed &&
                    boot175MovieAudioSeconds >=
                        playback.boot175HostClockBaseSeconds &&
                    playback.decodedFrames.size() > 1u)
                {
                    const uint64_t nominalIntervalQ32 =
                        playback.pictureIntervalQ32 != 0u
                            ? playback.pictureIntervalQ32
                            : kDefaultPictureIntervalQ32;

                    const double frameSeconds =
                        (static_cast<double>(nominalIntervalQ32) /
                         static_cast<double>(kPictureClockOne)) *
                        (1001.0 / 60000.0);

                    if (frameSeconds > 0.0)
                    {
                        const double audioProgressSeconds =
                            boot175MovieAudioSeconds -
                            playback.boot175HostClockBaseSeconds;

                        const uint64_t expectedAdvance =
                            static_cast<uint64_t>(
                                audioProgressSeconds /
                                frameSeconds);

                        const uint64_t expectedSourceProgress =
                            playback.boot175HostClockBaseSourceProgress +
                            expectedAdvance;

                        const uint64_t currentSourceProgress =
                            static_cast<uint64_t>(
                                playback.picturesServed) +
                            playback.boot175DroppedFrames;

                        // One picture will be presented normally below.
                        const uint64_t sourceProgressAfterNormalFrame =
                            currentSourceProgress + 1u;

                        if (expectedSourceProgress >
                            sourceProgressAfterNormalFrame)
                        {
                            const uint64_t wantedDrops =
                                expectedSourceProgress -
                                sourceProgressAfterNormalFrame;

                            const uint64_t availableDrops =
                                static_cast<uint64_t>(
                                    playback.decodedFrames.size() - 1u);

                            const uint64_t dropCount =
                                std::min(
                                    wantedDrops,
                                    availableDrops);

                            if (dropCount != 0u)
                            {
                                for (uint64_t i = 0u;
                                     i < dropCount;
                                     ++i)
                                {
                                    playback.decodedFrames.pop_front();
                                }

                                playback.boot175DroppedFrames +=
                                    dropCount;

                                // The replacement source picture must occupy
                                // the SAME presentation slot. Re-anchor its PTS
                                // to this slot so presentationTickForFrame()
                                // does not reintroduce the time we just
                                // recovered on the next invocation.
                                const MpegDecodedFrame &selectedFrame =
                                    playback.decodedFrames.front();

                                if (selectedFrame.pts90k >= 0)
                                {
                                    playback.firstPresentedPts90k =
                                        selectedFrame.pts90k;

                                    playback.ptsPresentationBaseTickQ32 =
                                        presentationTargetQ32;
                                }
                                else
                                {
                                    playback.firstPresentedPts90k = -1;
                                    playback.ptsPresentationBaseTickQ32 = 0u;
                                }

                                playback.nextPictureTickQ32 =
                                    presentationTargetQ32;

                                frameIntervalQ32 =
                                    decodedFrameIntervalQ32(
                                        playback,
                                        selectedFrame);

                                static uint32_t
                                    s_boot175DropLogCount = 0u;

                                if (s_boot175DropLogCount < 256u)
                                {
                                    std::cerr
                                        << "[dothack:boot175-av-sync]"
                                        << " stage=drop"
                                        << " n="
                                        << s_boot175DropLogCount++
                                        << " drop="
                                        << dropCount
                                        << " totalDropped="
                                        << playback.boot175DroppedFrames
                                        << " served="
                                        << playback.picturesServed
                                        << " sourceProgress="
                                        << (static_cast<uint64_t>(
                                                playback.picturesServed) +
                                            playback.boot175DroppedFrames)
                                        << " expected="
                                        << expectedSourceProgress
                                        << " audio="
                                        << boot175MovieAudioSeconds
                                        << " audioProgress="
                                        << audioProgressSeconds
                                        << " frameSeconds="
                                        << frameSeconds
                                        << " queued="
                                        << playback.decodedFrames.size()
                                        << std::endl;
                                }
                            }
                        }
                    }
                }

                frame = std::move(playback.decodedFrames.front());
                playback.decodedFrames.pop_front();
                playback.width = static_cast<uint32_t>(frame.width);
                playback.height = static_cast<uint32_t>(frame.height);
                width = playback.width;
                height = playback.height;
                frameCount = playback.picturesServed;
                playback.picturesServed += 1u;
                // BOOT 175:
                // The first picture handed to the guest after physical
                // audio playback begins defines the producer-side baseline.
                // Any existing movie preroll therefore remains intact.
                if (!playback.boot175HostClockArmed &&
                    boot175MovieAudioSeconds >= 0.0)
                {
                    playback.boot175HostClockArmed = true;
                    playback.boot175HostClockBaseSeconds =
                        boot175MovieAudioSeconds;
                    playback.boot175HostClockBaseSourceProgress =
                        static_cast<uint64_t>(
                            playback.picturesServed) +
                        playback.boot175DroppedFrames;

                    const uint64_t nominalIntervalQ32 =
                        playback.pictureIntervalQ32 != 0u
                            ? playback.pictureIntervalQ32
                            : kDefaultPictureIntervalQ32;

                    const double frameSeconds =
                        (static_cast<double>(nominalIntervalQ32) /
                         static_cast<double>(kPictureClockOne)) *
                        (1001.0 / 60000.0);

                    std::cerr
                        << "[dothack:boot175-av-sync]"
                        << " stage=armed"
                        << " served="
                        << playback.picturesServed
                        << " dropped="
                        << playback.boot175DroppedFrames
                        << " sourceBase="
                        << playback.boot175HostClockBaseSourceProgress
                        << " audioBase="
                        << playback.boot175HostClockBaseSeconds
                        << " frameSeconds="
                        << frameSeconds
                        << " nominalFps="
                        << (frameSeconds > 0.0
                                ? 1.0 / frameSeconds
                                : 0.0)
                        << std::endl;
                }

                // Low-volume progress diagnostics. These are deliberately
                // sparse so the probe does not recreate Boot 174's
                // line-by-line timing overhead.
                if (playback.boot175HostClockArmed &&
                    boot175MovieAudioSeconds >=
                        playback.boot175HostClockBaseSeconds &&
                    playback.picturesServed != 0u &&
                    (playback.picturesServed % 300u) == 0u)
                {
                    const uint64_t nominalIntervalQ32 =
                        playback.pictureIntervalQ32 != 0u
                            ? playback.pictureIntervalQ32
                            : kDefaultPictureIntervalQ32;

                    const double frameSeconds =
                        (static_cast<double>(nominalIntervalQ32) /
                         static_cast<double>(kPictureClockOne)) *
                        (1001.0 / 60000.0);

                    const uint64_t sourceProgress =
                        static_cast<uint64_t>(
                            playback.picturesServed) +
                        playback.boot175DroppedFrames;

                    const uint64_t sourceAdvance =
                        sourceProgress >=
                                playback.boot175HostClockBaseSourceProgress
                            ? sourceProgress -
                                  playback.boot175HostClockBaseSourceProgress
                            : 0u;

                    const double videoProgressSeconds =
                        static_cast<double>(sourceAdvance) *
                        frameSeconds;

                    const double audioProgressSeconds =
                        boot175MovieAudioSeconds -
                        playback.boot175HostClockBaseSeconds;

                    std::cerr
                        << "[dothack:boot175-av-sync]"
                        << " stage=sample"
                        << " served="
                        << playback.picturesServed
                        << " dropped="
                        << playback.boot175DroppedFrames
                        << " sourceProgress="
                        << sourceProgress
                        << " audioProgress="
                        << audioProgressSeconds
                        << " videoProgress="
                        << videoProgressSeconds
                        << " drift="
                        << (audioProgressSeconds -
                            videoProgressSeconds)
                        << std::endl;
                }

                playback.nextPictureTickQ32 = presentationTargetQ32 + frameIntervalQ32;
                playback.presentationEndTickQ32 = playback.nextPictureTickQ32;
                haveFrame = true;
                if (g_mpeg_stub_state.pictureTraceCount < 32u)
{
    std::cerr
        << "[dothack:mpeg-frame]"
        << " mpeg=0x" << std::hex << mpegAddr
        << " image=0x" << imageAddr
        << std::dec
        << " generation=" << g_mpeg_stub_state.cdStreamGeneration
        << " frame=" << frameCount
        << " queued=" << playback.decodedFrames.size()
        << " size=" << width << "x" << height
        << std::endl;

    ++g_mpeg_stub_state.pictureTraceCount;
}
            }
            else
            {
                width = playback.width;
                height = playback.height;
                frameCount = playback.picturesServed;
            }
        }

        mpegGuestWrite32(rdram, mpegAddr + 0x00u, width);
        mpegGuestWrite32(rdram, mpegAddr + 0x04u, height);
        mpegGuestWrite32(rdram, mpegAddr + 0x08u, frameCount);

        if (uint8_t *base = getMemPtr(rdram, mpegAddr))
        {
            const uint32_t iVar1 = *reinterpret_cast<uint32_t *>(base + 0x40);
            if (uint8_t *inner = getMemPtr(rdram, iVar1))
            {
                *reinterpret_cast<uint32_t *>(inner + 0xb0) = 1;
                *reinterpret_cast<uint32_t *>(inner + 0xd8) = (getRegU32(ctx, 5) & 0x0FFFFFFFu) | 0x20000000u;
                *reinterpret_cast<uint32_t *>(inner + 0xe4) = getRegU32(ctx, 6);
                *reinterpret_cast<uint32_t *>(inner + 0xdc) = 0;
                *reinterpret_cast<uint32_t *>(inner + 0xe0) = 0;
            }
        }

        if (haveFrame)
        {
            writeDecodedFrameToGuest(rdram, imageAddr, frame);
        }
        else if (frameCount == 0u)
        {
            writeBlankMpegFrame(rdram, imageAddr, width, height);
        }

        setReturnS32(ctx, 0);
    }

    void sceMpegGetPictureRAW8(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        TODO_NAMED("sceMpegGetPictureRAW8", rdram, ctx, runtime);
    }

    void sceMpegGetPictureRAW8xy(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        TODO_NAMED("sceMpegGetPictureRAW8xy", rdram, ctx, runtime);
    }

    void sceMpegInit(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        (void)rdram;
        std::lock_guard<std::mutex> lock(g_mpeg_stub_mutex);
        const uint64_t cdStreamGeneration = g_mpeg_stub_state.cdStreamGeneration;
        const uint64_t cdStreamBytesProduced = g_mpeg_stub_state.cdStreamBytesProduced;
        const uint64_t cdStreamBytesDemuxed = g_mpeg_stub_state.cdStreamBytesDemuxed;
        const bool cdStreamEofPending = g_mpeg_stub_state.cdStreamEofPending;
        const bool currentCdStreamEofSeen = g_mpeg_stub_state.currentCdStreamEofSeen;
        resetMpegStubStateUnlocked();
        g_mpeg_stub_state.initialized = true;
        g_mpeg_stub_state.cdStreamGeneration = cdStreamGeneration;
        g_mpeg_stub_state.cdStreamBytesProduced = cdStreamBytesProduced;
        g_mpeg_stub_state.cdStreamBytesDemuxed = cdStreamBytesDemuxed;
        g_mpeg_stub_state.cdStreamEofPending = cdStreamEofPending;
        g_mpeg_stub_state.currentCdStreamEofSeen = currentCdStreamEofSeen;
        setReturnU32(ctx, 0u);
    }

    void sceMpegIsEnd(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        (void)rdram;
        const uint32_t mpegAddr = getRegU32(ctx, 4);

        // BOOT 175:
        // Capture host audio time outside the MPEG mutex.
        const double boot175MovieAudioSeconds =
            runtime != nullptr
                ? runtime->audioBackend().movieAudioElapsedSeconds()
                : -1.0;

        std::lock_guard<std::mutex> lock(g_mpeg_stub_mutex);
        g_mpeg_stub_state.initialized = true;
        MpegPlaybackState &playback = getPlaybackState(mpegAddr);
        // Only the producer/demux EOF is authoritative. A sequence_end_code can
        // be observed while more PSS data is still buffered, and a decoder
        // failure before producer EOF may still recover on a later sequence.
        const bool producerEnded =
            g_mpeg_stub_state.currentCdStreamEofSeen &&
            playback.cdStreamGeneration == g_mpeg_stub_state.cdStreamGeneration;
        const bool ended = producerEnded &&
                           (playback.streamEnded || (playback.decoderFailed && playback.sawInput));
        const uint64_t presentationEnd = playback.presentationEndTickQ32;
        const uint64_t currentTickQ32 = runtime != nullptr
                                            ? (runtime->eeScheduler().currentVSyncTick() << 32u)
                                            : std::numeric_limits<uint64_t>::max();
        const bool presentationComplete =
            presentationEnd == std::numeric_limits<uint64_t>::max() ||
            currentTickQ32 >= presentationEnd;

        if (g_mpeg_stub_state.isEndTraceCount < 16u)
        {
            PS2_IF_AGRESSIVE_LOGS({
                std::cerr
    << "[dothack:mpeg-is-end]"
    << " mpeg=0x" << std::hex << mpegAddr
    << std::dec
    << " producerEof=" << producerEnded
    << " streamEnded=" << playback.streamEnded
    << " seqEnd=" << playback.sawSequenceEnd
    << " presentationComplete=" << presentationComplete
    << " queued=" << playback.decodedFrames.size()
    << " sawInput=" << playback.sawInput
    << std::endl;
            });
            ++g_mpeg_stub_state.isEndTraceCount;
        }

        const bool normalEnd =
            ended &&
            playback.decodedFrames.empty() &&
            presentationComplete;

        // .hack can consume the complete MPEG program while the host CD-stream
        // EOF bookkeeping still has not asserted producerEnded. Once the PSS
        // program and MPEG sequence have ended, every decoded picture has been
        // consumed, and presentation of the final picture is complete, the
        // decoder itself is authoritative enough to finish the movie.
        const bool decoderComplete =
            playback.streamEnded &&
            playback.sawInput &&
            playback.sawSequenceEnd &&
            playback.decodedFrames.empty() &&
            presentationComplete;

        const bool fallbackEnd =
            !producerEnded &&
            decoderComplete;

        // BOOT 175:
        // One final low-volume synchronization summary.
        if ((normalEnd || fallbackEnd) &&
            playback.boot175HostClockArmed &&
            !playback.boot175FinalSyncLogged)
        {
            playback.boot175FinalSyncLogged = true;

            const uint64_t nominalIntervalQ32 =
                playback.pictureIntervalQ32 != 0u
                    ? playback.pictureIntervalQ32
                    : kDefaultPictureIntervalQ32;

            const double frameSeconds =
                (static_cast<double>(nominalIntervalQ32) /
                 static_cast<double>(kPictureClockOne)) *
                (1001.0 / 60000.0);

            const uint64_t sourceProgress =
                static_cast<uint64_t>(
                    playback.picturesServed) +
                playback.boot175DroppedFrames;

            const uint64_t sourceAdvance =
                sourceProgress >=
                        playback.boot175HostClockBaseSourceProgress
                    ? sourceProgress -
                          playback.boot175HostClockBaseSourceProgress
                    : 0u;

            const double videoProgressSeconds =
                static_cast<double>(sourceAdvance) *
                frameSeconds;

            const double audioProgressSeconds =
                boot175MovieAudioSeconds >=
                        playback.boot175HostClockBaseSeconds
                    ? boot175MovieAudioSeconds -
                          playback.boot175HostClockBaseSeconds
                    : 0.0;

            std::cerr
                << "[dothack:boot175-av-sync]"
                << " stage=final"
                << " served="
                << playback.picturesServed
                << " dropped="
                << playback.boot175DroppedFrames
                << " sourceProgress="
                << sourceProgress
                << " audioProgress="
                << audioProgressSeconds
                << " videoProgress="
                << videoProgressSeconds
                << " drift="
                << (audioProgressSeconds -
                    videoProgressSeconds)
                << " frameSeconds="
                << frameSeconds
                << std::endl;
        }

        setReturnS32(ctx, (normalEnd || fallbackEnd) ? 1 : 0);
    }

    void sceMpegIsRefBuffEmpty(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        (void)rdram;
        (void)runtime;
        const uint32_t mpegAddr = getRegU32(ctx, 4);
        std::lock_guard<std::mutex> lock(g_mpeg_stub_mutex);
        const MpegPlaybackState &playback = getPlaybackState(mpegAddr);
        setReturnS32(ctx, playback.decodedFrames.empty() ? 1 : 0);
    }

    void sceMpegReset(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        (void)runtime;
        const uint32_t param_1 = getRegU32(ctx, 4);
        {
            std::lock_guard<std::mutex> lock(g_mpeg_stub_mutex);
            MpegPlaybackState &playback = getPlaybackState(param_1);
            MpegPlaybackState resetState = makeFreshPlaybackStatePreservingConfig(playback);
            if (playback.streamEnded || playback.decoderFailed)
            {
                resetState.sawInput = true;
                resetState.streamEnded = true;
                resetState.cdStreamGeneration = playback.cdStreamGeneration;
            }
            playback = std::move(resetState);
        }
        uint8_t *base = getMemPtr(rdram, param_1);
        if (!base)
        {
            return;
        }
        uint32_t inner = *reinterpret_cast<uint32_t *>(base + 0x40);
        if (inner == 0u)
            return;
        mpegGuestWrite32(rdram, param_1 + 0x00u, 0u);
        mpegGuestWrite32(rdram, param_1 + 0x04u, 0u);
        mpegGuestWrite32(rdram, param_1 + 0x08u, 0u);
        mpegGuestWrite32(rdram, inner + 0x00, 0u);
        mpegGuestWrite32(rdram, inner + 0x04, 0u);
        mpegGuestWrite32(rdram, inner + 0x08, 0u);
        mpegGuestWrite32(rdram, param_1 + 0x08, 0u);
        mpegGuestWrite32(rdram, inner + 0x80, 0xFFFFFFFFu);
        mpegGuestWrite32(rdram, inner + 0xAC, 0u);
        mpegGuestWrite32(rdram, 0x171904u, 0u);
    }

    void sceMpegResetDefaultPtsGap(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        (void)rdram;
        (void)runtime;
        setReturnS32(ctx, 0);
    }

    void sceMpegSetDecodeMode(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        (void)rdram;
        (void)runtime;
        const uint32_t mpegAddr = getRegU32(ctx, 4);
        const uint32_t mode = getRegU32(ctx, 5);
        std::lock_guard<std::mutex> lock(g_mpeg_stub_mutex);
        getPlaybackState(mpegAddr).decodeMode = mode;
        setReturnS32(ctx, 0);
    }

    void sceMpegSetDefaultPtsGap(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        (void)rdram;
        (void)runtime;
        setReturnS32(ctx, 0);
    }

    void sceMpegSetImageBuff(uint8_t *rdram, R5900Context *ctx, PS2Runtime *runtime)
    {
        (void)rdram;
        (void)runtime;
        const uint32_t mpegAddr = getRegU32(ctx, 4);
        const uint32_t imageBufferAddr = getRegU32(ctx, 5);
        std::lock_guard<std::mutex> lock(g_mpeg_stub_mutex);
        getPlaybackState(mpegAddr).imageBufferAddr = imageBufferAddr;
        setReturnS32(ctx, 0);
    }
}

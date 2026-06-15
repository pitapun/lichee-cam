// NintiDetect — LicheeRV Nano on-board YOLO detector.
// Base: ret7020 YoloCamera. Adapted: filter person/vehicle/animal (COCO),
// emit each kept detection as a JSON line over UDP (for the Python sidecar
// that does zones + MQTT + Home Assistant), and stream annotated MJPEG :7777.
#include "MJPEGWriter.h"
#include "yolo.hpp"
#include "cvi_venc.h"
#include "cvi_sys.h"
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <fstream>
#include <string>
#include <deque>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

volatile uint8_t interrupted = 0;
void interrupt_handler(int signum) {
    interrupted = 1;
}

// Global camera pointer for crash-cleanup (avoids VPSS leak on segfault)
static cv::VideoCapture *g_cap = nullptr;
static void crash_handler(int signum) {
    fprintf(stderr, "stream_yolo signal %d — releasing camera\n", signum);
    if (g_cap) { g_cap->release(); g_cap = nullptr; }
    signal(signum, SIG_DFL);
    raise(signum);
}

// COCO-80 class names.
static const char *COCO[80] = {
"person","bicycle","car","motorcycle","airplane","bus","train","truck","boat","traffic light",
"fire hydrant","stop sign","parking meter","bench","bird","cat","dog","horse","sheep","cow",
"elephant","bear","zebra","giraffe","backpack","umbrella","handbag","tie","suitcase","frisbee",
"skis","snowboard","sports ball","kite","baseball bat","baseball glove","skateboard","surfboard","tennis racket","bottle",
"wine glass","cup","fork","knife","spoon","bowl","banana","apple","sandwich","orange",
"broccoli","carrot","hot dog","pizza","donut","cake","chair","couch","potted plant","bed",
"dining table","toilet","tv","laptop","mouse","remote","keyboard","cell phone","microwave","oven",
"toaster","sink","refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush"};

// Map a COCO class id to the category we care about, or NULL to drop it.
static const char *category(int cls) {
    if (cls == 0) return "person";
    if (cls == 1 || cls == 2 || cls == 3 || cls == 5 || cls == 7) return "vehicle"; // bicycle/car/motorcycle/bus/truck
    if (cls >= 14 && cls <= 23) return "animal"; // bird..giraffe
    return NULL;
}

static int udp_fd = -1;
static struct sockaddr_in udp_addr;
static void udp_init(const char *ip, int port) {
    udp_fd = socket(AF_INET, SOCK_DGRAM, 0);
    memset(&udp_addr, 0, sizeof(udp_addr));
    udp_addr.sin_family = AF_INET;
    udp_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip, &udp_addr.sin_addr);
}
static void udp_send(const char *s) {
    if (udp_fd >= 0) sendto(udp_fd, s, strlen(s), 0, (struct sockaddr *)&udp_addr, sizeof(udp_addr));
}
static long now_ms() {
    struct timespec ts; clock_gettime(CLOCK_REALTIME, &ts);
    return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

static bool env_on(const char* name) {
    const char* v = getenv(name);
    return v && v[0] && strcmp(v, "0") != 0;
}

static long shutdown_t0_ms = 0;
static void shutdown_stage(const char* stage) {
    long now = now_ms();
    if (shutdown_t0_ms == 0) shutdown_t0_ms = now;
    fprintf(stderr, "[shutdown] stage=%s t=%ldms\n", stage, now - shutdown_t0_ms);
}

struct HdRecordJob {
    std::string path;
    cv::Mat image;
    std::string done_dir;
    bool marker;
};

static std::deque<HdRecordJob> hd_queue;
static std::mutex hd_queue_mutex;
static std::condition_variable hd_queue_cv;
static bool hd_writer_stop = false;

static void hd_writer_loop() {
    std::vector<int> params;
    params.push_back(cv::IMWRITE_JPEG_QUALITY);
    params.push_back(35);
    for (;;) {
        HdRecordJob job;
        {
            std::unique_lock<std::mutex> lock(hd_queue_mutex);
            hd_queue_cv.wait(lock, [] { return hd_writer_stop || !hd_queue.empty(); });
            if (hd_writer_stop && hd_queue.empty()) break;
            job = hd_queue.front();
            hd_queue.pop_front();
        }
        if (job.marker) {
            std::string done_path = job.done_dir + "/.hd_done";
            FILE *f = fopen(done_path.c_str(), "w");
            if (f) {
                fputs("done\n", f);
                fclose(f);
            }
            continue;
        }
        if (!job.path.empty() && !job.image.empty() && !cv::imwrite(job.path, job.image, params)) {
            fprintf(stderr, "[hdrec] write failed %s\n", job.path.c_str());
        }
    }
}

static void hd_enqueue_frame(const std::string &path, const cv::Mat &image) {
    std::lock_guard<std::mutex> lock(hd_queue_mutex);
    if (hd_queue.size() >= 48) {
        fprintf(stderr, "[hdrec] queue full, dropping frame\n");
        return;
    }
    HdRecordJob job;
    job.path = path;
    job.image = image.clone();
    job.marker = false;
    hd_queue.push_back(job);
    hd_queue_cv.notify_one();
}

static void hd_enqueue_done(const std::string &dir) {
    if (dir.empty()) return;
    std::lock_guard<std::mutex> lock(hd_queue_mutex);
    HdRecordJob job;
    job.done_dir = dir;
    job.marker = true;
    hd_queue.push_back(job);
    hd_queue_cv.notify_one();
}

// HlsStreamer: encodes frames via CHN0 VENC and writes Annex-B H264 to a FIFO
// for ffmpeg to segment into HLS.  Drain is synchronous (called inside send())
// so GetStream always runs right after SendFrame — eliminating the BUF_EMPTY
// race that plagued the old drain_loop thread.
class HlsStreamer {
public:
    HlsStreamer() : active(false), chn(0), fd(-1), event_fp_(nullptr), event_bytes_(0),
                    stream_started(false),
                    idr_pending(false), venc_active(false),
                    saved_w(0), saved_h(0), saved_fps(0), drain_requested_(false) {}

    bool start(int w, int h, int fps) {
        // Sanity floor — encoder rejects sub-multiples of 16.
        if (w < 64) w = 64;
        if (h < 64) h = 64;
        saved_w = w; saved_h = h; saved_fps = fps;
        sps_nalu.reserve(256);
        pps_nalu.reserve(256);

        static const char* FIFO = "/tmp/hls_feed.h264";
        while (fd < 0) {
            fd = open(FIFO, O_RDWR | O_NONBLOCK);
            if (fd < 0) usleep(200000);
        }
        fcntl(fd, F_SETPIPE_SZ, 1024 * 1024);
        { char tmp[4096]; while (read(fd, tmp, sizeof(tmp)) > 0) {} }  // flush stale

        CVI_VENC_StopRecvFrame(chn);
        CVI_VENC_DestroyChn(chn);
        if (!create_and_start_venc()) { close(fd); fd = -1; return false; }

        usleep(50000);  // 50ms: let encoder thread initialize before first SendFrame
        CVI_VENC_RequestIDR(chn, CVI_FALSE);
        active = true;
        if (!drain_thr_.joinable())
            drain_thr_ = std::thread([this]{ drain_thread_fn(); });
        fprintf(stderr, "[hls] started %dx%d@%d\n", w, h, fps);
        return true;
    }

    void pause_venc() {
        if (!venc_active) return;
        venc_active = false;
        drain_cv_.notify_all();  // wake drain thread so it sees venc_active=false
        usleep(60000);           // 60ms >> 5ms poll interval — drain exits cleanly
        fprintf(stderr, "[hls] pause CHN0\n");
        CVI_VENC_StopRecvFrame(chn);
        CVI_VENC_DestroyChn(chn);
    }

    void resume_venc() {
        if (venc_active) return;
        fprintf(stderr, "[hls] resume CHN0\n");
        if (!create_and_start_venc()) { fprintf(stderr, "[hls] resume failed\n"); return; }
        idr_pending = true;
    }

    void send(VIDEO_FRAME_INFO_S* frame) {
        if (!active || !frame || !venc_active) return;
        // Skip if drain thread hasn't caught up — avoids VENC queue overflow
        if (drain_requested_.load()) return;
        if (idr_pending.exchange(false)) {
            CVI_VENC_RequestIDR(chn, CVI_FALSE);
            fprintf(stderr, "[hls] IDR requested (resume)\n");
        }
        static int send_count = 0;
        long sf_t0 = now_ms();
        CVI_S32 ret = CVI_VENC_SendFrame(chn, frame, 100);
        long sf_ms = now_ms() - sf_t0;
        if (ret == CVI_SUCCESS || ret == CVI_TRUE) {
            if (++send_count <= 10 || send_count % 50 == 0)
                fprintf(stderr, "[hls] SendFrame ok #%d (ret=%d sf=%ldms)\n", send_count, ret, sf_ms);
            drain_requested_ = true;
            drain_cv_.notify_one();
        } else {
            fprintf(stderr, "[hls] SendFrame failed %#x\n", ret);
        }
    }

    void stop() {
        if (!active) return;
        stop_event_record();
        active = false;
        venc_active = false;
        drain_cv_.notify_all();
        CVI_VENC_StopRecvFrame(chn);
        CVI_VENC_DestroyChn(chn);
        if (drain_thr_.joinable()) {
            drain_thr_.detach();
            fprintf(stderr, "[hls] drain thread detached during shutdown\n");
        }
        if (fd >= 0) { close(fd); fd = -1; }
        fprintf(stderr, "[hls] stopped\n");
    }

    bool is_active() const { return active; }
    bool is_venc_active() const { return venc_active.load(); }
    bool is_drain_requested() const { return drain_requested_.load(); }
    VENC_CHN get_chn() const { return chn; }
    int get_w() const { return saved_w; }
    int get_h() const { return saved_h; }

    bool start_event_record(const std::string& path, long preroll_ms = 2500) {
        std::vector<uint8_t> preroll = snapshot_preroll(preroll_ms);
        FILE* f = fopen(path.c_str(), "wb");
        if (!f) {
            fprintf(stderr, "[hlsrec] fopen failed %s\n", path.c_str());
            return false;
        }
        size_t wrote = 0;
        if (!preroll.empty()) {
            wrote = fwrite(preroll.data(), 1, preroll.size(), f);
        }
        {
            std::lock_guard<std::mutex> lk(event_mtx_);
            if (event_fp_) {
                fflush(event_fp_);
                fclose(event_fp_);
            }
            event_fp_ = f;
            event_path_ = path;
            event_bytes_ = wrote;
        }
        fprintf(stderr, "[hlsrec] event start %s preroll=%zu wrote=%zu\n",
                path.c_str(), preroll.size(), wrote);
        return true;
    }

    void stop_event_record() {
        std::lock_guard<std::mutex> lk(event_mtx_);
        if (!event_fp_) return;
        fflush(event_fp_);
        fclose(event_fp_);
        fprintf(stderr, "[hlsrec] event stop bytes=%zu path=%s\n",
                event_bytes_, event_path_.c_str());
        event_fp_ = nullptr;
        event_path_.clear();
        event_bytes_ = 0;
    }

    std::vector<uint8_t> snapshot_preroll(long window_ms = 2500) {
        std::lock_guard<std::mutex> lk(preroll_mtx_);
        std::vector<uint8_t> out;
        if (preroll_.empty()) return out;
        long cutoff = now_ms() - window_ms;
        size_t start = preroll_.size();
        for (size_t i = 0; i < preroll_.size(); i++) {
            if (preroll_[i].key_start && preroll_[i].ts_ms >= cutoff) {
                start = i;
                break;
            }
        }
        if (start == preroll_.size()) {
            for (size_t i = preroll_.size(); i > 0; i--) {
                if (preroll_[i - 1].key_start) {
                    start = i - 1;
                    break;
                }
            }
        }
        if (start == preroll_.size()) start = 0;
        size_t total = 0;
        for (size_t i = start; i < preroll_.size(); i++) total += preroll_[i].data.size();
        out.reserve(total);
        for (size_t i = start; i < preroll_.size(); i++) {
            out.insert(out.end(), preroll_[i].data.begin(), preroll_[i].data.end());
        }
        return out;
    }

private:
    struct PrerollChunk {
        long ts_ms;
        bool key_start;
        std::vector<uint8_t> data;
    };

    void remember_preroll(std::vector<uint8_t>&& bytes, bool key_start) {
        if (bytes.empty()) return;
        long ts = now_ms();
        std::lock_guard<std::mutex> lk(preroll_mtx_);
        preroll_.push_back({ts, key_start, std::move(bytes)});
        const long keep_ms = 5000;
        long cutoff = ts - keep_ms;
        while (preroll_.size() > 1 && preroll_.front().ts_ms < cutoff) {
            preroll_.pop_front();
        }
    }

    void remember_preroll(const uint8_t* d, ssize_t sz, bool key_start) {
        if (!d || sz <= 0) return;
        std::vector<uint8_t> bytes(d, d + sz);
        remember_preroll(std::move(bytes), key_start);
    }

    void write_event_bytes(const uint8_t* d, size_t sz) {
        if (!d || sz == 0) return;
        std::lock_guard<std::mutex> lk(event_mtx_);
        if (!event_fp_) return;
        size_t wrote = fwrite(d, 1, sz, event_fp_);
        event_bytes_ += wrote;
    }

    bool create_and_start_venc() {
        VENC_CHN_ATTR_S attr;
        memset(&attr, 0, sizeof(attr));
        attr.stVencAttr.enType = PT_H264;
        attr.stVencAttr.u32MaxPicWidth = saved_w;
        attr.stVencAttr.u32MaxPicHeight = saved_h;
        attr.stVencAttr.u32PicWidth = saved_w;
        attr.stVencAttr.u32PicHeight = saved_h;
        attr.stVencAttr.u32BufSize = saved_w * saved_h * 2;
        attr.stVencAttr.u32Profile = 2;
        attr.stVencAttr.bByFrame = CVI_TRUE;
        attr.stVencAttr.stAttrH264e.bRcnRefShareBuf = CVI_FALSE;
        attr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
        attr.stRcAttr.stH264Cbr.u32Gop = saved_fps;  // IDR every ~1s at target fps
        attr.stRcAttr.stH264Cbr.u32StatTime = 1;
        attr.stRcAttr.stH264Cbr.u32SrcFrameRate = saved_fps;
        attr.stRcAttr.stH264Cbr.fr32DstFrameRate = saved_fps;
        attr.stRcAttr.stH264Cbr.u32BitRate = (saved_w >= 1920) ? 3000 : 1500;
        attr.stGopAttr.enGopMode = VENC_GOPMODE_NORMALP;
        attr.stGopAttr.stNormalP.s32IPQpDelta = 2;
        if (CVI_VENC_CreateChn(chn, &attr) != CVI_SUCCESS) {
            fprintf(stderr, "[hls] CreateChn failed\n");
            return false;
        }
        VENC_RECV_PIC_PARAM_S recv;
        memset(&recv, 0, sizeof(recv));
        recv.s32RecvPicNum = -1;
        if (CVI_VENC_StartRecvFrame(chn, &recv) != CVI_SUCCESS) {
            CVI_VENC_DestroyChn(chn);
            return false;
        }
        venc_active = true;
        return true;
    }

    void fifo_write(const uint8_t* d, ssize_t sz) {
        if (fd < 0 || sz <= 0) return;
        ssize_t n = write(fd, d, sz);
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            // FIFO full — ffmpeg likely dead. Drain pipe and reset stream so the
            // next reconnect starts from a clean IDR.
            char buf[4096];
            while (read(fd, buf, sizeof(buf)) > 0) {}
            if (stream_started) {
                fprintf(stderr, "[hls] FIFO full — drained, reset stream\n");
                stream_started = false;
            }
        }
    }

    void drain(int timeout_ms) {
        if (!venc_active || fd < 0) return;
        VENC_CHN_STATUS_S status;
        memset(&status, 0, sizeof(status));
        long deadline = now_ms() + timeout_ms;
        do {
            if (!venc_active) return;  // channel destroyed mid-poll
            if (CVI_VENC_QueryStatus(chn, &status) == CVI_SUCCESS
                    && status.u32CurPacks > 0) break;
            if (timeout_ms <= 0) return;
            usleep(5000);
        } while (now_ms() < deadline);
        if (!venc_active || status.u32CurPacks == 0) return;

        VENC_STREAM_S stream;
        memset(&stream, 0, sizeof(stream));
        stream.u32PackCount = status.u32CurPacks;
        pack_buf_.resize(status.u32CurPacks);
        memset(pack_buf_.data(), 0, pack_buf_.size() * sizeof(VENC_PACK_S));
        stream.pstPack = pack_buf_.data();

        if (CVI_VENC_GetStream(chn, &stream, timeout_ms) == CVI_SUCCESS) {
            for (CVI_U32 i = 0; i < stream.u32PackCount; i++) {
                VENC_PACK_S* p = &stream.pstPack[i];
                uint8_t* d = p->pu8Addr + p->u32Offset;
                ssize_t sz = (ssize_t)(p->u32Len - p->u32Offset);
                H264E_NALU_TYPE_E nalu = p->DataType.enH264EType;
                if (nalu == H264E_NALU_SPS) {
                    sps_nalu.assign(d, d + sz);
                } else if (nalu == H264E_NALU_PPS) {
                    pps_nalu.assign(d, d + sz);
                } else if (nalu == H264E_NALU_IDRSLICE
                           && !sps_nalu.empty() && !pps_nalu.empty()) {
                    if (!stream_started) {
                        stream_started = true;
                        fprintf(stderr, "[hls] first IDR — stream started\n");
                    }
                    std::vector<uint8_t> au;
                    au.reserve(sps_nalu.size() + pps_nalu.size() + (size_t)sz);
                    au.insert(au.end(), sps_nalu.begin(), sps_nalu.end());
                    au.insert(au.end(), pps_nalu.begin(), pps_nalu.end());
                    au.insert(au.end(), d, d + sz);
                    remember_preroll(std::move(au), true);
                    write_event_bytes(sps_nalu.data(), sps_nalu.size());
                    write_event_bytes(pps_nalu.data(), pps_nalu.size());
                    write_event_bytes(d, (size_t)sz);
                    fifo_write(sps_nalu.data(), sps_nalu.size());
                    fifo_write(pps_nalu.data(), pps_nalu.size());
                    fifo_write(d, sz);
                } else if (stream_started) {
                    remember_preroll(d, sz, false);
                    write_event_bytes(d, (size_t)sz);
                    fifo_write(d, sz);
                }
            }
            CVI_VENC_ReleaseStream(chn, &stream);
        }
    }

    void drain_thread_fn() {
        long idr_last_ms = 0;
        while (active) {
            {
                std::unique_lock<std::mutex> lk(drain_mtx_);
                drain_cv_.wait(lk, [this]{ return drain_requested_.load() || !active; });
                if (!active) break;
                // Keep drain_requested_=true during drain so main loop doesn't send
                // another frame while the encoder is still processing the current one
                // (would cause NOBUF). Cleared after drain completes below.
            }
            drain(500);
            drain_requested_ = false;  // allow main loop to send next frame
            // Request IDR here (VENC idle after drain) — avoids blocking main thread
            if (venc_active) {
                long now_t = now_ms();
                if (now_t - idr_last_ms >= 2000) {
                    CVI_VENC_RequestIDR(chn, CVI_FALSE);
                    idr_last_ms = now_t;
                }
            }
        }
    }

    bool active, stream_started;
    VENC_CHN chn;
    int fd;
    FILE* event_fp_;
    std::mutex event_mtx_;
    std::string event_path_;
    size_t event_bytes_;
    std::atomic<bool> idr_pending;
    std::atomic<bool> venc_active;
    std::vector<uint8_t> sps_nalu, pps_nalu;
    std::vector<VENC_PACK_S> pack_buf_;
    std::deque<PrerollChunk> preroll_;
    std::mutex preroll_mtx_;
    int saved_w, saved_h, saved_fps;
    std::thread drain_thr_;
    std::mutex drain_mtx_;
    std::condition_variable drain_cv_;
    std::atomic<bool> drain_requested_;
};

class H264Recorder {
public:
    H264Recorder() : active(false), chn_ready(false), chn(1), fp(nullptr), frames(0) {}

    // Call once at startup to pre-create CHN1 so start() never destroy/creates mid-run.
    bool init(int w, int h, int fps) {
        if (chn_ready) return true;
        VENC_CHN_ATTR_S attr;
        memset(&attr, 0, sizeof(attr));
        attr.stVencAttr.enType = PT_H264;
        attr.stVencAttr.u32MaxPicWidth = w;
        attr.stVencAttr.u32MaxPicHeight = h;
        attr.stVencAttr.u32PicWidth = w;
        attr.stVencAttr.u32PicHeight = h;
        attr.stVencAttr.u32BufSize = w * h;
        attr.stVencAttr.u32Profile = 2;
        attr.stVencAttr.bByFrame = CVI_TRUE;
        attr.stVencAttr.stAttrH264e.bRcnRefShareBuf = CVI_FALSE;
        attr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
        attr.stRcAttr.stH264Cbr.u32Gop = fps;
        attr.stRcAttr.stH264Cbr.u32StatTime = 1;
        attr.stRcAttr.stH264Cbr.u32SrcFrameRate = fps;
        attr.stRcAttr.stH264Cbr.fr32DstFrameRate = fps;
        attr.stRcAttr.stH264Cbr.u32BitRate = (w >= 1920) ? 6000 : 3000;
        attr.stGopAttr.enGopMode = VENC_GOPMODE_NORMALP;
        attr.stGopAttr.stNormalP.s32IPQpDelta = 2;
        // Try to create directly; if channel already exists, destroy first.
        if (CVI_VENC_CreateChn(chn, &attr) != CVI_SUCCESS) {
            CVI_VENC_StopRecvFrame(chn);
            CVI_VENC_DestroyChn(chn);
            if (CVI_VENC_CreateChn(chn, &attr) != CVI_SUCCESS) {
                fprintf(stderr, "[venc] init CreateChn failed\n");
                return false;
            }
        }
        VENC_RECV_PIC_PARAM_S recv;
        memset(&recv, 0, sizeof(recv));
        recv.s32RecvPicNum = -1;
        if (CVI_VENC_StartRecvFrame(chn, &recv) != CVI_SUCCESS) {
            fprintf(stderr, "[venc] init StartRecvFrame failed\n");
            CVI_VENC_DestroyChn(chn);
            return false;
        }
        chn_ready = true;
        fprintf(stderr, "[venc] CHN1 pre-initialized %dx%d@%d\n", w, h, fps);
        return true;
    }

    bool start(const std::string& path, int w, int h, int fps) {
        // Finalize any in-progress recording.
        if (active) {
            for (int i = 0; i < 10; i++) drain(100);
            if (fp) { fflush(fp); fclose(fp); fp = nullptr; }
            fprintf(stderr, "[venc] stop frames=%d path=%s\n", frames, out_path.c_str());
            active = false;
        }
        FILE* f = fopen(path.c_str(), "wb");
        if (!f) {
            fprintf(stderr, "[venc] fopen failed %s\n", path.c_str());
            return false;
        }
        // Ensure CHN1 is ready; init handles chn_ready guard.
        if (!init(w, h, fps)) { fclose(f); return false; }
        // Flush any stale packs from previous event.
        for (int i = 0; i < 5; i++) drain(50);
        CVI_VENC_RequestIDR(chn, CVI_FALSE);
        fp = f;
        active = true;
        frames = 0;
        out_path = path;
        fprintf(stderr, "[venc] start %s %dx%d@%d\n", path.c_str(), w, h, fps);
        return true;
    }

    void send(VIDEO_FRAME_INFO_S* frame) {
        if (!active || !frame) return;
        CVI_S32 ret = CVI_VENC_SendFrame(chn, frame, 200);
        if (ret == CVI_SUCCESS || ret == CVI_TRUE) {
            frames++;
            drain(1000);
        } else {
            fprintf(stderr, "[venc] SendFrame failed %#x\n", ret);
            drain(200);
        }
    }

    void stop() {
        if (!active) return;
        for (int i = 0; i < 10; i++) drain(100);
        // Phase 1 leak mitigation: keep CHN1 alive across events. The
        // CVITEK encoder driver leaks ~10KB per CreateChn/DestroyChn pair,
        // so cycling channels per event burns ~20MB over ~600 events and
        // OOMs stream_yolo. Leaving CHN1 in StartRecvFrame state is safe:
        // send() is gated by `active`, so no frames flow between events;
        // start() drains residual packs and issues a fresh IDR.
        if (fp) {
            fflush(fp);
            fclose(fp);
            fp = nullptr;
        }
        fprintf(stderr, "[venc] stop frames=%d path=%s (chn kept)\n", frames, out_path.c_str());
        active = false;
    }

    void destroy() {
        // Full teardown for process exit: stop() only closes fp now (keeps
        // CHN1 alive to avoid per-event leak), so do the actual VENC
        // shutdown here.
        stop();
        if (chn_ready) {
            CVI_VENC_StopRecvFrame(chn);
            CVI_VENC_DestroyChn(chn);
            chn_ready = false;
        }
    }

    bool is_active() const { return active; }

private:
    void drain(int timeout_ms) {
        if (!active || !fp) return;
        VENC_CHN_STATUS_S status;
        memset(&status, 0, sizeof(status));
        long deadline = now_ms() + timeout_ms;
        do {
            if (CVI_VENC_QueryStatus(chn, &status) == CVI_SUCCESS && status.u32CurPacks > 0) {
                break;
            }
            if (timeout_ms <= 0) return;
            usleep(10000);
        } while (now_ms() < deadline);
        if (status.u32CurPacks == 0) {
            return;
        }
        VENC_STREAM_S stream;
        memset(&stream, 0, sizeof(stream));
        stream.u32PackCount = status.u32CurPacks;
        stream.pstPack = (VENC_PACK_S*)calloc(status.u32CurPacks, sizeof(VENC_PACK_S));
        if (!stream.pstPack) return;
        CVI_S32 ret = CVI_VENC_GetStream(chn, &stream, timeout_ms);
        if (ret == CVI_SUCCESS) {
            for (CVI_U32 i = 0; i < stream.u32PackCount; i++) {
                VENC_PACK_S* pack = &stream.pstPack[i];
                fwrite(pack->pu8Addr + pack->u32Offset,
                       pack->u32Len - pack->u32Offset, 1, fp);
            }
            CVI_VENC_ReleaseStream(chn, &stream);
        }
        free(stream.pstPack);
    }

    bool active;
    bool chn_ready;
    VENC_CHN chn;
    FILE* fp;
    int frames;
    std::string out_path;
};

static std::string dirname_of(const std::string& path) {
    size_t pos = path.find_last_of('/');
    if (pos == std::string::npos) return ".";
    return path.substr(0, pos);
}

static void write_hd_done(const std::string& dir) {
    if (dir.empty()) return;
    std::string done_path = dir + "/.hd_done";
    FILE* f = fopen(done_path.c_str(), "w");
    if (f) {
        fputs("done\n", f);
        fclose(f);
    }
}

int main(int argc, char *argv[]) {
    setlinebuf(stdout);
    setlinebuf(stderr);
    signal(SIGINT,  interrupt_handler);
    signal(SIGTERM, interrupt_handler);
    signal(SIGSEGV, crash_handler);
    signal(SIGABRT, crash_handler);
    signal(SIGPIPE, SIG_IGN);

    if (argc < 4) {
        printf("Usage: %s <model.cvimodel> <class_cnt> <infer_size> [thresh] [--file <img.jpg>]\n", argv[0]);
        return 0;
    }
    // File test mode: stream_yolo model classes size thresh --file img.jpg
    for (int i = 1; i < argc - 1; ++i) {
        if (strcmp(argv[i], "--file") == 0) {
            float thr = (argc > 4) ? atof(argv[4]) : 0.35f;
            int sz = atoi(argv[3]);
            YoloModelDetector det2;
            if (det2.setup_model(argv[1], atoi(argv[2]), thr, 0.5f) != 1) {
                printf("setup_model failed\n"); return -1;
            }
            cv::Mat img = cv::imread(argv[i+1]);
            if (img.empty()) { printf("cannot read %s\n", argv[i+1]); return -1; }
            cv::resize(img, img, cv::Size(sz, sz));
            // Split BGR into planar buffers (model expects RGB planar)
            std::vector<cv::Mat> ch;
            cv::split(img, ch);  // ch[0]=B ch[1]=G ch[2]=R
            // Reorder to R,G,B for RGB input
            unsigned char *planeR = ch[2].data, *planeG = ch[1].data, *planeB = ch[0].data;
            VIDEO_FRAME_INFO_S fi = {};
            fi.stVFrame.u32Width  = sz; fi.stVFrame.u32Height = sz;
            fi.stVFrame.pu8VirAddr[0] = planeR;
            fi.stVFrame.pu8VirAddr[1] = planeG;
            fi.stVFrame.pu8VirAddr[2] = planeB;
            fi.stVFrame.u32Stride[0] = sz;
            fi.stVFrame.u32Stride[1] = sz;
            fi.stVFrame.u32Stride[2] = sz;
            fi.stVFrame.u64PhyAddr[0] = 0;
            fi.stVFrame.u64PhyAddr[1] = 0;
            fi.stVFrame.u64PhyAddr[2] = 0;
            cvtdl_object_t objs = det2.detect(&fi, false, false);
            printf("Detections: %u\n", objs.size);
            for (uint32_t k = 0; k < objs.size; ++k) {
                printf("  [%u] %s score=%.3f x1=%.1f y1=%.1f x2=%.1f y2=%.1f\n",
                       k, objs.info[k].name, objs.info[k].bbox.score,
                       objs.info[k].bbox.x1, objs.info[k].bbox.y1,
                       objs.info[k].bbox.x2, objs.info[k].bbox.y2);
            }
            det2.free_objects(&objs);
            det2.release();
            return 0;
        }
    }
    float thresh   = (argc > 4) ? atof(argv[4]) : 0.45f;
    int udp_port   = (argc > 5) ? atoi(argv[5]) : 5005;
    int infer_size = atoi(argv[3]);
    int disp_w     = (argc > 6) ? atoi(argv[6]) : infer_size;
    int disp_h     = (argc > 7) ? atoi(argv[7]) : disp_w;

    // Parse --zones "x,y,size;x,y,size;..." (up to 4). size optional, default
    // = infer_size. size < infer_size is clamped up; size is clamped to fit
    // inside (disp_w, disp_h). Backwards-compatible with old "x,y" form.
    struct Zone { int x, y, size; };
    std::vector<Zone> zones;
    for (int i = 1; i < argc - 1; ++i) {
        if (strcmp(argv[i], "--zones") == 0) {
            char *s = strdup(argv[i+1]);
            char *tok = strtok(s, ";");
            while (tok && zones.size() < 4) {
                int x = 0, y = 0, sz = infer_size;
                int n = sscanf(tok, "%d,%d,%d", &x, &y, &sz);
                if (n >= 2) {
                    Zone z; z.x = x; z.y = y; z.size = (n >= 3 ? sz : infer_size);
                    zones.push_back(z);
                }
                tok = strtok(NULL, ";");
            }
            free(s);
            break;
        }
    }
    if (zones.empty()) {
        Zone z; z.size = infer_size;
        z.x = (disp_w - z.size) / 2; z.y = (disp_h - z.size) / 2;
        zones.push_back(z);
    }
    // Enforce minimum size only. Zones may extend beyond frame; crop code pads with black.
    for (size_t i = 0; i < zones.size(); i++) {
        Zone &z = zones[i];
        if (z.size < infer_size) z.size = infer_size;
    }
    // Active follower: zone 0 re-centers each frame on previous frame's motion
    // centroid (smoothed 20%/frame). Triggered by --active-detector.
    bool active_follower = false;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--active-detector") == 0) { active_follower = true; break; }
    }
    fprintf(stderr, "[zones] count=%zu active_follower=%d", zones.size(), active_follower);
    for (size_t i = 0; i < zones.size(); i++)
        fprintf(stderr, " (%d,%d,%d)", zones[i].x, zones[i].y, zones[i].size);
    fprintf(stderr, "\n");

    // FPS cap: usleep at end of each frame to hit target_frame_ms. 0 = uncapped.
    int target_fps = 0;
    for (int i = 1; i < argc - 1; ++i) {
        if (strcmp(argv[i], "--fps") == 0) { target_fps = atoi(argv[i+1]); break; }
    }
    long target_frame_us = target_fps > 0 ? (1000000L / target_fps) : 0;
    fprintf(stderr, "[fps] target=%d (frame_us=%ld)\n", target_fps, target_frame_us);

    // Motion threshold: frame-mean brightness diff required to count as motion.
    // Lower = more sensitive. Default 3.0 (outdoor small targets). Passed via
    // --motion-thresh from sidecar (= UI motion_sensitivity / 5.0).
    float motion_thresh = 3.0f;
    for (int i = 1; i < argc - 1; ++i) {
        if (strcmp(argv[i], "--motion-thresh") == 0) { motion_thresh = atof(argv[i+1]); break; }
    }
    fprintf(stderr, "[motion] thresh=%.2f\n", motion_thresh);

    const bool ENABLE_HLS = !env_on("YOLO_DISABLE_HLS");
    const bool ENABLE_MJPEG = !env_on("YOLO_DISABLE_MJPEG");
    const bool ENABLE_NPU = !env_on("YOLO_DISABLE_NPU");
    const bool ENABLE_MOTION = !env_on("YOLO_DISABLE_MOTION");
    const bool FORCE_NPU = env_on("YOLO_FORCE_NPU");
    fprintf(stderr, "[isolation] hls=%d mjpeg=%d npu=%d motion=%d force_npu=%d\n",
            (int)ENABLE_HLS, (int)ENABLE_MJPEG, (int)ENABLE_NPU,
            (int)ENABLE_MOTION, (int)FORCE_NPU);

    cv::setNumThreads(1);  // single-core SoC: parallel_for_ overhead > benefit
    udp_init("127.0.0.1", udp_port);

    MJPEGWriter mjpeg(7777);
    cv::VideoCapture cap;
    g_cap = &cap;
    // CHN0 at display size; CHN1 at model input size for TDL (added in capture_cvi.cpp)
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  disp_w);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, disp_h);
    // Lower the sensor framerate to reduce ISP/VPSS heat. Patched capture_cvi.cpp
    // reads YOLO_CAP_FPS at start_streaming() and writes the GC4653 VTS register
    // directly so the sensor itself runs at the lower fps. Sensor floor ~2.75fps;
    // if user picks 1-2 we clamp sensor at 3 and let target_frame_us sleep do
    // the rest so the user-visible fps still matches the slider.
    if (target_fps > 0 && target_fps <= 30) {
        int sns_fps = target_fps < 3 ? 3 : target_fps;
        char buf[16]; snprintf(buf, sizeof(buf), "%d", sns_fps);
        setenv("YOLO_CAP_FPS", buf, 1);
        cap.set(cv::CAP_PROP_FPS, (double)sns_fps);
    }
    cap.open(0);
    if (!cap.isOpened()) { printf("Failed to open camera.\n"); return -1; }

    YoloModelDetector detector;
    cv::Mat bgr;
    cap >> bgr;
    if (ENABLE_MJPEG) {
        cv::Mat bgr_mjpeg;
        cv::resize(bgr, bgr_mjpeg, cv::Size(640, 360));
        mjpeg.write(bgr_mjpeg);
        mjpeg.start();
    }
    std::thread hd_writer(hd_writer_loop);

    if (ENABLE_NPU && detector.setup_model(argv[1], atoi(argv[2]), thresh, 0.5f) != 1) {
        printf("setup_model failed\n");
        return -2;
    }
    printf("NintiDetect running: model=%s classes=%s thresh=%.2f udp=127.0.0.1:%d mjpeg=:7777\n",
           argv[1], argv[2], thresh, udp_port);

    char buf[512];
    int fps_frames = 0;
    long fps_last = now_ms();

    struct TileDet { int dx1,dy1,dx2,dy2; float score; int cls; };
    std::vector<std::vector<TileDet> > zone_cache(zones.size());
    int active_zone = 0;
    cv::Mat tile_sq;  // pre-allocated, reused each frame
    cv::Mat last_disp; // last regular-path frame, used to keep HLS alive during captureRaw
    cv::Mat disp;
    cv::Mat hls_frame;
    cv::Mat hls_raw;
    cv::Mat small;
    cv::Mat diff;
    cv::Mat gray;
    cv::Mat mask;
    cv::Mat disp_out;
    std::vector<TileDet> all_dets;
    std::vector<bool> suppressed;

    // Inference frame skip: run NPU every INFER_EVERY frames, show cached boxes in between
    const int INFER_EVERY = 2;
    int frame_count = 0;

    // Browser MJPEG preview is only for aiming/config. Keep it low-res and
    // throttled so JPEG encoding does not heat the SoC.
    cv::Mat prev_small;
    // HLS must advertise a frame rate close to the frame cadence we can
    // actually sustain. If this is higher than the real send rate, players
    // fast-forward during playback because the H264 stream timestamps imply
    // more frames per second than the pipeline produced.
    int hls_fps = (target_fps > 0) ? std::min(target_fps, 5) : 5;
    int event_fps = target_fps > 0 ? target_fps : 25;
    // HLS output size: env override (set by sidecar), default to disp size.
    // Sidecar exposes this as a user-facing setting.
    int hls_w = disp_w, hls_h = disp_h;
    if (const char* env_w = getenv("YOLO_HLS_W")) { int v = atoi(env_w); if (v > 0) hls_w = v; }
    if (const char* env_h = getenv("YOLO_HLS_H")) { int v = atoi(env_h); if (v > 0) hls_h = v; }
    long last_mjpeg_ms = 0;
    const long MJPEG_FORCE_MS = ENABLE_HLS ? 2000 : 500;
    const char *HD_RECORD_CONTROL = "/tmp/ninti_hd_record_dir";
    const long HD_RECORD_INTERVAL_MS = 250;
    long last_hd_record_ms = 0;
    int hd_record_idx = 0;
    std::string hd_record_dir;
    H264Recorder h264rec;
    HlsStreamer hlsrec;
    // BGR→NV21 buffer for HLS. VENC needs physically-contiguous ION memory.
    cv::Mat hls_yuv;
    CVI_U64 hls_ion_phy = 0;
    void*   hls_ion_vir = nullptr;
    VIDEO_FRAME_INFO_S hls_frame_info;
    auto prepare_hls_frame = [&](const cv::Mat& bgr) -> VIDEO_FRAME_INFO_S* {
        int w = bgr.cols, h = bgr.rows;
        size_t y_size = (size_t)w * h;
        size_t uv_size = y_size / 2;
        size_t total = y_size + uv_size;
        // Allocate ION buffer on first call (or if size changes).
        if (hls_ion_vir == nullptr) {
            if (CVI_SYS_IonAlloc_Cached(&hls_ion_phy, &hls_ion_vir, "hls_nv21", total) != 0) {
                fprintf(stderr, "[hls] IonAlloc failed\n");
                return nullptr;
            }
            fprintf(stderr, "[hls] ION alloc %zu bytes phy=0x%llx vir=%p\n",
                    total, (unsigned long long)hls_ion_phy, hls_ion_vir);
        }
        uint8_t* buf = (uint8_t*)hls_ion_vir;
        // Direct BGR → NV21: single-pass, no intermediate I420 buffer.
        // NV21 layout: Y plane (w×h) + interleaved V,U plane (w×h/2 bytes).
        long cvt_t0 = now_ms();
        {
            uint8_t* y_out  = buf;
            uint8_t* vu_out = buf + y_size;  // NV21: V then U interleaved
            const int stride = (int)bgr.step[0];
            for (int row = 0; row < h; row++) {
                const uint8_t* src = bgr.data + row * stride;
                uint8_t* y_row = y_out + row * w;
                for (int col = 0; col < w; col++) {
                    int b = src[col*3], g = src[col*3+1], r = src[col*3+2];
                    y_row[col] = (uint8_t)((66*r + 129*g + 25*b + 4224) >> 8);
                }
                if ((row & 1) == 0) {
                    uint8_t* vu_row = vu_out + (row >> 1) * w;
                    for (int col = 0; col < w; col += 2) {
                        int b = src[col*3], g = src[col*3+1], r = src[col*3+2];
                        vu_row[col]   = (uint8_t)((112*r - 94*g - 18*b + 32896) >> 8);  // V
                        vu_row[col+1] = (uint8_t)((-38*r - 74*g + 112*b + 32896) >> 8); // U
                    }
                }
            }
        }
        long cvt_t1 = now_ms();
        long cvt_t2 = cvt_t1;  // no separate mcpy step
        long fl_t0 = now_ms();
        CVI_SYS_IonFlushCache(hls_ion_phy, hls_ion_vir, total);
        long fl_ms = now_ms() - fl_t0;
        static int fl_cnt = 0;
        if (++fl_cnt <= 5 || fl_cnt % 100 == 0)
            fprintf(stderr, "[hls] cvt=%ldms mcpy=%ldms ionflush=%ldms\n",
                    cvt_t1-cvt_t0, cvt_t2-cvt_t1, fl_ms);
        memset(&hls_frame_info, 0, sizeof(hls_frame_info));
        VIDEO_FRAME_S& vf = hls_frame_info.stVFrame;
        vf.u32Width  = w; vf.u32Height = h;
        vf.enPixelFormat  = PIXEL_FORMAT_NV21;
        vf.u32Stride[0]   = w; vf.u32Stride[1] = w;
        vf.u32Length[0]   = y_size; vf.u32Length[1] = uv_size;
        vf.pu8VirAddr[0]  = buf;
        vf.pu8VirAddr[1]  = buf + y_size;
        vf.u64PhyAddr[0]  = hls_ion_phy;
        vf.u64PhyAddr[1]  = hls_ion_phy + y_size;
        // Monotonic PTS in microseconds so the encoder timestamps frames correctly.
        struct timespec ts_pts; clock_gettime(CLOCK_MONOTONIC, &ts_pts);
        hls_frame_info.stVFrame.u64PTS = (uint64_t)ts_pts.tv_sec * 1000000ULL
                                        + ts_pts.tv_nsec / 1000;
        return &hls_frame_info;
    };
    auto read_hd_record_dir = [&]() -> std::string {
        std::string dir;
        std::ifstream ctl(HD_RECORD_CONTROL);
        if (ctl.good()) {
            std::getline(ctl, dir);
        }
        return dir;
    };

    // Last frame's motion centroid in disp coords (-1 = invalid).
    int motion_cx = -1, motion_cy = -1;

    // Motion-gated NPU: only infer while motion present (+ hangover) to keep chip cool
    long last_motion_ms = 0;
    const long NPU_HANGOVER_MS = 5000;

    // Stage timing accumulators (printed once/sec)
    long acc_cap=0, acc_motion=0, acc_infer=0, acc_nms_draw=0, acc_mjpeg=0;
    long acc_clone=0, acc_hls_send=0;

    // Pre-init CHN1 (event recorder) before the main loop so the first event
    // recording never stalls the loop with a 1-2s CVI_VENC_CreateChn call.
    // Capture one raw frame to discover sensor dimensions, then release it.
    int chn1_w = 0, chn1_h = 0;
    {
        VIDEO_FRAME_INFO_S* raw_f = (VIDEO_FRAME_INFO_S*)cap.captureRaw();
        if (raw_f) {
            VIDEO_FRAME_S& vf = raw_f->stVFrame;
            chn1_w = (int)vf.u32Width;
            chn1_h = (int)vf.u32Height;
            cap.releaseImagePtr();  // release before CreateChn (avoids VI stall)
            h264rec.init(chn1_w, chn1_h, event_fps);
        }
    }

    // Start HLS VENC before the main loop so CreateChn never runs while a VI
    // frame is in-flight (which causes VI GetChnFrame to block permanently).
    if (ENABLE_HLS) {
        if (!hlsrec.start(hls_w, hls_h, hls_fps))
            fprintf(stderr, "[hls] start failed — HLS disabled\n");
        else
            fprintf(stderr, "[hls] start %dx%d@%d\n", hls_w, hls_h, hls_fps);
    }

    struct timespec frame_start_ts;
    while (!interrupted) {
        clock_gettime(CLOCK_MONOTONIC, &frame_start_ts);
        long t0 = now_ms();

        // Event recording gets priority over detection and browser preview.
        // Pull raw VI frames and feed hardware VENC directly, avoiding VPSS/BGR,
        // JPEG, motion detection, and NPU work while the clip is being written.
        std::string raw_requested_hd_dir = read_hd_record_dir();
        if (false && (!raw_requested_hd_dir.empty() || !hd_record_dir.empty())) {
            // Keep HLS VENC running (CHN0 idle is fine) — don't stop/restart it
            // since CreateChn during captureRaw causes VI pipeline stalls.
            VIDEO_FRAME_INFO_S* raw_frame = (VIDEO_FRAME_INFO_S*)cap.captureRaw();
            long t1 = now_ms();
            acc_cap += t1 - t0;
            long ts = t1;

            if (raw_requested_hd_dir != hd_record_dir) {
                if (!hd_record_dir.empty()) {
                    hlsrec.pause_venc();
                    h264rec.stop();
                    hlsrec.resume_venc();
                    write_hd_done(hd_record_dir);
                }
                hd_record_dir = raw_requested_hd_dir;
                hd_record_idx = 0;
                last_hd_record_ms = ts - HD_RECORD_INTERVAL_MS;
                if (!hd_record_dir.empty() && raw_frame) {
                    VIDEO_FRAME_S& vf = raw_frame->stVFrame;
                    std::string h264_path = dirname_of(hd_record_dir) + "/event.h264";
                    h264rec.start(h264_path, (int)vf.u32Width, (int)vf.u32Height, event_fps);
                    fprintf(stderr, "[hdrec/raw] start dir=%s h264=%s\n",
                            hd_record_dir.c_str(), h264_path.c_str());
                }
            }
            bool hd_recording = !hd_record_dir.empty();
            if (hd_recording && raw_frame) {
                h264rec.send(raw_frame);
            }
            // send HLS at target_fps using last regular frame (rate-limited to avoid VENC overflow)
            static long hls_raw_last_ms = 0;
            long hls_now = now_ms();
            long hls_interval_ms = 1000L / hls_fps;
            if (ENABLE_HLS && hlsrec.is_active() && !last_disp.empty()
                    && (hls_now - hls_raw_last_ms) >= hls_interval_ms) {
                hls_raw_last_ms = hls_now;
                if (last_disp.cols != hlsrec.get_w() || last_disp.rows != hlsrec.get_h())
                    cv::resize(last_disp, hls_raw, cv::Size(hlsrec.get_w(), hlsrec.get_h()),
                               0, 0, cv::INTER_LINEAR);
                else
                    hls_raw = last_disp;
                VIDEO_FRAME_INFO_S* nv21 = prepare_hls_frame(hls_raw);
                if (nv21) hlsrec.send(nv21);
            }
            cap.releaseImagePtr();

            fps_frames++;
            long now = now_ms();
            if (now - fps_last >= 1000) {
                float fps = fps_frames * 1000.0f / (now - fps_last);
                fprintf(stderr, "[timing/raw] cap=%ld total=%ld ms (fps=%.1f)\n",
                        fps_frames ? acc_cap/fps_frames : 0,
                        fps_frames ? acc_cap/fps_frames : 0, fps);
                acc_cap=acc_motion=acc_infer=acc_nms_draw=acc_mjpeg=0;
                snprintf(buf, sizeof(buf), "{\"_fps\":%.1f}", fps);
                udp_send(buf);
                snprintf(buf, sizeof(buf),
                    "{\"_status\":1,\"fps\":%.1f,\"venc\":%d,\"chn1\":%d,\"mode\":\"raw\"}",
                    fps, (int)hlsrec.is_venc_active(), (int)h264rec.is_active());
                udp_send(buf);
                fps_frames = 0;
                fps_last = now;
            }
            continue;
        }

        cv::Mat frame;
        std::pair<void *, void *> imagePtrs = cap.capture(frame);
        if (imagePtrs.first == nullptr) { interrupted = 1; break; }
        VIDEO_FRAME_INFO_S* original_frame = (VIDEO_FRAME_INFO_S*)cap.getOriginalImagePtr();
        long t1 = now_ms(); acc_cap += t1 - t0;

        // Deep-copy out of DMA buffer immediately; all downstream ops use cached heap memory.
        disp.create(frame.rows, frame.cols, frame.type());
        frame.copyTo(disp);
        last_disp = disp;
        long t1a = now_ms();  // after clone

        long ts = t1;
        std::string requested_hd_dir = read_hd_record_dir();
        if (requested_hd_dir != hd_record_dir) {
            if (!hd_record_dir.empty()) {
                h264rec.stop();
                write_hd_done(hd_record_dir);
            }
            hd_record_dir = requested_hd_dir;
            hd_record_idx = 0;
            last_hd_record_ms = ts - HD_RECORD_INTERVAL_MS;
            if (!hd_record_dir.empty() && original_frame && chn1_w > 0) {
                std::string h264_path = dirname_of(hd_record_dir) + "/event.h264";
                h264rec.start(h264_path, chn1_w, chn1_h, event_fps);
                fprintf(stderr, "[hdrec] start dir=%s h264=%s %dx%d\n",
                        hd_record_dir.c_str(), h264_path.c_str(), chn1_w, chn1_h);
            }
        }
        bool hd_recording = !hd_record_dir.empty();
        // Feed the raw VI frame (sensor resolution, e.g. 2560x1440) to the
        // CHN1 H264 encoder while the event is active. Must run before
        // cap.releaseImagePtr() below, which releases original_frame.
        if (hd_recording && original_frame) {
            h264rec.send(original_frame);
        }
        // Rate-limit HLS to its declared output rate. Previously this branch
        // sent every loop iter, which over-fed VENC and made hlsrec.send()
        // block in drain() for 600-2000ms — dragging the whole loop (and
        // therefore event recording fps) down to ~1.
        static long hls_last_ms = 0;
        long hls_now = now_ms();
        long hls_interval_ms = hls_fps > 0 ? 1000L / hls_fps : 200;
        if (ENABLE_HLS && hlsrec.is_active() && !disp.empty()
                && (hls_now - hls_last_ms) >= hls_interval_ms) {
            hls_last_ms = hls_now;
            long pr_t0 = now_ms();
            // INTER_NEAREST: scalar INTER_LINEAR on c906 ate ~700ms per
            // 1920x1080 -> 1280x720 resize, choking the loop down to ~1 fps.
            // NEAREST is index-math + memcpy and runs in ~50ms; HLS at 5fps
            // is a preview stream, the quality drop is invisible.
            if (disp.cols != hlsrec.get_w() || disp.rows != hlsrec.get_h())
                cv::resize(disp, hls_frame, cv::Size(hlsrec.get_w(), hlsrec.get_h()),
                           0, 0, cv::INTER_NEAREST);
            else
                hls_frame = disp;
            VIDEO_FRAME_INFO_S* nv21 = prepare_hls_frame(hls_frame);
            static int pr_cnt = 0;
            if (++pr_cnt <= 5 || pr_cnt % 100 == 0)
                fprintf(stderr, "[diag] prepare_hls %ldms drain_req=%d\n", now_ms()-pr_t0, (int)hlsrec.is_drain_requested());
            static int hls_call = 0;
            if (++hls_call <= 15 || hls_call % 50 == 0)
                fprintf(stderr, "[hls] call send #%d nv21=%p venc=%d\n",
                        hls_call, (void*)nv21, (int)hlsrec.is_venc_active());
            if (nv21) hlsrec.send(nv21);
        }
        long t1b = now_ms();  // after HLS send
        acc_clone += t1a - t1; acc_hls_send += t1b - t1a;
        cap.releaseImagePtr();

        // Motion detection on raw frame (before box drawing) at 1/4 res
        bool motion = false;
        if (ENABLE_MOTION) {
            cv::resize(disp, small, cv::Size(disp.cols/4, disp.rows/4), 0, 0, cv::INTER_NEAREST);
            motion = prev_small.empty();
            if (!motion) {
                cv::absdiff(small, prev_small, diff);
                cv::Scalar m = cv::mean(diff);
                motion = (m[0] + m[1] + m[2]) / 3.0f > motion_thresh;
                // Centroid of moving pixels for active follower. Threshold diff to a
                // binary mask, take its first-order moments; require enough mass
                // before reporting a centroid (filters out noise).
                if (motion && active_follower) {
                    cv::cvtColor(diff, gray, cv::COLOR_BGR2GRAY);
                    cv::threshold(gray, mask, 20, 255, cv::THRESH_BINARY);
                    cv::Moments mom = cv::moments(mask, true);
                    if (mom.m00 > 200.0) {
                        motion_cx = (int)(mom.m10 / mom.m00) * 4;  // scale 1/4 -> disp
                        motion_cy = (int)(mom.m01 / mom.m00) * 4;
                    }
                }
            }
            small.copyTo(prev_small);
        }
        if (motion) last_motion_ms = ts;
        // Active follower: slide zone 0 toward previous frame's motion centroid.
        // 50% step per frame (snappy on low fps; still smooth on high fps).
        // Clamped to frame bounds.
        if (active_follower && motion_cx >= 0 && !zones.empty()) {
            Zone &z0 = zones[0];
            int target_x = motion_cx - z0.size / 2;
            int target_y = motion_cy - z0.size / 2;
            z0.x += (target_x - z0.x) / 2;
            z0.y += (target_y - z0.y) / 2;
            if (z0.x < 0) z0.x = 0;
            if (z0.y < 0) z0.y = 0;
            if (z0.x > disp_w - z0.size) z0.x = disp_w - z0.size;
            if (z0.y > disp_h - z0.size) z0.y = disp_h - z0.size;
        }
        bool npu_active = FORCE_NPU || ((ts - last_motion_ms) < NPU_HANGOVER_MS);
        long t2 = now_ms(); acc_motion += t2 - t1;

        // Clear stale boxes once we go idle so UDP stops emitting old detections
        if (!npu_active) {
            for (size_t i = 0; i < zone_cache.size(); i++) zone_cache[i].clear();
        }

        // Inference frame skip: only run NPU every INFER_EVERY frames, and only when motion active
        if (ENABLE_NPU && !hd_recording && npu_active && frame_count % INFER_EVERY == 0) {
            int z = active_zone;
            const Zone &zone = zones[z];
            // Padded crop: zone may extend beyond frame — out-of-bounds area filled black.
            // Scale visible portion directly into infer_size tile to avoid large intermediate buffer.
            tile_sq.create(infer_size, infer_size, disp.type());
            tile_sq.setTo(cv::Scalar(0, 0, 0));
            int sx = std::max(0, zone.x), sy = std::max(0, zone.y);
            int ex = std::min(disp_w, zone.x + zone.size);
            int ey = std::min(disp_h, zone.y + zone.size);
            if (ex > sx && ey > sy) {
                float sc2 = (float)infer_size / zone.size;
                int dx = (int)((sx - zone.x) * sc2);
                int dy = (int)((sy - zone.y) * sc2);
                int dw = std::min((int)((ex - sx) * sc2), infer_size - dx);
                int dh = std::min((int)((ey - sy) * sc2), infer_size - dy);
                if (dw > 0 && dh > 0) {
                    int interp = (zone.size == infer_size) ? cv::INTER_NEAREST : cv::INTER_AREA;
                    cv::resize(disp(cv::Rect(sx, sy, ex-sx, ey-sy)),
                               tile_sq(cv::Rect(dx, dy, dw, dh)),
                               cv::Size(dw, dh), 0, 0, interp);
                }
            }

            cvtdl_object_t objs = detector.detect(tile_sq);
            zone_cache[z].clear();
            const float sc = (float)zone.size / (float)infer_size;
            for (uint32_t i = 0; i < objs.size; i++) {
                if (!category(objs.info[i].classes)) continue;
                TileDet td;
                td.dx1 = (int)(objs.info[i].bbox.x1 * sc) + zone.x;
                td.dy1 = (int)(objs.info[i].bbox.y1 * sc) + zone.y;
                td.dx2 = (int)(objs.info[i].bbox.x2 * sc) + zone.x;
                td.dy2 = (int)(objs.info[i].bbox.y2 * sc) + zone.y;
                td.score = objs.info[i].bbox.score;
                td.cls   = objs.info[i].classes;
                zone_cache[z].push_back(td);
            }
            detector.free_objects(&objs);
            active_zone = (active_zone + 1) % (int)zones.size();
        }
        frame_count++;
        long t3 = now_ms(); acc_infer += t3 - t2;

        // Merge all zone caches
        all_dets.clear();
        for (size_t i = 0; i < zone_cache.size(); i++)
            all_dets.insert(all_dets.end(), zone_cache[i].begin(), zone_cache[i].end());

        // Cross-tile NMS: suppress lower-score duplicate that overlaps >40% IoU
        suppressed.assign(all_dets.size(), false);
        for (size_t i = 0; i < all_dets.size(); i++) {
            if (suppressed[i]) continue;
            for (size_t j = i + 1; j < all_dets.size(); j++) {
                if (suppressed[j] || all_dets[j].cls != all_dets[i].cls) continue;
                float ix1 = std::max(all_dets[i].dx1, all_dets[j].dx1);
                float iy1 = std::max(all_dets[i].dy1, all_dets[j].dy1);
                float ix2 = std::min(all_dets[i].dx2, all_dets[j].dx2);
                float iy2 = std::min(all_dets[i].dy2, all_dets[j].dy2);
                float inter = std::max(0.f,ix2-ix1) * std::max(0.f,iy2-iy1);
                if (inter < 1.f) continue;
                float ai = (float)(all_dets[i].dx2-all_dets[i].dx1)*(all_dets[i].dy2-all_dets[i].dy1);
                float aj = (float)(all_dets[j].dx2-all_dets[j].dx1)*(all_dets[j].dy2-all_dets[j].dy1);
                if (inter / (ai + aj - inter + 1e-5f) > 0.4f)
                    suppressed[all_dets[i].score >= all_dets[j].score ? j : i] = true;
            }
        }

        float fw = (float)disp.cols, fh = (float)disp.rows;
        for (size_t i = 0; i < all_dets.size(); i++) {
            if (suppressed[i]) continue;
            TileDet &td = all_dets[i];
            const char *cat  = category(td.cls);
            const char *name = (td.cls >= 0 && td.cls < 80) ? COCO[td.cls] : "obj";
            float cx = (td.dx1 + td.dx2) / 2.0f / fw;
            float cy = (td.dy1 + td.dy2) / 2.0f / fh;
            snprintf(buf, sizeof(buf),
                "{\"cat\":\"%s\",\"cls\":%d,\"name\":\"%s\",\"score\":%.3f,"
                "\"cx\":%.4f,\"cy\":%.4f,\"x1\":%.1f,\"y1\":%.1f,\"x2\":%.1f,\"y2\":%.1f,"
                "\"fw\":%.0f,\"fh\":%.0f,\"t\":%ld}",
                cat, td.cls, name, td.score, cx, cy,
                (float)td.dx1,(float)td.dy1,(float)td.dx2,(float)td.dy2, fw, fh, ts);
            udp_send(buf);
            char label[160];
            snprintf(label, sizeof(label), "%s %s %.2f", cat, name, td.score);
            cv::rectangle(disp, cv::Rect(td.dx1, td.dy1, td.dx2-td.dx1, td.dy2-td.dy1),
                          cv::Scalar(0,255,0), 2, 8, 0);
            cv::putText(disp, label, cv::Point(td.dx1, td.dy1>15?td.dy1-5:td.dy1+15),
                        cv::FONT_HERSHEY_DUPLEX, 0.6, cv::Scalar(0,255,0), 1);
        }
        long t4 = now_ms(); acc_nms_draw += t4 - t3;

        if (false && !hd_record_dir.empty() && ts - last_hd_record_ms >= HD_RECORD_INTERVAL_MS) {
            char hd_path[512];
            snprintf(hd_path, sizeof(hd_path), "%s/frame_%05d.jpg",
                     hd_record_dir.c_str(), hd_record_idx++);
            hd_enqueue_frame(hd_path, disp);
            last_hd_record_ms += HD_RECORD_INTERVAL_MS;
        }

        // MJPEG is secondary. While HD event recording is active, give disk
        // capture priority and let browser preview pause instead of slowing it.
        if (ENABLE_MJPEG && hd_record_dir.empty() && mjpeg.clientCount() > 0 && ts - last_mjpeg_ms >= MJPEG_FORCE_MS) {
            if (disp.cols != 640 || disp.rows != 360) {
                cv::resize(disp, disp_out, cv::Size(640, 360));
                mjpeg.write(disp_out);
            } else {
                mjpeg.write(disp);
            }
            last_mjpeg_ms = ts;
        }
        long t5 = now_ms(); acc_mjpeg += t5 - t4;

        fps_frames++;
        long now = t5;
        if (now - fps_last >= 1000) {
            float fps = fps_frames * 1000.0f / (now - fps_last);
            fprintf(stderr, "[timing/f] cap=%ld clone=%ld hls_send=%ld motion=%ld infer=%ld nms+draw=%ld mjpeg=%ld total=%ld ms (fps=%.1f)\n",
                    acc_cap/fps_frames, acc_clone/fps_frames, acc_hls_send/fps_frames,
                    acc_motion/fps_frames, acc_infer/fps_frames,
                    acc_nms_draw/fps_frames, acc_mjpeg/fps_frames,
                    (acc_cap+acc_motion+acc_infer+acc_nms_draw+acc_mjpeg)/fps_frames, fps);
            acc_cap=acc_motion=acc_infer=acc_nms_draw=acc_mjpeg=0;
            acc_clone=acc_hls_send=0;
            snprintf(buf, sizeof(buf), "{\"_fps\":%.1f}", fps);
            udp_send(buf);
            snprintf(buf, sizeof(buf),
                "{\"_status\":1,\"fps\":%.1f,\"venc\":%d,\"chn1\":%d,\"npu\":%d,\"mode\":\"normal\"}",
                fps, (int)hlsrec.is_venc_active(), (int)h264rec.is_active(), (int)npu_active);
            udp_send(buf);
            fps_frames = 0;
            fps_last = now;
            // Active follower: report current zone 0 position so the UI overlay
            // can render the moving box (sidecar relays via SSE).
            if (active_follower && !zones.empty()) {
                snprintf(buf, sizeof(buf), "{\"_active_zone\":[%d,%d,%d]}",
                         zones[0].x, zones[0].y, zones[0].size);
                udp_send(buf);
            }
        }

        // FPS cap: sleep remainder of target frame budget
        if (target_frame_us > 0) {
            struct timespec end_ts;
            clock_gettime(CLOCK_MONOTONIC, &end_ts);
            long used_us = (end_ts.tv_sec - frame_start_ts.tv_sec) * 1000000L
                         + (end_ts.tv_nsec - frame_start_ts.tv_nsec) / 1000L;
            long sleep_us = target_frame_us - used_us;
            if (sleep_us > 0) usleep((useconds_t)sleep_us);
        }
    }

    shutdown_stage("main loop exited");
    printf("Stopping...\n");
    shutdown_stage("before hlsrec.stop");
    hlsrec.stop();
    shutdown_stage("after hlsrec.stop");
    if (hls_ion_vir) {
        shutdown_stage("before hls ion free");
        CVI_SYS_IonFree(hls_ion_phy, hls_ion_vir);
        hls_ion_vir = nullptr;
        shutdown_stage("after hls ion free");
    }
    if (!hd_record_dir.empty()) {
        shutdown_stage("before h264rec.stop event");
        h264rec.stop();
        shutdown_stage("after h264rec.stop event");
        shutdown_stage("before write hd done");
        write_hd_done(hd_record_dir);
        shutdown_stage("after write hd done");
    }
    shutdown_stage("before h264rec.destroy");
    h264rec.destroy();
    shutdown_stage("after h264rec.destroy");
    shutdown_stage("before hd writer stop");
    {
        std::lock_guard<std::mutex> lock(hd_queue_mutex);
        hd_writer_stop = true;
    }
    hd_queue_cv.notify_one();
    shutdown_stage("before hd writer join");
    if (hd_writer.joinable()) hd_writer.join();
    shutdown_stage("after hd writer join");
    shutdown_stage("before cap.release");
    cap.release();
    shutdown_stage("after cap.release");
    shutdown_stage("before detector.release");
    detector.release();
    shutdown_stage("after detector.release");
    shutdown_stage("before udp close");
    if (udp_fd >= 0) close(udp_fd);
    shutdown_stage("after udp close");
    return 0;
}

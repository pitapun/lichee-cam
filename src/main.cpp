// NintiDetect — LicheeRV Nano on-board YOLO detector.
// Base: ret7020 YoloCamera. Adapted: filter person/vehicle/animal (COCO),
// emit each kept detection as a JSON line over UDP (for the Python sidecar
// that does zones + MQTT + Home Assistant), and stream annotated MJPEG :7777.
#include "MJPEGWriter.h"
#include "yolo.hpp"
#include "cvi_venc.h"
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
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

volatile uint8_t interrupted = 0;
void interrupt_handler(int signum) { interrupted = 1; }

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

class HlsStreamer {
public:
    HlsStreamer() : active(false), chn(2), running(false) {}

    bool start(int w, int h, int fps) {
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
        attr.stRcAttr.stH264Cbr.u32BitRate = (w >= 1920) ? 3000 : 1500;
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
        active = true;
        running = true;
        drain_th = std::thread(&HlsStreamer::drain_loop, this);
        fprintf(stderr, "[hls] streamer started %dx%d@%d\n", w, h, fps);
        return true;
    }

    void send(VIDEO_FRAME_INFO_S* frame) {
        if (!active || !frame) return;
        CVI_S32 ret = CVI_VENC_SendFrame(chn, frame, 100);
        if (ret != CVI_SUCCESS)
            fprintf(stderr, "[hls] SendFrame failed %#x\n", ret);
    }

    void stop() {
        if (!active && !running) return;
        fprintf(stderr, "[hls] stopping streamer\n");
        running = false;
        active = false;
        CVI_VENC_StopRecvFrame(chn);
        if (drain_th.joinable()) drain_th.join();
        CVI_VENC_DestroyChn(chn);
        fprintf(stderr, "[hls] streamer stopped\n");
    }

    bool is_active() const { return active; }

private:
    void drain_loop() {
        static const char* FIFO = "/tmp/hls_feed.h264";
        // Open O_RDWR so the FIFO is always "alive" (never sends EOF to readers
        // when ffmpeg dies and reconnects). O_NONBLOCK avoids blocking on open.
        // Then remove O_NONBLOCK so subsequent writes are blocking (back-pressure).
        int fd = -1;
        while (running && fd < 0) {
            fd = open(FIFO, O_RDWR | O_NONBLOCK);
            if (fd < 0) { usleep(200000); }
        }
        if (fd < 0) return;
        // Switch to blocking writes; increase pipe buffer for ~2s buffering.
        int fl = fcntl(fd, F_GETFL);
        fcntl(fd, F_SETFL, fl & ~O_NONBLOCK);
        fcntl(fd, F_SETPIPE_SZ, 1024 * 1024);
        fprintf(stderr, "[hls] fifo open (rdwr, blocking)\n");

        // Flush stale ring-buffer packs accumulated before ffmpeg connected.
        // Then force an IDR so ffmpeg sees a clean SPS+PPS+IDR at the very
        // start of its read — without this the stream has no parameter sets.
        for (int i = 0; i < 200; i++) {
            VENC_CHN_STATUS_S s; memset(&s, 0, sizeof(s));
            if (CVI_VENC_QueryStatus(chn, &s) != CVI_SUCCESS || s.u32CurPacks == 0) break;
            VENC_STREAM_S st; memset(&st, 0, sizeof(st));
            st.u32PackCount = s.u32CurPacks;
            st.pstPack = (VENC_PACK_S*)calloc(s.u32CurPacks, sizeof(VENC_PACK_S));
            if (!st.pstPack) break;
            CVI_VENC_GetStream(chn, &st, 100);
            CVI_VENC_ReleaseStream(chn, &st);
            free(st.pstPack);
        }
        CVI_VENC_RequestIDR(chn, CVI_TRUE);
        fprintf(stderr, "[hls] flushed stale packs, IDR requested\n");

        // CVI VENC emits SPS+PPS only on the first IDR (not on subsequent ones).
        // Store them here and prepend before every IDR so ffmpeg can sync after
        // any reconnect within at most one GOP period.
        std::vector<uint8_t> sps_nalu, pps_nalu;

        while (running) {
            VENC_CHN_STATUS_S status;
            memset(&status, 0, sizeof(status));
            CVI_S32 qs = CVI_VENC_QueryStatus(chn, &status);
            if (qs != CVI_SUCCESS) { usleep(50000); continue; }
            if (status.u32CurPacks == 0) { usleep(10000); continue; }
            VENC_STREAM_S stream;
            memset(&stream, 0, sizeof(stream));
            stream.u32PackCount = status.u32CurPacks;
            stream.pstPack = (VENC_PACK_S*)calloc(status.u32CurPacks, sizeof(VENC_PACK_S));
            if (!stream.pstPack) { usleep(10000); continue; }
            if (CVI_VENC_GetStream(chn, &stream, 1000) == CVI_SUCCESS) {
                for (CVI_U32 i = 0; i < stream.u32PackCount; i++) {
                    VENC_PACK_S* p = &stream.pstPack[i];
                    uint8_t* d = p->pu8Addr + p->u32Offset;
                    ssize_t sz = (ssize_t)(p->u32Len - p->u32Offset);
                    uint8_t nalu = (sz >= 5) ? (d[4] & 0x1f) : 0;
                    if (nalu == 7) {  // SPS
                        sps_nalu.assign(d, d + sz);
                    } else if (nalu == 8) {  // PPS
                        pps_nalu.assign(d, d + sz);
                    } else if (nalu == 5 && !sps_nalu.empty() && !pps_nalu.empty()) {
                        // IDR: prepend stored SPS+PPS so every IDR boundary is self-contained
                        write(fd, sps_nalu.data(), sps_nalu.size());
                        write(fd, pps_nalu.data(), pps_nalu.size());
                    }
                    write(fd, d, sz);
                }
                CVI_VENC_ReleaseStream(chn, &stream);
            }
            free(stream.pstPack);
        }
        close(fd);
    }

    bool active, running;
    VENC_CHN chn;
    std::thread drain_th;
};

class H264Recorder {
public:
    H264Recorder() : active(false), chn(1), fp(nullptr), frames(0) {}

    bool start(const std::string& path, int w, int h, int fps) {
        stop();
        FILE* f = fopen(path.c_str(), "wb");
        if (!f) {
            fprintf(stderr, "[venc] fopen failed %s\n", path.c_str());
            return false;
        }

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

        CVI_S32 ret = CVI_VENC_CreateChn(chn, &attr);
        if (ret != CVI_SUCCESS) {
            fprintf(stderr, "[venc] CreateChn failed %#x\n", ret);
            fclose(f);
            return false;
        }
        VENC_RECV_PIC_PARAM_S recv;
        memset(&recv, 0, sizeof(recv));
        recv.s32RecvPicNum = -1;
        ret = CVI_VENC_StartRecvFrame(chn, &recv);
        if (ret != CVI_SUCCESS) {
            fprintf(stderr, "[venc] StartRecvFrame failed %#x\n", ret);
            CVI_VENC_DestroyChn(chn);
            fclose(f);
            return false;
        }
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
        if (ret == CVI_SUCCESS) {
            frames++;
            drain(1000);
        } else {
            fprintf(stderr, "[venc] SendFrame failed %#x\n", ret);
            drain(200);
        }
    }

    void stop() {
        if (!active) return;
        for (int i = 0; i < 20; i++) {
            drain(100);
        }
        CVI_VENC_StopRecvFrame(chn);
        CVI_VENC_DestroyChn(chn);
        if (fp) {
            fflush(fp);
            fclose(fp);
            fp = nullptr;
        }
        fprintf(stderr, "[venc] stop frames=%d path=%s\n", frames, out_path.c_str());
        active = false;
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
    // Clamp size + position to frame bounds.
    for (size_t i = 0; i < zones.size(); i++) {
        Zone &z = zones[i];
        if (z.size < infer_size) z.size = infer_size;
        if (z.size > disp_w) z.size = disp_w;
        if (z.size > disp_h) z.size = disp_h;
        if (z.x < 0) z.x = 0;
        if (z.y < 0) z.y = 0;
        if (z.x > disp_w - z.size) z.x = disp_w - z.size;
        if (z.y > disp_h - z.size) z.y = disp_h - z.size;
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
    cv::Mat bgr_mjpeg;
    cv::resize(bgr, bgr_mjpeg, cv::Size(640, 360));
    mjpeg.write(bgr_mjpeg);
    mjpeg.start();
    std::thread hd_writer(hd_writer_loop);

    if (detector.setup_model(argv[1], atoi(argv[2]), thresh, 0.5f) != 1) {
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

    // Inference frame skip: run NPU every INFER_EVERY frames, show cached boxes in between
    const int INFER_EVERY = 2;
    int frame_count = 0;

    // Browser MJPEG preview is only for aiming/config. Keep it low-res and
    // throttled so JPEG encoding does not heat the SoC.
    cv::Mat prev_small;
    long last_mjpeg_ms = 0;
    const long MJPEG_FORCE_MS = 500;
    const char *HD_RECORD_CONTROL = "/tmp/ninti_hd_record_dir";
    const long HD_RECORD_INTERVAL_MS = 250;
    long last_hd_record_ms = 0;
    int hd_record_idx = 0;
    std::string hd_record_dir;
    H264Recorder h264rec;
    HlsStreamer hlsrec;
    const bool ENABLE_HLS = false;  // Temporarily disabled: MJPEG is the default setup view.
    int hls_fps = (target_fps > 0) ? target_fps : 25;
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

    struct timespec frame_start_ts;
    while (!interrupted) {
        clock_gettime(CLOCK_MONOTONIC, &frame_start_ts);
        long t0 = now_ms();

        // Event recording gets priority over detection and browser preview.
        // Pull raw VI frames and feed hardware VENC directly, avoiding VPSS/BGR,
        // JPEG, motion detection, and NPU work while the clip is being written.
        std::string raw_requested_hd_dir = read_hd_record_dir();
        if (!raw_requested_hd_dir.empty() || !hd_record_dir.empty()) {
            if (hlsrec.is_active()) {
                hlsrec.stop();
                usleep(300000);
            }
            VIDEO_FRAME_INFO_S* raw_frame = (VIDEO_FRAME_INFO_S*)cap.captureRaw();
            long t1 = now_ms();
            acc_cap += t1 - t0;
            long ts = t1;

            if (raw_requested_hd_dir != hd_record_dir) {
                if (!hd_record_dir.empty()) {
                    h264rec.stop();
                    write_hd_done(hd_record_dir);
                }
                hd_record_dir = raw_requested_hd_dir;
                hd_record_idx = 0;
                last_hd_record_ms = ts - HD_RECORD_INTERVAL_MS;
                if (!hd_record_dir.empty() && raw_frame) {
                    VIDEO_FRAME_S& vf = raw_frame->stVFrame;
                    std::string h264_path = dirname_of(hd_record_dir) + "/event.h264";
                    h264rec.start(h264_path, (int)vf.u32Width, (int)vf.u32Height, 25);
                    fprintf(stderr, "[hdrec/raw] start dir=%s h264=%s\n",
                            hd_record_dir.c_str(), h264_path.c_str());
                }
            }
            bool hd_recording = !hd_record_dir.empty();
            if (hd_recording && raw_frame) {
                h264rec.send(raw_frame);
            }
            // skip hlsrec during captureRaw: raw_frame is full-res, HLS VENC is 720p
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

        cv::Mat disp = frame;  // rotate done in VPSS (bMirror+bFlip)

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
            if (!hd_record_dir.empty() && original_frame) {
                VIDEO_FRAME_S& vf = original_frame->stVFrame;
                std::string h264_path = dirname_of(hd_record_dir) + "/event.h264";
                h264rec.start(h264_path, (int)vf.u32Width, (int)vf.u32Height, 25);
                fprintf(stderr, "[hdrec] start dir=%s h264=%s\n", hd_record_dir.c_str(), h264_path.c_str());
            }
        }
        bool hd_recording = !hd_record_dir.empty();
        if (hd_recording && original_frame) {
            h264rec.send(original_frame);
        }
        if (ENABLE_HLS && original_frame) {
            if (!hlsrec.is_active()) {
                VIDEO_FRAME_S& vf = original_frame->stVFrame;
                hlsrec.start((int)vf.u32Width, (int)vf.u32Height, hls_fps);
            }
            hlsrec.send(original_frame);
        }
        cap.releaseImagePtr();

        // Motion detection on raw frame (before box drawing) at 1/4 res
        cv::Mat small;
        cv::resize(disp, small, cv::Size(disp.cols/4, disp.rows/4), 0, 0, cv::INTER_NEAREST);
        bool motion = prev_small.empty();
        if (!motion) {
            cv::Mat diff;
            cv::absdiff(small, prev_small, diff);
            cv::Scalar m = cv::mean(diff);
            motion = (m[0] + m[1] + m[2]) / 3.0f > motion_thresh;
            // Centroid of moving pixels for active follower. Threshold diff to a
            // binary mask, take its first-order moments; require enough mass
            // before reporting a centroid (filters out noise).
            if (motion && active_follower) {
                cv::Mat gray, mask;
                cv::cvtColor(diff, gray, cv::COLOR_BGR2GRAY);
                cv::threshold(gray, mask, 20, 255, cv::THRESH_BINARY);
                cv::Moments mom = cv::moments(mask, true);
                if (mom.m00 > 200.0) {
                    motion_cx = (int)(mom.m10 / mom.m00) * 4;  // scale 1/4 → disp
                    motion_cy = (int)(mom.m01 / mom.m00) * 4;
                }
            }
        }
        small.copyTo(prev_small);
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
        bool npu_active = (ts - last_motion_ms) < NPU_HANGOVER_MS;
        long t2 = now_ms(); acc_motion += t2 - t1;

        // Clear stale boxes once we go idle so UDP stops emitting old detections
        if (!npu_active) {
            for (size_t i = 0; i < zone_cache.size(); i++) zone_cache[i].clear();
        }

        // Inference frame skip: only run NPU every INFER_EVERY frames, and only when motion active
        if (!hd_recording && npu_active && frame_count % INFER_EVERY == 0) {
            int z = active_zone;
            const Zone &zone = zones[z];
            cv::Mat region = disp(cv::Rect(zone.x, zone.y, zone.size, zone.size));
            // Resize zone crop down to infer_size. When zone.size == infer_size
            // this is an identity-copy (no-op); INTER_AREA gives best quality
            // for the >infer_size downscale case.
            int interp = (zone.size == infer_size) ? cv::INTER_NEAREST : cv::INTER_AREA;
            cv::resize(region, tile_sq, cv::Size(infer_size, infer_size), 0, 0, interp);

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
        std::vector<TileDet> all_dets;
        for (size_t i = 0; i < zone_cache.size(); i++)
            all_dets.insert(all_dets.end(), zone_cache[i].begin(), zone_cache[i].end());

        // Cross-tile NMS: suppress lower-score duplicate that overlaps >40% IoU
        std::vector<bool> suppressed(all_dets.size(), false);
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
        if (hd_record_dir.empty() && mjpeg.clientCount() > 0 && ts - last_mjpeg_ms >= MJPEG_FORCE_MS) {
            cv::Mat disp_out = disp;
            if (disp.cols != 640 || disp.rows != 360) {
                cv::resize(disp, disp_out, cv::Size(640, 360));
            }
            mjpeg.write(disp_out);
            last_mjpeg_ms = ts;
        }
        long t5 = now_ms(); acc_mjpeg += t5 - t4;

        fps_frames++;
        long now = t5;
        if (now - fps_last >= 1000) {
            float fps = fps_frames * 1000.0f / (now - fps_last);
            fprintf(stderr, "[timing/f] cap=%ld motion=%ld infer=%ld nms+draw=%ld mjpeg=%ld total=%ld ms (fps=%.1f)\n",
                    acc_cap/fps_frames, acc_motion/fps_frames, acc_infer/fps_frames,
                    acc_nms_draw/fps_frames, acc_mjpeg/fps_frames,
                    (acc_cap+acc_motion+acc_infer+acc_nms_draw+acc_mjpeg)/fps_frames, fps);
            acc_cap=acc_motion=acc_infer=acc_nms_draw=acc_mjpeg=0;
            snprintf(buf, sizeof(buf), "{\"_fps\":%.1f}", fps);
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
        if (target_frame_us > 0 && !hd_recording) {
            struct timespec end_ts;
            clock_gettime(CLOCK_MONOTONIC, &end_ts);
            long used_us = (end_ts.tv_sec - frame_start_ts.tv_sec) * 1000000L
                         + (end_ts.tv_nsec - frame_start_ts.tv_nsec) / 1000L;
            long sleep_us = target_frame_us - used_us;
            if (sleep_us > 0) usleep((useconds_t)sleep_us);
        }
    }

    printf("Stopping...\n");
    hlsrec.stop();
    if (!hd_record_dir.empty()) {
        h264rec.stop();
        write_hd_done(hd_record_dir);
    }
    {
        std::lock_guard<std::mutex> lock(hd_queue_mutex);
        hd_writer_stop = true;
    }
    hd_queue_cv.notify_one();
    if (hd_writer.joinable()) hd_writer.join();
    cap.release();
    detector.release();
    if (udp_fd >= 0) close(udp_fd);
    return 0;
}

// NintiDetect — LicheeRV Nano on-board YOLO detector.
// Base: ret7020 YoloCamera. Adapted: filter person/vehicle/animal (COCO),
// emit each kept detection as a JSON line over UDP (for the Python sidecar
// that does zones + MQTT + Home Assistant), and stream annotated MJPEG :7777.
#include "MJPEGWriter.h"
#include "yolo.hpp"
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
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

int main(int argc, char *argv[]) {
    setlinebuf(stdout);
    setlinebuf(stderr);
    signal(SIGINT,  interrupt_handler);
    signal(SIGTERM, interrupt_handler);
    signal(SIGSEGV, crash_handler);
    signal(SIGABRT, crash_handler);

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
    udp_init("127.0.0.1", udp_port);

    MJPEGWriter mjpeg(7777);
    cv::VideoCapture cap;
    g_cap = &cap;
    // CHN0 at display size; CHN1 at model input size for TDL (added in capture_cvi.cpp)
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  disp_w);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, disp_h);
    cap.open(0);
    if (!cap.isOpened()) { printf("Failed to open camera.\n"); return -1; }

    YoloModelDetector detector;
    cv::Mat bgr;
    cap >> bgr;
    mjpeg.write(bgr);
    mjpeg.start();

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
    std::vector<TileDet> tile_cache[2]; // cached dets per tile (interleaved)
    int active_tile = 0;
    cv::Mat tile_sq;  // pre-allocated, reused each frame

    while (!interrupted) {
        cv::Mat frame;
        std::pair<void *, void *> imagePtrs = cap.capture(frame);
        if (imagePtrs.first == nullptr) { interrupted = 1; break; }
        cap.releaseImagePtr();  // CHN1 not used; release immediately

        cv::Mat disp = frame;
        cv::rotate(disp, disp, cv::ROTATE_180);

        long ts = now_ms();
        const int tile_w = disp.cols / 2;
        const float ty_scale = (float)disp.rows / infer_size;

        // Interleaved: infer one tile per frame, use cache for the other
        {
            int t = active_tile;
            cv::Mat tile_region = disp(cv::Rect(t * tile_w, 0, tile_w, disp.rows));
            cv::resize(tile_region, tile_sq, cv::Size(infer_size, infer_size),
                       0, 0, cv::INTER_NEAREST);

            cvtdl_object_t objs = detector.detect(tile_sq);
            tile_cache[t].clear();
            for (uint32_t i = 0; i < objs.size; i++) {
                if (!category(objs.info[i].classes)) continue;
                TileDet td;
                td.dx1 = (int)(objs.info[i].bbox.x1)           + t * tile_w;
                td.dy1 = (int)(objs.info[i].bbox.y1 * ty_scale);
                td.dx2 = (int)(objs.info[i].bbox.x2)           + t * tile_w;
                td.dy2 = (int)(objs.info[i].bbox.y2 * ty_scale);
                td.score = objs.info[i].bbox.score;
                td.cls   = objs.info[i].classes;
                tile_cache[t].push_back(td);
            }
            detector.free_objects(&objs);
            active_tile = 1 - active_tile;
        }

        // Merge both tile caches
        std::vector<TileDet> all_dets;
        all_dets.insert(all_dets.end(), tile_cache[0].begin(), tile_cache[0].end());
        all_dets.insert(all_dets.end(), tile_cache[1].begin(), tile_cache[1].end());

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

        // Scale down for MJPEG: boxes drawn at 1280x720, stream at 640x360
        cv::Mat disp_out;
        cv::resize(disp, disp_out, cv::Size(disp.cols/2, disp.rows/2));
        mjpeg.write(disp_out);

        fps_frames++;
        long now = now_ms();
        if (now - fps_last >= 1000) {
            float fps = fps_frames * 1000.0f / (now - fps_last);
            snprintf(buf, sizeof(buf), "{\"_fps\":%.1f}", fps);
            udp_send(buf);
            fps_frames = 0;
            fps_last = now;
        }
    }

    printf("Stopping...\n");
    cap.release();
    detector.release();
    if (udp_fd >= 0) close(udp_fd);
    return 0;
}

#include "yolo.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#include "cvi_sys.h"

struct RuntimeDet {
    float x1;
    float y1;
    float x2;
    float y2;
    float score;
    int cls;
};

static const char *COCO_CLASSES[80] = {
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis",
    "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
    "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife",
    "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot",
    "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed",
    "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard",
    "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book",
    "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"
};

static float clampf(float v, float lo, float hi) {
    return std::max(lo, std::min(v, hi));
}

static float box_iou(const RuntimeDet &a, const RuntimeDet &b) {
    float xx1 = std::max(a.x1, b.x1);
    float yy1 = std::max(a.y1, b.y1);
    float xx2 = std::min(a.x2, b.x2);
    float yy2 = std::min(a.y2, b.y2);
    float w = std::max(0.0f, xx2 - xx1);
    float h = std::max(0.0f, yy2 - yy1);
    float inter = w * h;
    float area_a = std::max(0.0f, a.x2 - a.x1) * std::max(0.0f, a.y2 - a.y1);
    float area_b = std::max(0.0f, b.x2 - b.x1) * std::max(0.0f, b.y2 - b.y1);
    return inter / (area_a + area_b - inter + 1e-9f);
}

static std::vector<RuntimeDet> nms(std::vector<RuntimeDet> dets, float iou_thr) {
    std::sort(dets.begin(), dets.end(), [](const RuntimeDet &a, const RuntimeDet &b) {
        return a.score > b.score;
    });
    std::vector<RuntimeDet> out;
    std::vector<char> removed(dets.size(), 0);
    for (size_t i = 0; i < dets.size(); ++i) {
        if (removed[i]) continue;
        out.push_back(dets[i]);
        for (size_t j = i + 1; j < dets.size(); ++j) {
            if (!removed[j] && box_iou(dets[i], dets[j]) > iou_thr) {
                removed[j] = 1;
            }
        }
    }
    return out;
}

YoloModelDetector::YoloModelDetector() {}

int YoloModelDetector::setup_model(char *model_path, int model_class_cnt, float model_thresh,
                                   float model_nms_thresh, float model_scale, float model_mean) {
    (void)model_scale;
    (void)model_mean;

    class_count = model_class_cnt;
    threshold = model_thresh;
    nms_threshold = model_nms_thresh;

    CVI_RC ret = CVI_NN_RegisterModel(model_path, &model_handle);
    if (ret != CVI_RC_SUCCESS) {
        printf("CVI_NN_RegisterModel failed: 0x%x\n", ret);
        return -1;
    }
    CVI_NN_SetConfig(model_handle, OPTION_PROGRAM_INDEX, 0);

    ret = CVI_NN_GetInputOutputTensors(model_handle, &inputs, &input_num, &outputs, &output_num);
    if (ret != CVI_RC_SUCCESS || input_num < 1 || output_num < 1) {
        printf("CVI_NN_GetInputOutputTensors failed: 0x%x inputs=%d outputs=%d\n",
               ret, input_num, output_num);
        return -2;
    }

    size_t input_count = CVI_NN_TensorCount(&inputs[0]);
    input_size = (int)std::sqrt((double)input_count / 3.0);
    if (input_size <= 0 || input_count != (size_t)(3 * input_size * input_size)) {
        printf("Bad input tensor: count=%zu\n", input_count);
        return -3;
    }

    // --- detect model format ---
    use_multihead = false;
    use_split = false;
    size_t concat_count = (size_t)(4 + class_count) * 8400;
    for (int i = 0; i < output_num; ++i) {
        size_t cnt = CVI_NN_TensorCount(&outputs[i]);
        if (cnt == concat_count || cnt == concat_count * 1 /* trailing 1 dim */) {
            printf("Runtime YOLOv8 ready: input=%s count=%zu size=%d output=%s count=%zu classes=%d\n",
                   CVI_NN_TensorName(&inputs[0]), input_count, input_size,
                   CVI_NN_TensorName(&outputs[i]), cnt, class_count);
            // Move concat output to outputs[0] if not already
            if (i != 0) { CVI_TENSOR tmp = outputs[0]; outputs[0] = outputs[i]; outputs[i] = tmp; }
            return 1;
        }
    }

    // --- try split format (MaixCam: [4,N] bbox + [cls,N] sigmoid) ---
    {
        int n_anchors = (input_size/8)*(input_size/8) + (input_size/16)*(input_size/16) + (input_size/32)*(input_size/32);
        bool bbox_ok = (output_num >= 2 &&
                        CVI_NN_TensorCount(&outputs[0]) == (size_t)(4 * n_anchors) &&
                        CVI_NN_TensorCount(&outputs[1]) == (size_t)(class_count * n_anchors) &&
                        outputs[0].fmt == CVI_FMT_FP32 && outputs[1].fmt == CVI_FMT_FP32);
        if (bbox_ok) {
            use_split = true;
            printf("Runtime YOLOv8 split: input=%s size=%d anchors=%d classes=%d\n",
                   CVI_NN_TensorName(&inputs[0]), input_size, n_anchors, class_count);
            return 1;
        }
    }

    // --- try multi-head format (Sophgo TDL models) ---
    printf("[setup] output_num=%d, scanning tensors:\n", output_num);
    for (int i = 0; i < output_num && i < 20; ++i)
        printf("  [%d] %s count=%zu fmt=%d\n", i, CVI_NN_TensorName(&outputs[i]),
               CVI_NN_TensorCount(&outputs[i]), (int)outputs[i].fmt);
    int hs[3] = {input_size/8, input_size/16, input_size/32};
    for (int s = 0; s < 3; ++s) { dfl_idx[s] = -1; cls_idx[s] = -1; head_sizes[s] = hs[s]; }
    for (int i = 0; i < output_num; ++i) {
        size_t cnt = CVI_NN_TensorCount(&outputs[i]);
        for (int s = 0; s < 3; ++s) {
            int h = hs[s];
            if (cnt == (size_t)(64 * h * h) && dfl_idx[s] < 0) {
                dfl_idx[s] = i;
                dfl_scale[s] = CVI_NN_TensorQuantScale(&outputs[i]);
                dfl_zero[s]  = CVI_NN_TensorQuantZeroPoint(&outputs[i]);
                break;
            }
            if (cnt == (size_t)(class_count * h * h) && cls_idx[s] < 0) {
                cls_idx[s] = i;
                cls_scale[s] = CVI_NN_TensorQuantScale(&outputs[i]);
                cls_zero[s]  = CVI_NN_TensorQuantZeroPoint(&outputs[i]);
                break;
            }
        }
    }
    int found = 0;
    for (int s = 0; s < 3; ++s) if (dfl_idx[s] >= 0 && cls_idx[s] >= 0) found++;
    if (found == 3) {
        use_multihead = true;
        printf("Runtime YOLOv8 multihead: input=%s size=%d scales=%d/%d/%d classes=%d\n",
               CVI_NN_TensorName(&inputs[0]), input_size, hs[0], hs[1], hs[2], class_count);
        printf("  dfl fmt=%d scale=%.6f/%.6f/%.6f  cls fmt=%d scale=%.6f/%.6f/%.6f\n",
               (int)outputs[dfl_idx[0]].fmt, dfl_scale[0], dfl_scale[1], dfl_scale[2],
               (int)outputs[cls_idx[0]].fmt, cls_scale[0], cls_scale[1], cls_scale[2]);
        return 1;
    }

    printf("Unsupported model format: output_num=%d input_count=%zu classes=%d\n",
           output_num, input_count, class_count);
    return -3;
}

// DFL decode for INT8 tensors: dfl is int8_t* pointer, scale is dequant scale
static float dfl_val_i8(const int8_t *dfl, int coord, int anchor, int hh, float scale, int zero) {
    const int N = 16;
    const int8_t *base = dfl + coord * N * hh;
    float logits[16];
    for (int b = 0; b < N; ++b) logits[b] = ((int)base[b*hh+anchor] - zero) * scale;
    float max_v = logits[0];
    for (int b = 1; b < N; ++b) if (logits[b] > max_v) max_v = logits[b];
    float sum = 0.0f, result = 0.0f;
    for (int b = 0; b < N; ++b) { float e = expf(logits[b] - max_v); sum += e; result += b*e; }
    return result / (sum + 1e-9f);
}

// DFL decode for FP32 tensors
static float dfl_val(const float *dfl, int coord, int anchor, int hh) {
    const int N = 16;
    const float *base = dfl + coord * N * hh;
    float max_v = -1e30f;
    for (int b = 0; b < N; ++b) { float v = base[b*hh+anchor]; if (v > max_v) max_v = v; }
    float sum = 0.0f, result = 0.0f;
    for (int b = 0; b < N; ++b) { float e = expf(base[b*hh+anchor] - max_v); sum += e; result += b*e; }
    return result / (sum + 1e-9f);
}

cvtdl_object_t YoloModelDetector::detect(VIDEO_FRAME_INFO_S *frame_info, bool rotate_180, bool swap_rb) {
    cvtdl_object_t obj_meta = {0};
    if (!model_handle || !frame_info || input_num < 1 || output_num < 1) {
        return obj_meta;
    }

    VIDEO_FRAME_S &vf = frame_info->stVFrame;
    int width = (int)vf.u32Width;
    int height = (int)vf.u32Height;
    int copy_w = std::min(width, input_size);
    int copy_h = std::min(height, input_size);
    void *input_ptr = CVI_NN_TensorPtr(&inputs[0]);
    bool inp_fp32 = (inputs[0].fmt == CVI_FMT_FP32);
    memset(input_ptr, 0, CVI_NN_TensorSize(&inputs[0]));  // correct size regardless of fmt

    void *mapped[3] = {nullptr, nullptr, nullptr};
    for (int c = 0; c < 3; ++c) {
        int src_c = swap_rb ? (c == 0 ? 2 : (c == 2 ? 0 : 1)) : c;
        const unsigned char *src = vf.pu8VirAddr[src_c];
        if (!src && vf.u64PhyAddr[src_c] != 0 && vf.u32Length[src_c] != 0) {
            mapped[src_c] = CVI_SYS_MmapCache(vf.u64PhyAddr[src_c], vf.u32Length[src_c]);
            src = (const unsigned char *)mapped[src_c];
        }
        if (!src) {
            printf("frame plane %d has no virtual or mappable physical address\n", src_c);
            for (int m = 0; m < 3; ++m) {
                if (mapped[m]) CVI_SYS_Munmap(mapped[m], vf.u32Length[m]);
            }
            return obj_meta;
        }
        int stride = (int)vf.u32Stride[src_c];
        if (inp_fp32) {
            float *dst = (float *)input_ptr + c * input_size * input_size;
            for (int y = 0; y < copy_h; ++y) {
                const int sy = rotate_180 ? (copy_h - 1 - y) : y;
                const unsigned char *row = src + sy * stride;
                float *out = dst + y * input_size;
                for (int x = 0; x < copy_w; ++x) {
                    const int sx = rotate_180 ? (copy_w - 1 - x) : x;
                    out[x] = (float)row[sx] / 255.0f;
                }
            }
        } else {
            unsigned char *dst = (unsigned char *)input_ptr + c * input_size * input_size;
            for (int y = 0; y < copy_h; ++y) {
                const int sy = rotate_180 ? (copy_h - 1 - y) : y;
                const unsigned char *row = src + sy * stride;
                unsigned char *drow = dst + y * input_size;
                for (int x = 0; x < copy_w; ++x) {
                    const int sx = rotate_180 ? (copy_w - 1 - x) : x;
                    drow[x] = row[sx];
                }
            }
        }
    }
    for (int c = 0; c < 3; ++c) {
        if (mapped[c]) CVI_SYS_Munmap(mapped[c], vf.u32Length[c]);
    }

    CVI_RC ret = CVI_NN_Forward(model_handle, inputs, input_num, outputs, output_num);
    if (ret != CVI_RC_SUCCESS) {
        printf("CVI_NN_Forward failed: 0x%x\n", ret);
        return obj_meta;
    }

    return decode_outputs();
}

cvtdl_object_t YoloModelDetector::decode_outputs() {
    cvtdl_object_t obj_meta = {0};
    std::vector<RuntimeDet> dets;
    dets.reserve(128);

    if (use_split) {
        // --- split format (MaixCam: [4,N] ltrb + [cls,N] sigmoid) ---
        const int hs[3]      = {input_size/8, input_size/16, input_size/32};
        const int strides[3] = {8, 16, 32};
        int n_anchors = hs[0]*hs[0] + hs[1]*hs[1] + hs[2]*hs[2];
        const float *bbox = (const float *)CVI_NN_TensorPtr(&outputs[0]); // [4, N]
        const float *cls  = (const float *)CVI_NN_TensorPtr(&outputs[1]); // [C, N]
        if (!bbox || !cls) { return obj_meta; }
        int anchor_base = 0;
        for (int s = 0; s < 3; ++s) {
            int h = hs[s]; int stride = strides[s];
            for (int row = 0; row < h; ++row) {
                for (int col = 0; col < h; ++col) {
                    int i = anchor_base + row * h + col;
                    float best = 0.f; int best_c = -1;
                    for (int c = 0; c < class_count; ++c) {
                        float v = cls[c * n_anchors + i];
                        if (v > best) { best = v; best_c = c; }
                    }
                    if (best < threshold) continue;
                    float cx = (col + 0.5f) * stride;
                    float cy = (row + 0.5f) * stride;
                    float l  = bbox[0 * n_anchors + i] * stride;
                    float t  = bbox[1 * n_anchors + i] * stride;
                    float r  = bbox[2 * n_anchors + i] * stride;
                    float b  = bbox[3 * n_anchors + i] * stride;
                    RuntimeDet d;
                    d.x1 = clampf(cx - l, 0.f, (float)input_size);
                    d.y1 = clampf(cy - t, 0.f, (float)input_size);
                    d.x2 = clampf(cx + r, 0.f, (float)input_size);
                    d.y2 = clampf(cy + b, 0.f, (float)input_size);
                    d.score = best; d.cls = best_c;
                    float area = std::max(0.f,d.x2-d.x1)*std::max(0.f,d.y2-d.y1);
                    if (d.cls == 0 && area > 0.85f*input_size*input_size) continue;
                    dets.push_back(d);
                }
            }
            anchor_base += h * h;
        }
    } else if (!use_multihead) {
        // --- concat format (Milk-V / output0_Concat_f32) ---
        float *out = (float *)CVI_NN_TensorPtr(&outputs[0]);
        int channels = 4 + class_count;
        int boxes = (int)(CVI_NN_TensorCount(&outputs[0]) / channels);
        for (int i = 0; i < boxes; ++i) {
            float best = 0.0f; int cls = -1;
            for (int c = 0; c < class_count; ++c) {
                float score = out[(4 + c) * boxes + i];
                if (score > best) { best = score; cls = c; }
            }
            if (best < threshold) continue;
            float cx = out[0*boxes+i], cy = out[1*boxes+i];
            float w  = out[2*boxes+i], h  = out[3*boxes+i];
            RuntimeDet d;
            d.x1 = clampf(cx - w*0.5f, 0.f, (float)input_size);
            d.y1 = clampf(cy - h*0.5f, 0.f, (float)input_size);
            d.x2 = clampf(cx + w*0.5f, 0.f, (float)input_size);
            d.y2 = clampf(cy + h*0.5f, 0.f, (float)input_size);
            d.score = best; d.cls = cls;
            float area = std::max(0.f,d.x2-d.x1)*std::max(0.f,d.y2-d.y1);
            if (d.cls == 0 && area > 0.85f*input_size*input_size) continue;
            dets.push_back(d);
        }
    } else {
        // --- multi-head DFL format (Sophgo TDL YOLOv8) ---
        bool is_i8 = (outputs[dfl_idx[0]].fmt == CVI_FMT_INT8);
        int strides[3] = {input_size/head_sizes[0], input_size/head_sizes[1], input_size/head_sizes[2]};
        for (int s = 0; s < 3; ++s) {
            int h = head_sizes[s];
            int hh = h * h;
            int stride = strides[s];
            void *dfl_ptr = CVI_NN_TensorPtr(&outputs[dfl_idx[s]]);
            void *cls_ptr = CVI_NN_TensorPtr(&outputs[cls_idx[s]]);
            if (!dfl_ptr || !cls_ptr) { printf("[multihead] NULL tensor at scale %d\n",h); continue; }
            const int8_t  *dfl_i8  = (const int8_t *)dfl_ptr;
            const int8_t  *cls_i8  = (const int8_t *)cls_ptr;
            const float   *dfl_f32 = (const float  *)dfl_ptr;
            const float   *cls_f32 = (const float  *)cls_ptr;
            for (int row = 0; row < h; ++row) {
                for (int col = 0; col < h; ++col) {
                    int anchor = row * h + col;
                    // class scores with sigmoid + dequant if INT8
                    float best = 0.0f; int best_c = -1;
                    for (int c = 0; c < class_count; ++c) {
                        float raw = is_i8
                            ? ((int)cls_i8[c * hh + anchor] - cls_zero[s]) * cls_scale[s]
                            : cls_f32[c * hh + anchor];
                        float v = 1.0f / (1.0f + expf(-raw));
                        if (v > best) { best = v; best_c = c; }
                    }
                    if (best < threshold) continue;
                    // DFL decode
                    float l = is_i8 ? dfl_val_i8(dfl_i8, 0, anchor, hh, dfl_scale[s], dfl_zero[s])
                                    : dfl_val(dfl_f32, 0, anchor, hh);
                    float t = is_i8 ? dfl_val_i8(dfl_i8, 1, anchor, hh, dfl_scale[s], dfl_zero[s])
                                    : dfl_val(dfl_f32, 1, anchor, hh);
                    float r = is_i8 ? dfl_val_i8(dfl_i8, 2, anchor, hh, dfl_scale[s], dfl_zero[s])
                                    : dfl_val(dfl_f32, 2, anchor, hh);
                    float b = is_i8 ? dfl_val_i8(dfl_i8, 3, anchor, hh, dfl_scale[s], dfl_zero[s])
                                    : dfl_val(dfl_f32, 3, anchor, hh);
                    float cx = (col + 0.5f) * stride;
                    float cy = (row + 0.5f) * stride;
                    RuntimeDet d;
                    d.x1 = clampf(cx - l*stride, 0.f, (float)input_size);
                    d.y1 = clampf(cy - t*stride, 0.f, (float)input_size);
                    d.x2 = clampf(cx + r*stride, 0.f, (float)input_size);
                    d.y2 = clampf(cy + b*stride, 0.f, (float)input_size);
                    d.score = best; d.cls = best_c;
                    float area = std::max(0.f,d.x2-d.x1)*std::max(0.f,d.y2-d.y1);
                    if (d.cls == 0 && area > 0.85f*input_size*input_size) continue;
                    dets.push_back(d);
                }
            }
        }
    }

    std::vector<RuntimeDet> final = nms(dets, nms_threshold);
    obj_meta.size = final.size();
    obj_meta.width = input_size;
    obj_meta.height = input_size;
    if (obj_meta.size == 0) {
        return obj_meta;
    }

    obj_meta.info = (cvtdl_object_info_t *)calloc(obj_meta.size, sizeof(cvtdl_object_info_t));
    if (!obj_meta.info) {
        obj_meta.size = 0;
        return obj_meta;
    }
    for (uint32_t i = 0; i < obj_meta.size; ++i) {
        const RuntimeDet &d = final[i];
        cvtdl_object_info_t &info = obj_meta.info[i];
        info.classes = d.cls;
        info.bbox.x1 = d.x1;
        info.bbox.y1 = d.y1;
        info.bbox.x2 = d.x2;
        info.bbox.y2 = d.y2;
        info.bbox.score = d.score;
        if (d.cls >= 0 && d.cls < 80) {
            snprintf(info.name, sizeof(info.name), "%s", COCO_CLASSES[d.cls]);
        } else {
            snprintf(info.name, sizeof(info.name), "obj");
        }
    }
    return obj_meta;
}

cvtdl_object_t YoloModelDetector::detect(const cv::Mat& bgr_tile) {
    cvtdl_object_t obj_meta = {0};
    if (!model_handle || bgr_tile.empty() || input_num < 1 || output_num < 1)
        return obj_meta;

    int copy_w = std::min(bgr_tile.cols, input_size);
    int copy_h = std::min(bgr_tile.rows, input_size);
    void *input_ptr = CVI_NN_TensorPtr(&inputs[0]);
    bool inp_fp32 = (inputs[0].fmt == CVI_FMT_FP32);
    memset(input_ptr, 0, CVI_NN_TensorSize(&inputs[0]));

    // BGR interleaved → RGB planar (model plane order: 0=R, 1=G, 2=B)
    std::vector<cv::Mat> ch;
    cv::split(bgr_tile, ch);
    const int src_plane[3] = {2, 1, 0};
    for (int c = 0; c < 3; ++c) {
        const cv::Mat& src = ch[src_plane[c]];
        if (inp_fp32) {
            float *dst = (float*)input_ptr + c * input_size * input_size;
            for (int y = 0; y < copy_h; ++y) {
                const unsigned char *row = src.ptr<unsigned char>(y);
                float *out = dst + y * input_size;
                for (int x = 0; x < copy_w; ++x)
                    out[x] = (float)row[x] / 255.0f;
            }
        } else {
            unsigned char *dst = (unsigned char*)input_ptr + c * input_size * input_size;
            for (int y = 0; y < copy_h; ++y)
                memcpy(dst + y * input_size, src.ptr<unsigned char>(y), copy_w);
        }
    }

    CVI_RC ret = CVI_NN_Forward(model_handle, inputs, input_num, outputs, output_num);
    if (ret != CVI_RC_SUCCESS) {
        printf("CVI_NN_Forward (tile) failed: 0x%x\n", ret);
        return obj_meta;
    }
    return decode_outputs();
}

void YoloModelDetector::free_objects(cvtdl_object_t *objects) {
    if (!objects) return;
    if (objects->info) {
        free(objects->info);
    }
    memset(objects, 0, sizeof(*objects));
}

void YoloModelDetector::release() {
    if (model_handle != nullptr) {
        CVI_NN_CleanupModel(model_handle);
        model_handle = nullptr;
    }
    inputs = nullptr;
    outputs = nullptr;
    input_num = 0;
    output_num = 0;
}

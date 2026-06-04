#pragma once

#include "cvi_tdl.h"
#include "../../../sdk/sample/3rd/tpu/include/cviruntime.h"
#include <opencv2/opencv.hpp>

class YoloModelDetector {
	    public:
	        YoloModelDetector();
	        int setup_model(char *model_path, int model_class_cnt, float model_thresh, float model_nms_thresh, float model_scale=0.0039216, float model_mean=0.0);
	        cvtdl_object_t detect(VIDEO_FRAME_INFO_S *frame_info, bool rotate_180=true, bool swap_rb=false);
	        cvtdl_object_t detect(const cv::Mat& bgr_tile);
	        void free_objects(cvtdl_object_t *objects);
	        void release();

	    private:
	        cvtdl_object_t decode_outputs();
	        CVI_MODEL_HANDLE model_handle = nullptr;
	        CVI_TENSOR *inputs = nullptr;
	        CVI_TENSOR *outputs = nullptr;
	        int32_t input_num = 0;
	        int32_t output_num = 0;
	        int class_count = 80;
	        float threshold = 0.45f;
	        float nms_threshold = 0.5f;
	        int input_size = 640;
	        bool use_multihead = false;
	        bool use_split = false;
	        int dfl_idx[3] = {-1,-1,-1};
	        int cls_idx[3] = {-1,-1,-1};
	        int head_sizes[3] = {80,40,20};
	        float dfl_scale[3] = {1,1,1};
	        float cls_scale[3] = {1,1,1};
	        int   dfl_zero[3]  = {0,0,0};
	        int   cls_zero[3]  = {0,0,0};
};

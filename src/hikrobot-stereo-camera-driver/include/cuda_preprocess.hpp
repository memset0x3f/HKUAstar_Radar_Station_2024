#ifndef PREPROCESS_H
#define PREPROCESS_H

#include <opencv2/opencv.hpp>
#include <cuda_runtime.h>

void cuda_preprocess(cudaStream_t &stream, const cv::Mat& srcImg, float* dstData, const int dstHeight, const int dstWidth, bool doNormalize = true);
/*
srcImg:    source image for inference
dstData:   data after preprocess (resize / bgr to rgb / hwc to chw / normalize)
dstHeight: input height
dstWidth:  input width
*/

#endif  // PREPROCESS_H
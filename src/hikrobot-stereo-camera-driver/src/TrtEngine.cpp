#include "TrtEngine.hpp"
#include "cuda_preprocess.hpp"
#include <NvInfer.h>
#include <NvOnnxParser.h>
#include <algorithm>
#include <cassert>
#include <chrono>
#include <fstream>
#include <future>
#include <iostream>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/operations.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/cuda.hpp>
#include <string>
#include <vector>
#include <omp.h>
#include <ros/ros.h>

using namespace engine;

void Logger::log(Severity severity, const char* msg) noexcept
{
    // suppress info-level messages
    switch (severity) {
    case Severity::kINTERNAL_ERROR:
        std::cerr << "[INTERNAL_ERROR]: " << msg << std::endl;
        break;
    case Severity::kERROR:
        std::cerr << "[ERROR]: " << msg << std::endl;
        break;
    case Severity::kWARNING:
        std::cerr << "[WARNING]: " << msg << std::endl;
        break;
    default:
        break;
    }
}

/************** TrtEngineBase ****************/

TrtEngineBase::TrtEngineBase() : 
    logger(new Logger()),
    runtime(nvinfer1::createInferRuntime(*logger))
{
}

TrtEngineBase::~TrtEngineBase()
{
    // Release the engine
    engine.reset();
    // Release the runtime
    runtime.reset();
}

void TrtEngineBase::deserializeEngineFromFile(const std::string& enginePath){
    std::ifstream engineFile(enginePath, std::ios::binary);
    if (!engineFile)
    {
        logger->log(Logger::Severity::kERROR, "Error: Unable to open engine file");
        throw std::runtime_error("Error: Unable to open engine file");
    }
    engineFile.seekg(0, engineFile.end);
    size_t fileSize = engineFile.tellg();
    engineFile.seekg(0, engineFile.beg);

    std::unique_ptr<char[]> engineData(new char[fileSize]);
    engineFile.read(engineData.get(), fileSize);
    engineFile.close();

    engine = std::unique_ptr<nvinfer1::ICudaEngine>(runtime->deserializeCudaEngine(engineData.get(), fileSize));
}

void TrtEngineBase::build(const std::string &modelPath){
    deserializeEngineFromFile(modelPath);

    inputName = engine->getIOTensorName(0);
    outputName = engine->getIOTensorName(1);
    inputDims = engine->getTensorShape(inputName.c_str());
    outputDims = engine->getTensorShape(outputName.c_str());
    maxBatchSize = inputDims.d[0];
}

void TrtEngineBase::letterBox(const cv::Mat& img, cv::Mat& letterBoxed, cv::Size targetSize){
    float scale = std::min(static_cast<float>(targetSize.width) / img.cols, static_cast<float>(targetSize.height) / img.rows);

    cv::Mat resized;
    cv::resize(img, resized, cv::Size(), scale, scale, cv::INTER_LINEAR);
    // Use copyMakeBorder to pad the image
    int top = (targetSize.height - resized.rows) / 2;
    int bottom = targetSize.height - resized.rows - top;
    int left = (targetSize.width - resized.cols) / 2;
    int right = targetSize.width - resized.cols - left;
    cv::copyMakeBorder(resized, letterBoxed, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
}

BBox TrtEngineBase::letterBox2Original(const BBox& bbox, cv::Size originalSize, cv::Size boxSize){
    float scale = std::min(static_cast<float>(boxSize.width) / originalSize.width, static_cast<float>(boxSize.height) / originalSize.height);
    float dx = (boxSize.width - scale * originalSize.width) / 2;
    float dy = (boxSize.height - scale * originalSize.height) / 2;

    BBox originalBbox;
    originalBbox.x1 = (bbox.x1 - dx) / scale;
    originalBbox.y1 = (bbox.y1 - dy) / scale;
    originalBbox.x2 = (bbox.x2 - dx) / scale;
    originalBbox.y2 = (bbox.y2 - dy) / scale;
    originalBbox.score = bbox.score;
    originalBbox.classId = bbox.classId;

    return originalBbox;
}

void TrtEngineBase::preprocessBase(const ImageBatch &imgs, float *proceessed, bool doNormalize){
    assert(imgs.size() <= maxBatchSize);

    std::vector<cudaStream_t> streams(imgs.size());
    for(int i = 0; i < imgs.size(); ++i){
        cudaStreamCreate(&streams[i]);
    }

    for(int i = 0; i < imgs.size(); ++i){
        if(imgs[i].empty()) continue;
        // cv::Mat &img = imgs[i];
        int offset = i * inputDims.d[1] * inputDims.d[2] * inputDims.d[3];

        cuda_preprocess(streams[i], imgs[i], proceessed+offset, inputDims.d[2], inputDims.d[3], doNormalize);
    }
    // cudaDeviceSynchronize();

    // omp_set_num_threads(2);
    // #pragma omp parallel for
    for(int i = 0; i < imgs.size(); ++i){
        cudaStreamSynchronize(streams[i]);
        cudaStreamDestroy(streams[i]);
    }
}

/************** YoloEngine ****************/

YoloEngine::YoloEngine(float cls_thres, float nms_thres):CLS_THRES(cls_thres), NMS_THRES(nms_thres){}

void YoloEngine::infer(ImageBatch &imgs, std::vector<std::vector<BBox>> &bboxes){
    // assert(imgs.size() <= maxBatchSize);
    if(imgs.size() > maxBatchSize){
        imgs.resize(maxBatchSize);
    }

    auto context = std::unique_ptr<nvinfer1::IExecutionContext>(engine->createExecutionContext());
    cudaStream_t stream;
    cudaStreamCreate(&stream);

    size_t n_inputElem = inputDims.d[0] * inputDims.d[1] * inputDims.d[2] * inputDims.d[3];
    size_t n_outputElem = outputDims.d[0] * outputDims.d[1] * outputDims.d[2];
    size_t inputSize = n_inputElem * sizeof(float);
    size_t outputSize = n_outputElem * sizeof(float);

    // Allocate memory for input and output tensors
    void* buffers[2];
    cudaMalloc(&buffers[0], inputSize);
    cudaMalloc(&buffers[1], outputSize);
    context->setTensorAddress(inputName.c_str(), static_cast<float*>(buffers[0]));
    context->setTensorAddress(outputName.c_str(), static_cast<float*>(buffers[1]));

    
    // std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    preprocessBase(imgs, static_cast<float*>(buffers[0]), true);
    // std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    // ROS_INFO("Time Preprocess = %d [us]", (int)std::chrono::duration_cast<std::chrono::microseconds>(end - start).count());

    // Execute the network
    context->enqueueV3(stream);

    // Copy output data to host
    std::vector<float> output(n_outputElem);
    cudaMemcpyAsync(output.data(), buffers[1], outputSize, cudaMemcpyDeviceToHost, stream);

    // Release the memory
    cudaFreeAsync(buffers[0], stream);
    cudaFreeAsync(buffers[1], stream);

    cudaStreamSynchronize(stream);
    cudaStreamDestroy(stream);

    // Post-process the output
    std::vector<cv::Size> originalSize(imgs.size(), cv::Size());
    for (int i = 0; i < imgs.size(); ++i){
        if(!imgs[i].empty()){
            originalSize[i] = imgs[i].size();
        }
    }
    postprocess(output, bboxes, originalSize);
    
}

void YoloEngine::postprocess(
    const std::vector<float> &output, 
    std::vector<std::vector<BBox>>& results, 
    std::vector<cv::Size> &originalSize
){
    size_t n_rows = outputDims.d[1];
    size_t n_cols = outputDims.d[2];

    results.clear();
    results.resize(originalSize.size());
    for (int bid = 0; bid < originalSize.size(); ++bid){
        if(originalSize[bid].empty()){
            results[bid] = {};
            continue;
        }
        const float *data = output.data() + n_rows*n_cols*bid;
        std::vector<cv::Rect> boxes;
        std::vector<float> scores;
        std::vector<int> classes;

        for(int j = 0; j < n_cols; ++j){
            float max_conf = 0;
            int max_id;
            for(int i = 4; i < n_rows; ++i){
                int idx = i*n_cols + j;
                if(data[idx] > max_conf){
                    max_conf = data[idx];
                    max_id = i-4;
                }
            }
            if(max_conf > CLS_THRES){
                float x = data[j], y = data[j + n_cols];
                float w = data[j + 2*n_cols], h = data[j + 3*n_cols];
                float left = x-w/2, top = y-h/2;
                boxes.emplace_back(cv::Rect(left, top, w, h));
                scores.emplace_back(max_conf);
                classes.emplace_back(max_id);
            }
        }

        // NMS
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, scores, 0.0, NMS_THRES, indices);
        std::vector<BBox> bboxes;
        for (int id: indices){
            BBox bbox;
            bbox.x1 = boxes[id].x;
            bbox.y1 = boxes[id].y;
            bbox.x2 = boxes[id].x + boxes[id].width;
            bbox.y2 = boxes[id].y + boxes[id].height;
            bbox.score = scores[id];
            bbox.classId = classes[id];
            bbox = letterBox2Original(bbox, originalSize[bid], cv::Size(inputDims.d[3], inputDims.d[2]));
            bboxes.emplace_back(bbox);
        }
        results[bid] = bboxes;
    }

}

void YoloEngine::warmup(){
    // Generate a random image for warmup
    ImageBatch imgs(maxBatchSize, cv::Mat::zeros(inputDims.d[2], inputDims.d[3], CV_8UC3));
    std::vector<std::vector<BBox>> bboxes;
    for(int i = 0; i < 20; i++){
        infer(imgs, bboxes);
    }
}

/************** ReidEngine ****************/

void ReidEngine::infer(ImageBatch &imgs, std::vector<FeatureTensor> &features){
    auto context = std::unique_ptr<nvinfer1::IExecutionContext>(engine->createExecutionContext());

    size_t n_inputElem = inputDims.d[0] * inputDims.d[1] * inputDims.d[2] * inputDims.d[3];
    size_t n_outputElem = outputDims.d[0] * outputDims.d[1];
    size_t inputSize = n_inputElem * sizeof(float);
    size_t outputSize = n_outputElem * sizeof(float);
    size_t featureDim = outputDims.d[1];

    cudaStream_t stream;
    cudaStreamCreate(&stream);

    // Allocate memory for input and output tensors
    void* buffers[2];
    cudaMalloc(&buffers[0], inputSize);
    cudaMalloc(&buffers[1], outputSize);
    context->setTensorAddress(inputName.c_str(), static_cast<float*>(buffers[0]));
    context->setTensorAddress(outputName.c_str(), static_cast<float*>(buffers[1]));

    preprocessBase(imgs, static_cast<float*>(buffers[0]), false);

    // Execute the network
    context->enqueueV3(stream);

    // Copy output data to host
    std::vector<float> output(n_outputElem);
    cudaMemcpyAsync(output.data(), buffers[1], outputSize, cudaMemcpyDeviceToHost, stream);

    // Release the memory
    cudaFreeAsync(buffers[0], stream);
    cudaFreeAsync(buffers[1], stream);

    cudaStreamSynchronize(stream);
    cudaStreamDestroy(stream);

    // Post-process the output
    for (int i = 0; i < imgs.size(); ++i){
        if(!imgs[i].empty()){
            // std::copy(output.begin() + i*featureDim, output.begin() + (i+1)*featureDim, feature.begin());
            features.emplace_back(Eigen::Map<FeatureTensor>(output.data() + i*featureDim, featureDim));
        }
    }
}

void ReidEngine::warmup(){
    // Generate a random image for warmup
    ImageBatch imgs(maxBatchSize, cv::Mat::zeros(inputDims.d[2], inputDims.d[3], CV_8UC3));
    std::vector<FeatureTensor> features;
    for(int i = 0; i < 20; i++){
        infer(imgs, features);
    }
}
#ifndef ENGINE_H
#define ENGINE_H

#include <NvInfer.h>
#include <NvInferRuntime.h>
#include <NvInferRuntimeBase.h>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <string>
#include <opencv2/opencv.hpp>
#include <vector>
#include <Eigen/Dense>

namespace engine{
    class Logger : public nvinfer1::ILogger{
    public:
        void log(Severity severity, const char* msg) noexcept;
    };

    struct BBox{
        int x1, y1, x2, y2;       // Normalized coordinates
        float score;                // Confidence score
        int classId;                // Class ID
    };

    class TrtEngineBase{
        public:
            using ImageBatch = std::vector<cv::Mat>;
        protected:
            int maxBatchSize;
            std::string inputName, outputName;

            std::unique_ptr<Logger> logger;
            std::unique_ptr<nvinfer1::IRuntime> runtime;
            std::unique_ptr<nvinfer1::ICudaEngine> engine;
            nvinfer1::Dims inputDims, outputDims;

            void deserializeEngineFromFile(const std::string&);
            static void letterBox(const cv::Mat&, cv::Mat&, cv::Size);
            static BBox letterBox2Original(const BBox&, cv::Size, cv::Size);
            virtual void preprocessBase(const ImageBatch&, float *, bool doNormalize = true);
            // virtual void postprocess(const std::vector<float>&, std::vector<std::vector<BBox>>&, std::vector<cv::Size>&);

        public:
            TrtEngineBase();
            virtual ~TrtEngineBase();

            // Disable copy and assignment (unique_ptr is non-copyable)
            TrtEngineBase(const TrtEngineBase&) = delete;
            TrtEngineBase& operator=(const TrtEngineBase&) = delete;

            void build(const std::string&);
            // void infer(ImageBatch&, std::vector<std::vector<BBox>>&);
            // void warmup();
    };

    class YoloEngine: public TrtEngineBase{
        private:
            const float CLS_THRES;
            const float NMS_THRES;

            void postprocess(const std::vector<float>&, std::vector<std::vector<BBox>>&, std::vector<cv::Size>&);
        public:
            YoloEngine(float clas_thres = 0.3, float nms_thres = 0.4);
            void infer(ImageBatch&, std::vector<std::vector<BBox>>&);
            void warmup();
    };

    class ReidEngine: public TrtEngineBase{
        public:
            using FeatureTensor = Eigen::VectorXf;

            void infer(ImageBatch&, std::vector<FeatureTensor>&);
            void warmup();
    };

}

#endif  // ENGINE_H
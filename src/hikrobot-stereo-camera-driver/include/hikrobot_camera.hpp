#ifndef CAMERA_HPP
#define CAMERA_HPP
#include "opencv2/core/mat.hpp"
#include "opencv2/core/types.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "ros/init.h"
#include "ros/ros.h"
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <stdio.h>
#include <pthread.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <opencv2/opencv.hpp>
#include <sys/types.h>
#include <thread>
#include <utility>
#include "MvErrorDefine.h"
#include "CameraParams.h"
#include "MvCameraControl.h"
#include "shared.hpp"
#include <cuda_runtime.h>

namespace camera
{
    struct ThreadArgs {
        void *handle;
        cv::Mat frame;
        bool frame_empty;
        std::mutex mutex;
    };
//********** define ************************************/
#define MAX_IMAGE_DATA_SIZE (4 * 3270 * 2048)
    //********** frame ************************************/
    //cv::Mat frame;
    //********** frame_empty ******************************/
    //bool frame_empty = 0;
    //********** mutex ************************************/
    //pthread_mutex_t mutex;
    //********** CameraProperties config ************************************/
    enum CamerProperties
    {
        CAP_PROP_FRAMERATE_ENABLE,  //帧数可调
        CAP_PROP_FRAMERATE,         //帧数
        CAP_PROP_BURSTFRAMECOUNT,   //外部一次触发帧数
        CAP_PROP_HEIGHT,            //图像高度
        CAP_PROP_WIDTH,             //图像宽度
        CAP_PROP_EXPOSURE_TIME,     //曝光时间
        CAP_PROP_GAMMA_ENABLE,      //伽马因子可调
        CAP_PROP_GAMMA,             //伽马因子
        CAP_PROP_GAINAUTO,          //亮度
        CAP_PROP_SATURATION_ENABLE, //饱和度可调
        CAP_PROP_SATURATION,        //饱和度
        CAP_PROP_OFFSETX,           //X偏置
        CAP_PROP_OFFSETY,           //Y偏置
        CAP_PROP_TRIGGER_MODE,      //外部触发
        CAP_PROP_TRIGGER_SOURCE,    //触发源
        CAP_PROP_LINE_SELECTOR      //触发线

    };

    enum E_TriggerMode{
        TriggerMode_Off = 0,
        TriggerMode_On = 1
    };
    
    enum E_TiggerSource{
        TriggerSource_Line0 = 0,
        TriggerSource_Line1 = 1,
        TriggerSource_Line2 = 2,
        TriggerSource_Line3 = 3,
        TriggerSource_Counter0 = 4,
        TriggerSource_Software = 7,
        TriggerSource_FrequencyConverter = 8
    };

    //^ *********************************************************************************** //
    //^ ********************************** Camera Class************************************ //
    //^ *********************************************************************************** //
    class Camera
    {
    public:
        //********** 构造函数  ****************************/
        Camera(ros::NodeHandle &node, int ind);
        //********** 析构函数  ****************************/
        ~Camera();
        //********** 原始信息转换线程 **********************/
        void *HKWorkThread(void *p_handle);

        //********** 输出摄像头信息 ***********************/
        bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo);

        int getHeight();
        int getWidth();

        //********** 设置摄像头参数 ***********************/
        bool set(camera::CamerProperties type, float value);
        //********** 恢复默认参数 *************************/
        bool reset();
        //********** 读图10个相机的原始图像 ********************************/
        void ReadImg(cv::Mat &image);
        void TriggerCapture(cv::Mat &image);
        std::future<void> asyncGetImage(cv::Mat &image);

    private:
        ThreadArgs* args;
        std::unique_ptr<std::thread> workthread;
        //********** frame ************************************/
        //cv::Mat frame;
        //********** frame_empty ******************************/
        //bool frame_empty = 0;
        //********** mutex ************************************/
        //pthread_mutex_t mutex;    
        //********** handle ******************************/
        //void *handle;
        //********** nThreadID ******************************/
        pthread_t nThreadID;
        //********** yaml config ******************************/
        int nRet;
        int width;
        int height;
        int Offset_x;
        int Offset_y;
        bool FrameRateEnable;
        int FrameRate;

        int BurstFrameCount;
        int ExposureTime;
        bool GammaEnable;
        float Gamma;
        int GainAuto;
        bool SaturationEnable;
        int Saturation;
        int TriggerMode;
        int TriggerSource;
        int LineSelector;
        float Gain;

        int cam_index;

        int shmid;
        shared::shared_struct *shm;
    };
    //^ *********************************************************************************** //

    int Camera::getHeight(){
        return height;
    }

    int Camera::getWidth(){
        return width;
    }

    //^ ********************************** Camera constructor************************************ //
    Camera::Camera(ros::NodeHandle &node, int ind)
    {
        cam_index = ind;
        args = new ThreadArgs();
        args -> handle = NULL;
        args -> frame_empty = 0;
        //********** 读取待设置的摄像头参数 第三个参数是默认值 yaml文件未给出该值时生效 ********************************/
        node.param("width", width, 1280);
        node.param("height", height, 1024);
        node.param("FrameRateEnable", FrameRateEnable, false);
        node.param("FrameRate", FrameRate, 100);
        node.param("BurstFrameCount", BurstFrameCount, 10); // 一次触发采集的次数
        node.param("ExposureTime", ExposureTime, 4000);
        node.param("GammaEnable", GammaEnable, false);
        node.param("Gamma", Gamma, (float)0.7);
        node.param("GainAuto", GainAuto, 0);
        node.param("SaturationEnable", SaturationEnable, false);
        node.param("Saturation", Saturation, 128);
        node.param("Offset_x", Offset_x, 0);
        node.param("Offset_y", Offset_y, 0);
        node.param("TriggerMode", TriggerMode, static_cast<int>(TriggerMode_Off));
        node.param("TriggerSource", TriggerSource, static_cast<int>(TriggerSource_Software));
        node.param("LineSelector", LineSelector, 2);
        node.param("Gain", Gain, (float)20.0);

        //********** 枚举设备 ********************************/
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        nRet = MV_CC_EnumDevicesEx2(
            MV_GIGE_DEVICE | MV_USB_DEVICE, 
            &stDeviceList, 
            NULL, 
            SortMethod_UserID
        );
        if (MV_OK != nRet)
        {
            printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        if (stDeviceList.nDeviceNum > 0)
        {
            // if(stDeviceList.nDeviceNum == 1) {
            //     printf("Only 1 camera detected\n");
            //     exit(-1);
            // }
            
            printf("[device %d]:\n", ind);
            MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[ind];
            PrintDeviceInfo(pDeviceInfo);
        }
        else
        {
            printf("Find No Devices!\n");
            exit(-1);
        }

        //********** 选择设备并创建句柄 *************************/

        nRet = MV_CC_CreateHandle(&(args->handle), stDeviceList.pDeviceInfo[ind]);

        
        if (MV_OK != nRet)
        {
            printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
            exit(-1);
        }

        // 打开设备
        //********** frame **********/

        nRet = MV_CC_OpenDevice(args->handle);

        if (MV_OK != nRet)
        {
            printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
            exit(-1);
        }

        //设置 yaml 文件里面的配置
        this->set(CAP_PROP_FRAMERATE_ENABLE, FrameRateEnable);
        if (FrameRateEnable)
            this->set(CAP_PROP_FRAMERATE, FrameRate);
        // this->set(CAP_PROP_BURSTFRAMECOUNT, BurstFrameCount);
        this->set(CAP_PROP_HEIGHT, height);
        this->set(CAP_PROP_WIDTH, width);
        this->set(CAP_PROP_OFFSETX, Offset_x);
        this->set(CAP_PROP_OFFSETY, Offset_y);
        this->set(CAP_PROP_EXPOSURE_TIME, ExposureTime);
        // printf("\n%d\n",GammaEnable);
        this->set(CAP_PROP_GAMMA_ENABLE, GammaEnable);
        // printf("\n%d\n",GammaEnable);
        if (GammaEnable)
            this->set(CAP_PROP_GAMMA, Gamma);
        this->set(CAP_PROP_GAINAUTO, GainAuto);
        this->set(CAP_PROP_TRIGGER_MODE, TriggerMode);
        this->set(CAP_PROP_TRIGGER_SOURCE, TriggerSource);


        // Set Gain
        nRet = MV_CC_SetFloatValue(args->handle, "Gain", Gain);
        if (MV_OK == nRet)
        {
            printf("set Gain OK! value=%f\n", Gain);
        }
        else
        {
            printf("Set Gain Failed! nRet = [%x]\n\n", nRet);
        }

        // this->set(CAP_PROP_LINE_SELECTOR, LineSelector);

        //********** frame **********/
        //白平衡 非自适应（给定参数0）
        nRet = MV_CC_SetEnumValue(args->handle, "BalanceWhiteAuto", 2);
        // //白平衡度
        // int rgb[3] = {1742, 1024, 2371};
        // for (int i = 0; i < 3; i++)
        // {
        //     //********** frame **********/

        //     nRet = MV_CC_SetEnumValue(handle, "BalanceRatioSelector", i);
        //     nRet = MV_CC_SetIntValue(handle, "BalanceRatio", rgb[i]);
        // }
        if (MV_OK == nRet)
        {
            printf("set BalanceRatio OK! value=%f\n",0.0 );
        }
        else
        {
            printf("Set BalanceRatio Failed! nRet = [%x]\n\n", nRet);
        }
        this->set(CAP_PROP_SATURATION_ENABLE, SaturationEnable);
        if (SaturationEnable)
            this->set(CAP_PROP_SATURATION, Saturation);

        //********** 图像格式 **********/
        // 0x01100003:Mono10
        // 0x010C0004:Mono10Packed
        // 0x01100005:Mono12
        // 0x010C0006:Mono12Packed
        // 0x01100007:Mono16
        // 0x02180014:RGB8Packed
        // 0x02100032:YUV422_8
        // 0x0210001F:YUV422_8_UYVY
        // 0x01080008:BayerGR8
        // 0x01080009:BayerRG8
        // 0x0108000A:BayerGB8
        // 0x0108000B:BayerBG8
        // 0x0110000e:BayerGB10
        // 0x01100012:BayerGB12
        // 0x010C002C:BayerGB12Packed
        nRet = MV_CC_SetEnumValue(args->handle, "PixelFormat", 0x0108000A); // 目前 RGB  

        if (MV_OK == nRet)
        {
            printf("set PixelFormat OK ! value = RGB\n");
        }
        else
        {
            printf("MV_CC_SetPixelFormat fail! nRet [%x]\n", nRet);
        }
        MVCC_ENUMVALUE t = {0};
        //********** frame **********/

        nRet = MV_CC_GetEnumValue(args->handle, "PixelFormat", &t);

        if (MV_OK == nRet)
        {
            printf("PixelFormat :%d!\n", t.nCurValue); // 35127316
        }
        else
        {
            printf("get PixelFormat fail! nRet [%x]\n", nRet);
        }
        // 开始取流
        //********** frame **********/

        nRet = MV_CC_StartGrabbing(args->handle);

        if (MV_OK != nRet)
        {
            printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
            exit(-1);
        }

        /************* Shared Memory ***********/
        shmid = shmget((key_t)(ind + shared::idx_base), sizeof(shared::shared_struct), 0666 | IPC_CREAT);
        if (shmid == -1)
        {
            fprintf(stderr, "shmget failed\n");
            exit(EXIT_FAILURE);
        }

        shm = (shared::shared_struct *)shmat(shmid, 0, 0);
        if (shm == (void *)-1)
        {
            fprintf(stderr, "shmat failed\n");
            exit(EXIT_FAILURE);
        }
        else{
            printf("shared memory attached at %p\n", shm);
        }
        memset(shm->shared_img, 0, shared::shared_img_size);

        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);

        pthread_mutex_init(&(shm->shared_mutex), &attr);

        //********** frame **********/

        if(TriggerMode == TriggerMode_Off){
            std::function<void*(ThreadArgs*)> func = std::bind(&Camera::HKWorkThread, this, std::placeholders::_1);
            workthread = std::make_unique<std::thread>(func, args);
        }

    }

    //^ ********************************** Camera constructor************************************ //
    Camera::~Camera()
    {
        int nRet;
        //********** frame **********/

        if (TriggerMode == TriggerMode_Off)
            workthread->join();

        //********** frame **********/

        nRet = MV_CC_StopGrabbing(args->handle);

        if (MV_OK != nRet)
        {
            printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        printf("MV_CC_StopGrabbing succeed.\n");
        // 关闭设备
        //********** frame **********/

        nRet = MV_CC_CloseDevice(args->handle);

        if (MV_OK != nRet)
        {
            printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        printf("MV_CC_CloseDevice succeed.\n");
        // 销毁句柄
        //********** frame **********/

        nRet = MV_CC_DestroyHandle(args->handle);

        if (MV_OK != nRet)
        {
            printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
            exit(-1);
        }
        printf("MV_CC_DestroyHandle succeed.\n");

        delete args;

        /*******Shared Memory********/
        if(shmctl(shmid, IPC_RMID, 0) == -1){
            fprintf(stderr, "shmctl(IPC_RMID) failed\n");
            exit(EXIT_FAILURE);
        }
    }

    //^ ********************************** Camera constructor************************************ //
    bool Camera::set(CamerProperties type, float value)
    {
        switch (type)
        {
        case CAP_PROP_FRAMERATE_ENABLE:
        {
            //********** frame **********/

            nRet = MV_CC_SetBoolValue(args->handle, "AcquisitionFrameRateEnable", value);

            if (MV_OK == nRet)
            {
                printf("set AcquisitionFrameRateEnable OK! value=%f\n",value);
            }
            else
            {
                printf("Set AcquisitionFrameRateEnable Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_FRAMERATE:
        {
            //********** frame **********/

            nRet = MV_CC_SetFloatValue(args->handle, "AcquisitionFrameRate", value);

            if (MV_OK == nRet)
            {
                printf("set AcquisitionFrameRate OK! value=%f\n",value);
            }
            else
            {
                printf("Set AcquisitionFrameRate Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_BURSTFRAMECOUNT:
        {
            //********** frame **********/

            nRet = MV_CC_SetIntValue(args->handle, "AcquisitionBurstFrameCount", value);

            if (MV_OK == nRet)
            {
                printf("set AcquisitionBurstFrameCount OK!\n");
            }
            else
            {
                printf("Set AcquisitionBurstFrameCount Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_HEIGHT:
        {
            //********** frame **********/

            nRet = MV_CC_SetIntValue(args->handle, "Height", value); //图像高度

            if (MV_OK == nRet)
            {
                printf("set Height OK!\n");
            }
            else
            {
                printf("Set Height Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_WIDTH:
        {
            //********** frame **********/

            nRet = MV_CC_SetIntValue(args->handle, "Width", value); //图像宽度

            if (MV_OK == nRet)
            {
                printf("set Width OK!\n");
            }
            else
            {
                printf("Set Width Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_OFFSETX:
        {
            //********** frame **********/

            nRet = MV_CC_SetIntValue(args->handle, "OffsetX", value); //图像宽度

            if (MV_OK == nRet)
            {
                printf("set Offset X OK!\n");
            }
            else
            {
                printf("Set Offset X Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_OFFSETY:
        {
            //********** frame **********/

            nRet = MV_CC_SetIntValue(args->handle, "OffsetY", value); //图像宽度

            if (MV_OK == nRet)
            {
                printf("set Offset Y OK!\n");
            }
            else
            {
                printf("Set Offset Y Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_EXPOSURE_TIME:
        {
            //********** frame **********/

            nRet = MV_CC_SetFloatValue(args->handle, "ExposureTime", value); //曝光时间

            if (MV_OK == nRet)
            {
                printf("set ExposureTime OK! value=%f\n",value);
            }
            else
            {
                printf("Set ExposureTime Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_GAMMA_ENABLE:
        {
            //********** frame **********/

            nRet = MV_CC_SetBoolValue(args->handle, "GammaEnable", value); //伽马因子是否可调  默认不可调（false）

            if (MV_OK == nRet)
            {
                printf("set GammaEnable OK! value=%f\n",value);
            }
            else
            {
                printf("Set GammaEnable Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_GAMMA:
        {
            //********** frame **********/

            nRet = MV_CC_SetFloatValue(args->handle, "Gamma", value); //伽马越小 亮度越大

            if (MV_OK == nRet)
            {
                printf("set Gamma OK! value=%f\n",value);
            }
            else
            {
                printf("Set Gamma Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_GAINAUTO:
        {
            //********** frame **********/

            nRet = MV_CC_SetEnumValue(args->handle, "GainAuto", value); //亮度 越大越亮

            if (MV_OK == nRet)
            {
                printf("set GainAuto OK! value=%f\n",value);
            }
            else
            {
                printf("Set GainAuto Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_SATURATION_ENABLE:
        {
            //********** frame **********/

            nRet = MV_CC_SetBoolValue(args->handle, "SaturationEnable", value); //饱和度是否可调 默认不可调(false)

            if (MV_OK == nRet)
            {
                printf("set SaturationEnable OK! value=%f\n",value);
            }
            else
            {
                printf("Set SaturationEnable Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_SATURATION:
        {
            //********** frame **********/

            nRet = MV_CC_SetIntValue(args->handle, "Saturation", value); //饱和度 默认128 最大255

            if (MV_OK == nRet)
            {
                printf("set Saturation OK! value=%f\n",value);
            }
            else
            {
                printf("Set Saturation Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }

        case CAP_PROP_TRIGGER_MODE:
        {

            nRet = MV_CC_SetEnumValue(args->handle, "TriggerMode", value); //饱和度 默认128 最大255

            if (MV_OK == nRet)
            {
                printf("set TriggerMode OK!\n");
            }
            else
            {
                printf("Set TriggerMode Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_TRIGGER_SOURCE:
        {

            nRet = MV_CC_SetEnumValue(args->handle, "TriggerSource", value); //饱和度 默认128 最大255255

            if (MV_OK == nRet)
            {
                printf("set TriggerSource OK!\n");
            }
            else
            {
                printf("Set TriggerSource Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }
        case CAP_PROP_LINE_SELECTOR:
        {

            nRet = MV_CC_SetEnumValue(args->handle, "LineSelector", value); //饱和度 默认128 最大255

            if (MV_OK == nRet)
            {
                printf("set LineSelector OK!\n");
            }
            else
            {
                printf("Set LineSelector Failed! nRet = [%x]\n\n", nRet);
            }
            break;
        }

        default:
            return 0;
        }
        return nRet;
    }

    //^ ********************************** Camera constructor************************************ //
    bool Camera::reset()
    {
        nRet = this->set(CAP_PROP_FRAMERATE_ENABLE, FrameRateEnable);
        nRet = this->set(CAP_PROP_FRAMERATE, FrameRate) || nRet;
        // nRet = this->set(CAP_PROP_BURSTFRAMECOUNT, BurstFrameCount) || nRet;
        nRet = this->set(CAP_PROP_HEIGHT, height) || nRet;
        nRet = this->set(CAP_PROP_WIDTH, width) || nRet;
        nRet = this->set(CAP_PROP_OFFSETX, Offset_x) || nRet;
        nRet = this->set(CAP_PROP_OFFSETY, Offset_y) || nRet;
        nRet = this->set(CAP_PROP_EXPOSURE_TIME, ExposureTime) || nRet;
        nRet = this->set(CAP_PROP_GAMMA_ENABLE, GammaEnable) || nRet;
        nRet = this->set(CAP_PROP_GAMMA, Gamma) || nRet;
        nRet = this->set(CAP_PROP_GAINAUTO, GainAuto) || nRet;
        nRet = this->set(CAP_PROP_SATURATION_ENABLE, SaturationEnable) || nRet;
        nRet = this->set(CAP_PROP_SATURATION, Saturation) || nRet;
        nRet = this->set(CAP_PROP_TRIGGER_MODE, TriggerMode) || nRet;
        nRet = this->set(CAP_PROP_TRIGGER_SOURCE, TriggerSource) || nRet;
        nRet = this->set(CAP_PROP_LINE_SELECTOR, LineSelector) || nRet;
        return nRet;
    }

    //^ ********************************** PrintDeviceInfo ************************************ //
    bool Camera::PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo)
    {
        if (NULL == pstMVDevInfo)
        {
            printf("%s\n", "The Pointer of pstMVDevInfoList is NULL!");
            return false;
        }
        if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
        {
            printf("%s %x\n", "nCurrentIp:", pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp);                 //当前IP
            printf("%s %s\n\n", "chUserDefinedName:", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName); //用户定义名
        }
        else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
        {
            printf("UserDefinedName:%s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
        }
        else
        {
            printf("Not support.\n");
        }
        return true;
    }

    //^ ********************************** Camera constructor************************************ //
    void Camera::ReadImg(cv::Mat &image)
    {

        std::lock_guard<std::mutex> lock(args->mutex);
        if (args->frame_empty)
        {
            image = cv::Mat();
        }
        else
        {
            // image = (args->frame).clone();
            std::swap(args->frame, image);
            args->frame_empty = 1;
        }
    }

    void Camera::TriggerCapture(cv::Mat &image){
        nRet = MV_CC_SetCommandValue(args->handle, "TriggerSoftware");
        if (MV_OK != nRet)
        {
            printf("MV_CC_SetCommandValue fail! nRet [%x]\n", nRet);
            image = cv::Mat();
            return;
        }

        unsigned char *m_pBufForDriver = (unsigned char *)malloc(sizeof(unsigned char) * MAX_IMAGE_DATA_SIZE);
        unsigned char *m_pBufForSaveImage = (unsigned char *)malloc(MAX_IMAGE_DATA_SIZE);
        MV_FRAME_OUT_INFO_EX stImageInfo = {0};
        MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};

        nRet = MV_CC_GetOneFrameTimeout(args->handle, m_pBufForDriver, MAX_IMAGE_DATA_SIZE, &stImageInfo, 10000);
        if (nRet != MV_OK){
            printf("MV_CC_GetOneFrame times out! nRet [%x]\n", nRet);
            image = cv::Mat();
            return;
        }
        stConvertParam.nWidth = stImageInfo.nWidth;                 //ch:图像宽 | en:image width
        stConvertParam.nHeight = stImageInfo.nHeight;               //ch:图像高 | en:image height
        stConvertParam.pSrcData = m_pBufForDriver;                  //ch:输入数据缓存 | en:input data buffer
        stConvertParam.nSrcDataLen = MAX_IMAGE_DATA_SIZE;           //ch:输入数据大小 | en:input data size
        stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed; //ch:输出像素格式 | en:output pixel format                      //! 输出格式 RGB
        stConvertParam.pDstBuffer = m_pBufForSaveImage;             //ch:输出数据缓存 | en:output data buffer
        stConvertParam.nDstBufferSize = MAX_IMAGE_DATA_SIZE;        //ch:输出缓存大小 | en:output buffer size
        stConvertParam.enSrcPixelType = stImageInfo.enPixelType;    //ch:输入像素格式 | en:input pixel format                       //! 输入格式 RGB
        MV_CC_ConvertPixelType(args->handle, &stConvertParam);
        image = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, m_pBufForSaveImage);

        // cv::Mat shared_img;
        // cv::resize(image, shared_img, cv::Size(shared::sharedW, shared::sharedH));
        // pthread_mutex_lock(&(shm->shared_mutex));
        // memcpy(shm->shared_img, shared_img.data, shared::shared_img_size);
        // pthread_mutex_unlock(&(shm->shared_mutex));
    }

    std::future<void> Camera::asyncGetImage(cv::Mat &image){
        if(TriggerMode == TriggerMode_Off){
            return std::async(std::launch::async, [&]{
                do{
                    ReadImg(image);
                }while(image.empty());
            });
        }
        else{
            return std::async(std::launch::async, [&]{
                do{
                    TriggerCapture(image);
                }while(image.empty());
            });
        }
    }

    //^ ********************************** HKWorkThread1 ************************************ //
    void *Camera::HKWorkThread(void *args)
    {
        ThreadArgs *thread_args = (ThreadArgs *)args;
        void *p_handle = thread_args->handle;
        cv::Mat &frame = thread_args->frame;
        bool &frame_empty = thread_args->frame_empty;
        std::mutex &mutex = thread_args->mutex;
        
        double start;
        int nRet;
        unsigned char *m_pBufForDriver = (unsigned char *)malloc(sizeof(unsigned char) * MAX_IMAGE_DATA_SIZE);
        unsigned char *m_pBufForSaveImage;
        cudaMallocHost(&m_pBufForSaveImage, MAX_IMAGE_DATA_SIZE);
        MV_FRAME_OUT_INFO_EX stImageInfo = {0};
        MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
        cv::Mat tmp;
        int image_empty_count = 0; //空图帧数
        while (ros::ok())
        {
            nRet = MV_CC_GetOneFrameTimeout(p_handle, m_pBufForDriver, MAX_IMAGE_DATA_SIZE, &stImageInfo, 15);
            if (nRet != MV_OK)
            {
                if (++image_empty_count > 100)
                {
                    ROS_INFO("The Number of Faild Reading Exceed The Set Value!\n");
                    exit(-1);
                }
                continue;
            }
            image_empty_count = 0; //空图帧数
            //转换图像格式为BGR8

            stConvertParam.nWidth = stImageInfo.nWidth;                 //ch:图像宽 | en:image width
            stConvertParam.nHeight = stImageInfo.nHeight;               //ch:图像高 | en:image height
            stConvertParam.pSrcData = m_pBufForDriver;                  //ch:输入数据缓存 | en:input data buffer
            stConvertParam.nSrcDataLen = MAX_IMAGE_DATA_SIZE;           //ch:输入数据大小 | en:input data size
            stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed; //ch:输出像素格式 | en:output pixel format                      //! 输出格式 RGB
            stConvertParam.pDstBuffer = m_pBufForSaveImage;             //ch:输出数据缓存 | en:output data buffer
            stConvertParam.nDstBufferSize = MAX_IMAGE_DATA_SIZE;        //ch:输出缓存大小 | en:output buffer size
            stConvertParam.enSrcPixelType = stImageInfo.enPixelType;    //ch:输入像素格式 | en:input pixel format                       //! 输入格式 RGB

            std::lock_guard<std::mutex> lock(mutex);
            MV_CC_ConvertPixelType(p_handle, &stConvertParam);
            frame = cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, m_pBufForSaveImage);

            frame_empty = 0;

            // cv::Mat shared_img(shared::sharedH, shared::sharedW, CV_8UC3, shm->shared_img);
            // pthread_mutex_lock(&(shm->shared_mutex));
            // cv::resize(frame, shared_img, cv::Size(shared::sharedW, shared::sharedH));
            // // memcpy(shm->shared_img, shared_img.data, shared::shared_img_size);
            // pthread_mutex_unlock(&(shm->shared_mutex));
        }
        free(m_pBufForDriver);
        cudaFreeHost(m_pBufForSaveImage);
        return 0;
    }

} // namespace camera


// For python module
extern "C"{

    int getHeight(camera::Camera* cam){
        return cam->getHeight();
    }

    int getWidth(camera::Camera* cam){
        return cam->getWidth();
    }

    camera::Camera* createCamera(int idx){
        ros::NodeHandle nh;
        return new camera::Camera(nh, idx);
    }

    void triggerCapture(camera::Camera* cam, char *data){
        cv::Mat src;

        do{
            cam->TriggerCapture(src);
        }while(src.empty());
        std::copy(src.data, src.data + src.total() * src.elemSize(), data);
    }

    void getImage(camera::Camera* cam, char *data){
        cv::Mat src;

        do{
            cam->ReadImg(src);
        }while(src.empty());
        std::copy(src.data, src.data + src.total() * src.elemSize(), data);
    }

    void releaseCamera(camera::Camera* cam){
        delete cam;
    }
}

#endif

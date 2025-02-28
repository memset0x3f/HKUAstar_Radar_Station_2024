#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include "DataType.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <mutex>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <ostream>
#include <string>
#include <thread>
#include <vector>
#include <future>
#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <custom_msgs/yolo_point.h>
#include <custom_msgs/yolo_points.h>
#include "hikrobot_camera.hpp"
#include "TrtEngine.hpp"
#include "ros/console.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/this_node.h"
#include "ros/time.h"
#include <thread>
#include <functional>

#include "BoTSORT/BoTSORT.h"
#include "std_msgs/UInt32.h"

// 剪裁掉照片和雷达没有重合的视角，去除多余像素可以使rosbag包变小
#define FIT_LIDAR_CUT_IMAGE false
#if FIT_LIDAR_CUT_IMAGE
    #define FIT_min_x 420
    #define FIT_min_y 70
    #define FIT_max_x 2450
    #define FIT_max_y 2000
#endif 

int main(int argc, char **argv)
{
    //changed to stereo
    
    //********** variables    **********/
    std::vector<cv::Mat> src(2);
    //string src = "",image_pub = "";
    //********** rosnode init **********/
    ros::init(argc, argv, "hikrobot_camera");
    ros::NodeHandle right_camera, left_camera;
    camera::Camera MVS_left(left_camera, 0);
    camera::Camera MVS_right(right_camera, 1);

    ros::Publisher left_yolo_pub = left_camera.advertise<custom_msgs::yolo_points>("/left_yolo", 5);
    ros::Publisher right_yolo_pub = right_camera.advertise<custom_msgs::yolo_points>("/right_yolo", 5);

    //********** Image transport init **********/
    image_transport::ImageTransport right_cam_image(right_camera);
    image_transport::ImageTransport left_cam_image(left_camera);
    image_transport::CameraPublisher right_image_pub = right_cam_image.advertiseCamera("/hikrobot_camera/right/rgb", 5);
    image_transport::CameraPublisher left_image_pub = left_cam_image.advertiseCamera("/hikrobot_camera/left/rgb", 5);

    sensor_msgs::Image right_image_msg;
    sensor_msgs::Image left_image_msg;
    sensor_msgs::CameraInfo right_camera_info_msg;
    sensor_msgs::CameraInfo left_camera_info_msg;
    cv_bridge::CvImagePtr right_cv_ptr = boost::make_shared<cv_bridge::CvImage>();
    cv_bridge::CvImagePtr left_cv_ptr = boost::make_shared<cv_bridge::CvImage>();
    right_cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;  // 就是rgb格式 
    left_cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;  // 就是rgb格式

    bool calibrate_image_written = false;
    int id = 0;

    while (ros::ok())
    {
        std::chrono::steady_clock::time_point total_start = std::chrono::steady_clock::now();

        auto leftFuture = MVS_left.asyncGetImage(src[0]);
        auto rightFuture = MVS_right.asyncGetImage(src[1]);
        leftFuture.get();
        rightFuture.get();
        std::chrono::steady_clock::time_point getEnd = std::chrono::steady_clock::now();
        ROS_INFO("Time Get Image = %d [ms]", (int)std::chrono::duration_cast<std::chrono::milliseconds>(getEnd - total_start).count());
        

        // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        engine::YoloEngine::ImageBatch imgs = {src[0], src[1]};
        const int imgH = src[0].rows;
        const int imgW = src[0].cols;

        cv::Mat tmp;
        cv::resize(src[0], tmp, cv::Size(960, 540));
        cv::imshow("left_Image", tmp);
        cv::waitKey(1);

        cv::resize(src[1], tmp, cv::Size(960, 540));
        cv::imshow("right_Image", tmp);
        cv::waitKey(1);
    }

#ifdef RECORD
    std::cout << "releasing" << std::endl;
    leftVideoWriter.release();
#endif 
    return 0;
}

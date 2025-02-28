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

class HzReporter{
private:
    std::atomic<int> cnt;
    std::atomic<bool> ok;
    std::thread cnt_loop;
    ros::Publisher hz_pub;

public:
    HzReporter(ros::NodeHandle& handle):cnt(0), ok(true){
        hz_pub = handle.advertise<std_msgs::UInt32>("/hz", 5);
        cnt_loop = std::thread(std::bind(&HzReporter::loop, this));
    }

    void loop(){
        while(ok){
            // std::cout << "hz: " << cnt << std::endl;
            // ROS_INFO("hz: %d", cnt.load());
            std_msgs::UInt32 msg;
            msg.data = cnt.load();
            hz_pub.publish(msg);
            cnt = 0;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    void count(){
        cnt++;
    }

    ~HzReporter(){
        ok = false;
        cnt_loop.join();
    }
};

float fuseCarArmorConf(float carConf, float armorConf){
    return armorConf;
}

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

    //********** Define paths **********/
    std::string packageName = ros::this_node::getName().substr(1);
    std::string packagePath = ros::package::getPath(packageName);
    std::string modelPath = packagePath + "/models/";
    std::string configPathRoot = packagePath + "/config/";

    //*********** Recording **************/
#ifdef RECORD
    const int saveH = 1080, saveW = 1920;
    std::string recordingPath = packagePath + "/recording/";
    cv::VideoWriter leftVideoWriter, rightVideoWriter;
    //named by time 
    std::time_t t = std::time(0);
    std::tm* now = std::localtime(&t);
    int cc = cv::VideoWriter::fourcc('m','p','4','v'), fps = 25;
    std::string time = std::to_string(now->tm_year + 1900) + "-" + std::to_string(now->tm_mon + 1) + "-" + std::to_string(now->tm_mday) + "-" + std::to_string(now->tm_hour) + "-" + std::to_string(now->tm_min) + "-" + std::to_string(now->tm_sec);
    leftVideoWriter.open(recordingPath + "left_" + time + ".mp4", cc, fps, cv::Size(saveW, saveH));
    rightVideoWriter.open(recordingPath + "right_" + time + ".mp4", cc, fps, cv::Size(saveW, saveH));
    
#endif

    //********** Load yolo model **********/
    engine::YoloEngine car_model(0.2, 0.4), armor_model(0.8, 0.4);
    engine::ReidEngine reid_model;
    //model updating
    //car_model.build(modelPath + "car_1088_1920_b2.engine");
    car_model.build(modelPath + "radar_new_detection.engine");
    armor_model.build(modelPath + "armor_best.engine");
    reid_model.build(modelPath + "reid_b10.engine");
    car_model.warmup();
    armor_model.warmup();
    reid_model.warmup();

    //********** Load BoTSORT Tracker **********/
    std::vector<std::unique_ptr<BoTSORT>> trackers(2);
    trackers[0] = std::make_unique<BoTSORT>(
        configPathRoot + "tracker.ini",
        configPathRoot + "gmc.ini",
        configPathRoot + "reid.ini",
        modelPath + "radar2.onnx"
    );

    trackers[1] = std::make_unique<BoTSORT>(
        configPathRoot + "tracker.ini",
        configPathRoot + "gmc.ini",
        configPathRoot + "reid.ini",
        modelPath + "radar2.onnx"
    );


    bool calibrate_image_written = false;
    int id = 0;
    HzReporter hzReporter(left_camera);
    

    while (ros::ok())
    {
        hzReporter.count();
        std::chrono::steady_clock::time_point total_start = std::chrono::steady_clock::now();

        auto leftFuture = MVS_left.asyncGetImage(src[0]);
        auto rightFuture = MVS_right.asyncGetImage(src[1]);
        leftFuture.get();
        rightFuture.get();
        std::chrono::steady_clock::time_point getEnd = std::chrono::steady_clock::now();
        ROS_INFO("Time Get Image = %d [ms]", (int)std::chrono::duration_cast<std::chrono::milliseconds>(getEnd - total_start).count());
        


        // calibrate extrinsic
        // cv::imwrite("/home/inno2/livox_calib/left.png", src[0]);
        // cv::imwrite("/home/inno2/livox_calib/right.png", src[1]);
        // return 0;

        // write image for calibration
        // if(!calibrate_image_written) {
        //     cv::imwrite("/home/inno2/ws_radar/src/calibration_image_left.jpg", src[0]);
        //     cv::imwrite("/home/inno2/ws_radar/src/calibration_image_right.jpg", src[1]);
        //     calibrate_image_written = true;
        // }

        // std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        engine::YoloEngine::ImageBatch imgs = {src[0], src[1]};
        const int imgH = src[0].rows;
        const int imgW = src[0].cols;

        std::chrono::steady_clock::time_point car_start = std::chrono::steady_clock::now();
        /************* Car Inference *********************/
        std::vector<std::vector<engine::BBox>> cars_batch;
        car_model.infer(imgs, cars_batch);
 

        int offset = cars_batch[0].size();
        engine::YoloEngine::ImageBatch car_imgs;

        for(int i = 0; i < imgs.size(); ++i){
            if(cars_batch[i].empty()){
                continue;
            }
            // Crop cars for armor detection
            for (auto car : cars_batch[i]){
                int x1 = std::max(0, car.x1);
                int y1 = std::max(0, car.y1);
                int x2 = std::min(imgs[i].cols - 1, car.x2);
                int y2 = std::min(imgs[i].rows - 1, car.y2);
                cv::Rect roi(x1, y1, x2 - x1, y2 - y1);
                car_imgs.emplace_back(imgs[i](roi).clone());
            }
        }
        std::chrono::steady_clock::time_point car_end = std::chrono::steady_clock::now();  
        ROS_INFO("Time Car Inference = %d [ms]", (int)std::chrono::duration_cast<std::chrono::milliseconds>(car_end - car_start).count());


        std::chrono::steady_clock::time_point armor_start = std::chrono::steady_clock::now();
        /************* Armor Inference *********************/
        std::vector<std::vector<engine::BBox>> armors_batch;
        std::vector<std::vector<Detection>> detections(2);
        armor_model.infer(car_imgs, armors_batch);
        for(int p = 0; p < armors_batch.size(); ++p){
            int i = p >= offset ? 1 : 0;
            int j = p >= offset ? p - offset : p;
            
            auto &armors = armors_batch[p];
            auto &car = cars_batch[i][j];
            Detection det;
            int x1 = std::max(0, car.x1);
            int y1 = std::max(0, car.y1);
            int x2 = std::min(imgs[i].cols - 1, car.x2);
            int y2 = std::min(imgs[i].rows - 1, car.y2);
            det.bbox_tlwh.x = 1.0 * x1;
            det.bbox_tlwh.y = 1.0 * y1;
            det.bbox_tlwh.width = 1.0 * (x2 - x1);
            det.bbox_tlwh.height = 1.0 * (y2 - y1);

            if(armors.empty()){         // Unknow id
                car.classId = 255;
                det.class_id = car.classId;
                det.confidence = car.score;
            }
            else{
                auto argmax = std::max_element(armors.begin(), armors.end(), 
                    [](const engine::BBox& a, const engine::BBox& b){
                        return a.score < b.score;
                    }
                );
                car.classId = argmax->classId;

                det.class_id = car.classId;
                det.confidence = fuseCarArmorConf(car.score, argmax->score);
            }
            detections[i].push_back(det);
            ROS_INFO("Car ID: %d, Conf: %f", det.class_id, det.confidence);
        }
        std::chrono::steady_clock::time_point armor_end = std::chrono::steady_clock::now();
        ROS_INFO("Time Armor Inference = %d [ms]", (int)std::chrono::duration_cast<std::chrono::milliseconds>(armor_end - armor_start).count());


        std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
        /************* Tracking *********************/
        std::vector<engine::ReidEngine::FeatureTensor> features;
        reid_model.infer(car_imgs, features);
        std::vector<std::vector<engine::ReidEngine::FeatureTensor>> features_batch = {
            std::vector<engine::ReidEngine::FeatureTensor> (features.begin(), features.begin() + offset),
            std::vector<engine::ReidEngine::FeatureTensor> (features.begin() + offset, features.end())
        };
        for(int i = 0; i < imgs.size(); i++){
            if(detections[i].empty()){
                custom_msgs::yolo_points yolo_points;
                if(i == 0) left_yolo_pub.publish(yolo_points);
                else right_yolo_pub.publish(yolo_points);
            }
            else{
                custom_msgs::yolo_points yolo_points;
                std::vector<std::shared_ptr<Track>> tracks = trackers[i]->track(detections[i], src[i], features_batch[i]);

                if(i == 0) ROS_INFO("Tracks: %d", static_cast<int>(tracks.size()));

                for(auto &track : tracks){
                    custom_msgs::yolo_point point;
                    auto tlwh = track->get_tlwh();
                    point.width = static_cast<int>(tlwh[2]);
                    point.height = static_cast<int>(tlwh[3]);
                    point.x = static_cast<int>(tlwh[0] + tlwh[2]/2);
                    point.y = static_cast<int>(tlwh[1] + tlwh[3]/2);
                    if(track->class_id == 255){
                        point.color = point.id = 255;
                    }
                    else{
                        point.color = track->class_id / 6;
                        point.id = track->class_id % 6 + 1;
                    }
                    yolo_points.data.push_back(point);
                    ROS_INFO("Tracked Car: %d", track->class_id);

#ifdef DEBUG
                    // Debug
                    int x1 = static_cast<int>(tlwh[0]);
                    int y1 = static_cast<int>(tlwh[1]);
                    int x2 = x1 + static_cast<int>(tlwh[2]);
                    int y2 = y1 + static_cast<int>(tlwh[3]);
                    cv::rectangle(src[i], cv::Rect(x1, y1, x2 - x1, y2 - y1), cv::Scalar(0, 255, 0), 3, cv::LINE_8);
                    cv::putText(src[i], std::to_string(point.id), cv::Point(x1, y1-10), 
                        cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 255, 0), 2);
                    cv::putText(src[i], point.color == 0?"BLUE":"RED", cv::Point(x1+60, y1-10), cv::FONT_HERSHEY_SIMPLEX, 2, cv::Scalar(0, 255, 0), 2);
#endif            
                }
                if(i == 0) left_yolo_pub.publish(yolo_points);
                else right_yolo_pub.publish(yolo_points);

            }
        }
        std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        ROS_INFO("Time Tracking = %d [ms]", (int)std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count());

        std::chrono::steady_clock::time_point total_end = std::chrono::steady_clock::now();
        ROS_INFO("Time Total = %d [ms]\n", (int)std::chrono::duration_cast<std::chrono::milliseconds>(total_end - total_start).count());



#ifdef RECORD
        //check if recording
        if (!leftVideoWriter.isOpened())
        {
            ROS_ERROR("Error: VideoWriter not opened");
        }
        
        cv::Mat save_left, save_right;
        cv::resize(src[0], save_left, cv::Size(saveW,  saveH));
        cv::resize(src[0], save_right, cv::Size(saveW,  saveH));
        leftVideoWriter.write(save_left);
        rightVideoWriter.write(save_right);
#endif

#ifdef DEBUG
        // Debug
        cv::Mat tmp;
        cv::resize(src[0], tmp, cv::Size(960, 540));
        cv::imshow("left_Image", tmp);
        cv::waitKey(1);

        cv::resize(src[1], tmp, cv::Size(960, 540));
        cv::imshow("right_Image", tmp);
        cv::waitKey(1);
#endif 


        // display the image src[1] and show the rectangles

        // for(auto point: yolo_points.data) {
        //     cv::rectangle(src[1], cv::Rect(point.x - point.width / 2, point.y - point.height / 2, point.width, point.height), cv::Scalar(0, 255, 0), 3, cv::LINE_8);
        // }
        // cv::resize(img, img, cv::Size(1280, 720));
        // cout << img;
        // cv::imshow("Image", img);
        
        // publish image

        // cv::resize(src[1], src[1], cv::Size(1635, 1024));
        // right_cv_ptr->image = src[1];
        // right_image_msg = *(right_cv_ptr->toImageMsg());
        // right_image_msg.header.stamp = ros::Time::now();
        // right_image_msg.header.frame_id = "map";
        // right_camera_info_msg.header.stamp = right_image_msg.header.stamp;
        // right_camera_info_msg.header.frame_id = right_image_msg.header.frame_id;

        // cv::resize(src[0], src[0], cv::Size(1635, 1024));
        // left_cv_ptr->image = src[0];
        // left_image_msg = *(left_cv_ptr->toImageMsg());
        // left_image_msg.header.stamp = ros::Time::now();
        // left_image_msg.header.frame_id = "map";
        // left_camera_info_msg.header.stamp = left_image_msg.header.stamp;
        // left_camera_info_msg.header.frame_id = left_image_msg.header.frame_id;

        // left_image_pub.publish(left_image_msg, left_camera_info_msg);
        // right_image_pub.publish(right_image_msg, right_camera_info_msg);
    }

#ifdef RECORD
    std::cout << "releasing" << std::endl;
    leftVideoWriter.release();
#endif 
    return 0;
}

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "gui_pkg/qnode.hpp"
#include "sensor_msgs/image_encodings.h"

namespace gui_pkg {

QNode::QNode(int argc, char** argv) : init_argc(argc),init_argv(argv) {}

QNode::~QNode() {
    if(ros::isStarted()) {
        ros::shutdown();
        ros::waitForShutdown();
    }
    wait();
}

// void QNode::imgLeftCallback(const sensor_msgs::ImageConstPtr &msg) {
//     if(cameraCalibrating == leftImgRaw && isCalibrating) {
//         try {
//             cv_bridge::CvImageConstPtr cvPtr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
//             if(!cvPtr->image.empty()) {
                // imgLeft = cvPtr->image;
//                 cv::resize(imgLeft, imgLeft, cv::Size(calibrateMainWindowWidth, calibrateMainWindowLength));

//                 imageCalibrateMainWindow = QImage(imgLeft.data,
//                                                   imgLeft.cols, imgLeft.rows,
//                                                   imgLeft.step[0],
//                                                   QImage::Format_RGB888);

//                 Q_EMIT loggingCameraCalibrateMainWindow();
//                 Q_EMIT loggingCameraCalibrateSecondWindow();
//             }
//         } catch(cv_bridge::Exception &e) {
//             ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
//         }       
//     }
// }

// void QNode::imgRightCallback(const sensor_msgs::ImageConstPtr &msg) {
//     if(cameraCalibrating == rightImgRaw && isCalibrating) {
//         try {
//             cv_bridge::CvImageConstPtr cvPtr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
//             if(!cvPtr->image.empty()) {
//                 imgRight = cvPtr->image;
//                 cv::resize(imgRight, imgRight, cv::Size(calibrateMainWindowWidth, calibrateMainWindowLength));

//                 imageCalibrateMainWindow = QImage(imgRight.data,
//                                                   imgRight.cols, imgRight.rows,
//                                                   imgRight.step[0],
//                                                   QImage::Format_RGB888);

//                 Q_EMIT loggingCameraCalibrateMainWindow();
//                 Q_EMIT loggingCameraCalibrateSecondWindow();
//             }
//         } catch(cv_bridge::Exception &e) {
//             ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
//         }
//     }
// }

void QNode::leftDistCallback(const custom_msgs::dis_points &msg) {
    std::vector<custom_msgs::dis_point>().swap(leftDists);
    for (int i = 0; i < msg.data.size(); i++) {
        leftDists.push_back(msg.data[i]);
    }
}

void QNode::rightDistCallback(const custom_msgs::dis_points &msg) {
    std::vector<custom_msgs::dis_point>().swap(rightDists);
    for (int i = 0; i < msg.data.size(); i++) {
        rightDists.push_back(msg.data[i]);
    }
}

void QNode::robotPointCallback(const custom_msgs::points &msg) {
    RobotPoint p;
    std::vector<RobotPoint>().swap(robotPoints);

    for(size_t i = 0; i < msg.data.size(); i++) {
        p.point = QPoint(msg.data[i].x * uimapWidth, (1 - msg.data[i].y) * uimapLength);
        p.id = msg.data[i].id;
        robotPoints.push_back(p);
    }
    Q_EMIT loggingUimapUpdate();
}

void QNode::pubCalibration() {
    custom_msgs::point pointMsg;
    custom_msgs::points pointsMsg;
    if(cameraCalibrating == leftImgRaw) {
        for(int i = 0; i < 4; i++) {
            printf("leftPoints[%d]: %d %d\n",i , leftPoints[i].x(), leftPoints[i].y());
            pointMsg.id = i;
            pointMsg.x = leftPoints[i].x() * 1.0 / calibrateMainWindowWidth;
            pointMsg.y = leftPoints[i].y() * 1.0 / calibrateMainWindowLength;

            pointsMsg.data.push_back(pointMsg);
        }
        leftCalibrationPub.publish(pointsMsg);
    } else if(cameraCalibrating == rightImgRaw) {
         for(int i = 0; i < 4; i++) {
            printf("rightPoints[%d]: %d %d\n",i , rightPoints[i].x(), rightPoints[i].y());
            pointMsg.id = i;
            pointMsg.x = rightPoints[i].x() * 1.0 / calibrateMainWindowWidth;
            pointMsg.y = rightPoints[i].y() * 1.0 / calibrateMainWindowLength;

            pointsMsg.data.push_back(pointMsg);
        }
        rightCalibrationPub.publish(pointsMsg);       
    }
}

bool QNode::init() {
    ros::init(init_argc, init_argv, "gui_pkg");
    
    if(!ros::master::check()) {
        return false;
    }
    
    loadParams();
    ros::start();
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    
    // imageLeftSub = it.subscribe(leftImgRaw.toStdString(), 1, &QNode::imgLeftCallback, this);
    // imageRightSub = it.subscribe(rightImgRaw.toStdString(), 1, &QNode::imgRightCallback, this);
    // imageCalibrateMainWindow = QImage(imgLeft.data,
    //                                     imgLeft.cols, imgLeft.rows,
    //                                     imgLeft.step[0],
    //                                     QImage::Format_RGB888);

    robotPointsSub = nh.subscribe("/robot", 1, &QNode::robotPointCallback, this);
    leftCalibrationPub = nh.advertise<custom_msgs::points>(leftCalibrationTopic, 1);
    rightCalibrationPub = nh.advertise<custom_msgs::points>(rightCalibrationTopic, 1);

    leftDistSub = nh.subscribe(leftDistTopic, 1, &QNode::leftDistCallback, this);
    rightDistSub = nh.subscribe(rightDistTopic, 1, &QNode::rightDistCallback, this);
    start();
    return true;
}

void QNode::run() {
    //log(DEBUG, std::string("start running!"));
    ros::spin();
    std::cout << "ros shutting down" << std::endl;
    Q_EMIT rosShutdown();
}

void QNode::loadParams() {// load default values then read from yaml file
    std::string str;
    leftImgRaw = "/home/inno2/ws_radar/src/calibration_image_left.jpg";
    ros::param::get("/GUI/topic/left_img_raw", str);
    leftImgRaw = QString(str.c_str());

    rightImgRaw = "/home/inno2/ws_radar/src/calibration_image_right.jpg";
    ros::param::get("/GUI/topic/right_img_raw", str);
    rightImgRaw = QString(str.c_str());

    // centerImgRaw = "/hikrobot_camera/left/rgb"; //unknown yet
    // ros::param::get("/GUI/topic/center_img_raw", str);
    // centerImgRaw = QString(str.c_str());

    imgLeft = cv::imread(leftImgRaw.toStdString());
    cv::cvtColor(imgLeft, imgLeft, CV_BGR2RGB);
    imgRight = cv::imread(rightImgRaw.toStdString());
    cv::cvtColor(imgRight, imgRight, CV_BGR2RGB);

    leftCalibrationTopic = "/left/calibration";
    ros::param::get("/GUI/topic/left_calibration_topic", leftCalibrationTopic);
    rightCalibrationTopic = "/right/calibration";
    ros::param::get("/GUI/topic/right_calibration_topic", rightCalibrationTopic);

    calibrateRate = 3;
    ros::param::get("/GUI/calibrate_rate", calibrateRate);

    rawImageLength =2048;
    ros::param::get("/GUI/raw_image_length", rawImageLength);
    rawImageWidth = 3270;
    ros::param::get("/GUI/raw_image_width", rawImageWidth);

    battleColor = "red";
    ros::param::get("/battle_color", battleColor);

    uimapWidth = 360;
    uimapLength = 672;

    std::string ad(PROJECT_PATH);
    if(battleColor == std::string("red")) {
        ad += "/resources/images/red_minimap.png";
    }
    else {
        ad += "/resources/images/blue_minimap.png";
    }
    imgUimap = cv::imread(ad);
    cv::cvtColor(imgUimap, imgUimap, CV_BGR2RGB);
    cv::resize(imgUimap, imgUimap, cv::Size(uimapWidth, uimapLength));
    imageUimap = QImage(imgUimap.data, imgUimap.cols, imgUimap.rows, imgUimap.step[0], QImage::Format_RGB888);
    logoLength = 222;
    logoWidth = 222;
    ad = std::string(PROJECT_PATH);
    ad += "/resources/images/icon.png";
    imgLogo = cv::imread(ad);
    cv::cvtColor(imgLogo, imgLogo, CV_BGR2RGB);
    imageLogo.load(ad.c_str());
    imageLogo = imageLogo.scaled(logoWidth, logoLength, Qt::KeepAspectRatio);

    calibrateMainWindowWidth = 1227;
    calibrateMainWindowLength = 768;
    calibrateSecondWindowWidth = 618;
    calibrateSecondWindowLength = 618;

    cv::resize(imgLeft, imgLeft, cv::Size(calibrateMainWindowWidth, calibrateMainWindowLength));
    cv::resize(imgRight, imgRight, cv::Size(calibrateMainWindowWidth, calibrateMainWindowLength));

    // imgMainWindow = cv::Mat(displayMainWindowLength, displayMainWindowWidth, CV_8UC3, cv::Scalar(255, 255, 255));
    // imgLeft = cv::Mat(displayMainWindowLength, displayMainWindowWidth, CV_8UC3, cv::Scalar(255, 255, 255));
    // imgRight = cv::Mat(displayMainWindowLength, displayMainWindowWidth, CV_8UC3, cv::Scalar(255, 255, 255));
    // imgSecondWindow = cv::Mat(displaySecondWindowLength, displaySecondWindowWidth, CV_8UC3, cv::Scalar(255, 255, 255));

    isCalibrating = true;
    ros::param::get("/GUI/is_calibrating", isCalibrating);

    cameraCalibrating = "/home/inno2/ws_radar/src/calibration_image_left.jpg";
    ros::param::get("/GUI/camera_calibrating", str);
    cameraCalibrating = QString(str.c_str());

    float x = 0, y = 0;
    QPoint point;
    // read selected points from the param server
    for (int i = 0; i < 4; i++) {
        std::string xtopic = "/GUI/left_cam/calibration/point" + std::to_string(i) + "/x";
        std::string ytopic = "/GUI/left_cam/calibration/point" + std::to_string(i) + "/y";
        ros::param::get(xtopic, x);
        ros::param::get(ytopic, y);
        leftPoints[i] = QPoint(x * calibrateMainWindowWidth, y * calibrateMainWindowLength);
        
        xtopic = "/GUI/right_cam/calibration/point" + std::to_string(i) + "/x";
        ytopic = "/GUI/right_cam/calibration/point" + std::to_string(i) + "/y";       
        ros::param::get(xtopic, x);
        ros::param::get(ytopic, y);
        rightPoints[i] = QPoint(x * calibrateMainWindowWidth, y * calibrateMainWindowLength);
    }

    leftDistTopic = "/left/dis_points";
    rightDistTopic = "/right/dis_points";

    ifBeginToRecord = false;
    ifBeginToReplay = false;
    ifReplayDone = false;
    ifRecordDone = false;

    Q_EMIT loggingCameraCalibrateMainWindow();
    Q_EMIT loggingCameraCalibrateSecondWindow();
}

}
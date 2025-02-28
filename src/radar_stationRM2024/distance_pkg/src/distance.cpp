#include "opencv2/core/hal/interface.h"
#include <iostream>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/PointCloud2.h>
//include any additional custom msg types
#include <custom_msgs/yolo_points.h>
#include <custom_msgs/dis_point.h>
#include <custom_msgs/dis_points.h>
#include <custom_msgs/points.h>
using namespace std;
using namespace cv;
uint16_t times = 0;
vector<int> cnt;
vector<float> dists;
int img_width = 3270, img_height = 2048;
int queue_length = 5;

ros::Publisher left_distancePub;
ros::Publisher right_distancePub;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
vector<Mat> left_dep_queue;
vector<Mat> left_img2pc_queue;
Mat left_depths = Mat::zeros(img_height, img_width, CV_32FC1);
Mat left_img2pc = Mat_<Vec3f>(img_height, img_width, CV_32FC3);
Mat left_intrinsic = Mat_<float>(3, 3);
Mat left_extrinsic = Mat_<float>(3, 4);
Mat left_distCoeffs = Mat_<float>(5, 1);
vector<float> left_i;
vector<float> left_e;
vector<float> left_d;

vector<Mat> right_dep_queue;
vector<Mat> right_img2pc_queue;
Mat right_depths = Mat::zeros(img_height, img_width, CV_32FC1);
Mat right_img2pc = Mat_<Vec3f>(img_height, img_width, CV_32FC3);
Mat right_intrinsic = Mat_<float>(3, 3);
Mat right_extrinsic = Mat_<float>(3, 4);
Mat right_distCoeffs = Mat_<float>(5, 1);
vector<float> right_i;
vector<float> right_e;
vector<float> right_d;


custom_msgs::dis_points left_dis_it;
custom_msgs::dis_points right_dis_it;
custom_msgs::dis_points last_left_dis_it;
custom_msgs::dis_points last_right_dis_it;

struct DistNode {
    float depth, euDis;
    bool operator<(const DistNode &_)const {
        return depth < _.depth;
    }
};

float calcDis(Vec3f a, Vec3f b) {
    return sqrt(pow(a[0] - b[0], 2) + pow(a[1] - b[1], 2) + pow(a[2] -b[2], 2));
}

void depthOfRect(Rect rect, vector<Mat> &dep_queue, vector<Mat> &img2pc_queue, custom_msgs::yolo_point::_id_type id, float &depth, float& euDis) {
    vector<float> Dis;
    vector<Vec3f> depth2pc;
    for (int i = max(0, int(rect.y - 0.3 * rect.height)); i < min(int(rect.y + 0.3 * rect.height), img_height); i++) {
        for (int j = max(0, int(rect.x - 0.3 * rect.width)); j < min(int(rect.x + 0.3 * rect.width), img_width); j++) {
            for(uint8_t k = 0; k < dep_queue.size(); k++) {
                if (dep_queue[dep_queue.size() - 1].at<float>(i, j) > 0) {
                    Dis.push_back(dep_queue[dep_queue.size() - 1].at<float>(i, j));
                    depth2pc.push_back(img2pc_queue[img2pc_queue.size() - 1].at<Vec3f>(i, j));
                    break;
                }
                else if (k < dep_queue.size() - 1 && dep_queue[k + 1].at<float>(i, j) == 0 && dep_queue[k].at<float>(i, j) > 0) {
                    Dis.push_back(dep_queue[k].at<float>(i, j));
                    depth2pc.push_back(img2pc_queue[k].at<Vec3f>(i, j));
                    break;
                }
            }
        }
    }

    if(Dis.size() < 3) {
        cout << "No PC points in ROI!, Dis.size() = " << Dis.size() << ", rect is: " << rect << endl;
        depth = euDis = 0;
        return;
    }
    else {
        // kmeans
        Mat points(depth2pc.size(), 1, CV_32FC3);
        for(int i = 0; i < depth2pc.size(); i++) {
            points.at<Vec3f>(i, 0) = depth2pc[i];
        }
        int numClusters = 3;
        Mat labels, centers;
        kmeans(points, numClusters, labels, 
               cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 100, 0.1),
               3, cv::KMEANS_PP_CENTERS, centers);
        DistNode dnode[3] = {(DistNode){0, 0}, (DistNode){0, 0}, (DistNode){0, 0}};
        // float mean[3] = {0, 0, 0};
        // float euclideanDis[3] = {0, 0, 0};
        float eudis_sum = 0;
        int num[3] = {0, 0, 0};
        for(int i = 0; i < depth2pc.size(); i++) {
            dnode[labels.at<int>(i, 0)].depth += Dis[i];
            dnode[labels.at<int>(i, 0)].euDis += calcDis(Vec3f(0, 0, 0), points.at<Vec3f>(i, 0));
            eudis_sum += calcDis(Vec3f(0, 0, 0), points.at<Vec3f>(i, 0));
            // mean[labels.at<int>(i, 0)] += Dis[i];
            // euclideanDis[labels.at<int>(i, 0)] += calcDis(Vec3f(0, 0, 0), points.at<Vec3f>(i, 0));
            num[labels.at<int>(i, 0)]++;
            // cout << "point:" << depth2pc[i] << " label:" << labels.at<int>(i, 0) << endl;
        }


        for(int i = 0; i < 3; i++){
            if(num[i]) {
                dnode[i].depth /= num[i];
                dnode[i].euDis /= num[i];
            }
        }
        eudis_sum /= depth2pc.size();

        sort(dnode, dnode + 3);

        for(int i = 0 ; i < 3; i++) cout << "depth2pc_size: " << depth2pc.size() << " num: " << num[i] << " mean:" << dnode[i].depth << " " << "euclidean: " << dnode[i].euDis << endl;
        cout << endl;
        
        depth = dnode[1].depth;
        euDis = dnode[1].euDis;

        // calculate mean
        // float mean = 0;
        // for(float dis: Dis) {
        //     mean += dis;
        // }
        // mean /= Dis.size();
        // return mean;
    }
}

Mat PointCloud2Mat(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc) {
    // printf("%d\n", (int)pc->size());
    Mat ret = Mat::zeros(4, (int) pc->size(), CV_32F);
    for(int k = 0; k < ret.cols; k++) {
        for(int j = 0; j < 4; j++) {
            ret.at<float>(j, k) = pc->points[k].data[j];
        }
    }
    return ret;
}

void Mat2Img(Mat &input_depth, Mat &img2pc, Mat &input_pc, Mat &intrinsic, Mat &extrinsic, vector<Vec2i> &tmp) {
    Mat res = intrinsic * extrinsic * input_pc;
    for (int i = 0; i < res.cols; i++) {
        
        int x = round(res.at<float>(0, i) / res.at<float>(2, i));
        int y = round(res.at<float>(1, i) / res.at<float>(2, i));
        if (x >= 0 && x < img_width && y >= 0 && y < img_height) {
            input_depth.at<float>(y, x) = res.at<float>(2, i);
            img2pc.at<Vec3f>(y, x) = Vec3f(input_pc.at<float>(0, i),
                                                  input_pc.at<float>(1, i),
                                                  input_pc.at<float>(2, i));
            tmp.push_back(Vec2d(y, x));
        }
    }
}

void clearMat(Mat &input_depth, Mat &img2pc, vector<Vec2i> tmp) {
    for (int i = 0; i < tmp.size(); i++) {
        input_depth.at<float>(tmp[i][0], tmp[i][1]) = 0;
        img2pc.at<Vec3f>(tmp[i][0], tmp[i][1]) = Vec3f(0, 0, 0);
    }
}
// pcl::PointCloud<pcl::PointXYZ>::Ptr existing_cloud(new pcl::PointCloud<pcl::PointXYZ>);
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    // *existing_cloud += *cloud;
    // pcl::io::savePCDFileBinary("/home/inno2/livox_calib/pointcloud.pcd", *existing_cloud);
}

void left_yoloCallback(const custom_msgs::yolo_points::ConstPtr &msg)
{
    // start = std::chrono::high_resolution_clock::now();
    // auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(start - end_);
    // ROS_ERROR_STREAM("Time spent since last callback is " << elapsed.count());
    //clear the vector
    vector<custom_msgs::dis_point>().swap(left_dis_it.data);
    vector<Vec2i> tmp;
    Mat left_CloudMat;
    if(cloud) {
        left_CloudMat = PointCloud2Mat(cloud);
        Mat2Img(left_depths, left_img2pc, left_CloudMat, left_intrinsic, left_extrinsic, tmp);
        left_dep_queue.push_back(left_depths);
        left_img2pc_queue.push_back(left_img2pc);
        //pop front
        if(left_dep_queue.size() == queue_length) {
            left_dep_queue.erase(left_dep_queue.begin());
            left_img2pc_queue.erase(left_img2pc_queue.begin());
        }
    }
    if((*msg).text != "none") {
        for(int i = 0; i < (*msg).data.size(); i++) {
            custom_msgs::dis_point pit;
            pit.x = (*msg).data[i].x;
            pit.y = (*msg).data[i].y;
            depthOfRect(Rect((*msg).data[i].x, (*msg).data[i].y, (*msg).data[i].width, (*msg).data[i].height),
                        left_dep_queue, left_img2pc_queue,
                         (*msg).data[i].id, pit.dist, pit.euDis);
            pit.id = (*msg).data[i].id;
            pit.width = (*msg).data[i].width;
            pit.height = (*msg).data[i].height;
            pit.color = (*msg).data[i].color;
            if(pit.dist != 0)left_dis_it.data.push_back(pit);
            
        }
    }
    //clear mat
    if(cloud) {
        clearMat(left_depths, left_img2pc, tmp);
    }
    left_distancePub.publish(left_dis_it);
    // end_ = std::chrono::high_resolution_clock::now();
    // elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end_ - start);
    // ROS_ERROR_STREAM("Time spent running left_yolocallback is " << elapsed.count());
}


void right_yoloCallback(const custom_msgs::yolo_points::ConstPtr &msg) {
    //clear the vector
    vector<custom_msgs::dis_point>().swap(right_dis_it.data);
    vector<Vec2i>tmp;
    Mat right_CloudMat;
    if(cloud) {
        right_CloudMat = PointCloud2Mat(cloud);
        Mat2Img(right_depths, right_img2pc, right_CloudMat, right_intrinsic, right_extrinsic, tmp);
        right_dep_queue.push_back(right_depths);
        right_img2pc_queue.push_back(right_img2pc);
        //pop front
        if(right_dep_queue.size() == queue_length) {
            right_dep_queue.erase(right_dep_queue.begin());
            right_img2pc_queue.erase(right_img2pc_queue.begin());
        }
    }
    if((*msg).text != "none") {
        cout << (*msg).text << endl;
        for(int i = 0; i < (*msg).data.size(); i++) {
            custom_msgs::dis_point pit;
            pit.x = (*msg).data[i].x;
            pit.y = (*msg).data[i].y;
            depthOfRect(Rect((*msg).data[i].x, (*msg).data[i].y, (*msg).data[i].width, (*msg).data[i].height),
                        right_dep_queue, right_img2pc_queue,
                        (*msg).data[i].id, pit.dist, pit.euDis);
            pit.id = (*msg).data[i].id;
            pit.width = (*msg).data[i].width;
            pit.height = (*msg).data[i].height; 
            pit.color = (*msg).data[i].color;  
            if(pit.dist != 0)right_dis_it.data.push_back(pit);
       }
    }
    if(cloud) {
        clearMat(right_depths, right_img2pc, tmp);
    }
    right_distancePub.publish(right_dis_it);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "get_depth_node");
    ros::NodeHandle nh;
    // initialize parameters;
    ros::param::get("/left_cam/intrinsic_matrix", left_i);
    for(int i = 0; i < 9; i++) left_intrinsic.at<float>(i / 3, i % 3) = left_i[i];
    ros::param::get("/left_cam/extrinsic_matrix", left_e);
    for(int i = 0; i < 12; i++) left_extrinsic.at<float>(i / 4, i % 4) = left_e[i];
    ros::param::get("/left_cam/distortion_coefficients", left_d);
    for(int i = 0; i < 5; i++) left_distCoeffs.at<float>(i, 0) = left_d[i];

    // cout << left_intrinsic << "\n" << left_extrinsic << "\n" << left_distCoeffs << endl;

    ros::param::get("/right_cam/intrinsic_matrix", right_i);
    for(int i = 0; i < 9; i++) right_intrinsic.at<float>(i / 3, i % 3) = right_i[i];
    ros::param::get("/right_cam/extrinsic_matrix", right_e);
    for(int i = 0; i < 12; i++) right_extrinsic.at<float>(i / 4, i % 4) = right_e[i];
    ros::param::get("/right_cam/distortion_coefficients", right_d);
    for(int i = 0; i < 5; i++) right_distCoeffs.at<float>(i, 0) = right_d[i];

    //Kalman_init();
    
    ros::Subscriber cloud_sub;
    ros::Subscriber left_yolo_sub;
    ros::Subscriber right_yolo_sub;
    cloud_sub = nh.subscribe("/livox/lidar", 1, &pointCloudCallback);
    left_yolo_sub = nh.subscribe("/left_yolo", 1, &left_yoloCallback);
    right_yolo_sub = nh.subscribe("/right_yolo", 1, &right_yoloCallback);
    

    left_distancePub = nh.advertise<custom_msgs::dis_points>("/left/dis_points", 5);
    right_distancePub = nh.advertise<custom_msgs::dis_points>("/right/dis_points", 5);
    
    ros::Rate loop_rate(60);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

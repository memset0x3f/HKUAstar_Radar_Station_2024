#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <custom_msgs/points.h>
#include <custom_msgs/point.h>
#include <custom_msgs/dis_points.h>
#include <bitset>
//#include <opencv2/calib3d.hpp>

using namespace std;
using namespace cv;

ros::Publisher heroPub;
ros::Publisher sentryPub;
ros::Publisher robotPub;

int map_length = 15, map_width = 28; // in meters
double imgCols = 3270.0, imgRows = 2048.0;
Mat img;
Mat left_intrinsic = Mat_<double>(3, 3);
Mat left_distCoeffs = Mat_<double>(5, 1);
Mat left_R_jacob = Mat_<double>(3, 1);
Mat left_R = Mat_<double>(3, 3);
Mat left_T = Mat_<double>(3, 1);
Mat right_intrinsic = Mat_<double>(3, 3);
Mat right_distCoeffs = Mat_<double>(5, 1);
Mat right_R_jacob = Mat_<double>(3, 1);
Mat right_R = Mat_<double>(3, 3);
Mat right_T = Mat_<double>(3, 1);

Point2f outpost2d;
Point3f outpost3d;
Point3f sentry;
Point3f hero;
Point3f enemyHero;
Point3f enemySentry;

vector<Point3d> left_objectPoints(4);
vector<Point2d> left_imagePoints(4);
vector<Point3d> right_objectPoints(4);
vector<Point2d> right_imagePoints(4);

custom_msgs::points left_points;
custom_msgs::points right_points;
custom_msgs::points merged_points;

// for reading parameter input
vector<double> left_p;
vector<double> left_i;
vector<double> left_d;
vector<double> right_i;
vector<double> right_d;
vector<double> right_p;

// the x, y shift of the ui map
int x_shift = 0, y_shift = 0;

string color;
bool isBlue = 0;

double calculateDis(const custom_msgs::point &a, const custom_msgs::point &b) {
    double dist = sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    return dist;
}
// input: 3D wp and 2D pixel point; output: extrinsic of the left cam
void left_calibration(const custom_msgs::points &msg) {
    // tranform a point in a 1 x 1 coordinate to a 2048 x 3270 pixel coordinate of an image
    for(const auto &point: msg.data) {
        left_imagePoints[point.id] = Point2d(point.x, point.y);
        left_imagePoints[point.id].x *= imgCols;
        left_imagePoints[point.id].y *= imgRows;
        
    }
    //armour?(WIP)
    for(int i = 0; i < 4; i++) {
        printf("left_imagePoints[%d]: %f %f\n",i , left_imagePoints[i].x, left_imagePoints[i].y);
    }
    for(int i = 0; i < 4; i++) {
        printf("left_objectPoints[%d]:%f %f %f\n",i , left_objectPoints[i].x, left_objectPoints[i].y, left_objectPoints[i].z);
    }
    cout << "left_intrinsic: " << left_intrinsic << endl;
    cout << "left_distcoeffs: " << left_distCoeffs << endl;
    // rest
    Mat inlier;
    int suc = solvePnPRansac(left_objectPoints, left_imagePoints, left_intrinsic, left_distCoeffs, left_R_jacob,
                             left_T,
                             false, 100, 8.0, 0.99,
                             inlier, SOLVEPNP_AP3P);
    Rodrigues(left_R_jacob, left_R);
    cout << "suc: " << suc << endl;
    cout << "left Rotation Matrix: " << left_R << endl;
    cout << "left Translation Matrix: " << left_T << endl;
}

    void printMatrixDetails(const Mat& mat) {
        cout << " shape: " << mat.rows << "x" << mat.cols 
         << ", channels: " << mat.channels()
         << ", type: " << mat.type() << endl;
    }

std::unordered_map<int, int> yoloID2realID = {
    {1, 1},
    {2, 2},
    {3, 3},
    {4, 4},
    {5, 5},
    {6, 7},
    {255, 255}
};
// input: 2D pixel point and distance; output: 3D wp
void left_disPointsCallback(const custom_msgs::dis_points &input) {

    Mat invR, invM;
    invert(left_intrinsic, invM);
    invert(left_R, invR);
    vector<custom_msgs::point>().swap(left_points.data);
    for (int i = 0; i < input.data.size(); i++) {
        if (input.data[i].dist <= 0) continue;
        Mat x8_pixel;
        x8_pixel = (Mat_<double>(3, 1) << (double) input.data[i].x , (double) input.data[i].y, 1);
        x8_pixel *= (1000 * input.data[i].dist);
        // cout << invR.type() << " " << invM.type() << " " << x8_pixel.type() << " " << left_T.type() << endl;
        Mat calcWorld = invR * (invM * x8_pixel - left_T);
        calcWorld /= 1000;

        // printf("Matrix Calculation Completed!\n");

        custom_msgs::point point;
        point.x = calcWorld.at<double>(0, 0) / map_width;
        point.y = calcWorld.at<double>(1, 0) / map_length;
        point.z = calcWorld.at<double>(2, 0);
        point.id = (input.data[i].id == 255 ? 255 : (input.data[i].color ? yoloID2realID[input.data[i].id] : 100 + yoloID2realID[input.data[i].id]));  
              
        if(isBlue) {
            point.x = 1.0 - point.x;
            point.y = 1.0 - point.y;
        }
        
        cout << "left_point: " << "yolo_id: " << (int)input.data[i].id << "id: " << (int)point.id << " x: " << point.x * map_width<< " y: " << point.y * map_length << endl;

        if (!isBlue) { // we are red
            if(point.id >= 100)
                left_points.data.push_back(point);
        } else {
            if(point.id < 100)
                left_points.data.push_back(point);
        }
    }
    // calculate outpost distance? (WIP)

}
// input: 3D wp and 2D pixel point; output: extrinsic of the right cam
void right_calibration(const custom_msgs::points &msg) {
    for (const auto &point: msg.data) {
        right_imagePoints[point.id] = Point2d(point.x, point.y);
        right_imagePoints[point.id].x *= imgCols;
        right_imagePoints[point.id].y *= imgRows;
    }
    //armour(WIP)
    for(int i = 0; i < 4; i++) {
        printf("right_imagePoints[%d]: %f %f\n",i , right_imagePoints[i].x, right_imagePoints[i].y);
    }
    for(int i = 0; i < 4; i++) {
        printf("right_objectPoints[%d]:%f %f %f\n",i , right_objectPoints[i].x, right_objectPoints[i].y, right_objectPoints[i].z);
    }
    cout << "right_intrinsic: " << right_intrinsic << endl;
    cout << "right_distcoeffs: " << right_distCoeffs << endl;
    //rest
    Mat inlier;
    int suc = solvePnPRansac(right_objectPoints, right_imagePoints, right_intrinsic, right_distCoeffs, right_R_jacob,
                             right_T,
                             false, 100, 8.0, 0.99,
                             inlier, SOLVEPNP_AP3P);
    Rodrigues(right_R_jacob, right_R);
    cout << "suc: " << suc << endl;
    cout << "right Rotation Matrix: " << right_R << endl;
    cout << "right Translation Matrix: " << right_T << endl;
}

void right_disPointsCallback(const custom_msgs::dis_points &input) {
    Mat invR, invM;
    invert(right_intrinsic, invM);
    invert(right_R, invR);
    //clear
    vector<custom_msgs::point>().swap(right_points.data);
    for(int i = 0; i < input.data.size(); i++) {
        if (input.data[i].dist <= 0) continue;

        Mat x8_pixel = (Mat_<double>(3, 1) << (double)input.data[i].x, (double)input.data[i].y, 1);
        x8_pixel *= (1000 * input.data[i].dist);
        Mat calcWorld = invR * (invM * x8_pixel - right_T);
        calcWorld /= 1000;

        custom_msgs::point point;
        point.x = calcWorld.at<double>(0, 0) / map_width;
        point.y = calcWorld.at<double>(1, 0) / map_length;
        if(isBlue) {
            point.x = 1.0 - point.x;
            point.y = 1.0 - point.y;
        }
        point.z = calcWorld.at<double>(2, 0);
        point.id = (input.data[i].id == 255 ? 255 : (input.data[i].color ? yoloID2realID[input.data[i].id]: 100 + yoloID2realID[input.data[i].id]));

        cout << "right_point: " << "id: " << (int)point.id << " x: " << point.x * map_width<< " y: " << point.y * map_length << endl;

        if (!isBlue) { // we are red
            if(point.id >= 100)
                right_points.data.push_back(point);
        } else {
            if(point.id < 100)
                right_points.data.push_back(point);
        }
    }
}

void mergePoints() {
    vector<custom_msgs::point>().swap(merged_points.data);
    for (auto &l: left_points.data) {
        bool isDup = false;
        for (auto &r: right_points.data) {
            if(l.id == r.id){ // points are duplicate
                // consider different confidence for left and right based on the robot's position
                custom_msgs::point center;
                center.x = (l.x + r.x) / 2;
                center.y = (l.y + r.y) / 2;
                center.id = l.id;
                merged_points.data.emplace_back(center);
                
                r.id = 0; // mark the dup points;

                isDup = true;
                break;
            }
        }
        if(!isDup) {
            merged_points.data.emplace_back(l);
        }
    }
    
    for (auto &r: right_points.data) {
        if(r. id != 0) {
            merged_points.data.emplace_back(r);
        }
    }
}


int main(int argc, char **argv) {
    // value initialization


    // solvePnP
    ros::init(argc, argv, "uimap");
    ros::NodeHandle nh;

    ros::param::get("/uimap/x_shift", x_shift);
    ros::param::get("/uimap/y_shift", y_shift);

    ros::param::get("/left_cam/calibration/points", left_p);

    for (int i = 0; i < 4; i++) {
        left_objectPoints[i].x = left_p[i * 3 + 0];
        left_objectPoints[i].y = left_p[i * 3 + 1];
        left_objectPoints[i].z = left_p[i * 3 + 2];
    }

    ros::param::get("/right_cam/calibration/points", right_p);

    for (int i = 0; i < 4; i++) {
        right_objectPoints[i].x = right_p[i * 3 + 0];
        right_objectPoints[i].y = right_p[i * 3 + 1];
        right_objectPoints[i].z = right_p[i * 3 + 2];
    }

    ros::param::get("/battle_color", color);
    if(color == "red") isBlue = 0;
    else isBlue = 1;
    
    ros::param::get("/left_cam/intrinsic_matrix", left_i);
    for(int i = 0; i < 9; i++) left_intrinsic.at<double>(i / 3, i % 3) = left_i[i];
    ros::param::get("/left_cam/distortion_coefficients", left_d);
    for(int i = 0; i < 5; i++) left_distCoeffs.at<double>(i, 0) = left_d[i];

    ros::param::get("/right_cam/intrinsic_matrix", right_i);
    for(int i = 0; i < 9; i++) right_intrinsic.at<double>(i / 3, i % 3) = right_i[i];
    ros::param::get("/right_cam/distortion_coefficients", right_d);
    for(int i = 0; i < 5; i++) right_distCoeffs.at<double>(i, 0) = right_d[i];

    // read four DEFAULT image coordinates(WIP)
    for (int i = 0; i < 4; i++) {
        double x = 0, y = 0;
        string key = "/GUI/left_cam/calibration/point" + to_string(i + 1) + "/x";
        ros::param::get(key, x);
        key = "/GUI/left_cam/calibration/point" + to_string(i + 1) + "/y";
        ros::param::get(key, y);
        left_imagePoints[i] = cv::Point2d(x, y);
        left_imagePoints[i].x *= imgCols;
        left_imagePoints[i].y *= imgRows;

        key = "/GUI/right_cam/calibration/point" + to_string(i + 1) + "/x";
        ros::param::get(key, x);
        key = "/GUI/right_cam/calibration/point" + to_string(i + 1) + "/y";
        ros::param::get(key, y);
        right_imagePoints[i] = cv::Point2d(x, y);
        right_imagePoints[i].x *= imgCols;
        right_imagePoints[i].y *= imgRows;
    }

    // computer rotation and translation matrix USING DEFAULT PARAMETERS
    cout << left_T.type() << endl;
    solvePnPRansac(left_objectPoints, left_imagePoints, left_intrinsic, left_distCoeffs,
                   left_R_jacob, left_T, SOLVEPNP_AP3P);
    Rodrigues(left_R_jacob, left_R);
    cout << "left Rotation Matrix: " << left_R << " " << left_R.type() << endl;
    cout << "left Translation Matrix: " << left_T << " " << left_T.type() << endl;

    solvePnPRansac(right_objectPoints, right_imagePoints, right_intrinsic, right_distCoeffs,
                   right_R_jacob, right_T, SOLVEPNP_AP3P);
    Rodrigues(right_R_jacob, right_R);
    cout << "right Rotation Matrix: " << right_R << endl;
    cout << "right Translation Matrix: " << right_T << endl;


    ros::Subscriber left_imageSub = nh.subscribe("/left/calibration", 1, &left_calibration);
    ros::Subscriber left_disPointsSub = nh.subscribe("/left/dis_points", 1, &left_disPointsCallback);
    ros::Subscriber right_imageSub = nh.subscribe("/right/calibration", 1, &right_calibration);
    ros::Subscriber right_disPointsSub = nh.subscribe("/right/dis_points", 1, &right_disPointsCallback);

    heroPub = nh.advertise<custom_msgs::point>("/hero", 1);
    sentryPub = nh.advertise<custom_msgs::points>("/sentry", 1);
    robotPub = nh.advertise<custom_msgs::points>("/robot", 1);
    ros::Rate loopRate(20);
    
    //visualize component (WIP)

    while(ros::ok()) {
        ros::spinOnce();
        mergePoints();

        for(auto point : merged_points.data) {
            cout << "merged_point: " << "id: " << (int)point.id << " x: " << point.x * map_width << " y: " << point.y * map_length << endl;
        } cout << endl;

        robotPub.publish(merged_points);
        loopRate.sleep();
    }
    return 0;
}

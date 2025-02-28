#ifndef gui_pkg_QNODE_HPP_
#define gui_pkg_QNODE_HPP_

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QImage>
#include <QThread>
#include <QListWidgetItem>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <custom_msgs/points.h>
#include <custom_msgs/point.h>
#include <custom_msgs/game_state.h>
#include <custom_msgs/referee_warning.h>
#include <custom_msgs/supply_projectile_action.h>
#include <custom_msgs/dis_points.h>

struct RobotPoint {
    QPoint point;
    int id;
};

namespace gui_pkg {

enum LogLevel {
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL
};

struct LogInformation {
    LogLevel level;
    QString str; 
};

struct Robot {
    int hpCurrent;
    int hpMax;
};

class QNode : public QThread {
Q_OBJECT

public:
    QNode(int argc, char** argv);
    ~QNode();
    bool init();
    
    QImage imageMainWindow;
    QImage imageLeft;
    QImage imageRight;
    QImage imageCalibrateMainWindow;
    QImage imageCalibrateSecondWindow;
    QImage imageSecondWindow;
    QImage imageUimap;
    QImage imageLogo;

    QString leftImgRaw;
    QString rightImgRaw;
    QString cameraCalibrating;
    QString gameProgress; 

    QPoint mouseLocation;
    QPoint leftPoints[4];
    QPoint rightPoints[4];

    QListWidgetItem *listWidgetItem;
    LogInformation *logInformation;

    bool isCalibrating;
    int calibrateRate;
    int calibrateMainWindowWidth;
    int calibrateMainWindowLength;
    int calibrateSecondWindowWidth;
    int calibrateSecondWindowLength;
    int displayMainWindowWidth;
    int displayMainWindowLength;
    int displaySecondWindowWidth;
    int displaySecondWindowLength;
    int uimapWidth;
    int uimapLength;
    int rawImageWidth;
    int rawImageLength;
    int logoWidth;
    int logoLength;
    int stageRemainTime;

    // cv::Mat imgMainWindow;
    cv::Mat imgLeft;
    cv::Mat imgRight;
    cv::Mat imgUimap;
    cv::Mat imgLogo;
    // cv::Mat imgSecondWindow;

    std::vector<custom_msgs::dis_point> leftDists;
    std::vector<custom_msgs::dis_point> rightDists;

    unsigned short roiWarnState;

    std::string battleColor;
    
    Robot robot_red1;
    Robot robot_red2;
    Robot robot_red3;
    Robot robot_red4;
    Robot robot_red5;
    Robot robot_redGuard;
    Robot robot_redOutpost;
    Robot robot_redBase;
    Robot robot_blue1;
    Robot robot_blue2;
    Robot robot_blue3;
    Robot robot_blue4;
    Robot robot_blue5;
    Robot robot_blueGuard;
    Robot robot_blueOutpost;
    Robot robot_blueBase;

    std::vector<RobotPoint> robotPoints;


    void run();
    void loadParams();
    void robotPointCallback(const custom_msgs::points& msg);
    void leftDistCallback(const custom_msgs::dis_points& msg);
    void rightDistCallback(const custom_msgs::dis_points& msg);
    
    void pubCalibration();

Q_SIGNALS:
    // void loggingUpdated();
    void rosShutdown();
    void loggingCameraCalibrateMainWindow();
    void loggingCameraCalibrateSecondWindow();
    void loggingUimapUpdate();
    // void loggingGameStateUpdate();

private:
    int init_argc;
    char** init_argv;
    
    image_transport::Subscriber imageLeftSub;
    image_transport::Subscriber imageRightSub;
    ros::Subscriber robotPointsSub;
    ros::Subscriber gameStateSub;
    ros::Subscriber supplyProjectileActionSub;
    ros::Subscriber refereeWarningSub;
    ros::Subscriber leftDistSub;
    ros::Subscriber rightDistSub;


    ros::Publisher leftCalibrationPub;
    ros::Publisher rightCalibrationPub;

    std::string gameStateTopic;
    std::string supplyProjectileActionTopic;
    std::string refereeWarningTopic;
    std::string leftCalibrationTopic;
    std::string rightCalibrationTopic;
    std::string leftDistTopic;
    std::string rightDistTopic;

    bool ifBeginToRecord;
    bool ifRecordDone;
    bool ifBeginToReplay;
    bool ifReplayDone;
    int dart_first_close_time;
};


}

#endif
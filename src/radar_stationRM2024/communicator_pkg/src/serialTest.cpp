#include <ros/ros.h>
#include <ros/console.h>
#include <custom_msgs/points.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "serialTest");
    ros::NodeHandle nh;
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    ros::Publisher pub = nh.advertise<custom_msgs::points>("/serial_test", 100);

    ros::Rate loop_rate(10);
    while(ros::ok()){
        custom_msgs::points msg;
        custom_msgs::point pt;
        pt.id = 5;
        pt.x = 0.01;
        pt.y = 0.01;
        msg.data.push_back(pt);
        // pt.id = 102;
        // pt.x = 0.3;
        // pt.y = 0.3;
        // msg.data.push_back(pt);
        // pt.id = 3;
        // pt.x = 0.1;
        // pt.y = 0.1;
        // msg.data.push_back(pt);
        // ROS_DEBUG_STREAM("Publishing... " << static_cast<int>(msg.data[0].id) << " *** " << static_cast<int>(msg.data[1].id) <<" *** ");
        pub.publish(msg);
        loop_rate.sleep();
    }
    ros::spin();
    return 0;
}
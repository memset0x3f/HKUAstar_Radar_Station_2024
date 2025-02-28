#ifndef COMMUNICATOR_HPP
#define COMMUNICATOR_HPP

#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>
#include <mutex>
#include <ros/ros.h>
#include "data_structures.hpp"
#include "DJIcrcHelper.hpp"
#include "SerialPort.hpp"
#include <custom_msgs/points.h>
#include <custom_msgs/point.h>
#include <custom_msgs/SendData.h>
#include <custom_msgs/game_state.h>
#include <custom_msgs/referee_warning.h>
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/Float32MultiArray.h"

class Communicator{
public:
    
    Communicator(int argc, char** argv);
    /*receiver*/
    bool init();
    void initiate_double_dmg_cb(const ros::TimerEvent &event);
    void initiate_double_dmg_cb_temp(int temp);
    void readData(const ros::TimerEvent &event);
    // Publishers
    ros::Publisher robot_id_pub;
    ros::Publisher mark_data_pub;
    ros::Publisher hp_data_pub;
    ros::Publisher robot_pos_pub;
    ros::Publisher sentry_pos_pub;
    ros::Publisher gimbal_pos_pub;
    ros::Publisher game_status_pub;
    ros::Publisher robot_HP_pub;
    ros::Publisher dart_info_pub;
    ros::Publisher referee_warning_pub;
    // Subscriber
    ros::Subscriber sub_send_data;
    ros::Subscriber sub_hero_pos;
    // Timer
    ros::Timer timer_, timer1_, timer2_;

    /*Sender*/
    bool isFriendly(uint8_t id);
    void robot_pos_cb(const custom_msgs::points::ConstPtr msg);
    void hero_pos_cb(const custom_msgs::point::ConstPtr msg);
    void send_normal_points(const map_robot_data_t* map_data, std::chrono::steady_clock::time_point& last_send_time);
    void send_guess_points(map_robot_data_t* map_data, uint16_t* robot_pos_ptrs[], std::chrono::steady_clock::time_point& last_send_time);
    //subscriber
    ros::Subscriber sub_robot_mark_progress;
    ros::Subscriber sub_robot_pos;
    ros::Subscriber sub_ground_pos;
    ros::Subscriber sub_hero;
    ros::Subscriber sub_sentry;
    ros::Subscriber sub_robot;
    //publisher
    ros::Publisher pub_send_data;
    ros::Publisher pub_hero_pos;
    
private:
    uint16_t enemy_mark_data_now[3];
    bool IS_BLUE;
    //serial port
    SerialPort sp;
    int init_argc;
    char** init_argv;
    /*receiver*/
    uint8_t cmdID[2];
    // structures
    robot_status_t robotStatus;
    match_status_t matchstatus;
    // Game State
    custom_msgs::game_state game_state;
    // Variables
    uint8_t double_dmg_chances;
    uint8_t temp = 0;
    std::chrono::steady_clock::time_point double_dmg_timer;
    bool in_double_dmg = false;
    bool enemy_in_double_dmg = false;

    /*sender*/
    map_robot_data_t map_data;
    const std::unordered_map<int, std::vector<std::pair<int, int>>> guess_table = {
    {0, {{1100, 1400}, {900, 1400}}},
    {1, {{870, 1100}, {1340, 680}}},
    {2, {{560, 630}, {560, 870}}},
    {100, {{1700, 100}, {1900, 100}}},
    {101, {{1930, 400}, {1460, 820}}},
    {102, {{2240, 870}, {2240, 630}}}
    };
    std::unordered_map<int, int> enemy_mark_data_before;
    std::unordered_map<int, int> guess_index;
    std::unordered_map<int, int> milliseconds_after_lost;
    //coefficient
    int X_COEFF = 28;
    int Y_COEFF = 15;
};
#endif
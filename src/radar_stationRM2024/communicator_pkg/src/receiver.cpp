#include <iostream>
#include <string>
#include <ros/ros.h>
#include <cstdint>
#include <thread>
#include <chrono>
#include "../include/DJIcrcHelper.hpp"
#include "custom_msgs/game_state.h"
#include "custom_msgs/referee_warning.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "../include/SerialPort.hpp"
#include "../include/data_structures.hpp"
#include "custom_msgs/SendData.h"
#include "custom_msgs/points.h"
#include "custom_msgs/point.h"

using namespace std::chrono_literals;
using std::placeholders::_1;
// receivers
uint8_t cmdID[2];
bool IS_BLUE = false;
// structs
robot_status_t robotStatus;
match_status_t matchstatus;
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
ros::Timer timer_, timer1_;
// Game State
custom_msgs::game_state game_state;
// Serial Port
SerialPort sp;
// Variables
uint8_t double_dmg_chances;
uint8_t temp;
bool in_double_dmg = false, enemy_in_double_dmg = false;
// mark progress
uint16_t enemy_mark_data_now[3] = {0, 0, 0};
// 4 calibration points
custom_msgs::points calibration_points;

void initiate_double_dmg();

void readData(const ros::TimerEvent &event){
    int bytesAvailable = sp.getBufferBytes();
    uint8_t* buf = new uint8_t[bytesAvailable];
    // ROS_ERROR_STREAM("Entering readData, datalength is " << bytesAvailable << " bytes long!");
    int rawDataLength = 0, i = 0;
    while(rawDataLength < bytesAvailable){
        rawDataLength += sp.m_Receive(buf, bytesAvailable - rawDataLength);
    }
    /*
    frame_header 5 byte; cmd_id 2 bytel data n byte; frame_tail 2 byte
    */
    while(i < bytesAvailable){
        if(buf[i] == 165){ // frame_header.sof = 0xa5 = 165
            if(Verify_CRC8_Check_Sum(buf + i, frame_header_length)){
                size_t DataLength = (buf[i + 2] << 8) | buf[i + 1];
                if(Verify_CRC16_Check_Sum(buf + i, frame_header_length + 2 * sizeof(uint16_t) + DataLength)){
                    cmdID[0] = buf[i + 6]; cmdID[1] = buf[i + 5]; // true cmd_id takes up 4 bytes, so it needs two buffer spaces
                    // ROS_WARN_STREAM("i = " << i << ", Command id: 0x0" << static_cast<unsigned int>(cmdID[0]) << "0" << static_cast<unsigned int>(cmdID[1]));
                    int index = i + frame_header_length + sizeof(uint16_t);
                    switch(cmdID[0]){
                        case 0:{
                            switch(cmdID[1]){
                                case 1:{// 0x0001
                                    game_state.game_type = buf[index];
                                    game_state.game_progress = buf[index + 1];
                                    game_state.stage_remain_time = (buf[index + 3] << 8) | buf[index + 2];
                                    // game_state.sync_timestamp = (buf[index + 7] << 24) | (buf[index + 6] << 16) | (buf[index + 5]) << 
                                    //                             8 | buf[index + 4];
                                    game_status_pub.publish(game_state);
                                    break;
                                }
                                case 3:{// 0x0003
                                    game_state.red_1_robot_HP = (buf[index + 1] << 8) | buf[index];
                                    game_state.red_2_robot_HP = (buf[index + 3] << 8) | buf[index + 2];
                                    game_state.red_3_robot_HP = (buf[index + 5] << 8) | buf[index + 4];
                                    game_state.red_4_robot_HP = (buf[index + 7] << 8) | buf[index + 6];
                                    game_state.red_5_robot_HP = (buf[index + 9] << 8) | buf[index + 8];
                                    game_state.red_7_robot_HP = (buf[index + 11] << 8) | buf[index + 10];
                                    game_state.red_outpost_HP = (buf[index + 13] << 8) | buf[index + 12];
                                    game_state.red_base_HP = (buf[index + 15] << 8) | buf[index + 14];
                                    game_state.blue_1_robot_HP = (buf[index + 17] << 8) | buf[index + 16];
                                    game_state.blue_2_robot_HP = (buf[index + 19] << 8) | buf[index + 18];
                                    game_state.blue_3_robot_HP = (buf[index + 21] << 8) | buf[index + 20];
                                    game_state.blue_4_robot_HP = (buf[index + 23] << 8) | buf[index + 22];
                                    game_state.blue_5_robot_HP = (buf[index + 25] << 8) | buf[index + 24];
                                    game_state.blue_7_robot_HP = (buf[index + 27] << 8) | buf[index + 26];
                                    game_state.blue_outpost_HP = (buf[index + 29] << 8) | buf[index + 28];
                                    game_state.blue_base_HP = (buf[index + 31] << 8) | buf[index + 30];
                                    robot_HP_pub.publish(game_state);
                                    break;
                                }
                            }
                            break;
                        }
                        case 1:{
                            switch(cmdID[1]){
                                case 4:{// 0x0104 referee warning info
                                    custom_msgs::referee_warning referee_warning;
                                    referee_warning.level = buf[index];
                                    referee_warning.foul_robot_id = buf[index + 1];
                                    referee_warning.count = buf[index + 2];
                                    referee_warning_pub.publish(referee_warning);
                                    break;
                                }
                                case 5:{// input: 0x0105 dart info
                                    // game_state.dart_remaining_time = buf[index];
                                    // // game_state.dart_aim_state = buf[index + 1];
                                    // dart_info_pub.publish(game_state);
                                    // target = (buf[index + 1] & 0b01100000) >> 5;
                                    // if(target != prev_target && target != 0){
                                    //     target = prev_target;
                                    //     if(double_dmg_chances > 0 && !in_double_dmg){
                                    //         ros::NodeHandle nh;
                                    //         ros::Timer double_dmg_timer;
                                    //         ROS_INFO("1 seconds before double damage starts!");
                                    //         double_dmg_timer = nh.createTimer(ros::Duration(1), initiate_double_dmg_cb);
                                    //     }
                                    //     in_double_dmg = false;
                                    // }
                                    break;
                                }
                            }
                            break;
                        }
                        case 2:{
                            switch(cmdID[1]){
                                case 1:{// input: 0x0201 robot status; output: robot_id_pub
                                    robotStatus.robot_id = buf[index];
                                    IS_BLUE = robotStatus.robot_id > 100 ? true : false;
                                    ROS_ERROR_STREAM("our id: " << static_cast<int>(robotStatus.robot_id) << " our color: " << IS_BLUE);
                                    // other info
                                    auto msg = std_msgs::UInt8();
                                    msg.data = robotStatus.robot_id;
                                    robot_id_pub.publish(msg);
                                    break;
                                }
                                case 2:{
                                    break;
                                }
                                case 3:{// input: 0x0203 robot position; output: 0x0301
                                    if(robotStatus.robot_id == 7 || robotStatus.robot_id == 107){
                                        uint8_t send_buffer[sizeof(sentry_pos_frame)];
                                        sentry_pos_frame frame;
                                        frame.frame_header.sof = 0xa5;
                                        frame.frame_header.data_length = sizeof(interaction_data_header_t) + 8;
                                        frame.frame_header.seq = 0;
                                        frame.cmd_id = 0x0301;
                                        frame.data_header.data_cmd_id = 0x0201;
                                        frame.data_header.sender_id = robotStatus.robot_id;
                                        frame.data_header.receiver_id = robotStatus.robot_id + 2;
                                        // cpy robot position info from buf to frame and then send_buffer
                                        memcpy(&frame.sentry_position, buf + index, 8);
                                        memcpy(send_buffer, &frame.frame_header.sof, sizeof(sentry_pos_frame));
                                        Append_CRC8_Check_Sum(send_buffer, frame_header_length);
                                        Append_CRC16_Check_Sum(send_buffer, sizeof(sentry_pos_frame));
                                        if(sp.m_Send(send_buffer, sizeof(sentry_pos_frame))){
                                            ROS_FATAL_STREAM("successfully send sentry position to 0x0301");
                                        }else{
                                            ROS_ERROR_STREAM("failed to send sentry position to 0x0301");
                                        }
                                    }
                                    break;
                                }
                                case 0xB:{// input: 0x020B ground robot positions except sentry; output: 0x0301
                                    uint8_t send_buffer[sizeof(sentry_radar_frame)];//55
                                    sentry_radar_frame frame;
                                    frame.frame_header.sof = 0xa5;
                                    frame.frame_header.data_length = sizeof(interaction_data_header_t) + 40;
                                    frame.frame_header.seq = 0;
                                    frame.cmd_id = 0x0301;
                                    frame.data_header.data_cmd_id = 0x0200;
                                    frame.data_header.sender_id = robotStatus.robot_id;
                                    frame.data_header.receiver_id = robotStatus.robot_id + 2;
                                    memcpy(&frame.robot_position, buf+index, 40);
                                    memcpy(send_buffer, &frame.frame_header.sof, sizeof(sentry_radar_frame));
                                    Append_CRC8_Check_Sum(send_buffer, frame_header_length);
                                    Append_CRC16_Check_Sum(send_buffer, sizeof(sentry_radar_frame));
                                    if (sp.m_Send(send_buffer, sizeof(sentry_radar_frame))) 
                                        ROS_INFO_STREAM("Ground robot data sent successfully!");
                                    else 
                                        ROS_ERROR_STREAM("Fail to send the ground robot data!");
                                    break;
                                }
                                case 0xC:{// input: 0x020C mark data; output: mark_data_pub
                                    std_msgs::UInt8MultiArray mark_data;
                                    for(int j = 0; j < 6; j++){
                                        mark_data.data.push_back(buf[index + j]);
                                        // ROS_ERROR("Mark data: %d", buf[index + i]);
                                    }
                                    enemy_mark_data_now[0] = mark_data.data[0];
                                    enemy_mark_data_now[1] = mark_data.data[1];
                                    enemy_mark_data_now[2] = mark_data.data[5];
                                    mark_data_pub.publish(mark_data);
                                    break;
                                }
                                case 0xE:{// input: 0x020E double damage data
                                    double_dmg_chances = buf[index] & 0b00000011;
                                    if(double_dmg_chances > temp && !in_double_dmg){
                                        initiate_double_dmg();
                                        in_double_dmg = true;

                                        std::thread([=]() {
                                            std::this_thread::sleep_for(std::chrono::seconds(10));
                                            in_double_dmg = false;
                                        }).detach();
                                    }
                                    temp = double_dmg_chances;
                                    enemy_in_double_dmg = (buf[index] & 0b00000100) >> 2;
                                    break;
                                }
                                /*
                                other data...
                                */
                                break;
                            }
                            break;
                        }
                        case 3:{
                            switch(cmdID[1]){
                                case 1:{// input: 0x0301 custom messages, see the two above; output:
                                    ROS_ERROR("fuck");
                                    if(buf[index+1]==0x02 && buf[index]==0x00){
                                        index += 6; // skip data_header
                                        float pos;
                                        auto msg = std_msgs::Float32MultiArray();
                                        for(; index < 40; index += 4){
                                            memcpy(&pos, buf+index, 4);
                                            msg.data.push_back(pos);
                                        }
                                        robot_pos_pub.publish(msg);
                                    }
                                    else if(buf[index+1]==0x02 && buf[index]==0x01){//哨兵转发给雷达的哨兵坐标
                                        index += 6;
                                        float pos;
                                        auto msg = std_msgs::Float32MultiArray();
                                        memcpy(&pos, buf+index, 4);
                                        msg.data.push_back(pos);
                                        memcpy(&pos, buf+index+4, 4);
                                        msg.data.push_back(pos);
                                        sentry_pos_pub.publish(msg);
                                    }
                                    break;
                                }
                                case 3:{// input: 0x0303 云台手通过选手端; output: ros
                                    ROS_ERROR("fucked up!!");
                                    custom_msgs::point point;
                                    memcpy(&point.x, buf + index, 4);
                                    memcpy(&point.y, buf + index + 4, 4);
                                    memcpy(&point.id, buf + index + 9, 1);
                                    calibration_points.data.push_back(point);
                                    if(calibration_points.data.size() == 4){
                                        for(int j = 0; j < 4; j++){
                                            ROS_ERROR_STREAM("point " << j + 1 << " x = " << calibration_points.data[j].x << " y = " << calibration_points.data[j].y);
                                        }
                                    }
                                    gimbal_pos_pub.publish(point);
                                    break;
                                }
                                case 5:{// input: 0x0305 test
                                    break;
                                } 
                                /*
                                other data...
                                */
                            }
                            break;
                        }
                    }
                }
                else{
                    //ROS_ERROR("CRC16 FAILED");
                }
                i += frame_header_length + sizeof(uint16_t) + DataLength +sizeof(uint16_t);
            }
            else{
                //ROS_ERROR("CRC8 FAILED");
                i++;
            }
        }
        else{
            i++;
        }
    }
    delete[] buf;
}
// receive SendData.msg from map.cpp and send it to the serial port
void send_data_cb(const custom_msgs::SendData::ConstPtr &msg){
    uint8_t tbuf[256];
    memset(tbuf, 0, sizeof(tbuf));
    for(int i = 0; i < msg->length; i++){
        tbuf[i] = msg->data[i]; // uint8_t and char are the same
    }
    if (sp.m_Send(tbuf, msg->length)) // length of the whole frame
        ROS_ERROR_STREAM(msg->name << " Send successfully!");
    else 
        ROS_ERROR_STREAM("Fail to send the "<< msg->name << "!");
}
// initiate double damage once the target of the dart changes; whether this works remains to be seen;
void initiate_double_dmg(){
    static uint8_t radar_cmd = 0;
    uint8_t tbuf[16];
    memset(tbuf, 0, 16);

    radar_cmd_frame frame;
    frame.frame_header.sof = 0xA5;
    frame.frame_header.data_length = 7;
    frame.frame_header.seq = 0;
    memcpy(tbuf, &frame.frame_header, frame_header_length);
    Append_CRC8_Check_Sum(tbuf, frame_header_length);

    frame.cmd_id = 0x0301;
    memcpy(tbuf + 5, &frame.cmd_id, 2);

    frame.data_header.data_cmd_id = 0x0121;
    frame.data_header.sender_id = robotStatus.robot_id;
    frame.data_header.receiver_id = 0x8080;
    memcpy(tbuf + 7, &frame.data_header, 6);
    // cmd++ after each call
    frame.double_dmg_cmd = radar_cmd++;
    memcpy(tbuf + 13, &frame.double_dmg_cmd, 1);
    
    Append_CRC16_Check_Sum(tbuf, 16);

    if (sp.m_Send(tbuf, 16)) // length of the frame
        ROS_ERROR("Send dbl dmg successfully!");
    else 
        ROS_ERROR("Fail to send!");
}
/*test*/
void initiate_double_dmg_cb_temp(const ros::TimerEvent &event){
    static uint8_t radar_cmd_temp = 0;
    uint8_t tbuf[16];
    memset(tbuf, 0, 16);

    radar_cmd_frame frame;
    frame.frame_header.sof = 0xA5;
    frame.frame_header.data_length = 7;
    frame.frame_header.seq = 0;
    memcpy(tbuf, &frame.frame_header, frame_header_length);
    Append_CRC8_Check_Sum(tbuf, frame_header_length);

    frame.cmd_id = 0x0301;
    memcpy(tbuf + 5, &frame.cmd_id, 2);

    frame.data_header.data_cmd_id = 0x0121;
    frame.data_header.sender_id = robotStatus.robot_id;
    frame.data_header.receiver_id = 0x8080;
    memcpy(tbuf + 7, &frame.data_header, 6);

    // cmd++ after each call
    frame.double_dmg_cmd = 1;
    memcpy(tbuf + 13, &frame.double_dmg_cmd, 1);
    
    Append_CRC16_Check_Sum(tbuf, 16);

    if (sp.m_Send(tbuf, 16)) // length of the frame
        ROS_ERROR("Send dbl dmg successfully!");
    else 
        ROS_ERROR("Fail to send!");
}

int main(int argc, char** argv){
    ROS_INFO("Initialize the node communicator");
    ros::init(argc, argv, "communicator");
    ros::NodeHandle nh;
    //parameters
    std::string gameStateTopic = "/GUI/topic/game_state_topic";
    ros::param::get("/GUI/topic/game_state_topic", gameStateTopic);
    std::string supplyProjectileActionTopic = "/GUI/topic/supply_projectile_action_topic";
    ros::param::get("/GUI/topic/supply_projectile_action_topic", supplyProjectileActionTopic);
    std::string refereeWarningTopic = "/GUI/topic/referee_warning_topic";
    ros::param::get("/GUI/topic/referee_warning_topic", refereeWarningTopic);

    //publishers:
    ROS_INFO_STREAM("Advertise on topic robot_id with message type UInt8");
    robot_id_pub = nh.advertise<std_msgs::UInt8>("robot_id", 5);
    ROS_INFO_STREAM("Advertise on topic hp_data with message type UInt16MultiArray");
    hp_data_pub = nh.advertise<std_msgs::UInt16MultiArray>("hp_data", 5);
    ROS_INFO_STREAM("Advertise on topic mark_data with message type UInt8MultiArray");
    mark_data_pub = nh.advertise<std_msgs::UInt8MultiArray>("mark_data", 5);
    ROS_INFO_STREAM("Advertise on topic ground_robot_pos with message type Float32MultiArray");
    robot_pos_pub = nh.advertise<std_msgs::Float32MultiArray>("ground_robot_pos", 100);
    ROS_INFO_STREAM("Advertise on topic sentry_robot_pos with message type Float32MultiArray");
    sentry_pos_pub = nh.advertise<std_msgs::Float32MultiArray>("sentry_robot_pos", 100);
    ROS_INFO_STREAM("Advertise on topic gimbal_pos with message type point");
    gimbal_pos_pub = nh.advertise<custom_msgs::point>("gimbal_pos", 5);
    ROS_INFO_STREAM("Advertise on topic robot_HP with message type game_state");
    robot_HP_pub = nh.advertise<custom_msgs::game_state>(gameStateTopic, 5);
    game_status_pub = nh.advertise<custom_msgs::game_state>(gameStateTopic, 5);
    ROS_INFO_STREAM("Advertise on topic dart_info with message type game_state");
    dart_info_pub = nh.advertise<custom_msgs::game_state>(supplyProjectileActionTopic, 5);
    ROS_INFO_STREAM("Advertise on topic referee warning with message type referee_warning");
    referee_warning_pub = nh.advertise<custom_msgs::referee_warning>(refereeWarningTopic, 5);

    // subscriber: 订阅 map.cpp 中发布的信息并转发给裁判系统
    // sub_send_data = nh.subscribe("/send_data", 256, &send_data_cb);
    // sub_hero_pos = nh.subscribe("/hero_pos", 256, &send_data_cb);
    // //timer: 回调函数 readData 会负责发布信息
    ROS_INFO_STREAM("Timer created to call function readData");
    timer_= nh.createTimer(ros::Duration(1.0), readData);
    /*test*/
    //timer1_= nh.createTimer(ros::Duration(1.0), initiate_double_dmg_cb_temp);
    
    if(sp.OpenPort("/dev/ttyUSB0", 115200, 0,8,1))
        ROS_INFO_STREAM("Open serial port successfully");
    else
        ROS_ERROR_STREAM("Fail to open the serial port!!");

    ros::spin();
    return 0;
}
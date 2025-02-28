// input: data from the radar station and sentry
// output: "send_data" to Communicator.cpp
#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <ros/ros.h>
#include "../include/data_structures.hpp"
#include "../include/DJIcrcHelper.hpp"
#include "../include/SerialPort.hpp"
#include "ros/console.h"
#include "ros/param.h"
#include <custom_msgs/points.h>
#include <custom_msgs/point.h>
#include <custom_msgs/SendData.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

#define X_COEFF 28
#define Y_COEFF 15

SerialPort sp;
//color
bool IS_BLUE;

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
//mark progress
uint16_t enemy_mark_data_now[3];

// float Dis(float x1, float y1, float x2, float y2){
//     return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
// }

const std::unordered_map<int, std::vector<std::pair<int, int>>> guess_table = {
    {0, {{1100, 1400}, {900, 1400}}},
    {1, {{870, 1100}, {1340, 680}}},
    {2, {{560, 630}, {560, 870}}},
    {100, {{1700, 100}, {1900, 100}}},
    {101, {{1930, 400}, {1460, 820}}},
    {102, {{2240, 870}, {2240, 630}}}
};
std::unordered_map<int, int> enemy_mark_data_before = {
    {0, 0},
    {1, 0},
    {2, 0}
};
std::unordered_map<int, int> guess_index = {
    {0, 0},
    {1, 0},
    {2, 0}
};

bool isFriendly(int id){
    if((IS_BLUE && id >= 100) || (!IS_BLUE && id < 100)) return true;
    return false;
}

void send_normal_points(map_robot_data_t* map_data, std::chrono::steady_clock::time_point& last_send_time);
void send_guess_points(map_robot_data_t* map_data, uint16_t* robot_data_ptrs[], std::chrono::steady_clock::time_point& last_send_time);

void robot_pos_cb(const custom_msgs::points::ConstPtr msg){
    static auto last_send_time = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send_time);

    uint16_t id;
    float rx, ry;
    map_robot_data_t map_data;
    for(const custom_msgs::point &point: msg->data){
        id = point.id;
        rx = X_COEFF * point.x * 100;
        ry = Y_COEFF * point.y * 100;
        if(isFriendly(id)){
            //ROS_ERROR_STREAM(std::to_string(rx).c_str());
            //ROS_ERROR("Is a friend or itself");
            continue;
        }

        int offset = id - (static_cast<int>(!IS_BLUE) * 100) - 1;
        if(offset == 6) offset = 5;
        uint16_t *p = (uint16_t*)&map_data + offset*2;
        *p = static_cast<uint16_t>(rx);
        *(p+1) = static_cast<uint16_t>(ry);
        // ROS_WARN("id: %d, x: %f, y: %f", id, rx, ry);
        // ROS_WARN("offset: %d, x: %d, y: %d", offset, *p, *(p+1));
    }
    
    //send at 5hz
    if(elapsed.count() >= 200){
        uint16_t* robot_pos_ptrs[3];
        robot_pos_ptrs[0] = (uint16_t*)&map_data;
        robot_pos_ptrs[1] = (uint16_t*)&map_data + 2;
        robot_pos_ptrs[2] = (uint16_t*)&map_data + 10;
        if(*robot_pos_ptrs[0] == 0 && *(robot_pos_ptrs[0] + 1) == 0 ||
          *robot_pos_ptrs[1] == 0 && *(robot_pos_ptrs[1] + 1) == 0 ||
          *robot_pos_ptrs[2] == 0 && *(robot_pos_ptrs[2] + 1) == 0){
            ROS_WARN_STREAM("hero x: " << *robot_pos_ptrs[0] << " hero y: " << *(robot_pos_ptrs[0] + 1) << " engin x: " << *robot_pos_ptrs[1] << " engin y: " << *(robot_pos_ptrs[1] + 1) << " sentry x: " << *robot_pos_ptrs[2] << " sentry y: " << *(robot_pos_ptrs[2] + 1));
            send_guess_points(&map_data, robot_pos_ptrs, last_send_time);
        } else {
            ROS_WARN_STREAM("hero x: " << *robot_pos_ptrs[0] << " hero y: " << *(robot_pos_ptrs[0] + 1) << " engin x: " << *robot_pos_ptrs[1] << " engin y: " << *(robot_pos_ptrs[1] + 1) << " sentry x: " << *robot_pos_ptrs[2] << " sentry y: " << *(robot_pos_ptrs[2] + 1));
            send_normal_points(&map_data, last_send_time);
        }
    }
}

void hero_pos_cb(const custom_msgs::points::ConstPtr msg){
    uint8_t id;
    float rx, ry;
    for(const custom_msgs::point &point: msg->data){
        id = point.id;
        rx = X_COEFF * point.x;
        ry = Y_COEFF * point.y;
        if(isFriendly(id)){
            ROS_ERROR_STREAM("Is a friend or itself, its id: " << static_cast<int>(id) << " our color is: " << IS_BLUE);
            continue;
        }
        const uint16_t cmd_id = 0x0301;
        uint8_t frame[24];
        memset(frame, 0, 24);
        uint8_t frame_size = 24;

        // header 1+2+1+1
        frame_header_t header;
        header.sof = 0xA5;
        header.data_length = 15;
        header.seq = 0;
        header.crc8 = 0;
        memcpy(frame, &header, 5);
        Append_CRC8_Check_Sum(frame, frame_header_length); // frame[HEADER_LEN - 1] = CRC8, 即第四位

        // protocol command id 0x0301 size: 2
        memcpy(frame+5, &cmd_id, 2);

        // data_header 2+2+2
        interaction_data_header_t interaction_data;
        interaction_data.data_cmd_id = 0x0201;
        interaction_data.sender_id = IS_BLUE * 100 + 9;
        interaction_data.receiver_id = IS_BLUE * 100 + 7;
        memcpy(frame+7, &interaction_data, 6);

        // data 1+4+4
        map_hero_data_t map_data;
        map_data.target_robot_id = id;
        map_data.target_position_x = rx;
        map_data.target_position_y = ry;
        memcpy(frame+13, &map_data, 9);

        // CRC16 2
        Append_CRC16_Check_Sum(frame, frame_size);

        // send frame
        if(sp.m_Send(frame, frame_size) > 0){
            ROS_ERROR("Hero pos sent successfully!");
        } else {
            ROS_ERROR("Hero pos sent fucked up");
        }
    }
}

// void hero_pos_cb(const custom_msgs::points::ConstPtr msg)
// {   
//     const uint16_t cmd_id = 0x0301;
//     uint8_t frame[24];
//     memset(frame, 0, 24);
//     uint8_t frame_size = 24;

//     // header 1+2+1+1
//     frame_header_t header;
//     header.sof = 0xA5;
//     header.data_length = 15;
//     header.seq = 0;
//     header.crc8 = 0;
//     memcpy(frame, &header, 5);
//     Append_CRC8_Check_Sum(frame, 5); // frame[HEADER_LEN - 1] = CRC8, 即第四位

//     // protocol command id 0x0301 size: 2
//     memcpy(frame+5, &cmd_id, 2);

//     // data_header 2+2+2
//     interaction_data_header_t interaction_data;
//     interaction_data.data_cmd_id = 0x0201;
//     interaction_data.sender_id = 109;
//     interaction_data.receiver_id = 107;
//     memcpy(frame+7, &interaction_data, 6);

//     // data 1+4+4
//     map_robot_data_t map_data;
//     map_data.target_robot_id = 5;
//     map_data.target_position_x = 0.5;
//     map_data.target_position_y = 1.9;
//     memcpy(frame+13, &map_data, 9);

//     // CRC16 2
//     Append_CRC16_Check_Sum(frame, frame_size);

//     // send frame
//     sp.m_Send(frame, frame_size);
// }

int main(int argc, char **argv){
    ros::init(argc, argv, "frame_data");
    ros::NodeHandle nh;
    // // color
    // std::string color;
    // ros::param::get("battle_color", color);
    // if(color == "red") IS_BLUE = false;
    // else IS_BLUE = true;
    //subsrcibers
    //to be implemented
    sub_hero = nh.subscribe<custom_msgs::points>("/serial_test", 10, &hero_pos_cb);
    sub_robot = nh.subscribe<custom_msgs::points>("/serial_test", 10, &robot_pos_cb);
    if(sp.OpenPort("/dev/ttyUSB0", 115200, 0,8,1))
        ROS_INFO_STREAM("Open serial port successfully");
    else
        ROS_ERROR_STREAM("Fail to open the serial port!!");

    ros::spin();
    return 0;
}

void send_normal_points(map_robot_data_t* map_data, std::chrono::steady_clock::time_point& last_send_time){
    const uint16_t cmd_id = 0x0305;
    uint8_t frame[33];
    memset(frame, 0, 33);
    uint8_t frame_size = 33;

    // header 1+2+1+1
    frame_header_t header;
    header.sof = 0xA5;
    header.data_length = 24;
    header.seq = 0;
    header.crc8 = 0;
    memcpy(frame, &header, frame_header_length);
    Append_CRC8_Check_Sum(frame, frame_header_length); // frame[HEADER_LEN - 1] = CRC8, 即第四位

    // protocol command id 0x0305 size: 2
    memcpy(frame+5, &cmd_id, 2);

    // data 2*12
    memcpy(frame+7, map_data, 24);

    // CRC16 2
    Append_CRC16_Check_Sum(frame, frame_size);

    // send frame
    if(sp.m_Send(frame, frame_size) > 0){
        ROS_ERROR("robot pos sent successfully!");
    } else {
        ROS_ERROR("robot pos sent fucked up");
    }
    last_send_time = std::chrono::steady_clock::now();

    // synchronize enemy_mark_data_before
    for(int i = 0; i < 3; i++){
        enemy_mark_data_before[i] = enemy_mark_data_now[i];
    }
}

void send_guess_points(map_robot_data_t* map_data, uint16_t* robot_pos_ptrs[], std::chrono::steady_clock::time_point& last_send_time){
    for(int i = 0; i < 3; i++){
        if(*robot_pos_ptrs[i] == 0 && *(robot_pos_ptrs[i] + 1) == 0){
            if(enemy_mark_data_now[i] < 120 &&
              (enemy_mark_data_now[i] - enemy_mark_data_before[i]) <= 0){
                guess_index[i] = 1 - guess_index[i];
                *robot_pos_ptrs[i] = guess_table.at(!IS_BLUE * 100 + i).at(guess_index[i]).first;
                *(robot_pos_ptrs[i] + 1) = guess_table.at(!IS_BLUE * 100 + i).at(guess_index[i]).second;
                enemy_mark_data_before[i] = enemy_mark_data_now[i];
            } else {
                *robot_pos_ptrs[i] = guess_table.at(!IS_BLUE * 100 + i).at(guess_index[i]).first;
                *(robot_pos_ptrs[i] + 1) = guess_table.at(!IS_BLUE * 100 + i).at(guess_index[i]).second;
                enemy_mark_data_before[i] = enemy_mark_data_now[i];
            }
        }
    }
    const uint16_t cmd_id = 0x0305;
    uint8_t frame[33];
    memset(frame, 0, 33);
    uint8_t frame_size = 33;

    // header 1+2+1+1
    frame_header_t header;
    header.sof = 0xA5;
    header.data_length = 24;
    header.seq = 0;
    header.crc8 = 0;
    memcpy(frame, &header, frame_header_length);
    Append_CRC8_Check_Sum(frame, frame_header_length); // frame[HEADER_LEN - 1] = CRC8, 即第四位

    // protocol command id 0x0305 size: 2
    memcpy(frame+5, &cmd_id, 2);

    // data 2*12
    memcpy(frame+7, map_data, 24);

    // CRC16 2
    Append_CRC16_Check_Sum(frame, frame_size);

    // send frame
    if(sp.m_Send(frame, frame_size) > 0){
        ROS_ERROR("guessed robot pos sent successfully!");
    } else {
        ROS_ERROR("robot pos sent fucked up");
    }
    last_send_time = std::chrono::steady_clock::now();
}
/*abandoned*/
// // send data through ros to 裁判系统, then to 己方所有选手端
// void Send_data(struct map_robot_data_frame *frame){
//     // fill in frame header
//     uint8_t tbuf[20];
//     frame->frame_header.sof = 0xa5;
//     frame->frame_header.data_length = map_robot_data_length;
//     frame->frame_header.seq = 0;
//     frame->cmd_id = 0x0305;
//     // tbuf now contains both the frame header and the ID & position data
//     memcpy(tbuf, &frame->frame_header, sizeof(map_robot_data_frame));
//     Append_CRC8_Check_Sum(tbuf, frame_header_length);
//     Append_CRC16_Check_Sum(tbuf, sizeof(map_robot_data_frame));
    
//     if(sp.m_Send(tbuf, sizeof(map_robot_data_frame)) > 0){
//         ROS_ERROR("Send robot data successfully");
//     } else{
//         ROS_ERROR("Failed to send data");
//     }
//     // custom_msgs::SendData message; // append custom msg
//     // message.length = sizeof(map_robot_data_frame);
//     // for(int i=0; i < message.length; ++i){
//     //     message.data.push_back(tbuf[i]);
//     // }
//     // message.name = "map enemy data frame";
//     // ROS_INFO("Publishing data with length %d and name '%s", message.length, message.name.c_str());
//     // pub_send_data.publish(message);
// }

// // firstly fill in the data of a frame, and then pass to Send_data to add header and publish
// void send_position(uint16_t target_id, float x, float y){
//     map_robot_data_frame frame;
//     Fill_map_robot_data(&frame.data, target_id, x, y);
//     Send_data(&frame);
// }



#include "communicator.hpp"
#include "std_msgs/UInt8MultiArray.h"
#include <chrono>

Communicator::Communicator(int argc, char** argv) : init_argc(argc), init_argv(argv) {
    enemy_mark_data_before = {
    {0, 0},
    {1, 0},
    {2, 0}
    };
    guess_index = {
    {0, 0},
    {1, 0},
    {2, 0}
    };
    milliseconds_after_lost = {
        {1, 0},
        {2, 0},
        {3, 0},
        {4, 0},
        {5, 0},
        {6, 0},
        {7, 0}
    };
}

bool Communicator::init(){
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);

    ros::init(init_argc, init_argv, "communicator");
    ros::NodeHandle nh;

    // Publishers
    robot_id_pub = nh.advertise<std_msgs::UInt8MultiArray>("robot_id", 5);
    hp_data_pub = nh.advertise<std_msgs::UInt16MultiArray>("hp_data", 5);
    mark_data_pub = nh.advertise<std_msgs::UInt8MultiArray>("mark_data", 5);
    robot_pos_pub = nh.advertise<std_msgs::Float32MultiArray>("ground_robot_pos", 100);
    sentry_pos_pub = nh.advertise<std_msgs::Float32MultiArray>("sentry_robot_pos", 100);
    gimbal_pos_pub = nh.advertise<custom_msgs::point>("gimbal_pos", 5);
    robot_HP_pub = nh.advertise<custom_msgs::game_state>("/GUI/topic/game_state_topic", 5);
    game_status_pub = nh.advertise<custom_msgs::game_state>("/GUI/topic/game_state_topic", 5);
    dart_info_pub = nh.advertise<custom_msgs::game_state>("/GUI/topic/supply_projectile_action_topic", 5);
    referee_warning_pub = nh.advertise<custom_msgs::referee_warning>("/GUI/topic/referee_warning_topic", 5);

    ROS_INFO_STREAM("Timer created to call function readData");
    timer_= nh.createTimer(ros::Duration(1.0), &Communicator::readData, this);
    /*test*/
    // timer1_= nh.createTimer(ros::Duration(1.0), &Communicator::initiate_double_dmg_cb_temp, this);
    timer1_ = nh.createTimer(ros::Duration(1.0), &Communicator::initiate_double_dmg_cb, this);

    /*sender*/
    sub_robot = nh.subscribe("/robot", 10, &Communicator::robot_pos_cb, this);
    sub_hero = nh.subscribe("/hero", 10, &Communicator::hero_pos_cb, this);
    
    if(sp.OpenPort("/dev/ttyUSB0", 115200, 0,8,1))
        ROS_INFO_STREAM("Open serial port successfully");
    else{
        ROS_ERROR_STREAM("Fail to open the serial port!!");
        return false;
    }
    return true;
}

void Communicator::readData(const ros::TimerEvent &event){
    int bytesAvailable = sp.getBufferBytes();
    uint8_t* buf = new uint8_t[bytesAvailable];
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
                    // ROS_WARN_STREAM("i = " << i << ", Command id: 0x0" << static_cast<uint16_t>(cmdID[0]) << "0" << static_cast<uint16_t>(cmdID[1]));
                    int index = i + frame_header_length + sizeof(uint16_t);
                    switch(cmdID[0]){
                        case 0:{
                            switch(cmdID[1]){
                                case 1:{// 0x0001
                                    game_state.game_type = buf[index] | 0b00001111;
                                    game_state.game_progress = (buf[index] & 0b11110000) >> 4;
                                    if(game_state.game_progress == 2 || game_state.game_progress == 3){
                                        double_dmg_timer = std::chrono::steady_clock::now();
                                    }
                                    game_state.stage_remain_time = (buf[index + 2] << 8) | buf[index + 1];
                                    // game_state.sync_timestamp = (buf[i + index + 7] << 24) | (buf[i + index + 6] << 16) | (buf[i + index + 5]) << 
                                    //                             8 | buf[i + index + 4];
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
                                    // // game_state.dart_aim_state = buf[i + index + 1];
                                    // dart_info_pub.publish(game_state);
                                    // target = (buf[i + index + 1] & 0b01100000) >> 5;
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
                                    // ROS_ERROR_STREAM("our id: " << static_cast<int>(robotStatus.robot_id) << " our color: " << IS_BLUE);
                                    // other info
                                    std_msgs::UInt8MultiArray msg;
                                    msg.data.push_back(robotStatus.robot_id);
                                    robot_id_pub.publish(msg);
                                    break;
                                }
                                case 2:{
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
                                    ROS_WARN_STREAM("3 enemies mark data: " << static_cast<int>(enemy_mark_data_now[0]) << ", " << static_cast<int>(enemy_mark_data_now[1]) << ", " << static_cast<int>(enemy_mark_data_now[2]));
                                    mark_data_pub.publish(mark_data);
                                    break;
                                }
                                case 0xE:{// input: 0x020E double damage data
                                    // double_dmg_chances = buf[index] & 0b00000011;
                                    // enemy_in_double_dmg = (buf[index] & 0b00000100) >> 2;
                                    // ROS_FATAL_STREAM("Current double damage chances: " << static_cast<int>(double_dmg_chances) << ", enemy in double damage: " << static_cast<int>(enemy_in_double_dmg));
                                    // static int flag = 0;
                                    // if(double_dmg_chances > temp){
                                    //     flag++;
                                    // }
                                    // if(!enemy_in_double_dmg){
                                    //     ROS_ERROR("flag = %d", flag);
                                    //     initiate_double_dmg(flag);
                                    // }
                                    // temp = double_dmg_chances;
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
                                    // custom_msgs::point point;
                                    // memcpy(&point.x, buf+index, 4);
                                    // memcpy(&point.y, buf+index + 4, 4);
                                    // memcpy(&point.id, buf + index + 9, 1);
                                    // calibration_points.data.push_back(point);
                                    // if(calibration_points.data.size() == 4){
                                    //     for(int i = 0; i < 4; i++){
                                    //         ROS_ERROR_STREAM("point " << i + 1 << " x = " << calibration_points.data[i].x << " y = " << calibration_points.data[i].y);
                                    //     }
                                    // }
                                    // gimbal_pos_pub.publish(point);
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
                    ROS_ERROR("CRC16 FAILED");
                }
                i += frame_header_length + sizeof(uint16_t) + DataLength +sizeof(uint16_t);
            }
            else{
                ROS_ERROR("CRC8 FAILED");
                i++;
            }
        }
        else{
            i++;
        }
    }
}

void Communicator::initiate_double_dmg_cb(const ros::TimerEvent &event){
    initiate_double_dmg_cb_temp(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(700));
    initiate_double_dmg_cb_temp(2);
}

/*test*/
void Communicator::initiate_double_dmg_cb_temp(int temp){
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
    frame.double_dmg_cmd = temp;
    memcpy(tbuf + 13, &frame.double_dmg_cmd, 1);
    
    Append_CRC16_Check_Sum(tbuf, 16);

    if (sp.m_Send(tbuf, 16)) // length of the frame
        ROS_ERROR("Send dbl dmg successfully!");
    else 
        ROS_ERROR("Fail to send!");
}

/*sender*/
bool Communicator::isFriendly(uint8_t id){
    if((IS_BLUE && id >= 100) || (!IS_BLUE && id < 100)) return true;
    return false;
}

void Communicator::robot_pos_cb(const custom_msgs::points::ConstPtr msg){
    static auto last_send_time = std::chrono::steady_clock::now();

    uint16_t id;
    bool used_id[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    float rx, ry;
    for(const custom_msgs::point &point: msg->data){
        id = point.id;
        if(isFriendly(id)){
            //ROS_ERROR_STREAM(std::to_string(rx).c_str());
            ROS_WARN_STREAM("Detected robot id is: " << static_cast<int>(id) << ", Is a friend!");
            continue;
        }

        if(id != 255){
            // found robot, reset its time
            milliseconds_after_lost[id % 100] = 0;
        } else {
            // found a idless robot, guess a id
            for(int j = 3; j < 6; j++){
                if(!used_id[j] && j != 6){
                    id = !IS_BLUE * 100 + j;
                    used_id[j] = true;
                    break;
                }
            }
        }
        used_id[id % 100] = true;
        rx = X_COEFF * point.x * 100;
        ry = Y_COEFF * point.y * 100;
        
        int offset = id - (static_cast<int>(!IS_BLUE) * 100) - 1;
        if(offset == 6) offset = 5;
        uint16_t *p = (uint16_t*)&map_data + offset*2;
        *p = static_cast<uint16_t>(rx);
        *(p+1) = static_cast<uint16_t>(ry);
        // ROS_WARN("id: %d, x: %f, y: %f", id, rx, ry);
        // ROS_WARN("offset: %d, x: %d, y: %d", offset, *p, *(p+1));
    }

    //send at 5hz
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send_time);

    // check if seconds after last is longer than 3,if true, change it to 0
    for(int j = 1; j < 8; j++){
        milliseconds_after_lost[j] += elapsed.count();
        if(milliseconds_after_lost[j] > 3000){
            if(j == 7){
                j = 6;
            }
            uint16_t *p = (uint16_t*)&map_data + 2 * (j - 1);
            *p = 0;
            *(p + 1) = 0;
        }
    }

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

void Communicator::hero_pos_cb(const custom_msgs::point::ConstPtr msg){
    uint8_t id;
    float rx, ry;
    
    id = msg->id;
    rx = X_COEFF * msg->x;
    ry = Y_COEFF * msg->y;
    if(isFriendly(id)){
        ROS_WARN_STREAM("Detected hero id is: " << static_cast<int>(id) << ", Is a friend!");
        return;
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
        ROS_ERROR_STREAM("Hero: " << static_cast<int>(id) << " pos sent successfully!");
    } else {
        ROS_ERROR("Hero pos sent fucked up");
    }
    
}

void Communicator::send_normal_points(const map_robot_data_t* map_data, std::chrono::steady_clock::time_point& last_send_time){
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

void Communicator::send_guess_points(map_robot_data_t* map_data, uint16_t* robot_pos_ptrs[], std::chrono::steady_clock::time_point& last_send_time){
    // take a guess
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

    Append_CRC16_Check_Sum(frame, frame_size);

    // send frame
    if(sp.m_Send(frame, frame_size) > 0){
        ROS_WARN("guessed robot pos sent successfully!");
    } else {
        ROS_ERROR("robot pos sent fucked up");
    }
    last_send_time = std::chrono::steady_clock::now();

    // DO NOT synchronize enemy_mark_data_before
}

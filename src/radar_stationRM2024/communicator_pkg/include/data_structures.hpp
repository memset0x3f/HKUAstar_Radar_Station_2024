#ifndef DATA_STRUCTURES_H
#define DATA_STRUCTURES_H

#include <cstddef>
#include <stdint.h>

#pragma pack(push, 1)
struct frame_header_t
{
    uint8_t sof;         ///< 起始字节(0xA5)
    uint16_t data_length; ///< 数据长度
    uint8_t seq;         ///< 包序号
    uint8_t crc8;        ///< 帧头CRC8校验
};


//001 match status
struct match_status_t
{
    uint8_t game_type : 4;     ///< 比赛类型：1:机甲大师赛 2:单项赛 3:人工智能挑战赛
    uint8_t game_progress : 4; ///< 当前比赛阶段：0:未开始比赛 1:准备阶段 2:自检阶段 3:5s倒计时 4:对战中 5:比赛结算中
    uint16_t stage_remain_time; ///< 当前阶段剩余时间(s)
    uint64_t sync_timestamp;
};

//003 robot hp
struct game_robot_HP_t
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
};
//0x0104 referee warning
struct referee_warning_t{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
};
//0x0105 dart info
struct dart_info_t{
    uint8_t dart_remaining_time;
    uint8_t dart_aim_state;
};
//0x0201 robot status
struct robot_status_t
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t remain_HP;
    uint16_t max_HP;
    uint16_t shooter_id1_17mm_cooling_rate;
    uint16_t shooter_id1_17mm_cooling_limit;
    uint16_t shooter_id1_17mm_speed_limit;
    uint16_t shooter_id2_17mm_cooling_rate;
    uint16_t shooter_id2_17mm_cooling_limit;
    uint16_t shooter_id2_17mm_speed_limit;
    uint16_t shooter_id1_42mm_cooling_rate;
    uint16_t shooter_id1_42mm_cooling_limit;
    uint16_t shooter_id1_42mm_speed_limit;
    uint16_t chassis_power_limit;
    uint8_t mains_power_gimbal_output : 1;
    uint8_t mains_power_chassis_output : 1;
    uint8_t mains_power_shooter_output : 1;
};

//0x0203 self position
struct robot_pos_t
{
    float x;
    float y;
    float z;
    float angle;
};

//0x020B ground robot postion data sent from server to sentry
struct ground_robot_position_t
{
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float standard_5_x;
    float standard_5_y;
};

//0x020C mark data
struct radar_mark_data_t
{
    uint8_t mark_hero_progress;
    uint8_t mark_engineer_progress;
    uint8_t mark_standard_3_progress;
    uint8_t mark_standard_4_progress;
    uint8_t mark_standard_5_progress;
    uint8_t mark_sentry_progress;
};

//0x0301
struct interaction_data_header_t
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
};
//0x0301->0x0200
struct sentry_radar_frame
{
    frame_header_t frame_header;//5
    uint16_t cmd_id;//2
    interaction_data_header_t data_header;//6
    uint8_t robot_position[40];//40
    uint16_t crc16;//2
};
//0x0301->0x0201
struct sentry_pos_frame
{
    frame_header_t frame_header;//5
    uint16_t cmd_id;//2
    interaction_data_header_t data_header;//6
    uint8_t sentry_position[8];//8
    uint16_t crc16;//2
};

//0x0121 double damage
struct radar_cmd_frame
{
    frame_header_t frame_header;// 5
    uint16_t cmd_id;// 2
    interaction_data_header_t data_header;// 6
    uint8_t double_dmg_cmd;// 1
    uint16_t crc16;// 2
};

//0x0305 target robot position
struct map_robot_data_t
{
    map_robot_data_t()
    :   hero_position_x(0), hero_position_y(0),
        engineer_position_x(0), engineer_position_y(0),
        infantry_3_position_x(0), infantry_3_position_y(0),
        infantry_4_position_x(0), infantry_4_position_y(0),
        infantry_5_position_x(0), infantry_5_position_y(0),
        sentry_position_x(0), sentry_position_y(0)
    {}
    uint16_t hero_position_x;
    uint16_t hero_position_y;
    uint16_t engineer_position_x;
    uint16_t engineer_position_y;
    uint16_t infantry_3_position_x;
    uint16_t infantry_3_position_y;
    uint16_t infantry_4_position_x;
    uint16_t infantry_4_position_y;
    uint16_t infantry_5_position_x;
    uint16_t infantry_5_position_y;
    uint16_t sentry_position_x;
    uint16_t sentry_position_y;
};
struct map_hero_data_t
{
    uint8_t target_robot_id;
    float target_position_x;
    float target_position_y;
};
struct map_robot_data_frame
{
    frame_header_t frame_header; //5
    uint16_t cmd_id; //2
    map_robot_data_t data; //9
    uint16_t crc16; //2
};

struct map_hero_data_frame
{
    frame_header_t frame_header; // 5
    uint16_t cmd_id; // 2
    interaction_data_header_t data_header; // 6
    map_robot_data_t data; // 10
    uint16_t crc16; // 2
};

#pragma pack(pop)



const size_t frame_header_length = sizeof(frame_header_t);
const size_t map_robot_data_length = sizeof(map_robot_data_t);

extern uint16_t enemy_mark_data_now[3];
extern bool IS_BLUE;

#endif

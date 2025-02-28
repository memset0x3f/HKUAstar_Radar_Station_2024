cmake_minimum_required(VERSION 3.0.2)
project(custom_msgs)

find_package(catkin REQUIRED COMPONENTS
    message_generation
    message_runtime
    sensor_msgs
    std_msgs
)

add_message_files(
    FILES
    point.msg
    points.msg
    dis_point.msg
    dis_points.msg
    yolo_point.msg
    yolo_points.msg
    game_state.msg
    referee_warning.msg
    supply_projectile_action.msg
    SendData.msg
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    DEPENDS
)

#include "communicator.hpp"

int main(int argc, char** argv){
    Communicator communicator(argc, argv);
    if(communicator.init()){
        ros::spin();
    } else {
        ROS_FATAL("the whole communicator fucked up!!!");
    }
    return 0;
}
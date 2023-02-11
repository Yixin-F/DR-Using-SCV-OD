#include "ssc.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "ufo");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug); 
    ROS_INFO("\033[1;32m----> ufo Started.\033[0m");

    // test 2023.2.6
    SSC ssc;
    ssc.segDF();

    ros::spin();

    return 0;
}
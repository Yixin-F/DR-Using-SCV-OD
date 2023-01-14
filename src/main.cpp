#include "ssc.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "ufo");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug); 
    ROS_INFO("\033[1;32m----> ufo Started.\033[0m");

    // data load
    pcl::PointCloud<pcl::PointXYZI>::Ptr test(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::io::loadPCDFile("/home/fyx/ufo_hiahia/src/test/000179.pcd", *test);

    // ssc test: 2023.1.12 start ~
    SSC ssc;

    ssc.process(test);

    ros::spin();

    return 0;
}
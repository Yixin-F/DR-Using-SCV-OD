#include "ssc.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "ufo");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug); 
    ROS_INFO("\033[1;32m----> ufo Started.\033[0m");

    // // data load
    // pcl::PointCloud<pcl::PointXYZI>::Ptr test(new pcl::PointCloud<pcl::PointXYZI>());
    // pcl::io::loadPCDFile("/home/fyx/ufo_hiahia/src/test/000179.pcd", *test);

    // // ssc test: 2023.1.12 start ~2023.1.17
    // SSC ssc(0);

    // ssc.process(test);

    // ssc.segment();

    // ssc.recognize();

    SSC ssc(999);
    std::string pose_path = "/home/fyx/ufo_final/src/dataset/session/01_session/transformations.pcd";
    std::string cloud_path = "/home/fyx/ufo_final/src/dataset/session/01_session/Scans/";
    pcl::PointCloud<Pose>::Ptr pose(new pcl::PointCloud<Pose>());
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_vec;
    ssc.getPose(pose, pose_path);
    ssc.getCloud(cloud_vec, cloud_path);
    std::cout << "pose :" << pose->points.size() << " cloud_vec: " << cloud_vec.size() << std::endl;

    std::vector<Frame> frame_ssc;
    for(int i = 0; i < 2; i++){
        SSC ssc(i);
        ssc.process(cloud_vec[i]);
        ssc.segment();
        ssc.recognize();
    }

    ros::spin();

    return 0;
}
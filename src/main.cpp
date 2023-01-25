#include "ssc.h"

int main(int argc, char** argv){
    ros::init(argc, argv, "ufo");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug); 
    ROS_INFO("\033[1;32m----> ufo Started.\033[0m");

    // data load
    pcl::PointCloud<pcl::PointXYZI>::Ptr test(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::io::loadPCDFile("/home/fyx/ufo_hiahia/src/test/000179.pcd", *test);

    // // ssc test: 2023.1.12 start ~2023.1.17
    // SSC ssc;

    // ssc.process(test);
    // ssc.intensityVisualization(ssc.cloud_use);

    // ssc.segment();

    // ssc.recognize();
    // std::string path = "/home/fyx/ufo_hiahia/src/test/";
    // ssc.saveSegCloud(ssc.frame_ssc, path, ssc.id, "_seg.pcd");

    // std::cout << ssc.frame_ssc.cluster_set.size() << std::endl;
    // int count = 0;
    // for(int i = 0; i < ssc.frame_ssc.cluster_set.size(); i++){
    //     std::cout << ssc.frame_ssc.cluster_set[i].occupy_vcs.size() << " ";
    //     count += ssc.frame_ssc.cluster_set[i].occupy_voxels.size();
    // }
    // std::cout << std::endl;
    // std::cout << count << std::endl;

    // std::cout << ssc.frame_ssc.vox_cloud->points.size() << std::endl;
    // for(int i = 0; i < ssc.frame_ssc.vox_cloud->points.size(); i++){
    //     std::cout << ssc.frame_ssc.vox_cloud->points[i].intensity << " ";
    // }

    SSC ssc;
    std::string pose_path = "/home/fyx/ufo_final/src/dataset/session/01_session/transformations.pcd";
    std::string cloud_path = "/home/fyx/ufo_final/src/dataset/session/01_session/Scans/";
    pcl::PointCloud<Pose>::Ptr pose(new pcl::PointCloud<Pose>());
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_vec;
    ssc.getPose(pose, pose_path);
    ssc.getCloud(cloud_vec, cloud_path);

    std::vector<Frame> frames;
    std::vector<Pose> poses;
    for(int i = 0; i < 6; i++){
        ssc.process(cloud_vec[i]);
        ssc.segment();
        ssc.recognize();
        std::string path = "/home/fyx/ufo_hiahia/src/test/";
        ssc.saveSegCloud(ssc.frame_ssc, path, ssc.id, "_seg.pcd");
        frames.emplace_back(ssc.frame_ssc);
        poses.emplace_back(pose->points[i]);
        ssc.reset();
        ssc.id ++;
        std::cout << std::endl;
    }

    std::vector<Frame> frame2Initial;
    std::vector<Pose> pose2Initial;
    for(int k = 0; k < 5; k++){
        frame2Initial.emplace_back(frames[k]);
        pose2Initial.emplace_back(pose->points[k]);
    }
    std::cout << "start initialization" << std::endl;
    Frame frame_initial = ssc.intialization(frame2Initial, pose2Initial);

    ros::spin();

    return 0;
}
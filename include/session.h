#ifndef _SESSION_H_
#define _SESSION_H_

#include "utility.h"
#include "common.h"

class Session: public Utility{
public:
    // pcl::PointCloud<Pose>::Ptr pose_1;
    // pcl::PointCloud<Pose>::Ptr pose_2;

    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> session_seg_1;
    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> session_seg_2;

    Session();
    ~Session();

    void allocateMemory();
    void getPose(pcl::PointCloud<Pose>::Ptr& pose_, const std::string& pose_path_);
    void getCloudSeg(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& session_seg_, pcl::PointCloud<Pose>::Ptr& pose_, const std::string& in_path_, const std::string& out_path_);
    void getReloInfo(std::vector<cv::Mat>& relo_vec_, const std::string& relo_path_, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& build_vec_, const std::string& build_path_);

};

#endif
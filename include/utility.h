#pragma once

#ifndef _UTILITY_H_
#define _UTILITY_H_

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/image_encodings.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/features/esf.h>   

#include <vector>
#include <cmath>
#include <algorithm>
#include <stdlib.h>
#include <unordered_map>
#include <utility>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <random>
#include <filesystem> // requires gcc version >= 8
#include <yaml-cpp/yaml.h> // yaml

#include "assert.h"
#include "tictoc.h"

namespace fs = std::filesystem; // file-process

// pose-point cloud from lio-sam
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   
} EIGEN_ALIGN16;                  

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))
typedef PointXYZIRPYT  Pose;

// apiric-format of point
struct PointAPRI{
    float x, y, z;
    float range;
    float angle;
    float azimuth;
    float intensity = 0.f;
    int range_idx = -1;
    int sector_idx = -1 ;
    int azimuth_idx = -1;
    int voxel_idx = -1;  // id in voxel cloud
};

// voxel-type in hash cloud
struct Voxel{
    int range_idx;
    int sector_idx;
    int azimuth_idx;
    int label = -1;
    pcl::PointXYZI center;   // the point center's intensity is its id in voxel cloud
    std::vector<int> ptIdx;  // the vector of id in noground cloud
    std::vector<float> intensity_record;
    float intensity_av = 0.f;
    float intensity_cov = 0.f;
};

// feature values
struct FeatureValue{
    FeatureValue() = delete;
    FeatureValue(std::string value_name_, double value_) : name(value_name_), value(value_) {}
    ~FeatureValue() {}

    std::string name = "";
    double value = 0.0;
};

// one type of feature
struct Feature{
    Feature() = delete;
    Feature(std::string feature_name): name(feature_name) {}
    ~Feature() {}

    std::string name = "";
    std::vector<FeatureValue> feature_values;
};

// one cluster
struct Cluster{
    Cluster(){
        allocateMemory();
    }
    ~Cluster() {}
    
    void allocateMemory(){
        cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    }

    int track_id = -1;  // tracking
    int name = -1;  
    int type = -1;  // building, tree, car
    int state = -1;   //  dynamic 1, static 0 
    int color[3];
    std::pair<pcl::PointXYZI, pcl::PointXYZI> bounding_box;
    std::vector<int> occupy_pts;  // pt id in cloud_use, prepared to evalute
    std::vector<int> occupy_voxels;  // id in hash cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    Eigen::MatrixXd feature_matrix;
};

// one frame
struct Frame{
    Frame(){
        allocateMemory();
    }
    ~Frame() {}
    void allocateMemory(){
        vox_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        cloud_use.reset(new pcl::PointCloud<pcl::PointXYZI>());
    }

    int id;
    int max_name;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_use;
    std::unordered_map<int, Voxel> hash_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr vox_cloud;  // voxel cloud
    std::unordered_map<int, Cluster> cluster_set;
};

class Utility{
public:
    std::string out_path;
    int kNumOmpCores;
    bool save;
    bool mapping_init;
    bool is_pcd;
    int skip;

    std::string data_path;
    std::string label_path;
    std::string pose_path;
    int init;
    int start;
    int end;

    std::string calib_path;
    std::string seg_path;
    std::string pcd_path;
    std::string map_path;
 
    float sensor_height;
    float min_dis;
    float max_dis;
    float min_angle;
    float max_angle;
    float min_azimuth;
    float max_azimuth;
    float range_res;
    float sector_res;
    float azimuth_res;

    float max_intensity;
    float correct_ratio;
    float correct_radius;
    int search_num;

    int iteration;
    int toBeClass;
    int search_c;
    float intensity_diff;
    float intensity_cov;
    float occupancy;

    int building;
    int tree;
    int car;

    std::vector<int> dynamic_label;
    std::vector<float> tr_v;
    Eigen::Matrix4f tr;

    double kOneThird;
    double kLinearityMax;
    double kPlanarityMax;
    double kScatteringMax;
    double kOmnivarianceMax;
    double kAnisotropyMax;
    double kEigenEntropyMax;
    double kChangeOfCurvatureMax;
    double kNPointsMax;

    ros::NodeHandle nh;

    virtual ~Utility(){}

    Utility(){
        nh.param<std::string>("common/out_path_", out_path,  " ");
        nh.param<int>("common/kNumOmpCores_", kNumOmpCores, 6);
        nh.param<bool>("common/save_", save, true);
        nh.param<bool>("common/mapping_init_", mapping_init, false);
        nh.param<bool>("common/is_pcd_", is_pcd, false);
        nh.param<int>("common/skip_", skip, 2);

        nh.param<std::string>("session/data_path_", data_path, " ");
        nh.param<std::string>("session/label_path_", label_path, " ");
        nh.param<std::string>("session/pose_path_", pose_path, " ");
        nh.param<int>("session/init_", init, 5);
        nh.param<int>("session/start_", start, 5);
        nh.param<int>("session/end_", end, 50);

        nh.param<std::string>("ssc/calib_path_", calib_path, " ");
        nh.param<std::string>("ssc/seg_path_", seg_path, " ");
        nh.param<std::string>("ssc/pcd_path_", pcd_path, " ");
        nh.param<std::string>("ssc/map_path_", map_path, " ");

        nh.param<float>("ssc/sensor_height_", sensor_height, 2.0);
        nh.param<float>("ssc/min_dis_", min_dis, 0.0);
        nh.param<float>("ssc/max_dis_", max_dis, 50.0);
        nh.param<float>("ssc/min_angle_", min_angle, 0.0);
        nh.param<float>("ssc/max_angle_", max_angle, 360.0);
        nh.param<float>("ssc/min_azimuth_", min_azimuth, -30.0);
        nh.param<float>("ssc/max_azimuth_", max_azimuth, 60.0);
        nh.param<float>("ssc/range_res_", range_res, 0.2);
        nh.param<float>("ssc/sector_res_", sector_res, 1.2);
        nh.param<float>("ssc/azimuth_res_", azimuth_res, 2.0);
        nh.param<float>("ssc/max_intensity_", max_intensity, 200.0);
        nh.param<float>("ssc/correct_radius_", correct_radius, 0.5);
        nh.param<float>("ssc/correct_ratio_", correct_ratio, 0.5);
        nh.param<int>("ssc/search_num_", search_num, 10);
        nh.param<int>("ssc/iteration_", iteration, 3);
        nh.param<int>("ssc/toBeClass_", toBeClass, 1);
        nh.param<int>("ssc/search_c_", search_c, 2);
        nh.param<float>("ssc/intensity_diff_", intensity_diff, 50);
        nh.param<float>("ssc/intensity_cov_", intensity_cov, 20);
        nh.param<float>("ssc/occupancy_", occupancy, 0.6);
        nh.param<int>("ssc/building_", building, 0);
        nh.param<int>("ssc/tree_", tree, 1);
        nh.param<int>("ssc/car_", car, 2);
        nh.param<std::vector<int>>("ssc/dynamic_label_", dynamic_label, std::vector<int>());
        nh.param<std::vector<float>>("ssc/tr_", tr_v, std::vector<float>());
        tr = Eigen::Map<const Eigen::Matrix<float, -1, -1, Eigen::RowMajor>>(tr_v.data(), 4, 4);

        nh.param<double>("feature/kOneThird_", kOneThird, 0.333);
        nh.param<double>("feature/kLinearityMax_",  kLinearityMax, 740.0);
        nh.param<double>("feature/kPlanarityMax_", kPlanarityMax, 959.0);
        nh.param<double>("feature/kScatteringMax_", kScatteringMax, 1248.0);
        nh.param<double>("feature/kOmnivarianceMax_", kOmnivarianceMax, 0.278636);
        nh.param<double>("feature/kAnisotropyMax_", kAnisotropyMax, 1248.0);
        nh.param<double>("feature/kEigenEntropyMax_", kEigenEntropyMax, 0.956129);
        nh.param<double>("feature/kChangeOfCurvatureMax_", kChangeOfCurvatureMax, 0.99702);
        nh.param<double>("feature/kNPointsMax_", kNPointsMax, 13200.0);
    }

    void fsmkdir(std::string _path){
        if(!fs::is_directory(_path) || !fs::exists(_path)) 
            fs::create_directories(_path);   // restore new
    } 

    template<typename CloudT> 
    sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, CloudT thisCloud, ros::Time thisStamp, std::string thisFrame){
        sensor_msgs::PointCloud2 tmp_cloud;
        pcl::toROSMsg(*thisCloud, tmp_cloud);
        tmp_cloud.header.stamp = thisStamp;
        tmp_cloud.header.frame_id = thisFrame;
        if(thisPub->getNumSubscribers() != 0){
            thisPub->publish(tmp_cloud);
        }
        return tmp_cloud;
    }

    template<typename T> 
    float rad2deg(const T& radians){
        return (float)radians * 180.0 / M_PI; 
    }

    template<typename T> 
    float deg2rad(const T& degrees){
        return (float)degrees * M_PI / 180.0; 
    }
    
    template<typename PointT> 
    float pointDistance3d(const PointT& p1, const PointT& p2){
        return (float)sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
    }

    template<typename PointT> 
    float pointDistance2d(const PointT& p1, const PointT& p2){
        return (float)sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y));
    }

    template<typename PointT> 
    float pointDistance3d(const PointT& p1){
        return (float)sqrt((p1.x)*(p1.x) + (p1.y)*(p1.y) + (p1.z)*(p1.z));
    }

    template<typename PointT> 
    float pointDistance2d(const PointT& p1){
        return (float)sqrt((p1.x)*(p1.x) + (p1.y)*(p1.y));
    }

    template<typename PointT> 
    float getPolarAngle(const PointT& p){
        if(p.x == 0 && p.y == 0){
            return 0.f;
        }
        else if(p.y >= 0){
            return (float)rad2deg((float)atan2(p.y, p.x));
        }
        else if(p.y < 0){
            return (float)rad2deg((float)atan2(p.y, p.x) + 2*M_PI);
        }
    }

    template<typename PointT> 
    float getAzimuth(const PointT& p){
        return (float)rad2deg((float)atan2(p.z, (float)pointDistance2d(p)));
    }

    template<typename CloudT> 
    void transformCloud(const CloudT& cloudIn_, const Eigen::Affine3f& transCur_, CloudT& cloudOut_){
        int cloudSize = cloudIn_->points.size();
        cloudOut_->points.resize(cloudSize);
    
        #pragma omp parallel for num_threads(kNumOmpCores)
        for(int i = 0; i < cloudSize; i++){
            cloudOut_->points[i].x = transCur_(0,0) * cloudIn_->points[i].x + transCur_(0,1) * cloudIn_->points[i].y + transCur_(0,2) * cloudIn_->points[i].z + transCur_(0,3);
            cloudOut_->points[i].y = transCur_(1,0) * cloudIn_->points[i].x + transCur_(1,1) * cloudIn_->points[i].y + transCur_(1,2) * cloudIn_->points[i].z + transCur_(1,3);
            cloudOut_->points[i].z = transCur_(2,0) * cloudIn_->points[i].x + transCur_(2,1) * cloudIn_->points[i].y + transCur_(2,2) * cloudIn_->points[i].z + transCur_(2,3);
            cloudOut_->points[i].intensity = cloudIn_->points[i].intensity;
        }
    }

    template<typename CloudT>
    void saveCloud(const CloudT& cloud_, const std::string& path_, const int& id, const std::string& name_){  // root path
        std::string save_path = path_ + std::to_string(id) + name_;
        cloud_->height = 1;
        cloud_->width = cloud_->points.size();
        if(save){
            if(cloud_->points.size() == 0 || pcl::io::savePCDFile(save_path, *cloud_) == -1){
                ROS_WARN("%s save error ", (std::to_string(id) + name_).c_str());
            }
            ROS_DEBUG("cloud save: %s save success, pt_num: %d", save_path.c_str(), (int)cloud_->points.size());
        }
    }

    template<typename CloudT>
    void loadCloud(CloudT& cloud_, const std::string& path_){
        if(pcl::io::loadPCDFile(path_, *cloud_) == -1){
            ROS_WARN("pose file %s load error", path_.c_str());
            ROS_BREAK();
        }
        else{
            ROS_DEBUG("cloud load: %s load success, pt_num: %d", path_.c_str(), (int)cloud_->points.size());
        }
    }

    template<typename CloudT>
    void getCloudByVec(const CloudT& cloud_, const std::vector<int>& vec_, const CloudT& cloud_out_){
        for(auto& it : vec_){
            cloud_out_->points.push_back(cloud_->points[it]);
        }
    }


    template<typename T>
    void addVec(std::vector<T>& vec_central_, const std::vector<T>& vec_add_){
        vec_central_.insert(vec_central_.end(), vec_add_.begin(), vec_add_.end());
    }

    template<typename T>
    void reduceVec(std::vector<T>& vec_central_, const std::vector<T>& vec_reduce_){
        for(auto it = vec_reduce_.begin(); it != vec_reduce_.end(); it++){
            vec_central_.erase(std::remove(vec_central_.begin(), vec_central_.end(), *it), vec_central_.end());
        }
    }

    template<typename T>
    void sampleVec(std::vector<T>& vec_central_){
        std::sort(vec_central_.begin(), vec_central_.end());
        vec_central_.erase(std::unique(vec_central_.begin(), vec_central_.end()), vec_central_.end());
    }

    bool findNameInVec(const int& name_, const std::vector<int>& vec_){
        if(std::count(vec_.begin(), vec_.end(), name_)){
            return true;
        }
        else{
            return false;
        }
    }
    
    bool findNameInVec(const std::vector<int>& vec1_, const std::vector<int>& vec2_){
        for(auto& i : vec2_){
            if(findNameInVec(i, vec1_)){
                return true;
            }
            else{
                continue;
            }
        }
        return false;
    }

    Eigen::MatrixXd turnVec2Matrix(const std::vector<FeatureValue>& vec_){
        int size = vec_.size();
        Eigen::MatrixXd m(1, size);
        for(int i = 0; i < vec_.size(); i++){   
            m(0, i) = vec_[i].value;   // row add
        }
        return m;
    }

    Eigen::Vector3f rotationMatrixToEulerAngles(Eigen::Matrix3f &R){
        float sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
        bool singular = sy < 1e-6;
        float x, y, z;
        if (!singular)
        {
            x = atan2( R(2,1), R(2,2));
            y = atan2(-R(2,0), sy);
            z = atan2( R(1,0), R(0,0));
        }
        else
        {
            x = atan2(-R(1,2), R(1,1));
            y = atan2(-R(2,0), sy);
            z = 0;
        }
        return {x, y, z};
    }
};

#endif
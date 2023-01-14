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
    std::vector<int> ptIdx;  // the vector of id in noground cloud
    pcl::PointXYZI center;   // the point center's intensity is its id in voxel cloud
    std::unordered_map<int, int> clusterNameAndNum;
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
        cluster.reset(new pcl::PointCloud<pcl::PointXYZI>());
    }
    
    std::vector<int> occupy_aprics;
    std::vector<int> occupy_voxels;
    std::string type = "";
    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster;
    pcl::PointXYZI cluster_center;
    std::vector<Feature> feature_set;
};

// one frame
struct Frame{
    Frame(){
        allocateMemory();
    }
    ~Frame() {}
    void allocateMemory(){
        center_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    }

    float min_dis;
    float max_dis;
    std::vector<Cluster> cluster_set;
    pcl::PointCloud<pcl::PointXYZI>::Ptr center_cloud;
};

namespace fs = std::filesystem; // file-process
typedef PointXYZIRPYT  Pose;

class Utility{
public:
    std::string out_path;
    int kNumOmpCores;
    bool save;
    bool mapping_init;
    float downsample_size;

    std::string bin_path_1;
    std::string pcd_path_1;
    std::string pose_path_1;
    std::string out_path_1;
    std::string bin_path_2;
    std::string pcd_path_2;
    std::string pose_path_2;
    std::string out_path_2;

    std::string segment_path_1;
    std::string segment_path_2;
    std::string relo_path_1;
    std::string relo_path_2;
    std::string map_path_1;
    std::string map_path_2;
    float min_dis_add;
    float max_dis_ratio;
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

    int toBeClass;
    float intensity_diff;
    float curvature_diff;
    float intensity_cov;
    float curvature_cov;
    int vox_search;
    float fusion_thre;
    float occupancy;
    int iterator_times;
    float height;
    int building;
    int tree;
    int car;
    int other;

    double kOneThird;
    double kLinearityMax;
    double kPlanarityMax;
    double kScatteringMax;
    double kOmnivarianceMax;
    double kAnisotropyMax;
    double kEigenEntropyMax;
    double kChangeOfCurvatureMax;
    double kNPointsMax;
    int element_num;
    int same_num;
    float same_score;
    float linearity_thre;
    float planarity_thre;
    float scattering_thre;

    ros::NodeHandle nh;

    virtual ~Utility(){}

    Utility(){
        nh.param<std::string>("common/out_path_", out_path,  " ");
        nh.param<int>("common/kNumOmpCores_", kNumOmpCores, 6);
        nh.param<bool>("common/save_", save, true);
        nh.param<bool>("common/mapping_init_", mapping_init, false);
        nh.param<float>("common/downsample_size_", downsample_size, 0.1);

        nh.param<std::string>("session/bin_path_1_", bin_path_1, " ");
        nh.param<std::string>("session/pcd_path_1_", pcd_path_1, " ");
        nh.param<std::string>("session/pose_path_1_", pose_path_1, " ");
        nh.param<std::string>("session/out_path_1_", out_path_1, " ");
        nh.param<std::string>("session/bin_path_2_", bin_path_2, " ");
        nh.param<std::string>("session/pcd_path_2_", pcd_path_2, " ");
        nh.param<std::string>("session/pose_path_2_", pose_path_2,  " ");
        nh.param<std::string>("session/out_path_2_", out_path_2, " ");

        nh.param<std::string>("ssc/segment_path_1_", segment_path_1, " ");
        nh.param<std::string>("ssc/segment_path_2_", segment_path_2, " ");
        nh.param<std::string>("ssc/relo_path_1_", relo_path_1, " ");
        nh.param<std::string>("ssc/relo_path_2_", relo_path_2, " ");
        nh.param<std::string>("ssc/map_path_1_", map_path_1, " ");
        nh.param<std::string>("ssc/map_path_2_", map_path_2, " ");
        nh.param<float>("ssc/min_dis_add_", min_dis_add, 1.0);
        nh.param<float>("ssc/max_dis_ratio_", max_dis_ratio, 0.8);
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

        nh.param<int>("ssc/toBeClass_", toBeClass, 1);
        nh.param<float>("ssc/intensity_diff_", intensity_diff, 50);
        nh.param<float>("ssc/curvature_diff_", curvature_diff, 2.5);
        nh.param<float>("ssc/intensity_cov_", intensity_cov, 20);
        nh.param<float>("ssc/curvature_cov_", curvature_cov, 1.5);
        nh.param<int>("ssc/vox_search_", vox_search, 27);
        nh.param<float>("ssc/fusion_thre_", fusion_thre, 0.4);
        nh.param<float>("ssc/occupancy_", occupancy, 0.6);
        nh.param<float>("ssc/height_", height, 2.0);
        nh.param<int>("ssc/iterator_times_", iterator_times, 3);
        nh.param<int>("ssc/building_", building, 0);
        nh.param<int>("ssc/tree_", tree, 1);
        nh.param<int>("ssc/car_", car, 2);
        nh.param<int>("ssc/other_", other, 3);

        nh.param<double>("feature/kOneThird_", kOneThird, 0.333);
        nh.param<double>("feature/kLinearityMax_",  kLinearityMax, 740.0);
        nh.param<double>("feature/kPlanarityMax_", kPlanarityMax, 959.0);
        nh.param<double>("feature/kScatteringMax_", kScatteringMax, 1248.0);
        nh.param<double>("feature/kOmnivarianceMax_", kOmnivarianceMax, 0.278636);
        nh.param<double>("feature/kAnisotropyMax_", kAnisotropyMax, 1248.0);
        nh.param<double>("feature/kEigenEntropyMax_", kEigenEntropyMax, 0.956129);
        nh.param<double>("feature/kChangeOfCurvatureMax_", kChangeOfCurvatureMax, 0.99702);
        nh.param<double>("feature/kNPointsMax_", kNPointsMax, 13200.0);
        nh.param<int>("feature/element_num_", element_num, 18);
        nh.param<int>("feature/same_num_", same_num, 6);
        nh.param<float>("feature/same_score_", same_score, 0.5);
        nh.param<float>("feature/linearity_thre_", linearity_thre, 0.01);
        nh.param<float>("feature/planarity_thre_", planarity_thre, 0.01);
        nh.param<float>("feature/scattering_thre_", scattering_thre, 0.01);


    }

    void fsmkdir(std::string _path){
        // if(fs::exists(_path))
        //     fs::remove(_path);   // remove old
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
        if(1){
            if(cloud_->points.size() == 0 || pcl::io::savePCDFile(save_path, *cloud_) == -1){
                ROS_WARN("%s save error ", (std::to_string(id) + name_).c_str());
            }
            ROS_DEBUG("cloud save: %s save success, pt_num %d", save_path.c_str(), (int)cloud_->points.size());
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

    Eigen::MatrixXd turnVec2Matrix(const std::vector<FeatureValue>& vec_){
        int size = vec_.size();
        Eigen::MatrixXd m(1, size);
        for(int i = 0; i < vec_.size(); i++){   
            m(0, i) = vec_[i].value;   // row add
        }
        return m;
    }
    
};

#endif
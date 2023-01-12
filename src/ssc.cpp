#include "ssc.h"

int SSC::id = 0;

SSC::~SSC() {}

SSC::SSC(){
    allocateMemory();
    std::cout << "----  SSC INITIALIZATION  ----" << "\n"
                       << "range_res: " << range_res << " sector_res: " << sector_res << " azimuth_res: " << azimuth_res << "\n"
                       << "min_dis_add: " << min_dis_add << " max_dis_ratio: " << max_dis_ratio << "\n"
                       << "min_angle: " << min_angle << " max_angle: " << max_angle << "\n"
                       << "min_azimuth: " << min_azimuth << " max_azimuth: " << max_azimuth << "\n"
                       << std::endl;               
}

void SSC::allocateMemory(){
    src_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    ng_cloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
    ng_cloud_calib.reset(new pcl::PointCloud<pointCalib>());
    PatchworkGroundSeg.reset(new PatchWork<pcl::PointXYZI>());
}

void SSC::reset(){
    range_num = -1;
    sector_num = -1;
    azimuth_num = -1;
    bin_num = -1;

    min_dis = -1; 
    max_dis = -1;

    Frame frame_new;
    frame_ssc = frame_new;

    src_cloud->clear();
    ng_cloud->clear();
    ng_cloud_calib->clear();
}

void SSC::extractGroudByPatchWork(const pcl::PointCloud<pcl::PointXYZI>::Ptr& src_cloud_){
    downSample(src_cloud_, src_cloud, downsample_size);
    ROS_DEBUG("frame %d: original pointcloud size: %d, downsampled point cloud size: %d", id, (int)src_cloud_->points.size(), (int)src_cloud->points.size());
    id ++;  // use for next frame

    double time_pw;
    pcl::PointCloud<pcl::PointXYZI>::Ptr g_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    PatchworkGroundSeg->estimate_ground(*src_cloud, *g_cloud, *ng_cloud, time_pw);
    std::string save_path = "/home/fyx/ufo_hiahia/src/test/";
    saveCloud(ng_cloud, save_path, 179, "_ng.pcd");
    ROS_DEBUG("patchwork extract ground: time_use(ms): %0.2f, points_num: %d", time_pw / 1e+3, (int)ng_cloud->points.size());
}

void SSC::intensityAndCurvatureCalibration(){
    
}


void SSC::downSample(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_down_, float leaf_size_){
    pcl::VoxelGrid<pcl::PointXYZI> filter;
    filter.setInputCloud(cloud_);
    filter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    filter.filter(*cloud_down_);
    // std::string save_path = "/home/fyx/ufo_hiahia/src/test/";
    // saveCloud(cloud_down_, save_path, 179, "_downSample.pcd");
}

void SSC::intensityVisualization(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_){
    int cloud_size = cloud_->points.size();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>());
    cloud_rgb->height = 1;
    cloud_rgb->width = cloud_size;
    for(size_t i = 0; i < cloud_size; i++){
        pcl::PointXYZRGB pt_rgb;
        pt_rgb.x = cloud_->points[i].x;
        pt_rgb.y = cloud_->points[i].y;
        pt_rgb.z = cloud_->points[i].z;
        float pt_intensity = cloud_->points[i].intensity;
        if(pt_intensity > 255.f){
            pt_rgb.r = 255.f;
            pt_rgb.g = 255.f;
            pt_rgb.b = 255.f;
        }
        else{
            pt_rgb.r = pt_intensity;
            pt_rgb.g = pt_intensity;
            pt_rgb.b = 0.f;
        }
        cloud_rgb->points.push_back(pt_rgb);
    }
    std::string save_path = "/home/fyx/ufo_hiahia/src/test/";
    saveCloud(cloud_rgb, save_path, 179, "_intensityShow.pcd");
}


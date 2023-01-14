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
    PatchworkGroundSeg.reset(new PatchWork<pcl::PointXYZI>());
    cloud_use.reset(new pcl::PointCloud<pcl::PointXYZI>());
}

void SSC::reset(){
    range_num = -1;
    sector_num = -1;
    azimuth_num = -1;
    bin_num = -1;

    min_dis = 999999.f; 
    max_dis = -999999.f;
    sensor_height = -999999.f;

    Frame frame_new;
    frame_ssc = frame_new;

    cloud_use->clear();
}

void SSC::process(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn_){
    // get params, min_dis, max_dis, sensor_height
    getCloudInfo(cloudIn_);

    // extract ground
    pcl::PointCloud<pcl::PointXYZI>::Ptr ng_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    ng_cloud = extractGroudByPatchWork(cloudIn_);

    // downsample and select
    pcl::PointCloud<pcl::PointXYZI>::Ptr ng_cloud_down(new pcl::PointCloud<pcl::PointXYZI>());
    downSampleAndDistanceSelect(ng_cloud, ng_cloud_down, downsample_size);

    // calibrate intensity by curvature
    intensityCalibrationByCurvature(ng_cloud_down);

    // intenisty save
    intensityVisualization(ng_cloud_down);
}

void SSC::getCloudInfo(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn_){
    float min_z = 999999.f;
    float max_z = -999999.f;
    for(size_t i = 0; i < cloudIn_->points.size(); i++){
        pcl::PointXYZI pt = cloudIn_->points[i];
        float pt_dis = pointDistance2d(pt);
        if(pt_dis <= min_dis){
            min_dis = pt_dis;
        }
        if(pt_dis >= max_dis){
            max_dis = pt_dis;
        }
        float pt_z = pt.z;
        if(pt_z <= min_z){
            min_z = pt_z;
        }
        if(pt_z >= max_z){
            max_z = pt_z;
        }
    }
    sensor_height = std::fabs(min_z) - 0.2;
    min_dis += min_dis_add;
    max_dis *= max_dis_ratio;
    ROS_INFO("frame %d info: senor height: %0.2f, original pointcloud size: %d, min_dis: %0.2f, max_dis: %0.2f", id, sensor_height, (int)cloudIn_->points.size(), min_dis, max_dis);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr SSC::extractGroudByPatchWork(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn_){
    double time_pw;
    pcl::PointCloud<pcl::PointXYZI>::Ptr g_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr ng_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    PatchworkGroundSeg->set_sensor(sensor_height);
    PatchworkGroundSeg->estimate_ground(*cloudIn_, *g_cloud, *ng_cloud, time_pw);
    ROS_DEBUG("patchwork extract ground: time_use(ms): %0.2f, points_num: %d", time_pw / 1e+3, (int)ng_cloud->points.size());
    return ng_cloud;
}

void SSC::intensityCalibrationByCurvature(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn_){
    TicToc calibration_t("intensity and curvature calibration");
    int cloud_size = cloudIn_->points.size();
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloudIn_);

    std::vector<int> invalid_idxs;
    for(size_t i = 0; i < cloud_size; i++){
        if(cloudIn_->points[i].intensity >= max_intensity){
            cloudIn_->points[i].intensity = 200.f;
            invalid_idxs.emplace_back(i);
        }
    }
    for(auto& k : invalid_idxs){
        std::vector<int> pt_idxs;
        std::vector<float> pt_dis;
        kdtree.radiusSearch(cloudIn_->points[k], correct_radius, pt_idxs, pt_dis);
        float inten_correct = 0.f;
        std::vector<int> pt_correct;
        for(auto& j : pt_idxs){
            if(!findNameInVec(j, invalid_idxs)){
                pt_correct.emplace_back(j);
            }
        }
        if(pt_correct.size() != 0){
            for(auto& l : pt_correct){
                inten_correct += cloudIn_->points[l].intensity;
            }
            cloudIn_->points[k].intensity = inten_correct / pt_correct.size();
        }
        else{
            cloudIn_->points[k].intensity = max_intensity;
        }
    }

    // normal estimation
    pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> normal;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    normal.setNumberOfThreads(10);
    normal.setInputCloud(cloudIn_);
	normal.setSearchMethod(tree);
	normal.setKSearch(10);
	normal.compute(*normals);

    for(size_t i = 0; i < cloud_size; i++){
        if(cloudIn_->points[i].z > sensor_height){
            continue;
        }
        pcl::Normal n = normals->points[i];
        Eigen::Vector3f normal_v(n.normal_x, n.normal_y, n.normal_z);
        pcl::PointXYZI pt = cloudIn_->points[i];
        Eigen::Vector3f pt_v(pt.x, pt.y, pt.z);
        float angleCos = std::fabs(normal_v.dot(pt_v) / (normal_v.norm() * pt_v.norm()));
        if(angleCos < 0.3){
            angleCos = 0.3;
        }
        if((pt.intensity / angleCos) > max_intensity){
            cloudIn_->points[i].intensity = max_intensity;
        }
        else{
            cloudIn_->points[i].intensity = pt.intensity / angleCos;
        }
    }

    // // simple model used in intensity-slam
    // for(size_t i = 0; i < cloud_size; i++){
    //     float dis = pointDistance2d(cloud_->points[i]);
    //     float ratio = std::fabs(cloud_->points[i].z / dis);
    //     float angleRad = std::atan(ratio);
    //     if(angleRad < M_PI / 18) angleRad = M_PI / 18;
    //     if(angleRad > M_PI / 7.5) angleRad = M_PI / 7.5;
    //     float new_intensity = cloud_->points[i].intensity * std::cos(angleRad) / cos(M_PI / 7.5);
    //     cloud_->points[i].intensity = new_intensity;
    //     pcl::PointXYZI pt = cloud_->points[i];
    //     pointCalib pt_calib;
    //     pt_calib.x = pt.x;
    //     pt_calib.y = pt.y;
    //     pt_calib.z = pt.z;
    //     pt_calib.intensity = pt.intensity;
    //     ng_cloud_calib->points.push_back(pt_calib);
    // }

    cloud_use = cloudIn_;
    ROS_DEBUG("calibrate intensity: calibrated cloud_size: %d", (int)cloud_use->points.size());
}


void SSC::downSampleAndDistanceSelect(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_down_, float leaf_size_){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::VoxelGrid<pcl::PointXYZI> filter;
    filter.setInputCloud(cloud_);
    filter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    filter.filter(*cloud_tmp);
    for(size_t i =0; i < cloud_tmp->points.size(); i++){
        if(pointDistance2d(cloud_tmp->points[i]) < min_dis){
            continue;
        }
        cloud_down_->points.push_back(cloud_tmp->points[i]);
    }
    ROS_DEBUG("downsample and dis-select: downsampled pointcloud size: %d", (int)cloud_down_->points.size());
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
        if(pt_intensity > max_intensity){
            pt_rgb.r = 255.f;
            pt_rgb.g = 0.f;
            pt_rgb.b = 0.f;
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


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
    downSampleAndDistanceSelect(src_cloud_, src_cloud, downsample_size);
    ROS_DEBUG("frame %d: original pointcloud size: %d, downsampled and distance selectpoint cloud size: %d", id, (int)src_cloud_->points.size(), (int)src_cloud->points.size());
    id ++;  // use for next frame

    double time_pw;
    pcl::PointCloud<pcl::PointXYZI>::Ptr g_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    PatchworkGroundSeg->estimate_ground(*src_cloud, *g_cloud, *ng_cloud, time_pw);
    std::string save_path = "/home/fyx/ufo_hiahia/src/test/";
    saveCloud(ng_cloud, save_path, 179, "_ng.pcd");
    ROS_DEBUG("patchwork extract ground: time_use(ms): %0.2f, points_num: %d", time_pw / 1e+3, (int)ng_cloud->points.size());
}

void SSC::intensityAndCurvatureCalibration(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_, pcl::PointCloud<pointCalib>::Ptr& cloud_calib_){
    TicToc calibration_t("intensity and curvature calibration");
    int cloud_size = cloud_->points.size();
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloud_);

    std::vector<int> invalid_idxs;
    for(size_t i = 0; i < cloud_size; i++){
        if(cloud_->points[i].intensity >= max_intensity){
            cloud_->points[i].intensity = 200.f;
            invalid_idxs.emplace_back(i);
        }
    }
    for(auto& k : invalid_idxs){
        std::vector<int> pt_idxs;
        std::vector<float> pt_dis;
        kdtree.radiusSearch(cloud_->points[k], correct_radius, pt_idxs, pt_dis);
        float inten_correct = 0.f;
        std::vector<int> pt_correct;
        for(auto& j : pt_idxs){
            if(!findNameInVec(j, invalid_idxs)){
                pt_correct.emplace_back(j);
            }
        }
        if(pt_correct.size() != 0){
            for(auto& l : pt_correct){
                inten_correct += cloud_->points[l].intensity;
            }
            cloud_->points[k].intensity = inten_correct / pt_correct.size();
            // std::cout << pt_correct.size() << " " << cloud_->points[k].intensity << std::endl;
        }
        else{
            cloud_->points[k].intensity = max_intensity;
        }
    }

    // normal estimation
    pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> normal;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    normal.setNumberOfThreads(10);
    normal.setInputCloud(cloud_);
	normal.setSearchMethod(tree);
	normal.setKSearch(10);
	normal.compute(*normals);

    // int count = 0;  // record
    // int count_max = 0;
    for(size_t i = 0; i < cloud_size; i++){
        if(cloud_->points[i].z > sensor_height){
            pcl::PointXYZI pt = cloud_->points[i];
            pointCalib pt_calib;
            pt_calib.x = pt.x;
            pt_calib.y = pt.y;
            pt_calib.z = pt.z;
            pt_calib.intensity = pt.intensity;
            ng_cloud_calib->points.push_back(pt_calib);
            continue;
        }
        // count ++;
        pcl::Normal n = normals->points[i];
        Eigen::Vector3f normal_v(n.normal_x, n.normal_y, n.normal_z);
        pcl::PointXYZI pt = cloud_->points[i];
        Eigen::Vector3f pt_v(pt.x, pt.y, pt.z);
        float angleCos = std::fabs(normal_v.dot(pt_v) / (normal_v.norm() * pt_v.norm()));
        if(angleCos < 0.3){
            angleCos = 0.3;
        }
        // std::cout << "angleCos: " << angleCos;

        pointCalib pt_calib;
        pt_calib.x = pt.x;
        pt_calib.y = pt.y;
        pt_calib.z = pt.z;
        if((pt.intensity / angleCos) > max_intensity){
            cloud_->points[i].intensity = max_intensity;
            // count_max ++;
        }
        else{
            cloud_->points[i].intensity = pt.intensity / angleCos;
        }
        pt_calib.intensity = cloud_->points[i].intensity;
        pt_calib.curvature = n.curvature;
        // std::cout << "n.curvature: " << n.curvature << std::endl;
        ng_cloud_calib->points.push_back(pt_calib);
    }
    // std::cout << count << " " << count_max <<  std::endl;

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

    ROS_DEBUG("calibration info: time_use(ms): %0.2f, calibrated cloud_size: %d", (float)calibration_t.toc(), (int)ng_cloud_calib->points.size());
}


void SSC::downSampleAndDistanceSelect(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_down_, float leaf_size_){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::VoxelGrid<pcl::PointXYZI> filter;
    filter.setInputCloud(cloud_);
    filter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    filter.filter(*cloud_tmp);
    // float min_z = 999.f;
    for(size_t i =0; i < cloud_tmp->points.size(); i++){
        // if(cloud_tmp->points[i].z < min_z){  // get the precise senor_height
        //     min_z = cloud_tmp->points[i].z;
        // }
        if(pointDistance2d(cloud_tmp->points[i]) < min_dis_add){
            continue;
        }
        cloud_down_->points.push_back(cloud_tmp->points[i]);
    }
    // std::cout << min_z << std::endl;
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


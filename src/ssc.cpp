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
    apri_vec.clear();
    hash_cloud.clear();

    cloud_use->clear();
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
    sensor_height = std::fabs(min_z) - 0.5;
    // if(min_dis <= min_dis_add){
    //     min_dis = min_dis_add;
    // }
    // min_dis += min_dis_add;
    if(1){
        min_dis = min_dis_add;
    }
    max_dis *= max_dis_ratio;

    range_num = (int)std::ceil((max_dis - min_dis) / range_res);   // get ssc params
    sector_num = (int)std::ceil((max_angle - min_angle) / sector_res);
    azimuth_num = (int)std::ceil((max_azimuth - min_azimuth) / azimuth_res);
    bin_num = range_num * sector_num * azimuth_num;
    ROS_INFO("frame %d info: senor height: %0.2f, original pointcloud size: %d, min_dis: %0.2f, max_dis: %0.2f, range_num: %d, sector_num: %d, azimuth_num: %d, bin_num: %d", id, sensor_height, (int)cloudIn_->points.size(), min_dis, max_dis, range_num, sector_num, azimuth_num, bin_num);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr SSC::extractGroudByPatchWork(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn_){
    double time_pw;
    pcl::PointCloud<pcl::PointXYZI>::Ptr g_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr ng_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    PatchworkGroundSeg->set_sensor(sensor_height);
    PatchworkGroundSeg->estimate_ground(*cloudIn_, *g_cloud, *ng_cloud, time_pw);
    ROS_DEBUG("patchwork extract ground: time_use(ms): %0.2f, pointcloud without ground size:: %d", time_pw / 1e+3, (int)ng_cloud->points.size());
    return ng_cloud;
}

void SSC::intensityCalibrationByCurvature(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn_){
    int cloud_size = cloudIn_->points.size();

    // // model 1: correct intensity by searching neighbors
    // pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    // kdtree.setInputCloud(cloudIn_);
    // std::vector<int> invalid_idxs;
    // for(size_t i = 0; i < cloud_size; i++){
    //     if(cloudIn_->points[i].intensity >= max_intensity){
    //         cloudIn_->points[i].intensity = 200.f;
    //         if(cloudIn_->points[i].z > sensor_height || pointDistance2d(cloudIn_->points[i]) > max_dis * correct_ratio){
    //             continue;
    //         }
    //         invalid_idxs.emplace_back(i);
    //     }
    // }
    // for(auto& k : invalid_idxs){
    //     std::vector<int> pt_idxs;
    //     std::vector<float> pt_dis;
    //     kdtree.radiusSearch(cloudIn_->points[k], correct_radius, pt_idxs, pt_dis);
    //     float inten_correct = 0.f;
    //     std::vector<int> pt_correct;
    //     for(auto& j : pt_idxs){
    //         if(!findNameInVec(j, invalid_idxs)){
    //             pt_correct.emplace_back(j);
    //         }
    //     }
    //     if(pt_correct.size() != 0){
    //         for(auto& l : pt_correct){
    //             inten_correct += cloudIn_->points[l].intensity;
    //         }
    //         cloudIn_->points[k].intensity = inten_correct / pt_correct.size();
    //     }
    //     else{
    //         cloudIn_->points[k].intensity = max_intensity;
    //     }
    // }
    // pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> normal;
    // pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    // pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    // normal.setNumberOfThreads(10);
    // normal.setInputCloud(cloudIn_);
	// normal.setSearchMethod(tree);
	// normal.setKSearch(10);
	// normal.compute(*normals);
    // for(size_t i = 0; i < cloud_size; i++){
    //     if(cloudIn_->points[i].z > sensor_height || pointDistance2d(cloudIn_->points[i]) > max_dis * correct_ratio){
    //         continue;
    //     }
    //     pcl::Normal n = normals->points[i];
    //     Eigen::Vector3f normal_v(n.normal_x, n.normal_y, n.normal_z);
    //     pcl::PointXYZI pt = cloudIn_->points[i];
    //     Eigen::Vector3f pt_v(pt.x, pt.y, pt.z);
    //     float angleCos = std::fabs(normal_v.dot(pt_v) / (normal_v.norm() * pt_v.norm()));
    //     if(angleCos < 0.3){
    //         angleCos = 0.3;
    //     }
    //     if((pt.intensity / angleCos) > max_intensity){
    //         cloudIn_->points[i].intensity = max_intensity;
    //         apri_vec[i].intensity = cloudIn_->points[i].intensity;
    //     }
    //     else{
    //         cloudIn_->points[i].intensity = pt.intensity / angleCos;
    //         apri_vec[i].intensity = cloudIn_->points[i].intensity;
    //         // std::cout << apri_vec[i].intensity << " ";
    //     }
    // }

    // model 2: normal estimation in your interesting region
    for(size_t i = 0; i < cloud_size; i++){
        if(cloudIn_->points[i].intensity > max_intensity){
            cloudIn_->points[i].intensity = max_intensity;
        }
    }
    std::vector<int> id_correct;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_correct(new pcl::PointCloud<pcl::PointXYZI>());
    for(size_t i = 0; i < cloud_size; i++){
        pcl::PointXYZI pt = cloudIn_->points[i];
        if(pt.z > sensor_height || pointDistance2d(pt) > max_dis * correct_ratio){
            apri_vec[i].intensity = pt.intensity;
            continue;
        }
        else{
            id_correct.emplace_back(i);
            cloud_correct->points.push_back(pt);
        }
    }
    pcl::NormalEstimationOMP<pcl::PointXYZI, pcl::Normal> normal;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    normal.setNumberOfThreads(kNumOmpCores);
    normal.setInputCloud(cloud_correct);
	normal.setSearchMethod(tree);
	normal.setKSearch(search_num);
	normal.compute(*normals);

    if(id_correct.size() != normals->points.size()){
        ROS_WARN("intensity calibration error about correct ratio");
        return ;
    }
    for(size_t i = 0; i < normals->points.size(); i++){
        pcl::Normal n = normals->points[i];
        Eigen::Vector3f normal_v(n.normal_x, n.normal_y, n.normal_z);
        pcl::PointXYZI pt = cloudIn_->points[id_correct[i]];
        Eigen::Vector3f pt_v(pt.x, pt.y, pt.z);
        float angleCos = std::fabs(normal_v.dot(pt_v) / (normal_v.norm() * pt_v.norm()));
        if(angleCos < 0.3){
            angleCos = 0.3;
        }
        if((pt.intensity / angleCos) > max_intensity){
            cloudIn_->points[id_correct[i]].intensity = max_intensity;
            apri_vec[id_correct[i]].intensity = cloudIn_->points[id_correct[i]].intensity;
        }
        else{
            cloudIn_->points[id_correct[i]].intensity = pt.intensity / angleCos;
            apri_vec[id_correct[i]].intensity = cloudIn_->points[id_correct[i]].intensity;
            // std::cout << apri_vec[i].intensity << " ";
        }
    }

    
    // // model 3: simple model used in intensity-slam
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
    // ROS_DEBUG("calibrate intensity: calibrated cloud_size: %d", (int)cloud_use->points.size());
}


void SSC::downSampleAndMakeApriVec(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_down_, float leaf_size_){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::VoxelGrid<pcl::PointXYZI> filter;
    filter.setInputCloud(cloud_);
    filter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    filter.filter(*cloud_tmp);
    int count = 0;
    for(size_t i =0; i < cloud_tmp->points.size(); i++){
        pcl::PointXYZI pt = cloud_tmp->points[i];
        float dis = pointDistance2d(pt);
        float angle = getPolarAngle(pt);
        float azimuth = getAzimuth(pt);
        if(dis < min_dis || dis > max_dis) continue;
        if(angle < min_angle || angle > max_angle) continue;
        if(azimuth < min_azimuth || azimuth > max_azimuth) continue;
        cloud_down_->points.push_back(pt);
        count ++;

        PointAPRI apri;
        apri.x = pt.x;
        apri.y = pt.y;
        apri.z = pt.z;
        apri.range = dis;
        apri.angle = angle;
        apri.azimuth = azimuth;
        if(apri.range_idx != -1 || apri.sector_idx != -1 || apri.azimuth_idx != -1 || apri.voxel_idx != -1){
            ROS_WARN("pt %d can't generate apri", (int)i);
            continue;
        }
        apri.range_idx = std::ceil((dis - min_dis) / range_res) - 1;
        apri.sector_idx = std::ceil((angle - min_angle) / sector_res) - 1;
        apri.azimuth_idx = std::ceil((azimuth - min_azimuth) / azimuth_res) -1;
        apri.voxel_idx = apri.azimuth_idx * range_num * sector_num + apri.range_idx * sector_num + apri.sector_idx;
        if(apri.voxel_idx > bin_num){
            ROS_WARN("pt %d can't find its bin", (int)i);
            continue;
        }
        apri_vec.emplace_back(apri);
    }
    if(apri_vec.size() != count){
        ROS_WARN("not all pts can't turn to be a apri, please check");
    }
    ROS_DEBUG("downsample and select: downsampled pointcloud size: %d", (int)cloud_down_->points.size());
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
    saveCloud(cloud_rgb, save_path, 179, "_use.pcd");
}

void SSC::process(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn_){
    TicToc process_t("pre-process");
    // get params, min_dis, max_dis, sensor_height
    getCloudInfo(cloudIn_);

    // extract ground
    pcl::PointCloud<pcl::PointXYZI>::Ptr ng_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    ng_cloud = extractGroudByPatchWork(cloudIn_);

    // downsample and select
    pcl::PointCloud<pcl::PointXYZI>::Ptr ng_cloud_down(new pcl::PointCloud<pcl::PointXYZI>());
    downSampleAndMakeApriVec(ng_cloud, ng_cloud_down, downsample_size);

    // calibrate intensity by curvature
    intensityCalibrationByCurvature(ng_cloud_down);

    // intenisty save
    intensityVisualization(cloud_use);

    // make hash cloud
    makeHashCloud(apri_vec);

    // get voxel cloud
    getVoxelCloudFromHashCloud(hash_cloud);

    ROS_DEBUG("pre-process: time_use(ms): %0.2f, valid pointcloud size: %d, apri_vec size: %d, hash_cloud size: %d", (float)process_t.toc(), (int)cloud_use->points.size(), (int)apri_vec.size(), (int)hash_cloud.size());
}

void SSC::makeHashCloud(const std::vector<PointAPRI>& apriIn_){
    std::unordered_map<int, Voxel>::iterator it_find;
    for(size_t i = 0; i < apriIn_.size(); i++){
        PointAPRI apri = apriIn_[i];
        it_find = hash_cloud.find(apri.voxel_idx);
        if(it_find != hash_cloud.end()){
            it_find->second.ptIdx.emplace_back(i);
            it_find->second.intensity_record.emplace_back(apri.intensity);
            it_find->second.intensity_av += apri.intensity;
        }
        else{
            Voxel voxel;
            voxel.ptIdx.emplace_back(i);
            voxel.intensity_record.emplace_back(apri.intensity);
            voxel.intensity_av += apri.intensity;
            float range_center = (apri.range_idx * 2 + 1) / 2 * range_res;
            float sector_center = deg2rad((apri.sector_idx * 2 + 1) / 2 * sector_res);
            float azimuth_center = deg2rad((apri.azimuth_idx * 2 + 1) / 2 * azimuth_res) + deg2rad(min_azimuth);
            voxel.center.x = range_center * std::cos(sector_center);
            voxel.center.y = range_center * std::sin(sector_center);
            voxel.center.z = range_center * std::tan(azimuth_center);
            voxel.center.intensity = apri.voxel_idx;
            hash_cloud.insert(std::make_pair(apri.voxel_idx, voxel));
        }
    }
    int count = 0;
    for(auto& vox : hash_cloud){
        count += vox.second.ptIdx.size();
        vox.second.intensity_av /= vox.second.ptIdx.size();
        for(auto& in : vox.second.intensity_record){
            vox.second.intensity_cov += std::pow((in - vox.second.intensity_av), 2);   // /10, otherwise it's too large
        }
        vox.second.intensity_cov /= vox.second.ptIdx.size();
        // std::cout << "vox.second.ptIdx.size(): " << vox.second.ptIdx.size() <<   " vox.second.intensity_av: " << vox.second.intensity_av << " vox.second.intensity_cov: " << vox.second.intensity_cov << std::endl;
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr SSC::getVoxelCloudFromHashCloud(const std::unordered_map<int, Voxel>& hashCloud_){
    pcl::PointCloud<pcl::PointXYZI>::Ptr voxel_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    for(auto& vox : hashCloud_){
        voxel_cloud->points.push_back(vox.second.center);
    }
    std::string save_path = "/home/fyx/ufo_hiahia/src/test/";
    saveCloud(voxel_cloud, save_path, 179, "_vox.pcd");
    return voxel_cloud;
}


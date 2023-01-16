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
    cloud_vox.reset(new pcl::PointCloud<pcl::PointXYZI>());
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
    cloud_vox->clear();
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
    // if(min_dis <= min_dis_add){
    //     min_dis = min_dis_add;
    // }
    // min_dis += min_dis_add;
    if(1){
        min_dis = min_dis_add;
    }
    max_dis = max_dis_ratio;

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
    ROS_DEBUG("patchwork extract ground: time_use(ms): %0.2f, pointcloud with no ground size:: %d", time_pw / 1e+3, (int)ng_cloud->points.size());
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
        ROS_WARN("not all pts can turn to be a apri, please check");
    }
    // ROS_DEBUG("downsample and select: downsampled pointcloud size: %d", (int)cloud_down_->points.size());
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
            voxel.range_idx = apri.range_idx;
            voxel.sector_idx = apri.sector_idx;
            voxel.azimuth_idx = apri.azimuth_idx;
            float range_center = (apri.range_idx * 2 + 1) / 2 * range_res + min_dis;
            float sector_center = deg2rad((apri.sector_idx * 2 + 1) / 2 * sector_res) + min_angle;
            float azimuth_center = deg2rad((apri.azimuth_idx * 2 + 1) / 2 * azimuth_res) + deg2rad(min_azimuth);
            voxel.center.x = range_center * std::cos(sector_center);
            voxel.center.y = range_center * std::sin(sector_center);
            voxel.center.z = range_center * std::tan(azimuth_center);
            voxel.center.intensity = apri.voxel_idx;
            hash_cloud.insert(std::make_pair(apri.voxel_idx, voxel));
        }
    }

    for(auto& vox : hash_cloud){
        vox.second.intensity_av /= vox.second.ptIdx.size();
        for(auto& in : vox.second.intensity_record){
            vox.second.intensity_cov += std::pow((in - vox.second.intensity_av), 2);   // /10, otherwise it's too large
        }
        vox.second.intensity_cov /= vox.second.ptIdx.size();
        // std::cout << "vox.second.ptIdx.size(): " << vox.second.ptIdx.size() <<   " vox.second.intensity_av: " << vox.second.intensity_av << " vox.second.intensity_cov: " << vox.second.intensity_cov << std::endl;
    }
}

void SSC::getVoxelCloudFromHashCloud(const std::unordered_map<int, Voxel>& hashCloud_){
    for(auto& vox : hashCloud_){
        cloud_vox->points.push_back(vox.second.center);
    }
    std::string save_path = "/home/fyx/ufo_hiahia/src/test/";
    saveCloud(cloud_vox, save_path, 179, "_vox.pcd");
}

void SSC::clusterAndCreateFrame(const std::vector<PointAPRI>& apri_vec_, std::unordered_map<int, Voxel>& hash_cloud_){
    int cluster_name = 4;
    std::vector<int> clusterIdxs = std::vector<int>(apri_vec_.size(), -1);

    TicToc cluster_t("cluster");
    for(int i = 0; i < apri_vec_.size(); i++){
        PointAPRI apri = apri_vec_[i];
        std::unordered_map<int, Voxel>::iterator it_find1;
        std::unordered_map<int, Voxel>::iterator it_find2;
        std::vector<int> neighbors;  // restore a lot of apri-neighbors idxs

        it_find1 = hash_cloud_.find(apri.voxel_idx);
        if(it_find1 != hash_cloud_.end()){
            std::vector<int> neighbor = findVoxelNeighbors(apri.range_idx, apri.sector_idx, apri.azimuth_idx, 1);   
            for(int k = 0; k < neighbor.size(); k++){
                it_find2 =  hash_cloud_.find(neighbor[k]);   
                if(it_find2 != hash_cloud_.end()){
                    addVec(neighbors, it_find2->second.ptIdx);
                }
            }
        }

        neighbors.swap(neighbors);
        if(neighbors.size() > 0){
            for(int n = 0; n < neighbors.size(); n++){
                int oc = clusterIdxs[i];
                int nc = clusterIdxs[neighbors[n]];

                if(oc != -1 && nc != -1){
                    if(oc != nc){
                        mergeClusters(clusterIdxs, oc, nc);  // merge
                    }
                }
                else{
                    if(nc != -1){
                        clusterIdxs[i] = nc;
                    }
                    else{
                        if(oc != -1){
                            clusterIdxs[neighbors[n]] = oc;
                        }
                    }
                }    
            }
        }

        if(clusterIdxs[i] == -1){
            cluster_name ++;   //   a new class
            clusterIdxs[i] = cluster_name;  // just encode the cluster name
            for(int m = 0; m < neighbors.size(); m++){
                clusterIdxs[neighbors[m]] = cluster_name;
            }
        }
    }

    std::unordered_map<int, std::vector<int>> cluster_pt;  // cluster name + pt id
    std::unordered_map<int, std::vector<int>> cluster_vox;  // cluster name + voxel id
    std::unordered_map<int, std::vector<int>>::iterator it_p;
    std::unordered_map<int, std::vector<int>>::iterator it_v;
    for(size_t i = 0; i < clusterIdxs.size(); i++){
        it_p = cluster_pt.find(clusterIdxs[i]);
        it_v = cluster_vox.find(clusterIdxs[i]);
        if(it_p != cluster_pt.end()){
            it_p->second.emplace_back(i);
            it_v->second.emplace_back(apri_vec_[i].voxel_idx);
        }
        else{
            std::vector<int> pt_vec;
            std::vector<int> vox_vec;
            pt_vec.emplace_back(i);
            vox_vec.emplace_back(apri_vec_[i].voxel_idx);
            cluster_pt.insert(std::make_pair(clusterIdxs[i], pt_vec));
            cluster_vox.insert(std::make_pair(clusterIdxs[i], vox_vec));
        }
    }

    int pt_count = 0;
    for(auto& c : cluster_pt){
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>());
        cloud_cluster = getCloudByIdx(cloud_use, c.second);
        if(!refineClusterByBoundingBox(cloud_cluster)){
            continue;
        }
        else{
            Cluster cluster;
            cluster.occupy_pts = c.second;
            cluster.occupy_voxels = cluster_vox[c.first];
            cluster.cloud_observe.emplace_back(std::make_pair(id, cloud_cluster));
            pcl::PointXYZI center = getCenterOfCloud(cloud_cluster);
            cluster.cluster_center = center;
            frame_ssc.cluster_set.emplace_back(cluster);
            frame_ssc.center_cloud->points.push_back(center);
            pt_count += c.second.size();
        }
    }

    int final_clusterNum = frame_ssc.cluster_set.size();
    for(int j = 0; j < final_clusterNum; j++){
        for(auto& v : frame_ssc.cluster_set[j].occupy_voxels){
            hash_cloud[v].label = j;
        }
    }

    std::string save_path = "/home/fyx/ufo_hiahia/src/test/";
    saveCloud(frame_ssc.center_cloud, save_path, 179, "_clusterCenter.pcd");
    ROS_DEBUG("cluster and create frame: time_use(ms): %0.2f, cluster_num: %d, pt_num: %d", (float)cluster_t.toc(), final_clusterNum, pt_count);
}

std::vector<int> SSC::findVoxelNeighbors(const int& range_idx_, const int& sector_idx_, const int& azimuth_idx_, int size_){
    std::vector<int> neighborIdxs;
    for(int x = range_idx_ - size_; x <= range_idx_ + size_; x++){
        if(x > range_num -1 || x < 0) {continue;}
        for(int y = sector_idx_ - size_; y <= sector_idx_ + size_; y++){
            if(y > sector_num -1 || y < 0) {continue;}
            for(int z = azimuth_idx_ - size_; z <= azimuth_idx_ + size_; z++){
                if(z > azimuth_num - 1 || z < 0) {continue;}
                neighborIdxs.emplace_back(x * sector_num + y + z * range_num * sector_num);  
            }
        }
    }
    return neighborIdxs;
}

void SSC::mergeClusters(std::vector<int>& clusterIdxs_, const int& idx1_, const int& idx2_){
    for(int i = 0; i < clusterIdxs_.size(); i++){
        if(clusterIdxs_[i] == idx1_){
            clusterIdxs_[i] = idx2_;
        }
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr SSC::getCloudByIdx(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_from_, const std::vector<int>& idx_vec_){
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZI>());
    for(auto& idx : idx_vec_){
        cloud_cluster->points.push_back(cloud_from_->points[idx]);
    }
    return cloud_cluster;
}

std::pair<pcl::PointXYZI, pcl::PointXYZI> SSC::getBoundingBoxOfCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_){
    pcl::PointXYZI point_min, point_max;
    pcl::getMinMax3D(*cloud_, point_min, point_max);
    return std::make_pair(point_min, point_max);
}

pcl::PointXYZI SSC::getCenterOfCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_){
    Eigen::Vector4f centroid;	
    pcl::compute3DCentroid(*cloud_, centroid);
    pcl::PointXYZI center;
    center.x = centroid(0);
    center.y = centroid(1);
    center.z = centroid(2);
    return center;
}

bool SSC::refineClusterByBoundingBox(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_cluster_){
    std::pair<pcl::PointXYZI, pcl::PointXYZI> bounding_box = getBoundingBoxOfCloud(cloud_cluster_);
    pcl::PointXYZI point_min = bounding_box.first;
    pcl::PointXYZI point_max = bounding_box.second;
    float diff_z = point_max.z - point_min.z;
    if(point_min.z > sensor_height / 2 || diff_z <= 0.2){
        return false;
    }
    else{
        if(point_min.z < (- sensor_height) || cloud_cluster_->points.size() >=toBeClass){
            return true;
        }
        else{
            return false;
        }
    }
}

void SSC::saveSegCloud(Frame& frame_ssc){
    cv::RNG rng(12345);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    rgb_ptr->height = 1;
    int count = 0;
    for(auto& c : frame_ssc.cluster_set){
        int r, g, b;
        r = rng.uniform(20, 150);   // random get rgb
        g = rng.uniform(20, 200); 
        b = rng.uniform(20, 200); 
        for(size_t i = 0; i < c.occupy_pts.size(); i++){
            pcl::PointXYZRGB pt_rgb;
            pcl::PointXYZI pt = cloud_use->points[c.occupy_pts[i]];
            pt_rgb.x = pt.x;
            pt_rgb.y = pt.y;
            pt_rgb.z = pt.z;
            pt_rgb.r = r;
            pt_rgb.g = g;
            pt_rgb.b = b;
            rgb_ptr->points.push_back(pt_rgb);
            count ++;
        }
        c.color[0] = r;
        c.color[1] = g;
        c.color[2] = b;
    }
    rgb_ptr->width = count;
    std::string save_path = "/home/fyx/ufo_hiahia/src/test/";
    saveCloud(rgb_ptr, save_path, 179, "_seg.pcd");
}

void SSC::refineClusterByIntensity(Frame& frame_ssc){
    std::vector<std::vector<int>> fusion_set;
    for(int i = 0; i < frame_ssc.cluster_set.size(); i++){
        std::vector<int> voxel_c = frame_ssc.cluster_set[i].occupy_voxels;
        std::vector<int> candidate_c;
        for(auto& v: voxel_c){
            std::vector<int> candidate;
            std::unordered_map<int, Voxel>::iterator it_find; 
            std::vector<int> vox = findVoxelNeighbors(hash_cloud[v].range_idx, hash_cloud[v].sector_idx, hash_cloud[v].azimuth_idx, search_c);
            for(auto& n : vox){
                it_find = hash_cloud.find(n);
                if(it_find != hash_cloud.end() && hash_cloud[n].intensity_cov <= intensity_cov && std::fabs(hash_cloud[v].intensity_av - hash_cloud[n].intensity_av) <= intensity_diff){
                    if(it_find->second.label != -1){
                        candidate.emplace_back(it_find->second.label);
                    }
                }
            }
            addVec(candidate_c, candidate);
        }
        sampleVec(candidate_c);
        if(candidate_c.size() >= 2){
            fusion_set.emplace_back(candidate_c);
        }
    }

    std::vector<std::vector<int>> fusion_set_tmp;
    std::vector<int> invalid_id;
    for(int k = 0; k < fusion_set.size(); k++){
        if(findNameInVec(k, invalid_id)){
            continue;
        }
        std::vector<int> set = fusion_set[k];
        for(int j = k; j < fusion_set.size(); j++){
            if(findNameInVec(set, fusion_set[j])){
                addVec(set, fusion_set[j]);
                invalid_id.emplace_back(j);
            }
            else{
                continue;
            }
        }
        sampleVec(set);
        std::sort(set.begin(), set.end());
        fusion_set_tmp.emplace_back(set);
    }
    fusion_set.swap(fusion_set_tmp);

    // // test
    // std::cout << "fusion_set: " << fusion_set.size() << std::endl;
    // for(auto& fusion : fusion_set){
    //     for(auto& f : fusion){
    //         std::cout << f << " ";
    //     }
    //     std::cout << std::endl;
    // }
    
    // update
    std::vector<int> erase_id;
    for(auto& fusion : fusion_set){
        for(int c = 1; c < fusion.size(); c++){
            addVec(frame_ssc.cluster_set[fusion[0]].occupy_pts, frame_ssc.cluster_set[fusion[c]].occupy_pts);
            addVec(frame_ssc.cluster_set[fusion[0]].occupy_voxels, frame_ssc.cluster_set[fusion[c]].occupy_voxels);
            *frame_ssc.cluster_set[fusion[0]].cloud_observe[0].second += *frame_ssc.cluster_set[fusion[c]].cloud_observe[0].second;
            erase_id.emplace_back(fusion[c]);
        }
        frame_ssc.cluster_set[fusion[0]].cluster_center = getCenterOfCloud(frame_ssc.cluster_set[fusion[0]].cloud_observe[0].second);
    }

    std::vector<Cluster> cluster_set_tmp;
    for(int p = 0; p < frame_ssc.cluster_set.size(); p++){
        if(!findNameInVec(p, erase_id)){
            cluster_set_tmp.emplace_back(frame_ssc.cluster_set[p]);
        }
    }
    frame_ssc.cluster_set.swap(cluster_set_tmp);

    frame_ssc.center_cloud->clear();
    for(auto& c : frame_ssc.cluster_set){  // center cloud
        frame_ssc.center_cloud->points.push_back(c.cluster_center);
    }
    // std::cout << "erase_id: " << erase_id.size() << " frame_ssc.center_cloud->points : " << frame_ssc.center_cloud->points.size() << std::endl;

}

void SSC::segment(){
    TicToc segment_t("segment");
    // cvc
    clusterAndCreateFrame(apri_vec, hash_cloud);

    // intensity compensate
    refineClusterByIntensity(frame_ssc);

    // save segment cloud
    saveSegCloud(frame_ssc);

    ROS_INFO("frame %d segment: time_use(ms): %0.2f, refined cluster_num: %d", id, (float)segment_t.toc(), (int)frame_ssc.cluster_set.size());
}
#include "ssc.h"

template<typename T>
bool swap_if_gt(T& a, T& b) {
    if (a > b) {
    std::swap(a, b);
    return true;
  }
  return false;
}

bool fileSort(std::string name1_, std::string name2_){  // filesort by name
    std::string::size_type iPos1 = name1_.find_last_of('/') + 1;
	std::string filename1 = name1_.substr(iPos1, name1_.length() - iPos1);
	std::string name1 = filename1.substr(0, filename1.rfind("."));

    std::string::size_type iPos2 = name2_.find_last_of('/') + 1;
    std::string filename2 = name2_.substr(iPos2, name2_.length() - iPos2);
	std::string name2 = filename2.substr(0, filename2.rfind(".")); 

    return std::stoi(name1) < std::stoi(name2);
}

bool sort1(const std::pair<int, Cluster>& cluster1_, const std::pair<int, Cluster>& cluster2_){
    return cluster1_.second.occupy_voxels >= cluster2_.second.occupy_voxels;
}

int SSC::id = 0;

SSC::~SSC() {}

SSC::SSC(){
    allocateMemory();
    range_num = (int)std::ceil((max_dis - min_dis) / range_res);   // get ssc params
    sector_num = (int)std::ceil((max_angle - min_angle) / sector_res);
    azimuth_num = (int)std::ceil((max_azimuth - min_azimuth) / azimuth_res);
    bin_num = range_num * sector_num * azimuth_num;

    calib_save =  out_path + calib_path;
    fsmkdir(calib_save);
    seg_save = out_path + seg_path;
    fsmkdir(seg_save);
    pcd_save = out_path + pcd_path;
    fsmkdir(pcd_save);
    map_save = out_path + map_path;
    fsmkdir(map_save);

    std::cout << "----  SSC INITIALIZATION  ----" << "\n"
                       << "range_res: " << range_res << " sector_res: " << sector_res << " azimuth_res: " << azimuth_res << "\n"
                       << "min_dis: " << min_dis << " max_dis: " << max_dis << " range_num: " << range_num << "\n"
                       << "min_angle: " << min_angle << " max_angle: " << max_angle << " sector_num: " << sector_num << "\n"
                       << "min_azimuth: " << min_azimuth << " max_azimuth: " << max_azimuth << " azimuth_num: " << azimuth_num << "\n"
                       << "data_path: " << data_path <<  "\n"
                       << "label_path: " << label_path << "\n"
                       << "pose_path: " << pose_path << "\n"
                       << "map_init: " << init << "\n"
                       << "calib_save: " << calib_save << "\n"
                       << "seg_save: " << seg_save << "\n"
                       << std::endl;         
}

void SSC::allocateMemory(){
    PatchworkGroundSeg.reset(new PatchWork<pcl::PointXYZI>());
    cloud_use.reset(new pcl::PointCloud<pcl::PointXYZI>());
}

void SSC::reset(){
    Frame frame_new;
    frame_ssc = frame_new;

    apri_vec.clear();
    hash_cloud.clear();
    cloud_use->clear();
}

pcl::PointCloud<pcl::PointXYZI>::Ptr SSC::extractGroudByPatchWork(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn_){
    double time_pw;
    pcl::PointCloud<pcl::PointXYZI>::Ptr g_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr ng_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    g_cloud_vec.emplace_back(g_cloud);
    PatchworkGroundSeg->set_sensor(sensor_height);
    PatchworkGroundSeg->estimate_ground(*cloudIn_, *g_cloud, *ng_cloud, time_pw);
    return ng_cloud;
}

void SSC::intensityCalibrationByCurvature(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn_){
    int cloud_size = cloudIn_->points.size();

    // correct
    for(size_t i = 0; i < cloud_size; i++){
        if(cloudIn_->points[i].intensity > max_intensity){
            cloudIn_->points[i].intensity = max_intensity;
        }
    }
    
    // get ROI
    std::vector<int> id_correct;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_correct(new pcl::PointCloud<pcl::PointXYZI>());
    for(size_t i = 0; i < cloud_size; i++){
        pcl::PointXYZI pt = cloudIn_->points[i];
        if(pt.z > (sensor_height / 3) || pointDistance2d(pt) > max_dis * correct_ratio){
            continue;
        }
        else{
            id_correct.emplace_back(i);
            cloud_correct->points.push_back(pt);
        }
    }

    // calibrate intensity by curvature
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
        }
        else{
            cloudIn_->points[id_correct[i]].intensity = pt.intensity / angleCos;
        }
    }
}

void SSC::makeApriVec(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_){
    for(size_t i =0; i < cloud_->points.size(); i++){
        pcl::PointXYZI pt = cloud_->points[i];
        float dis = pointDistance2d(pt);
        float angle = getPolarAngle(pt);
        float azimuth = getAzimuth(pt);
        if(dis < min_dis || dis > max_dis) continue;
        if(angle < min_angle || angle > max_angle) continue;
        if(azimuth < min_azimuth || azimuth > max_azimuth) continue;

        cloud_use->points.push_back(pt);  
        frame_ssc.cloud_use->points.push_back(pt);  // get cloud_use

        PointAPRI apri;
        apri.x = pt.x;
        apri.y = pt.y;
        apri.z = pt.z;
        apri.range = dis;
        apri.angle = angle;
        apri.azimuth = azimuth;
        apri.intensity = pt.intensity;
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
            pt_rgb.r = 0.f;
            pt_rgb.g = pt_intensity;
            pt_rgb.b = pt_intensity;
        }
        cloud_rgb->points.push_back(pt_rgb);
    }

    saveCloud(cloud_rgb, calib_save, id, "_calib.pcd");
}

void SSC::process(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn_){
    TicToc process_t("pre-process");
    // give frame id
    frame_ssc.id = id;

    // extract ground
    pcl::PointCloud<pcl::PointXYZI>::Ptr ng_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    ng_cloud = extractGroudByPatchWork(cloudIn_);

    // calibrate intensity by curvature
    intensityCalibrationByCurvature(ng_cloud);

    // make apri vec
    makeApriVec(ng_cloud);

    // make hash cloud
    makeHashCloud(apri_vec);

    ROS_INFO("pre-process: time_use(ms): %0.2f, original pointcloud size: %d, valid pointcloud size: %d, apri_vec size: %d, hash_cloud size: %d", (float)process_t.toc(), (int)cloudIn_->points.size(), (int)cloud_use->points.size(), (int)apri_vec.size(), (int)hash_cloud.size());

    // visualize intensity
    intensityVisualization(ng_cloud);
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
            vox.second.intensity_cov += std::pow((in - vox.second.intensity_av), 2);  
        }
        vox.second.intensity_cov /= vox.second.ptIdx.size();
    }
}

void SSC::getVoxelCloudFromHashCloud(std::unordered_map<int, Voxel>& hashCloud_){
    for(auto& vox : hashCloud_){
        frame_ssc.vox_cloud->points.push_back(vox.second.center);
    }
    std::string save_path = "/home/fyx/ufo_hiahia/src/test/";
    saveCloud(frame_ssc.vox_cloud, save_path, id, "_vox.pcd");
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

    frame_ssc.max_name = cluster_name ++;

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

    for(auto& c : cluster_pt){
        Cluster cluster;
        cluster.name = c.first;
        cluster.occupy_pts = c.second;
        cluster.occupy_voxels = cluster_vox[c.first];
        getCloudByVec(cloud_use, c.second, cluster.cloud);
        sampleVec(cluster.occupy_voxels);
        frame_ssc.cluster_set.insert(std::make_pair(cluster.name, cluster));
    }

    for(auto& c : frame_ssc.cluster_set){
        for(auto& v : c.second.occupy_voxels){
            hash_cloud[v].label = c.first;
        }
    }
    ROS_DEBUG("cvc: time_use(ms): %0.2f, cluster_num: %d", (float)cluster_t.toc(), (int)frame_ssc.cluster_set.size());
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

void SSC::refineClusterByBoundingBox(Frame& frame_ssc_){
    std::vector<int> erase_id;
    for(auto& c : frame_ssc_.cluster_set){
        c.second.bounding_box = getBoundingBoxOfCloud(c.second.cloud);
        pcl::PointXYZI point_min = c.second.bounding_box.first;
        pcl::PointXYZI point_max = c.second.bounding_box.second;
        float diff_z = point_max.z - point_min.z;
        // if(point_min.z > 0.f ||  (c.second.occupy_pts.size() < toBeClass) || (point_max.z < - sensor_height / 2)){  // parkinglot
        if(point_min.z > 0.f ||  (c.second.occupy_pts.size() < toBeClass) || (point_max.z < - sensor_height / 2) || diff_z < 0.1){ 
            erase_id.emplace_back(c.first);
        }
        else{
            continue;
        }
    }
    for(auto& e : erase_id){
        for(auto& v : frame_ssc_.cluster_set[e].occupy_voxels){
            hash_cloud[v].label = -1;
        }
        frame_ssc_.cluster_set.erase(e);
    }
}

void SSC::saveSegCloud(Frame& frame_ssc, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_, const std::string& path_, int mode){
    cv::RNG rng(12345);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    rgb_ptr->height = 1;
    int count = 0;
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_ptr_d(new pcl::PointCloud<pcl::PointXYZRGB>());
    // rgb_ptr_d->height = 1;
    // int count_d = 0;
    for(auto& c : frame_ssc.cluster_set){
        int r, g, b;
        if(c.second.state == 1){  
            if(mode == 3){  // skip dynamic object
                // for(size_t i = 0; i < c.second.occupy_pts.size(); i++){
                //     pcl::PointXYZRGB pt_rgb;
                //     pcl::PointXYZI pt = cloud_->points[c.second.occupy_pts[i]];
                //     pt_rgb.x = pt.x;
                //     pt_rgb.y = pt.y;
                //     pt_rgb.z = pt.z;
                //     pt_rgb.r = 255.f;
                //     pt_rgb.g = 0.f;
                //     pt_rgb.b = 0.f;
                //     rgb_ptr_d->points.push_back(pt_rgb);
                //     count_d ++;
                // }
                continue;
            }
        
            r = 255.f;  // dynamic is red
            g = 0.f;
            b = 0.f;
        }
        else{
            if(c.second.type == car || c.second.type == -1){
                if(mode == 1){  // random color
                    r = rng.uniform(20, 200);   // car is random color to test
                    g = rng.uniform(20, 200); 
                    b = rng.uniform(20, 200); 
                }
                else{  // track  mode = 2
                    r = c.second.color[0];
                    g = c.second.color[1];
                    b = c.second.color[2];
                }
            }
            else if(c.second.type == building){  
                r = 139.f;
                g = 90.f;
                b = 0.f;
            }
            else if(c.second.type == tree){
                r = 162.f;
                g = 205.f;
                b = 90.f;
            }
            else{
                r = 255.f;
                g = 255.f;
                b = 255.f;
            }
        }

        for(size_t i = 0; i < c.second.occupy_pts.size(); i++){
            pcl::PointXYZRGB pt_rgb;
            pcl::PointXYZI pt = cloud_->points[c.second.occupy_pts[i]];
            pt_rgb.x = pt.x;
            pt_rgb.y = pt.y;
            pt_rgb.z = pt.z;
            pt_rgb.r = r;
            pt_rgb.g = g;
            pt_rgb.b = b;
            rgb_ptr->points.push_back(pt_rgb);
            count ++;
        }
    }
    rgb_ptr->width = count;
    pcl::VoxelGrid<pcl::PointXYZRGB> sample;  // downsampling
    sample.setInputCloud(rgb_ptr);
    sample.setLeafSize(0.05, 0.05, 0.05);
    sample.filter(*rgb_ptr);
    saveCloud(rgb_ptr, path_, frame_ssc.id, "_seg.pcd");

    // if(mode == 3){
    //     rgb_ptr_d->width = count_d;
    //     pcl::VoxelGrid<pcl::PointXYZRGB> sample_d;  // downsampling
    //     sample_d.setInputCloud(rgb_ptr_d);
    //     sample_d.setLeafSize(0.05, 0.05, 0.05);
    //     sample_d.filter(*rgb_ptr_d);
    //     saveCloud(rgb_ptr_d, "/home/fyx/ufo_hiahia/src/test/", frame_ssc.id, "_dynamic.pcd");
    // }
    
}

void SSC::refineClusterByIntensity(Frame& frame_ssc){
    int original_num = frame_ssc.cluster_set.size();
    int stage = 0;
    int iter = iteration;
    while(iter){
        TicToc refine_t("refine");
        std::vector<std::pair<int, Cluster>> clusters(frame_ssc.cluster_set.begin(), frame_ssc.cluster_set.end());
        std::sort(clusters.begin(), clusters.end(), sort1);
        std::vector<int> invalid_name;
        std::unordered_map<int, std::vector<int>> fusion_map;
        for(auto& c : clusters){
            if(findNameInVec(c.first, invalid_name)){
                continue;
            }
            std::vector<int> neighbor_name;
            std::vector<int> neighbor_vox;
            for(auto& v : c.second.occupy_voxels){
                std::vector<int> vox = findVoxelNeighbors(hash_cloud[v].range_idx, hash_cloud[v].sector_idx, hash_cloud[v].azimuth_idx, search_c);
                for(auto& n : vox){
                    std::unordered_map<int, Voxel>::iterator it_find = hash_cloud.find(n);
                    if(it_find != hash_cloud.end() && hash_cloud[n].intensity_cov <= intensity_cov && std::fabs(hash_cloud[v].intensity_av - hash_cloud[n].intensity_av) <= intensity_diff){
                        neighbor_vox.emplace_back(n);
                    }
                }
            }
            sampleVec(neighbor_vox);

            for(auto& n : neighbor_vox){
                if(!findNameInVec(hash_cloud[n].label, invalid_name)){
                    neighbor_name.emplace_back(hash_cloud[n].label);
                }
            }
            sampleVec(neighbor_name);

            if(neighbor_name.size() > 1){
                addVec(invalid_name, neighbor_name);
                fusion_map.insert(std::make_pair(c.first, neighbor_name));
            }
            sampleVec(invalid_name);
        }

        // update
        for(auto& cn : fusion_map){
            Cluster cluster_fusion;
            for(auto& f : cn.second){
                cluster_fusion.name = f;
                addVec(cluster_fusion.occupy_pts, frame_ssc.cluster_set[f].occupy_pts);
                addVec(cluster_fusion.occupy_voxels, frame_ssc.cluster_set[f].occupy_voxels);
                *cluster_fusion.cloud += *frame_ssc.cluster_set[f].cloud;
                frame_ssc.cluster_set.erase(f);
            }
            for(auto v : cluster_fusion.occupy_voxels){
                hash_cloud[v].label = cluster_fusion.name;
            }
            frame_ssc.cluster_set.insert(std::make_pair(cluster_fusion.name, cluster_fusion));
        }
        
        iter --;
        stage ++;
        ROS_DEBUG("refine stage %d: time_use(ms): %0.2f, cluster_num: %d", stage, (float)refine_t.toc(), (int)frame_ssc.cluster_set.size());
    }
}

void SSC::segment(){
    TicToc segment_t("segment");
    // cvc
    clusterAndCreateFrame(apri_vec, hash_cloud);
    int first_num = frame_ssc.cluster_set.size();
    
    // intensity compensate
    refineClusterByIntensity(frame_ssc);
    int second_num = frame_ssc.cluster_set.size();

    // boundingbox
    refineClusterByBoundingBox(frame_ssc);
    int third_num = frame_ssc.cluster_set.size();

    frame_ssc.hash_cloud = hash_cloud;

    ROS_INFO("segment: time_use(ms): %0.2f, first cluster_num: %d, second cluster_num: %d, third cluster_num: %d", (float)segment_t.toc(), first_num, second_num, third_num);

}

Feature SSC::getDescriptorByEigenValue(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster_cloud_){
    Eigen::Matrix3f covariance;	
    Eigen::Vector4f centeroid;		
    pcl::compute3DCentroid(*cluster_cloud_, centeroid);
    pcl::computeCovarianceMatrix(*cluster_cloud_, centeroid, covariance);	

    // eigen value
    constexpr bool compute_eigenvectors = false;
    Eigen::EigenSolver<Eigen::Matrix3f> eigenvalues_solver(covariance, compute_eigenvectors);
    std::vector<float> eigenvalues(3, 0.0);
    eigenvalues.at(0) = eigenvalues_solver.eigenvalues()[0].real();
    eigenvalues.at(1) = eigenvalues_solver.eigenvalues()[1].real();
    eigenvalues.at(2) = eigenvalues_solver.eigenvalues()[2].real();
    if(eigenvalues_solver.eigenvalues()[0].imag() != 0.0 || eigenvalues_solver.eigenvalues()[1].imag() != 0.0 || eigenvalues_solver.eigenvalues()[2].imag() != 0.0 ){
        ROS_WARN("Eigenvalues should not have non-zero imaginary component");;
        ROS_BREAK();
    }

    swap_if_gt(eigenvalues.at(0), eigenvalues.at(1));  // sort
    swap_if_gt(eigenvalues.at(0), eigenvalues.at(2));
    swap_if_gt(eigenvalues.at(1), eigenvalues.at(2));

    double sum_eigenvalues = eigenvalues.at(0) + eigenvalues.at(1) + eigenvalues.at(2);
    double e1 = eigenvalues.at(0) / sum_eigenvalues;
    double e2 = eigenvalues.at(1) / sum_eigenvalues;
    double e3 = eigenvalues.at(2) / sum_eigenvalues;
    const double sum_of_eigenvalues = e1 + e2 + e3;

    Feature eigenvalue_feature("eigenvalue");  // add 8 features 

    double linearity = std::fabs((e1 - e2) / e1 / kLinearityMax);
    eigenvalue_feature.feature_values.emplace_back(FeatureValue("linearity", linearity));
    // std::cout << "linearity: " << linearity << std::endl;

    double planarity = std::fabs((e2 - e3) / e1 / kPlanarityMax);
    eigenvalue_feature.feature_values.emplace_back(FeatureValue("planarity", planarity));
    // std::cout << "planarity: " << planarity << std::endl;

    double scattering = std::fabs(e3 / e1 / kScatteringMax);
    eigenvalue_feature.feature_values.emplace_back(FeatureValue("scattering", scattering));
    // std::cout << "scattering: " << scattering << std::endl;

    double omnivariance = std::fabs(std::pow(e1 * e2 * e3, kOneThird) / kOmnivarianceMax);
    eigenvalue_feature.feature_values.emplace_back(FeatureValue("omnivariance", omnivariance));
    // std::cout << "omnivariance: " << omnivariance << std::endl;

    double anisotropy = std::fabs((e1 - e3) / e1 / kAnisotropyMax);
    eigenvalue_feature.feature_values.emplace_back(FeatureValue("anisotropy", anisotropy));
    // std::cout << "anisotropy: " << anisotropy << std::endl;

    double eigen_entropy = std::fabs((e1 * std::log(e1)) + (e2 * std::log(e2)) + (e3 * std::log(e3)) / kEigenEntropyMax);
    eigenvalue_feature.feature_values.emplace_back(FeatureValue("eigen_entropy",eigen_entropy));
    // std::cout << "eigen_entropy: " << eigen_entropy << std::endl;

    double change_of_curvature = std::fabs(e3 / sum_of_eigenvalues / kChangeOfCurvatureMax);
    eigenvalue_feature.feature_values.emplace_back(FeatureValue("change_of_curvature", change_of_curvature));
    // std::cout << "change_of_curvature: " << change_of_curvature << std::endl;

    std::pair<pcl::PointXYZI, pcl::PointXYZI> bounding_box = getBoundingBoxOfCloud(cluster_cloud_);
    pcl::PointXYZI point_min = bounding_box.first;
    pcl::PointXYZI point_max = bounding_box.second;
    double diff_x, diff_y, diff_z;
    diff_x = point_max.x - point_min.x;
    diff_y = point_max.y - point_min.y;
    diff_z = point_max.z - point_min.z;
    if(diff_z > diff_x || diff_z > diff_y){
        eigenvalue_feature.feature_values.emplace_back(FeatureValue("point_up", 1));
    }
    else{
        eigenvalue_feature.feature_values.emplace_back(FeatureValue("point_up", 0));
    }

    eigenvalue_feature.feature_values.emplace_back(FeatureValue("max_z", point_max.z));
    eigenvalue_feature.feature_values.emplace_back(FeatureValue("min_z", point_min.z));

    eigenvalue_feature.feature_values.emplace_back(FeatureValue("type", -1));

    if(eigenvalue_feature.feature_values.size() != 11){
        ROS_WARN("get descriptor by eigen value error");
        ROS_BREAK();
    }
    return eigenvalue_feature;
}

Feature SSC::getDescriptorByEnsembleShape(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster_cloud_){
    pcl::ESFEstimation<pcl::PointXYZI, pcl::ESFSignature640> esf_estimator_;
    pcl::PointCloud<pcl::ESFSignature640>::Ptr signature(new pcl::PointCloud<pcl::ESFSignature640>);
    esf_estimator_.setInputCloud(cluster_cloud_);
    esf_estimator_.compute(*signature);
    if(signature->size() != 1){
        ROS_WARN("esf_estimator use wrong");;
        ROS_BREAK();
    }

    Feature ensembleshape_feature("ensemble_shape");
    const int bin_size = 64;
    const int dim = 10;
    double histogram[dim];
    for(int i = 0; i < 640; i++){
        histogram[i % 10] += signature->points[0].histogram[i];
    }
    for(int h = 0; h < dim; h++){
        ensembleshape_feature.feature_values.emplace_back(FeatureValue("esf_" + std::to_string(h), histogram[h]));
    }
    if(ensembleshape_feature.feature_values.size() != dim){
        ROS_WARN("ensembleshape_feature_matrix calculate error");
        ROS_BREAK();
    }

    return ensembleshape_feature;
}

Eigen::MatrixXd SSC::getFeature21(const Eigen::MatrixXd& eigenvalue_matrix_, const Eigen::MatrixXd& ensembleshape_matrix_){
    Eigen::MatrixXd f_21;
    f_21.setZero();
    f_21.resize(1, 21);
    f_21.block(0, 0, 1, 11) = eigenvalue_matrix_;
    f_21.block(0, 11, 1, 10) = ensembleshape_matrix_;
    return f_21;
}

bool SSC::regionGrowing(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster_cloud_){
    pcl::search::Search<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>());
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cluster_cloud_);
    normal_estimator.setKSearch(toBeClass);
    normal_estimator.compute (*normals);

    pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> reg;
    reg.setMinClusterSize (toBeClass * 15); 
    reg.setMaxClusterSize (100000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (toBeClass * 2);
    reg.setInputCloud (cluster_cloud_);
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (7.0 / 180.0 * M_PI);  // TODO: ? it is hard to get this value
    reg.setCurvatureThreshold (0.6);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);
    std::vector<int> plane_pts;
    for(int i = 0; i < clusters.size(); i++){
        for(auto& p : clusters[i].indices){
            plane_pts.emplace_back(p);
        }
    }

    if(plane_pts.size() >= cluster_cloud_->points.size() * 0.2){
        return true;
    }
    else{
        return false;
    }
}

void SSC::recognize(Frame& frame_ssc_){
    TicToc recognize_t("recognize");
    const double linearity_th = 0.02;
    const double planarity_th = 0.1;
    const double height =  1.0;
    const double diff_x = 8;
    for(auto& c : frame_ssc_.cluster_set){
        Feature eigen_f = getDescriptorByEigenValue(c.second.cloud);
        Eigen::MatrixXd eigen_f_11 = turnVec2Matrix(eigen_f.feature_values);
        Eigen::MatrixXd f_11 = eigen_f_11;
        
        if(regionGrowing(c.second.cloud)){
            f_11(0, 10) = (double)building;
            c.second.type = building;
            c.second.feature_matrix = f_11;
        }
        else{
            if(f_11(0, 8) > height){
                f_11(0, 10) = (double)tree;
                c.second.type = tree;
                c.second.feature_matrix = f_11;
            }
            else{
                f_11(0, 10) = (double)car;
                c.second.type = car;
                c.second.feature_matrix = f_11;
            }
        }
        // std::cout << "feature_matrix: " << c.second.feature_matrix << std::endl;
    }
    ROS_INFO("recognize: time_use(ms): %0.2f", (float)recognize_t.toc());
}

float SSC::compareFeature(const Eigen::MatrixXd& feature1_, const Eigen::MatrixXd& feature2_){
    float diff = 0.;
    Eigen::MatrixXd f_diff = (feature1_ - feature2_).cwiseAbs();
    diff += (f_diff (0, 0) * 0.5);  // linearity
    diff += (f_diff (0, 1) * 0.5);  // planarity
    diff += (f_diff (0, 2) * 0.2);  // scattering
    diff += (f_diff (0, 3) * 0.2);  // omnivariance
    diff += (f_diff (0, 4) * 0.2);  // anisotropy
    diff += (f_diff (0, 5) * 0.2);  // eigen_entropy
    diff += (f_diff (0, 6) * 0.2);  // change_of_curvature
    diff += (f_diff (0, 7) * 0.6);  // point_up
    diff += (f_diff (0, 8) * 0.2);  // max_z
    diff += (f_diff (0, 9) * 0.0);  // type
    return diff;
}

void SSC::getPose(){
    if(is_pcd){
        pcl::PointCloud<Pose>::Ptr pose_tmp(new pcl::PointCloud<Pose>());
        loadCloud(pose_tmp, pose_path);
        int all = pose_tmp->points.size();
        if(start < 0 || end > all){
            ROS_WARN("the start or end index set error");
            ros::shutdown();
        }
        for(size_t i = start; i < end; i++){
            pose_vec.emplace_back(pose_tmp->points[i]);
        }
    }
    else{  // TODO:
        std::fstream pose_file;
        std:string line;
        pose_file.open(pose_path, std::ios::in);
        int count = 0;

        // if(tr.size() != 12){
        //     ROS_WARN("extrinsic velo to cam error");
        //     ros::shutdown();
        // }

        // Eigen::Matrix4f trans;
        // trans(0, 0) = tr[0]; trans(0, 1) = tr[1]; trans(0, 2) = tr[2]; trans(0, 3) = tr[3];
        // trans(1, 0) = tr[4]; trans(1, 1) = tr[5]; trans(1, 2) = tr[6]; trans(1, 3) = tr[7];
        // trans(2, 0) = tr[8]; trans(2, 1) = tr[9]; trans(2, 2) = tr[10]; trans(2, 3) = tr[11];
        // trans(3, 0) = 0.0; trans(3, 1) = 0.0; trans(3, 2) = 0.0; trans(3, 3) = 1.0;

        while(getline(pose_file, line)){
            if(count < start || (count - start) % skip != 0){
                count ++;
                continue;
            }
            if(count >= end){
                break;
            }
            std::vector<std::string> line_s;
            float pose_v[12];
            boost::split(line_s, line, boost::is_any_of(" "));
            // std::cout << "line: " << count << " ";
            for(int l = 0; l < line_s.size(); l++){
                pose_v[l] = atof(line_s[l].c_str()); 
                // std::cout << pose_v[l] << " ";
            }
            // std::cout << std::endl;

            Eigen::Matrix4f cam;
            cam(0, 0) = pose_v[0]; cam(0, 1) = pose_v[1]; cam(0, 2) = pose_v[2]; cam(0, 3) = pose_v[3];
            cam(1, 0) = pose_v[4]; cam(1, 1) = pose_v[5]; cam(1, 2) = pose_v[6]; cam(1, 3) = pose_v[7];
            cam(2, 0) = pose_v[8]; cam(2, 1) = pose_v[9]; cam(2, 2) = pose_v[10]; cam(2, 3) = pose_v[11];
            cam(3, 0) = 0.0; cam(3, 1) = 0.0; cam(3, 2) = 0.0; cam(3, 3) = 1.0;

            Eigen::Matrix4f velo_to_cam = tr.inverse() * cam * tr;
            trans_vec.emplace_back(velo_to_cam);
            Pose pose;
            Eigen::Matrix3f rotation;
            pose.x = velo_to_cam(0, 3);
            pose.y = velo_to_cam(1, 3);
            pose.z = velo_to_cam(2, 3);
            rotation(0, 0) = velo_to_cam(0, 0);
            rotation(0, 1) = velo_to_cam(0, 1);
            rotation(0, 2) = velo_to_cam(0, 2);
            rotation(1, 0) = velo_to_cam(1, 0);
            rotation(1, 1) = velo_to_cam(1, 1);
            rotation(1, 2) = velo_to_cam(1, 2);
            rotation(2, 0) = velo_to_cam(2, 0);
            rotation(2, 1) = velo_to_cam(2, 1);
            rotation(2, 2) = velo_to_cam(2, 2);
            Eigen::Vector3f rpy = rotationMatrixToEulerAngles(rotation);
            // Eigen::Vector3f rpy = rotation.eulerAngles(2, 1, 0);  // there is error about .eulerAngles()
            pose.roll = rpy[0];
            pose.pitch = rpy[1];
            pose.yaw = rpy[2];

            count ++;
            pose_vec.emplace_back(pose);
        }
    }
    
    ROS_DEBUG("load pose size: %d", (int)pose_vec.size());
}

void SSC::getCloud(){
    if(is_pcd){
        std::vector<std::string> cloud_name;
        for(auto& entry_ : fs::directory_iterator(data_path)){
            cloud_name.emplace_back(entry_.path());
        }
        std::sort(cloud_name.begin(), cloud_name.end(), fileSort);

        int all = cloud_name.size();
        if(start < 0 || end > all){
            ROS_WARN("the start or end index set error");
            ros::shutdown();
        }

        for(size_t i = start; i < end; i++){
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
            loadCloud(cloud_tmp, cloud_name[i]);
            cloud_vec.emplace_back(cloud_tmp);
        }
    }
    else{  //TODO:
        std::vector<std::string> bin_name;
        for(auto& entry_ : fs::directory_iterator(data_path)){
            bin_name.emplace_back(entry_.path());
        }
        std::sort(bin_name.begin(), bin_name.end(), fileSort);

        std::vector<std::string> label_name;
        for(auto& entry_ : fs::directory_iterator(label_path)){
            label_name.emplace_back(entry_.path());
        }
        std::sort(label_name.begin(), label_name.end(), fileSort);

        if(bin_name.size() != label_name.size()){
            ROS_WARN("bins or labels load error");
            ros::shutdown();
        }

        std::vector<uint32_t> labels;
        int it = 0;
        for(size_t i = start; i < end; i = i + skip){
            pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZI>());  // use in segDF
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());  // save

            std::ifstream in_label(label_name[i], std::ios::binary);
            if(!in_label.is_open()){
                ROS_ERROR("can't open %s", label_name[i].c_str());
                ros::shutdown();
            }
            in_label.seekg(0, std::ios::end);
            uint32_t num_points = in_label.tellg() / sizeof(uint32_t);
            in_label.seekg(0, std::ios::beg);
            std::vector<uint32_t> values_label(num_points);
            in_label.read((char *)&values_label[0], num_points * sizeof(uint32_t));
            std::ifstream in_cloud(bin_name[i], std::ios::binary);
            std::vector<float> values_cloud(4 * num_points);
            in_cloud.read((char *)&values_cloud[0], 4 * num_points * sizeof(float));

            // std::cout << values_label.size() << std::endl;
            // addVec(labels, values_label);

            for(uint32_t k = 0; k < num_points; k++){
                if(values_label[k] &  0xFFFF == 0){  // unlabeled
                    continue;
                }

                pcl::PointXYZI xyzi;
                xyzi.x = values_cloud[k * 4];
                xyzi.y = values_cloud[k * 4 + 1];
                xyzi.z = values_cloud[k * 4 + 2];
                xyzi.intensity = values_cloud[k * 4 + 3] * max_intensity;

                pcl::PointXYZRGB rgb;
                rgb.x = values_cloud[k * 4];
                rgb.y = values_cloud[k * 4 + 1];
                rgb.z = values_cloud[k * 4 + 2];
                
                if(findNameInVec((values_label[k] &  0xFFFF), dynamic_label)){
                    rgb.r = 255.f;
                    rgb.g = 0.f;
                    rgb.b = 0.f;
                }
                else{
                    rgb.r = 205.f;
                    rgb.g = 192.f;
                    rgb.b = 176.f;
                }
                raw_cloud->points.emplace_back(xyzi);
                rgb_cloud->points.emplace_back(rgb);
            }
            in_label.close();
            in_cloud.close();

            pcl::VoxelGrid<pcl::PointXYZI> sample;  // downsampling
            sample.setInputCloud(raw_cloud);
            sample.setLeafSize(0.08, 0.08, 0.08);
            sample.filter(*raw_cloud);

            pcl::VoxelGrid<pcl::PointXYZRGB> sample2;  // downsampling
            sample2.setInputCloud(rgb_cloud);
            sample2.setLeafSize(0.08, 0.08, 0.08);
            sample2.filter(*rgb_cloud);

            cloud_vec.emplace_back(raw_cloud);

            pcl::transformPointCloud(*rgb_cloud, *rgb_cloud, trans_vec[it]);
            saveCloud(rgb_cloud, pcd_save, i, "_semantickitti.pcd");
            it ++;
        }

        // sampleVec(labels);
        // for(auto& l : labels){
        //     std::cout << l << " ";
        // }
        // std::cout << std::endl;
    }
    
    ROS_DEBUG("load cloud size: %d", (int)cloud_vec.size());
} 

Frame SSC::intialization(const std::vector<Frame>& frames_, const std::vector<Pose>& poses_){
    TicToc init_t("initialization");
    // get based frame
    int max_num = 999999;
    int id_based;
    for(int i = 0; i < frames_.size(); i++){
        if(frames_[i].cluster_set.size() <= max_num){
            max_num = frames_[i].cluster_set.size();
            id_based = i;
        }
    }

    // initialize
    Frame frame_based = frames_[id_based];
    Pose pose_based = poses_[id_based];
    Eigen::Affine3f trans_based = pcl::getTransformation(pose_based.x, pose_based.y, pose_based.z, pose_based.roll, pose_based.pitch, pose_based.yaw);

    for(int i = 0; i < frames_.size(); i++){
        if(i == id_based){
            continue;
        }
        Frame frame_i = frames_[i];
        Pose pose_i = poses_[i];
        Eigen::Affine3f trans_i = pcl::getTransformation(pose_i.x, pose_i.y, pose_i.z, pose_i.roll, pose_i.pitch, pose_i.yaw);
        Eigen::Affine3f trans_bi = trans_based.inverse() * trans_i;

        // test transform
        // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_o(new pcl::PointCloud<pcl::PointXYZI>());  
        // transformCloud(frame_i.cloud_use, trans_bi, cloud_o);
        // std::cout << cloud_o->points.size() << std::endl;
        // saveCloud(cloud_o, "/home/fyx/ufo_hiahia/src/out/seg/", frame_i.id, "_o.pcd");
        
        for(auto& c : frame_i.cluster_set){
            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>());
            transformCloud(c.second.cloud, trans_bi, cluster);

            std::unordered_map<int, std::vector<int>> remap_name;
            for(int k = 0; k < cluster->points.size(); k++){
                pcl::PointXYZI pt = cluster->points[k];
                float dis = pointDistance2d(pt);
                float angle = getPolarAngle(pt);
                float azimuth = getAzimuth(pt);
                int range_idx = std::ceil((dis - min_dis) / range_res) - 1;
                int sector_idx = std::ceil((angle - min_angle) / sector_res) - 1;
                int azimuth_idx = std::ceil((azimuth - min_azimuth) / azimuth_res) -1;
                int voxel_idx = azimuth_idx * range_num * sector_num + range_idx * sector_num + sector_idx;

                std::unordered_map<int, Voxel>::iterator it_find = frame_based.hash_cloud.find(voxel_idx);
                if(it_find != frame_based.hash_cloud.end() && it_find->second.label != -1){
                    std::unordered_map<int, std::vector<int>>::iterator l_find = remap_name.find(it_find->second.label);
                    if(l_find == remap_name.end()){
                        std::vector<int> vec;
                        vec.emplace_back(it_find->first);
                        remap_name.insert(std::make_pair(it_find->second.label, vec));
                    }
                    else{
                        l_find->second.emplace_back(it_find->first);
                    }
                }
            }
            
            // update
            if(remap_name.size() > 1){
                Cluster cluster_fusion;
                std::vector<int> erase_id;
                for(auto& re : remap_name){

                    // test
                    // std::unordered_map<int, Cluster>::iterator find = frame_based.cluster_set.find(re.first);
                    // if(find == frame_based.cluster_set.end()){
                    //     std::cout << "error" << std::endl;
                    // }

                    sampleVec(re.second);
                    if(((float)re.second.size() / (float)frame_based.cluster_set[re.first].occupy_voxels.size()) >= occupancy){
                        erase_id.emplace_back(re.first);
                        cluster_fusion.name = re.first;
                        addVec(cluster_fusion.occupy_pts, frame_based.cluster_set[re.first].occupy_pts);
                        addVec(cluster_fusion.occupy_voxels, frame_based.cluster_set[re.first].occupy_voxels);
                        *cluster_fusion.cloud += *frame_based.cluster_set[re.first].cloud;
                    }
                }
                for(auto& e : erase_id){
                    frame_based.cluster_set.erase(e);
                }
                frame_based.cluster_set.insert(std::make_pair(cluster_fusion.name, cluster_fusion));
                for(auto& v : cluster_fusion.occupy_voxels){
                    frame_based.hash_cloud[v].label = cluster_fusion.name;
                }
            }
        }
    }

    // initialize tacking map
    recognize(frame_based);
    frame_set[frame_based.id - start] = frame_based;
    mapping_init = true;

    ROS_INFO("initialization: time_use(ms): %0.2f, based_id: %d, initialized cluster_num: %d", (float)init_t.toc(), frame_based.id, (int)frame_based.cluster_set.size());
    return frame_based;
}

void SSC::tracking(Frame& frame_pre_, Frame& frame_next_, Pose pose_pre_, Pose pose_next_){
    // transform previous frame to next frame
    TicToc track_t("tracking");

    int next_cluster = frame_next_.cluster_set.size();
    Eigen::Affine3f trans_next = pcl::getTransformation(pose_next_.x, pose_next_.y, pose_next_.z, pose_next_.roll, pose_next_.pitch, pose_next_.yaw);
    Eigen::Affine3f trans_pre = pcl::getTransformation(pose_pre_.x, pose_pre_.y, pose_pre_.z, pose_pre_.roll, pose_pre_.pitch, pose_pre_.yaw);
    Eigen::Affine3f trans_np = trans_next.inverse() * trans_pre;

    int dynamic_num = 0;
    cv::RNG rng(12345);
    for(auto& c : frame_pre_.cluster_set){
        if(c.second.type != car){
            continue;
        }

        if(c.second.track_id == -1){
            c.second.track_id = name;
            c.second.color[0] = rng.uniform(20, 200);
            c.second.color[1] = rng.uniform(20, 200);
            c.second.color[2] = rng.uniform(20, 200);
            name ++;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>());
        transformCloud(c.second.cloud, trans_np, cluster);

        std::unordered_map<int, std::vector<int>> remap_name;
        for(int k = 0; k < cluster->points.size(); k++){
                pcl::PointXYZI pt = cluster->points[k];
                float dis = pointDistance2d(pt);
                float angle = getPolarAngle(pt);
                float azimuth = getAzimuth(pt);
                int range_idx = std::ceil((dis - min_dis) / range_res) - 1;
                int sector_idx = std::ceil((angle - min_angle) / sector_res) - 1;
                int azimuth_idx = std::ceil((azimuth - min_azimuth) / azimuth_res) -1;
                int voxel_idx = azimuth_idx * range_num * sector_num + range_idx * sector_num + sector_idx;

                // std::vector<int> neighbor_vox = findVoxelNeighbors(range_idx, sector_idx, azimuth_idx, search_c);
                // for(auto& n : neighbor_vox){
                //     std::unordered_map<int, Voxel>::iterator it_find = frame_next_.hash_cloud.find(n);
                //     if(it_find != frame_next_.hash_cloud.end() && it_find->second.label != -1){
                //         std::unordered_map<int, std::vector<int>>::iterator l_find = remap_name.find(it_find->second.label);
                //         if(l_find == remap_name.end()){
                //             std::vector<int> vec;
                //             vec.emplace_back(it_find->first);
                //             remap_name.insert(std::make_pair(it_find->second.label, vec));
                //         }
                //         else{
                //             l_find->second.emplace_back(it_find->first);
                //         }
                //     }
                // }

                std::unordered_map<int, Voxel>::iterator it_find = frame_next_.hash_cloud.find(voxel_idx);
                if(it_find != frame_next_.hash_cloud.end() && it_find->second.label != -1){
                    std::unordered_map<int, std::vector<int>>::iterator l_find = remap_name.find(it_find->second.label);
                    if(l_find == remap_name.end()){
                        std::vector<int> vec;
                        vec.emplace_back(it_find->first);
                        remap_name.insert(std::make_pair(it_find->second.label, vec));
                    }
                    else{
                        l_find->second.emplace_back(it_find->first);
                    }
                }

            }

            for(auto& re : remap_name){
                sampleVec(re.second);
            }

            if(remap_name.size() == 0){
                c.second.state = 1;
                dynamic_num ++;
            }

            else if(remap_name.size() == 1){
                std::unordered_map<int, std::vector<int>>::iterator it = remap_name.begin();
                
                // int pt_num = 0;
                // for(auto& vp : it->second){
                //     pt_num += frame_next_.hash_cloud[vp].ptIdx.size();
                // }

                if(((float)it->second.size() / (float)frame_next_.cluster_set[it->first].occupy_voxels.size()) < occupancy){
                // if(((float)pt_num / (float)frame_next_.cluster_set[it->first].occupy_pts.size()) < occupancy){
                    if(frame_next_.cluster_set[it->first].type == car){
                        c.second.state = 1;
                        dynamic_num ++;
                    }
                    else{
                        c.second.state = 0;
                        // Cluster cluster_new;
                        // cluster_new.track_id = c.second.track_id;
                        // cluster_new.name = frame_next_.max_name ++;
                        // // cluster_new.type = car;
                        // cluster_new.type = frame_next_.cluster_set[it->first].type;
                        // cluster_new.color[0] = c.second.color[0];
                        // cluster_new.color[1] = c.second.color[1];
                        // cluster_new.color[2] = c.second.color[2];
                        // cluster_new.occupy_voxels = it->second;
                        // reduceVec(frame_next_.cluster_set[it->first].occupy_voxels, cluster_new.occupy_voxels);
                        // for(auto& v : it->second){
                        //     frame_next_.hash_cloud[v].label = cluster_new.name;
                        //     addVec(cluster_new.occupy_pts, frame_next_.hash_cloud[v].ptIdx);
                        // }
                        // getCloudByVec(frame_next_.cloud_use, cluster_new.occupy_pts, cluster_new.cloud);
                        // *cluster_new.cloud += *cluster;
                        // reduceVec(frame_next_.cluster_set[it->first].occupy_pts, cluster_new.occupy_pts);
                        // frame_next_.cluster_set.insert(std::make_pair(cluster_new.name, cluster_new));
                    }    
                }

                else{
                    if(frame_next_.cluster_set[it->first].type == car){
                        c.second.state = 0;
                        frame_next_.cluster_set[it->first].track_id = c.second.track_id;
                        *frame_next_.cluster_set[it->first].cloud += *cluster;
                        frame_next_.cluster_set[it->first].color[0] = c.second.color[0];
                        frame_next_.cluster_set[it->first].color[1] = c.second.color[1];
                        frame_next_.cluster_set[it->first].color[2] = c.second.color[2];
                    }
                    // c.second.state = 0;
                    // frame_next_.cluster_set[it->first].track_id = c.second.track_id;
                    // *frame_next_.cluster_set[it->first].cloud += *cluster;
                    // frame_next_.cluster_set[it->first].color[0] = c.second.color[0];
                    // frame_next_.cluster_set[it->first].color[1] = c.second.color[1];
                    // frame_next_.cluster_set[it->first].color[2] = c.second.color[2];
                }
            }

            else if(remap_name.size() > 1){
                c.second.state = 0;
            //     Cluster cluster_new;
            //     cluster_new.track_id = c.second.track_id;
            //     cluster_new.name = frame_next_.max_name ++;
            //     cluster_new.type = car;
            //     cluster_new.color[0] = c.second.color[0];
            //     cluster_new.color[1] = c.second.color[1];
            //     cluster_new.color[2] = c.second.color[2];

            //     for(auto& re : remap_name){
            //         if(frame_next_.cluster_set[re.first].type == car && ((float)re.second.size() / (float)frame_next_.cluster_set[re.first].occupy_voxels.size()) >= occupancy){
            //             addVec(cluster_new.occupy_pts, frame_next_.cluster_set[re.first].occupy_pts);
            //             addVec(cluster_new.occupy_voxels, frame_next_.cluster_set[re.first].occupy_voxels);
            //             frame_next_.cluster_set.erase(re.first);
            //         }
            //     }
            //     getCloudByVec(frame_next_.cloud_use, cluster_new.occupy_pts, cluster_new.cloud);
            //     *cluster_new.cloud += *cluster;
            //     for(auto& v : cluster_new.occupy_voxels){
            //         frame_next_.hash_cloud[v].label = cluster_new.name;
            //     }
            //     frame_next_.cluster_set.insert(std::make_pair(cluster_new.name, cluster_new));
            }
    }

    ROS_INFO("frame %d is tracked in frame %d: time_use(ms): %0.2f, cluster_num of frame_next is %d, after compensatation is %d, dynamic object_num: %d",frame_pre_.id, frame_next_.id, (float)track_t.toc(), next_cluster, (int)frame_next_.cluster_set.size(), dynamic_num);
}

void SSC::segDF(){
    id = start;
    getPose();
    getCloud();      
    std::cout << "\n";

    for(auto& cloud : cloud_vec){
        ROS_INFO("frame %d is added into the segDF", id);
        process(cloud);
        segment();
        saveSegCloud(frame_ssc, frame_ssc.cloud_use, seg_save, 1);
        recognize(frame_ssc);
        frame_set.emplace_back(frame_ssc);
        reset();
        id += skip;
        std::cout << "\n";
    }

    // std::vector<Frame> frame_init;
    // std::vector<Pose> pose_init;
    // for(int i = 0; i < init; i++){
    //     frame_init.emplace_back(frame_set[i]);
    //     pose_init.emplace_back(pose_vec[i]);
    // }

    // frame_based = intialization(frame_init, pose_init);
    // saveSegCloud(frame_based, frame_based.cloud_use);
    // std::cout << "\n";


    // int start_df = frame_based.id - start;
    for(int i = 0 ; i < frame_set.size() - 1; i ++ ){
        tracking(frame_set[i], frame_set[i + 1], pose_vec[i], pose_vec[i + 1]);
    }

    for(int i = 0; i < frame_set.size() - 1; i ++){
        Eigen::Affine3f trans_i = pcl::getTransformation(pose_vec[i].x, pose_vec[i].y, pose_vec[i].z, pose_vec[i].roll, pose_vec[i].pitch, pose_vec[i].yaw);
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
        transformCloud(frame_set[i].cloud_use, trans_i, cloud);
        saveSegCloud(frame_set[i], cloud, map_save, 3);

        pcl::VoxelGrid<pcl::PointXYZI> sample2;  // downsampling
        sample2.setInputCloud(g_cloud_vec[i]);
        // sample2.setLeafSize(1.0, 1.0, 1.0);
        sample2.setLeafSize(0.08, 0.08, 0.08);
        sample2.filter(*g_cloud_vec[i]);

        pcl::PointCloud<pcl::PointXYZI>::Ptr g_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        transformCloud(g_cloud_vec[i], trans_i, g_cloud);
        // * g_cloud += * g_cloud_vec[i];
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb(new pcl::PointCloud<pcl::PointXYZRGB>());
        for(size_t k = 0; k < g_cloud->points.size(); k++){
            pcl::PointXYZRGB pt;
            pt.x = g_cloud->points[k].x;
            pt.y = g_cloud->points[k].y;
            pt.z = g_cloud->points[k].z;
            pt.r = 255.f;
            pt.g = 222.f;
            pt.b = 173.f;
            rgb->points.push_back(pt);
        }
        saveCloud(rgb, map_save, frame_set[i].id, "_g.pcd");
    }   
}


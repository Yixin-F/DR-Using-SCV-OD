#include "utility.h"
#include <pcl/filters/extract_indices.h>

int main(int argc, char** argv){
    pcl::PointCloud<pcl::PointXYZI>::Ptr static_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr ori_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::io::loadPCDFile(argv[1], *ori_cloud);
    pcl::io::loadPCDFile(argv[2], *static_cloud);
    // pcl::VoxelGrid<pcl::PointXYZI> down;
    // down.setLeafSize(0.1, 0.1, 0.1);
    // down.setInputCloud(ori_cloud);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ori_cloud_down(new pcl::PointCloud<pcl::PointXYZI>());
    // down.filter(*ori_cloud_down);
    *ori_cloud_down += *ori_cloud;
    pcl::KdTreeFLANN<pcl::PointXYZI> kd;
    kd.setInputCloud(ori_cloud_down);
    std::vector<int> static_id;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ()); 
    for(int i = 0; i < static_cloud->points.size(); i++){
        pcl::PointXYZI pt = static_cloud->points[i];
        std::vector<int> id;
        std::vector<float> dis;
        kd.nearestKSearch(pt, 1, id, dis);
        inliers->indices.emplace_back(id[0]);
    }
    
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(ori_cloud_down);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*dynamic_cloud);
    
    std::string name = std::string(argv[3]) + "dynamic_cloud.pcd";
    pcl::io::savePCDFile(name, *dynamic_cloud);
    std::cout << "done" << std::endl;
    return 0;

}
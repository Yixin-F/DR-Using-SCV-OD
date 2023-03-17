#include "utility.h"

std::vector<int> ground{40, 44, 48, 49, 71, 72};
std::vector<int> building{50, 51, 52, 60};
std::vector<int> tree{70, 80, 81};

bool findNameInVec(const int& name_, const std::vector<int>& vec_){
    if(std::count(vec_.begin(), vec_.end(), name_)){
        return true;
    }
    else{
        return false;
    }
}

int check(int in_){
    if(findNameInVec(in_, ground)){
        return 40;
    }
    else if(findNameInVec(in_, building)){
        return 50;
    }
    else if(findNameInVec(in_, tree)){
        return 70;
    }
    else{
        return 0;
    }
}

int ground_s[2];   // P, N
int  building_s[2];
int  tree_s[2];
int  pd_s[2];

int ground_num = 0;
int building_num = 0;
int tree_num = 0;
int pd_num = 0;

int main(int argc, char ** argv){
    std::cout << "wait.." << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plot(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile(argv[1], *plot);
    std::cout << "plot: " << plot->points.size() <<std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr truth(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(argv[2], *truth);
    std::cout << "truth: " << truth->points.size() << std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr estimate(new pcl::PointCloud<pcl::PointXYZI>);

    for(size_t i = 0; i < plot->points.size(); i++){
        pcl::PointXYZRGB rgb = plot->points[i];
        if(rgb.r == 255.f && rgb.g == 222.f && rgb.b == 173.f){  // ground
            pcl::PointXYZI xyzi;
            xyzi.x = rgb.x;
            xyzi.y = rgb.z;
            xyzi.z = rgb.z;
            xyzi.intensity = 40;
            estimate->points.emplace_back(xyzi);
        }
        else if(rgb.r == 139.f && rgb.g == 90.f && rgb.b == 0.f){ // building
            pcl::PointXYZI xyzi;
            xyzi.x = rgb.x;
            xyzi.y = rgb.z;
            xyzi.z = rgb.z;
            xyzi.intensity = 50;
            estimate->points.emplace_back(xyzi);
        }
        else if(rgb.r == 162.f && rgb.g == 205.f && rgb.b == 90.f){  // tree
            pcl::PointXYZI xyzi;
            xyzi.x = rgb.x;
            xyzi.y = rgb.z;
            xyzi.z = rgb.z;
            xyzi.intensity = 70;
            estimate->points.emplace_back(xyzi);
        }
        else{
            pcl::PointXYZI xyzi;
            xyzi.x = rgb.x;
            xyzi.y = rgb.z;
            xyzi.z = rgb.z;
            xyzi.intensity = 0;
            estimate->points.emplace_back(xyzi);
        }
    }

    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(estimate);
    for(size_t i = 0; i < truth->points.size(); i++){
        std::cout << i << " ";
        pcl::PointXYZI pt = truth->points[i];
        std::vector<int> id;
        std::vector<float> dis;
        kdtree.nearestKSearch(pt, 1, id, dis);
        if(check((int)pt.intensity & 0xFFFF) == 40){  // ground
            ground_num ++;
            if(estimate->points[id[0]].intensity == 40){
                ground_s[0] ++;
                continue;
            }
            else{
                ground_s[1] ++;
                continue;
            }
        }
        else if(check((int)pt.intensity & 0xFFFF) == 50){  // building
            building_num ++;
            if(estimate->points[id[0]].intensity == 50 || estimate->points[id[0]].intensity == 70){
                building_s[0] ++;
                continue;
            }
            else{
                building_s[1] ++;
                continue;
            }
        }
        else if(check((int)pt.intensity & 0xFFFF) == 70){  // tree
            tree_num ++;
            if(estimate->points[id[0]].intensity == 70 || estimate->points[id[0]].intensity == 50){
                tree_s[0] ++;
                continue;
            }
            else{
                tree_s[1] ++;
                continue;
            }
        }
        else{
            pd_num ++;
            // || estimate->points[id[0]].intensity == 70
            if(estimate->points[id[0]].intensity == 0  || dis[0] > 0.5){
                pd_s[0] ++;
                continue;
            }
            else{
                pd_s[1] ++;
                continue;
            }
        }
    }    

    std::cout << "estimate: " << estimate->points.size() << std::endl;
    std::cout << "ground :" << "size: " <<  ground_num << " P: " << (float)ground_s[0]/ground_num << " N: " << (float)ground_s[1]/ground_num << "\n"
                       << "building :"<< "size: " <<  building_num << " P: " << (float)building_s[0]/building_num << " N: " << (float)building_s[1]/building_num << "\n"
                       << "tree :" << "size: " <<  tree_num << " P: " << (float)tree_s[0]/tree_num << " N: " << (float)tree_s[1]/tree_num << "\n"
                       << "pd :" << "size: " <<  pd_num << " P: " << (float)pd_s[0]/pd_num << " N: " << (float)pd_s[1]/pd_num << std::endl;

}
#include "utility.h"

bool fileSort(std::string name1_, std::string name2_){  // filesort by name
    std::string::size_type iPos1 = name1_.find_last_of('/') + 1;
	std::string filename1 = name1_.substr(iPos1, name1_.length() - iPos1);
	std::string name1 = filename1.substr(0, filename1.rfind("."));

    std::string::size_type iPos2 = name2_.find_last_of('/') + 1;
    std::string filename2 = name2_.substr(iPos2, name2_.length() - iPos2);
	std::string name2 = filename2.substr(0, filename2.rfind(".")); 

    return std::stoi(name1) < std::stoi(name2);
}

int main(int argc, char** argv){
    std::string in = std::string(argv[1]);
    std::string out = std::string(argv[2]);
    // std::cout << 1 << std::endl;
    std::vector<std::string> cloud_name;
    for(auto& entry_ : fs::directory_iterator(in)){
        // std::cout << entry_.path() << std::endl;
        cloud_name.emplace_back(entry_.path());
    }
    std::sort(cloud_name.begin(), cloud_name.end(), fileSort);
    // std::cout << in << " " << out  << std::endl;
    // for(auto& name : cloud_name){
    //     std::cout << name << std::endl;
    // }
    int count = 0;
    for(int i = 0; i < cloud_name.size(); i = i+2){
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr g(new pcl::PointCloud<pcl::PointXYZRGB>());
        std::cout << cloud_name[i] << std::endl;
        pcl::io::loadPCDFile(cloud_name[i], *g);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr ng(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::io::loadPCDFile(cloud_name[i+1], *ng);
        std::cout << cloud_name[i+1] << std::endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr use(new pcl::PointCloud<pcl::PointXYZRGB>());
        *use+=*g;
        *use+=*ng;
        pcl::PointCloud<pcl::PointXYZI>::Ptr save(new pcl::PointCloud<pcl::PointXYZI>());
        save->height = 1;
        for(int k = 0; k < use->points.size(); k++){
            pcl::PointXYZI pt;
            pt.x = use->points[k].x;
            pt.y = use->points[k].y;
            pt.z = use->points[k].z;
            pt.intensity = 0.0;
            save->points.emplace_back(pt);
        }
        save->width = use->points.size();
        std::string use_name = out + std::to_string(count) + ".pcd";
        pcl::io::savePCDFile(use_name, *save);
        std::cout << use_name << std::endl;
        count ++;
    }
    return 0;
}
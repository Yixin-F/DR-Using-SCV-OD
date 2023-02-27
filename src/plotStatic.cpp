#include "utility.h"

// argv: 0: estimated static map name   1: save path
std::vector<int> dynamic{252, 253, 254, 255, 256, 257, 258, 259};

bool findNameInVec(const int& name_, const std::vector<int>& vec_){
    if(std::count(vec_.begin(), vec_.end(), name_)){
        return true;
    }
    else{
        return false;
    }
}

int main(int argc, char** argv){
    std::cout << "wait.." << std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr plot(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(argv[1], *plot);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr plot_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    plot_rgb->height = 1;
    int count = 0;
    for(size_t i = 0; i < plot->points.size(); i++){
        pcl::PointXYZI xyz = plot->points[i];
        pcl::PointXYZRGB rgb;
        rgb.x = xyz.x;
        rgb.y = xyz.y;
        rgb.z = xyz.z;
        
        uint32_t label = static_cast<uint32_t>(plot->points[i].intensity);
        
        if(findNameInVec(label & 0xFFFF, dynamic)){
            rgb.r = 255.f;
            rgb.g = 0.f;
            rgb.b = 0.f;
            // std::cout << label << " ";
        }
        else{
            rgb.r = 205.f;
            rgb.g = 192.f;
            rgb.b = 176.f;
        }
        plot_rgb->points.emplace_back(rgb);
        count ++;
    }
    plot_rgb->width = count;
    std::string save_path = (std::string)argv[2] + "/666_remain.pcd";
    pcl::io::savePCDFile(save_path, *plot_rgb);
    std::cout << "done.." << std::endl;

}
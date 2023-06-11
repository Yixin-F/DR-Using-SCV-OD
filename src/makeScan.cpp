#include <vector>
#include <cmath>
#include <algorithm>
#include <stdlib.h>
#include <unordered_map>
#include <utility>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <random>
#include <filesystem> // requires gcc version >= 8

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>

#include "utility.h"


namespace fs = std::filesystem; // file-process

bool fileSort(std::string name1_, std::string name2_){  // filesort by name
    std::string::size_type iPos1 = name1_.find_last_of('/') + 1;
	std::string filename1 = name1_.substr(iPos1, name1_.length() - iPos1);
	std::string name1 = filename1.substr(0, filename1.rfind("."));

    std::string::size_type iPos2 = name2_.find_last_of('/') + 1;
    std::string filename2 = name2_.substr(iPos2, name2_.length() - iPos2);
	std::string name2 = filename2.substr(0, filename2.rfind(".")); 

    return std::stoi(name1) < std::stoi(name2);
}

Eigen::Vector3f rotationMatrixToEulerAngles(Eigen::Matrix3f &R){
        float sy = sqrt(R(0,0) * R(0,0) + R(1,0) * R(1,0));
        bool singular = sy < 1e-6;
        float x, y, z;
        if (!singular)
        {
            x = atan2( R(2,1), R(2,2));
            y = atan2(-R(2,0), sy);
            z = atan2( R(1,0), R(0,0));
        }
        else
        {
            x = atan2(-R(1,2), R(1,1));
            y = atan2(-R(2,0), sy);
            z = 0;
        }
        return {x, y, z};
    }

    template<typename CloudT> 
    void transformCloud(const CloudT& cloudIn_, const Eigen::Affine3f& transCur_, CloudT& cloudOut_){
        int cloudSize = cloudIn_->points.size();
        cloudOut_->points.resize(cloudSize);
    
        #pragma omp parallel for num_threads(8)
        for(int i = 0; i < cloudSize; i++){
            cloudOut_->points[i].x = transCur_(0,0) * cloudIn_->points[i].x + transCur_(0,1) * cloudIn_->points[i].y + transCur_(0,2) * cloudIn_->points[i].z + transCur_(0,3);
            cloudOut_->points[i].y = transCur_(1,0) * cloudIn_->points[i].x + transCur_(1,1) * cloudIn_->points[i].y + transCur_(1,2) * cloudIn_->points[i].z + transCur_(1,3);
            cloudOut_->points[i].z = transCur_(2,0) * cloudIn_->points[i].x + transCur_(2,1) * cloudIn_->points[i].y + transCur_(2,2) * cloudIn_->points[i].z + transCur_(2,3);
            cloudOut_->points[i].intensity = cloudIn_->points[i].intensity;
        }
    }
const std::string pcd_path = "/media/fyx/Yixin F/seu_mid360/highway/highway_PCD/";
const std::string new_pcd_path = "/media/fyx/Yixin F/seu_mid360/highway/highway_use/";
const std::string pose_path = "/media/fyx/Yixin F/seu_mid360/highway/highway_poses.txt";
const std::string new_pose_path = "/media/fyx/Yixin F/seu_mid360/highway/poses_use.txt";

const std::string erasor_pose_path = "/media/fyx/Yixin F/seu_mid360/highway/poses_erasor.txt";
const std::string erasor_pcd_path = "/media/fyx/Yixin F/seu_mid360/highway/highway_erasor/";

const int start = 900;
const int end = 1100;

const int interval = 3;


std::vector<Eigen::Matrix3f> rot_name;
std::vector<Eigen::Vector3f> trans_name;
std::fstream pose_out;
std::fstream erasor_out;

int main(){
    
    pose_out.open(new_pose_path, std::ios::out);
    erasor_out.open(erasor_pose_path, std::ios::out);

    std::vector<std::string> pcd_name;
    for(auto& entry_ : fs::directory_iterator(pcd_path)){
        // std::cout << entry_.path() << std::endl;
        pcd_name.emplace_back(entry_.path());
    }
    // std::cout << 0 << std::endl;
    std::sort(pcd_name.begin(), pcd_name.end(), fileSort);
    // std::cout << 1 << std::endl;

    std::fstream pose_file;
    std::string line;
    pose_file.open(pose_path, std::ios::in);
    while(getline(pose_file, line)){
        std::vector<std::string> line_s;
        boost::split(line_s, line, boost::is_any_of(" "));
        Eigen::Matrix3f rot;
        Eigen::Vector3f trans;
        rot(0, 0) = atof(line_s[0].c_str());
        rot(0, 1) = atof(line_s[1].c_str());
        rot(0, 2) = atof(line_s[2].c_str());
        trans(0) = atof(line_s[3].c_str());
        rot(1, 0) = atof(line_s[4].c_str());
        rot(1, 1) = atof(line_s[5].c_str());
        rot(1, 2) = atof(line_s[6].c_str());
        trans(1) = atof(line_s[7].c_str());
        rot(2, 0) = atof(line_s[8].c_str());
        rot(2, 1) = atof(line_s[9].c_str());
        rot(2, 2) = atof(line_s[10].c_str());
        trans(2) = atof(line_s[11].c_str());
        // std::cout << rot << " " << trans << std::endl;
        rot_name.emplace_back(rot);
        trans_name.emplace_back(trans);
    }

    std::cout << pcd_name.size() << " " << rot_name.size() << std::endl;
    if(pcd_name.size() != rot_name.size()){
        std::cerr << "error" << std::endl;
        return -1;
    }

    pcl::PointCloud<Pose>::Ptr transformt(new pcl::PointCloud<Pose>());

    int count = 0;
    for(int i = 0; i < pcd_name.size() - interval; i = i + interval){
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::io::loadPCDFile(pcd_name[i], *cloud1);
        pcl::io::loadPCDFile(pcd_name[i + 1], *cloud2);
        pcl::io::loadPCDFile(pcd_name[i + 2], *cloud3);

        Eigen::Matrix3f rot1 = rot_name[i];
        Eigen::Matrix3f rot2 = rot_name[i + 1];
        Eigen::Matrix3f rot3 = rot_name[i + 2];
        Eigen::Vector3f rpy1 = rotationMatrixToEulerAngles(rot1);
        Eigen::Vector3f rpy2 = rotationMatrixToEulerAngles(rot2);
        Eigen::Vector3f rpy3 = rotationMatrixToEulerAngles(rot3);
        Eigen::Vector3f trans1 = trans_name[i];
        Eigen::Vector3f trans2 = trans_name[i + 1];
        Eigen::Vector3f trans3 = trans_name[i + 2];
        Eigen::Affine3f trans_1 = pcl::getTransformation(trans1(0), trans1(1), trans1(2), rpy1(0), rpy1(1), rpy1(2));
        Eigen::Affine3f trans_2 = pcl::getTransformation(trans2(0), trans2(1), trans2(2), rpy2(0), rpy2(1), rpy2(2));
        Eigen::Affine3f trans_3 = pcl::getTransformation(trans3(0), trans3(1), trans3(2), rpy3(0), rpy3(1), rpy3(2));

        Pose posei;
        posei.x = trans2(0);
        posei.y = trans2(1);
        posei.z = trans2(2);
        posei.roll = rpy2(0);
        posei.pitch = rpy2(1);
        posei.yaw = rpy2(2);
        transformt->points.push_back(posei);


        Eigen::Affine3f  trans_21 = trans_2.inverse() * trans_1;
        Eigen::Affine3f  trans_23 = trans_2.inverse() * trans_3;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud21(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud23(new pcl::PointCloud<pcl::PointXYZI>());

        transformCloud(cloud1, trans_21, cloud21);
        transformCloud(cloud3, trans_23, cloud23);
        *cloud2 += *cloud21;
        *cloud2 += *cloud23;

        // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud22(new pcl::PointCloud<pcl::PointXYZI>());
        // cloud22->height = 1;
        // cloud22->width = cloud2->points.size();
        // transformCloud(cloud2, trans_2, cloud22);

        // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud21(new pcl::PointCloud<pcl::PointXYZI>());
        // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud23(new pcl::PointCloud<pcl::PointXYZI>());
        // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud22(new pcl::PointCloud<pcl::PointXYZI>());
        // transformCloud(cloud1, trans_1, cloud21);
        // transformCloud(cloud3, trans_3, cloud23);
        // transformCloud(cloud2, trans_2, cloud22);
        // *cloud22 += *cloud21;
        // *cloud22 += *cloud23;


        std::string name = new_pcd_path + std::to_string(count) + ".pcd";
        pcl::io::savePCDFile(name, *cloud2);
        if(count >= start && count <= end){
            std::string erasor = erasor_pcd_path + std::to_string(count - start) + ".pcd";
            pcl::io::savePCDFile(erasor, *cloud2);
            std::cout << "save: " << count << std::endl;
            Eigen::Quaternionf q2(rot2);
            Eigen::Vector4f vec = q2.coeffs();
            erasor_out << count - start <<", " << count << ", " << trans_name[i + 1](0) << ", " << trans_name[i + 1](1) << ", " << trans_name[i + 1](2) << ", "
                                    << vec(0) << ", " << vec(1) << ", " << vec(2) << ", " << vec(4) << "\n";
        }

        // std::string name = new_pcd_path + std::to_string(count) + ".bin";
        // count ++;
        // std::ofstream myFile(name.c_str(), std::ios::out | std::ios::binary | std::ios::app);
        // std::cout << cloud2->points.size() << std::endl;
        // for(int k = 0; k < cloud2->points.size(); k++){
        //     myFile.write((char*)& cloud2->points[k].x, sizeof(float));
        //     myFile.write((char*)& cloud2->points[k].y, sizeof(float));
        //     myFile.write((char*)& cloud2->points[k].z, sizeof(float));
        //     myFile.write((char *)& cloud2->points[k].intensity, sizeof(float));
        // }
        // myFile.close();

        
    
        pose_out << rot_name[i + 1](0, 0) << " " << rot_name[i + 1](0, 1) << " " << rot_name[i + 1](0, 2) << " " <<   trans_name[i + 1](0) << " "
                             << rot_name[i + 1](1, 0) << " " << rot_name[i + 1](1, 1) << " " << rot_name[i + 1](1, 2) << " " <<   trans_name[i + 1](1) << " "
                             << rot_name[i + 1](2, 0) << " " << rot_name[i + 1](2, 1) << " " << rot_name[i + 1](2, 2) << " " <<   trans_name[i + 1](2) << "\n";

        std::cout <<"pcd: " << i + 1 << std::endl;
        count ++;
    }
    transformt->height = 1;
    transformt->width = count;
    pcl::io::savePCDFile("/media/fyx/Yixin F/seu_mid360/urban/urban_transform.pcd", *transformt);
    pose_out.close();

    return 0;
}


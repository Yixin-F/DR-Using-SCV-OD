#ifndef SSC_H_
#define SSC_H_

#include "utility.h"
#include "patchwork.h"

class SSC: public Utility{
public:
    static int id;

    int range_num = -1;  // upate in each ssc 
    int sector_num = -1;
    int azimuth_num = -1;
    int bin_num = -1;

    float min_dis = -1;  // refer to the previous frame
    float max_dis = -1;

    Frame frame_ssc;

    pcl::PointCloud<pcl::PointXYZI>::Ptr src_cloud;  // original cloud
    boost::shared_ptr<PatchWork<pcl::PointXYZI>> PatchworkGroundSeg;   // patchwork
    pcl::PointCloud<pcl::PointXYZI>::Ptr ng_cloud;  // noground cloud
    pcl::PointCloud<pointCalib>::Ptr ng_cloud_calib;

    ~SSC();
    SSC();

    // allocate memory and reset
    void allocateMemory();
    void reset();

    // extract ground by pathwork
    void extractGroudByPatchWork(const pcl::PointCloud<pcl::PointXYZI>::Ptr& src_cloud_);

    // calibrate point by intensity and curvature
    void intensityAndCurvatureCalibration();

    // generate hashcloud
    std::pair<std::pair<float, float>, std::vector<PointAPRIC>> makeAPRIC(const pcl::PointCloud<pointCalib>::Ptr& cloudCalib_);  // return min_dis&max_dis of original pointcloud and apric_vec without range constraint
    std::pair<std::vector<PointAPRIC>, std::unordered_map<int, Voxel>> makeHashCloud(const std::vector<PointAPRIC>& apriIn_, const float& minDis_, const float& maxDis_);  // return apric_vec with range constraint and hashcloud without label


    // tool
    void downSample(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_down_, float leaf_size_);
    void intensityVisualization(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_);
    pcl::PointCloud<pcl::PointXYZI>::Ptr getVoxelFromHashCloud(const std::unordered_map<int, Voxel>& hashCloud_);
    float occupancyUseVoxel(const std::vector<int>& voxels1_, const std::vector<int>& voxels2_);




};


#endif
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

    float min_dis = 999999.f;  // refer to the previous frame
    float max_dis = -999999.f;
    double sensor_height = -999999.f;

    Frame frame_ssc;

    boost::shared_ptr<PatchWork<pcl::PointXYZI>> PatchworkGroundSeg;   // patchwork
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_use;  // noground cloud

    ~SSC();
    SSC();

    // allocate memory and reset
    void allocateMemory();
    void reset();

    // pre-process pointcloud
    void process(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn_);
    void getCloudInfo(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn_);
    pcl::PointCloud<pcl::PointXYZI>::Ptr extractGroudByPatchWork(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn_);  // extract ground by pathwork
    void intensityCalibrationByCurvature(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn_);  // calibrate point by intensity by curvature
    void downSampleAndDistanceSelect(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_down_, float leaf_size_);
    void intensityVisualization(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_);

    // generate hashcloud
    std::pair<std::pair<float, float>, std::vector<PointAPRIC>> makeAPRIC(const pcl::PointCloud<pointCalib>::Ptr& cloudCalib_);  // return min_dis&max_dis of original pointcloud and apric_vec without range constraint
    std::pair<std::vector<PointAPRIC>, std::unordered_map<int, Voxel>> makeHashCloud(const std::vector<PointAPRIC>& apriIn_, const float& minDis_, const float& maxDis_);  // return apric_vec with range constraint and hashcloud without label


    // tool
    pcl::PointCloud<pcl::PointXYZI>::Ptr getVoxelFromHashCloud(const std::unordered_map<int, Voxel>& hashCloud_);
    float occupancyUseVoxel(const std::vector<int>& voxels1_, const std::vector<int>& voxels2_);




};


#endif
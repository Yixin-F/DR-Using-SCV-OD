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

    std::vector<PointAPRI> apri_vec;
    std::unordered_map<int, Voxel> hash_cloud;
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
    void downSampleAndMakeApriVec(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_down_, float leaf_size_);
    void intensityVisualization(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_);
    void makeHashCloud(const std::vector<PointAPRI>& apriIn_);  
    pcl::PointCloud<pcl::PointXYZI>::Ptr getVoxelCloudFromHashCloud(const std::unordered_map<int, Voxel>& hashCloud_);

    // segment
    void cluster()

    // tool
    
    float occupancyUseVoxel(const std::vector<int>& voxels1_, const std::vector<int>& voxels2_);




};


#endif
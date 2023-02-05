#ifndef SSC_H_
#define SSC_H_

#include "utility.h"
#include "patchwork.h"

class SSC: public Utility{
public:
    static int id;

    int range_num; 
    int sector_num;
    int azimuth_num;
    int bin_num;

    std::string calib_save;
    std::string seg_save;
    std::string map_save;

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_vec;
    std::vector<Pose> pose_vec;

    std::vector<PointAPRI> apri_vec;
    std::unordered_map<int, Voxel> hash_cloud;
    Frame frame_ssc;

    boost::shared_ptr<PatchWork<pcl::PointXYZI>> PatchworkGroundSeg;   // patchwork
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_use;  // noground cloud with intenisty calibrated

    std::unordered_map<int, Cluster> cluster_track;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster_map;

    ~SSC();
    SSC();

    // allocate memory and reset
    void allocateMemory();
    void reset();

    // pre-process pointcloud
    void process(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn_);
    pcl::PointCloud<pcl::PointXYZI>::Ptr extractGroudByPatchWork(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn_);  // extract ground by pathwork
    void intensityCalibrationByCurvature(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloudIn_);  // calibrate point by intensity by curvature
    void makeApriVec(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_);
    void intensityVisualization(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_);
    void makeHashCloud(const std::vector<PointAPRI>& apriIn_);  
    
    // segment
    void segment();
    void clusterAndCreateFrame(const std::vector<PointAPRI>& apri_vec_, std::unordered_map<int, Voxel>& hash_cloud_);
    std::vector<int> findVoxelNeighbors(const int& range_idx_, const int& sector_idx_, const int& azimuth_idx_, int size_);
    void mergeClusters(std::vector<int>& clusterIdxs_, const int& idx1_, const int& idx2_);
    pcl::PointCloud<pcl::PointXYZI>::Ptr getCloudByIdx(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_from_, const std::vector<int>& idx_vec_);
    pcl::PointXYZI getCenterOfCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_);
    std::pair<pcl::PointXYZI, pcl::PointXYZI> getBoundingBoxOfCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_);
    bool refineClusterByBoundingBox(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_cluster_);
    void refineClusterByIntensity(Frame& frame_ssc);
    void getVoxelCloudFromHashCloud(std::unordered_map<int, Voxel>& hashCloud_);
    void saveSegCloud(Frame& frame_ssc, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_);

    // feature recognize
    void recognize();
    Feature getDescriptorByEigenValue(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster_cloud_);
    Feature getDescriptorByEnsembleShape(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster_cloud_);
    Eigen::MatrixXd getFeature21(const Eigen::MatrixXd& eigenvalue_matrix_, const Eigen::MatrixXd& ensembleshape_matrix_);
    bool regionGrowing(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster_cloud_);
    
    // dynamic detect
    void removert();
    Frame intialization(const std::vector<Frame>& frames_, const std::vector<Pose>& poses_);
    void tracking(Frame& frame_pre_, Frame& frame_next_, Pose pose_pre_, Pose pose_next_);
    float compareFeature(const Eigen::MatrixXd& feature1_, const Eigen::MatrixXd& feature2_);

    // tool
    void getPose();
    void getCloud();

};


#endif
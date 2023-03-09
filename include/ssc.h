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
    std::string pcd_save;
    std::string map_save;
    std::string evaluate_save;

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_vec;
    std::vector<Pose> pose_vec;
    std::vector<Eigen::Matrix4f> trans_vec;

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> g_cloud_vec;

    std::vector<PointAPRI> apri_vec;
    std::unordered_map<int, Voxel> hash_cloud;
    Frame frame_ssc;

    boost::shared_ptr<PatchWork<pcl::PointXYZI>> PatchworkGroundSeg;   // patchwork
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_use;  // noground cloud with intenisty calibrated

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_original;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_dynamic;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_static;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr instance_map;

    

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> eva_ori;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_eva_ori;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_eva_static;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_eva_dynamic;

    
    Frame frame_based;  // mapping based frame
    int name = 0;  // record cluster tracked
    std::vector<Frame> frame_set;

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
    pcl::PointXYZI getCenterOfCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_);
    std::pair<pcl::PointXYZI, pcl::PointXYZI> getBoundingBoxOfCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_);
    void refineClusterByBoundingBox(Frame& frame_ssc_);
    void refineClusterByIntensity(Frame& frame_ssc);
    void getVoxelCloudFromHashCloud(std::unordered_map<int, Voxel>& hashCloud_);
    void saveSegCloud(Frame& frame_ssc, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud_, const std::string& path_, int mode);

    // feature recognize
    void recognize(Frame& frame_ssc_);
    Feature getDescriptorByEigenValue(const Cluster& cluster_);
    Feature getDescriptorByEnsembleShape(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster_cloud_);
    Eigen::MatrixXd getFeature21(const Eigen::MatrixXd& eigenvalue_matrix_, const Eigen::MatrixXd& ensembleshape_matrix_);
    bool regionGrowing(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cluster_cloud_);
    
    // dynamic detect
    Frame intialization(const std::vector<Frame>& frames_, const std::vector<Pose>& poses_);
    void tracking(Frame& frame_pre_, Frame& frame_next_, Pose pose_pre_, Pose pose_next_);
    float compareFeature(const Eigen::MatrixXd& feature1_, const Eigen::MatrixXd& feature2_);

    // main
    void getPose();
    void getCloud();
    void segDF();

    // tool
    void getObjectToDetection(const Frame& frame_);
    void getDynamic(const Frame& frame_);
    void getStatic(const Frame& frame_);
    void getTFAndPF(const pcl::PointCloud<pcl::PointXYZI>::Ptr& original_, const pcl::PointCloud<pcl::PointXYZI>::Ptr& result_);
    void recordIntensity(std::unordered_map<int, Voxel>& hash_);
};


#endif
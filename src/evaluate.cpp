# include "utility.h"

// void evaluate(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& original_, const pcl::PointCloud<pcl::PointXYZ>::Ptr& static_, const pcl::PointCloud<pcl::PointXYZ>::Ptr& dynamic_, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& mapShow_){
//     pcl::KdTreeFLANN<pcl::PointXYZ> kd_s;
//     kd_s.setInputCloud(static_);
//     pcl::KdTreeFLANN<pcl::PointXYZ> kd_d;
//     kd_d.setInputCloud(dynamic_);
//     mapShow_->height = 1;
//     int count = 0;
//     for(size_t i = 0; i < original_->points.size(); i++){
//         pcl::PointXYZRGB ori = original_->points[i];
//         std::cout << i << " ";
//         // float dis = std::sqrt(ori.x * ori.x + ori.y * ori.y);
//         // if(dis < 1.8 || dis > 50.0){
//         //     continue;
//         // }
//         if(ori.g != 0 || ori.b != 0){  // predicted static
//             pcl::PointXYZ xyz;
//             xyz.x = ori.x;
//             xyz.y = ori.y;
//             xyz.z = ori.z;

//             std::vector<int> id;
//             std::vector<float> dis;
//             kd_s.radiusSearch(xyz, 0.1, id, dis);
//             if(id.size() != 0){  // TP, green
//                 ori.r = 0.f;
//                 ori.g = 255.f;
//                 ori.b = 127.f;
//                 mapShow_->points.push_back(ori);
//                 count ++;
//             } 
//             else{ 
//                 std::vector<int> id2;
//                 std::vector<float> dis2;
//                 kd_d.radiusSearch(xyz, 0.1, id2, dis2);
//                 if(id2.size() != 0){  // FN, orange
//                     ori.r = 255.f;
//                     ori.g = 165.f;
//                     ori.b = 0.f;
//                     mapShow_->points.push_back(ori);
//                     count ++;
//                 }
//             }
//         }
//         else{  // predicted dynamic
//             pcl::PointXYZ xyz;
//             xyz.x = ori.x;
//             xyz.y = ori.y;
//             xyz.z = ori.z;

//             std::vector<int> id;
//             std::vector<float> dis;
//             kd_d.radiusSearch(xyz, 0.1, id, dis);
//             if(id.size() != 0){  // TN, blue
//                 ori.r = 0.f;
//                 ori.g = 255.f;
//                 ori.b = 255.f;
//                 mapShow_->points.push_back(ori);
//                 count ++;
//             } 
//             else{
//                 std::vector<int> id2;
//                 std::vector<float> dis2;
//                 kd_s.radiusSearch(xyz, 0.1, id2, dis2);
//                 if(id2.size() != 0){  // FN, pink
//                     ori.r = 255.f;
//                     ori.g = 192.f;
//                     ori.b = 203.f;
//                     mapShow_->points.push_back(ori);
//                     count ++;
//                 }
//             }
//         }
//     }
//     mapShow_->width = count;
// }

void evaluate(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& original_, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& static_, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& dynamic_, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& mapShow_){
    mapShow_->height = 1;
    int count = 0;
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_s;
    kd_s.setInputCloud(static_);
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kd_d;
    kd_d.setInputCloud(dynamic_);
    for(size_t i = 0; i < original_->points.size(); i++){
        pcl::PointXYZRGB ori = original_->points[i];
        // std::cout << i << " ";
        // float dis = std::sqrt(ori.x * ori.x + ori.y * ori.y);
        // std::cout << dis << " ";
        // if(dis < 1.8 || dis > 50.0){
        //     continue;
        // }
        if(ori.g != 0){  // predicted static
            std::vector<int> id;
            std::vector<float> dis;
            kd_s.radiusSearch(ori, 0.15, id, dis);
            // std::cout << id.size() << " ";
            if(id.size() != 0){  // TP, green
                ori.r = 0.f;
                ori.g = 255.f;
                ori.b = 127.f;
                mapShow_->points.push_back(ori);
                count ++;
            } 
            else{ 
                std::vector<int> id2;
                std::vector<float> dis2;
                kd_d.radiusSearch(ori, 0.1, id2, dis2);
                if(id2.size() != 0){  // FN, orange
                    ori.r = 255.f;
                    ori.g = 165.f;
                    ori.b = 0.f;
                    mapShow_->points.push_back(ori);
                    count ++;
                }
            }
        }
        else{  // predicted dynamic
            std::vector<int> id;
            std::vector<float> dis;
            kd_d.radiusSearch(ori, 0.15, id, dis);
            if(id.size() != 0){  // TN, blue
                ori.r = 0.f;
                ori.g = 255.f;
                ori.b = 255.f;
                mapShow_->points.push_back(ori);
                count ++;
            } 
            else{
                std::vector<int> id2;
                std::vector<float> dis2;
                kd_s.radiusSearch(ori, 0.1, id2, dis2);
                if(id2.size() != 0){  // FN, pink
                    ori.r = 255.f;
                    ori.g = 192.f;
                    ori.b = 203.f;
                    mapShow_->points.push_back(ori);
                    count ++;
                }
            }
        }
    }
    mapShow_->width = count;
}

int main(){
    // const std::string path = "/home/fyx/ufo_hiahia/src/evaluate/";
    // const std::string original_path = path + "original.bin";
    // const std::string static_path = path + "static.bin";
    // const std::string dynamic_path = path + "dynamic.bin";
    const std::string path = "/home/fyx/ufo_hiahia/src/evaluate/";
    const std::string original_path = path + "original.pcd";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr original(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::io::loadPCDFile(original_path, *original);
    std::cout << "original: " << original->points.size() << std::endl;
    const std::string static_path = path + "static.pcd";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr static_c(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::io::loadPCDFile(static_path, *static_c);
    std::cout << "static: " << static_c->points.size() << std::endl;
    const std::string dynamic_path = path + "dynamic.pcd";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr dynamic(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::io::loadPCDFile(dynamic_path, *dynamic);
    std::cout << "dynamic: " << dynamic->points.size() << std::endl;
    

    // // original map
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr original(new pcl::PointCloud<pcl::PointXYZRGB>());
    // std::ifstream ori_cloud(original_path, std::ios::binary);
    // ori_cloud.seekg(0, std::ios::end);
    // // ori_cloud.seekg(0, std::ios::beg);
    // uint32_t num_ori = ori_cloud.tellg() / sizeof(float) / 6;
    // // ori_cloud.seekg(0, std::ios::beg);
    // std::vector<float> values_ori(6 * num_ori);
    // ori_cloud.read((char *)&values_ori[0], 6 * num_ori * sizeof(float));
    // for(uint32_t k = 0; k < num_ori; k++){
    //     pcl::PointXYZRGB rgb;
    //     rgb.x = values_ori[k * 6];
    //     rgb.y = values_ori[k * 6 + 1];
    //     rgb.z = values_ori[k * 6 + 2];
    //     rgb.r = values_ori[k * 6 + 3];
    //     rgb.g = values_ori[k * 6 + 4];
    //     rgb.b = values_ori[k * 6 + 5];
    //     original->points.push_back(rgb);
    // }
    // std::cout << "original: " << num_ori << std::endl;
 
    // // static map
    // pcl::PointCloud<pcl::PointXYZ>::Ptr static_c(new pcl::PointCloud<pcl::PointXYZ>());
    // std::ifstream s_cloud(static_path, std::ios::binary);
    // s_cloud.seekg(0, std::ios::end);
    // // s_cloud.seekg(0, std::ios::beg);
    // uint32_t num_s = s_cloud.tellg() / sizeof(float) / 6;
    // // s_cloud.seekg(0, std::ios::end);
    // // s_cloud.seekg(0, std::ios::beg);
    // std::vector<float> values_s(6 * num_s);
    // s_cloud.read((char *)&values_s[0], 6 * num_s * sizeof(float));
    // for(uint32_t k = 0; k < num_s; k++){
    //     pcl::PointXYZ xyz;
    //     xyz.x = values_s[k * 6];
    //     xyz.y = values_s[k * 6 + 1];
    //     xyz.z = values_s[k * 6 + 2];
    //     static_c->points.push_back(xyz);
    // }
    // std::cout << "static: " << num_s << std::endl;

    // // dynamic map
    // pcl::PointCloud<pcl::PointXYZ>::Ptr dynamic_c(new pcl::PointCloud<pcl::PointXYZ>());
    // std::ifstream d_cloud(dynamic_path, std::ios::binary);
    // d_cloud.seekg(0, std::ios::end);
    // // d_cloud.seekg(0, std::ios::beg);
    // uint32_t num_d = d_cloud.tellg() / sizeof(float) / 6;
    // // d_cloud.seekg(0, std::ios::beg);
    // std::vector<float> values_d(6 * num_d);
    // d_cloud.read((char *)&values_d[0], 6 * num_d * sizeof(float));
    // for(uint32_t k = 0; k < num_d; k++){
    //     pcl::PointXYZ xyz;
    //     xyz.x = values_d[k * 6];
    //     xyz.y = values_d[k * 6 + 1];
    //     xyz.z = values_d[k * 6 + 2];
    //     dynamic_c->points.push_back(xyz);
    // }
    // std::cout << "dynamic: " << num_d << std::endl;

    // evaluate map
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr e_map(new pcl::PointCloud<pcl::PointXYZRGB>());
    std::cout << "wait" << std::endl;
    evaluate(original, static_c, dynamic, e_map);
    std::cout << "evaluate: " << e_map->points.size() << std::endl;
    std::string save = "/home/fyx/ufo_hiahia/src/evaluate/evaluate.pcd";
    pcl::io::savePCDFile(save, *e_map);
    std::cout << "save evaluate" << std::endl;

}
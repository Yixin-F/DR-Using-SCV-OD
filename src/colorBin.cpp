#include "utility.h"

int main(int argc, char** argv){

    std::cout << "bin: " << argv[1] << " pcd: " << argv[2] << std::endl;

    std::ifstream input(argv[1], std::ios::binary);
    if (!input.is_open()) {
        std::cerr << "Failed to open input file: " << argv[1] << std::endl;
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointXYZI point;
    for (int i = 0; input.good() && !input.eof(); i++) {
		input.read((char*)&point.x, 3 * sizeof(float));
		input.read((char*)&point.intensity, sizeof(float));
        pcl::PointXYZRGB point_rgb;
        point_rgb.r = std::atof(argv[3]);
        point_rgb.g = std::atof(argv[4]);
        point_rgb.b = std::atof(argv[5]);
        point_rgb.x = point.x;
        point_rgb.y = point.y;
        point_rgb.z = point.z;
		cloud->push_back(point_rgb);
	}
    input.close();

    // while (input_file.read((char *)&point.x, 4 * sizeof(float))) {
    //     pcl::PointXYZRGB point_rgb;
    //     point_rgb.r = std::atof(argv[3]);
    //     point_rgb.g = std::atof(argv[4]);
    //     point_rgb.b = std::atof(argv[5]);
    //     point_rgb.x = point.x;
    //     point_rgb.y = point.y;
    //     point_rgb.z = point.z;

    //     cloud->push_back(point_rgb);
    // }
    // input_file.close();
    std::cout << cloud->points.size() << std::endl;

    pcl::io::savePCDFile(argv[2], *cloud);
    return 0;
}
#include <opencv2/core/core.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <iterator>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/features/don.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace std;

int user_data;

float get_float(string message)
{
   float check = std::atof(message.c_str());
   return check;
}

std::vector<std::vector<std::string> >
ReadDataFromCSV(std::string filename)
{
    std::ifstream file(filename.c_str());
    std::vector<std::vector<std::string> > dataList;

    std::string line = "";
    while (getline(file, line))
    {
        std::vector<std::string> vec;
        boost::algorithm::split(vec, line, boost::is_any_of(","));
        dataList.push_back(vec);
    }
    return dataList;
}

void
pcdWriter(std::vector<std::vector<std::string> > rawData, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud) {

    cloud->width = (uint32_t) rawData.size();

    std::cout << cloud->width << std::endl;
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        cloud->points[i].x = get_float(rawData[i][0]);
        cloud->points[i].y = get_float(rawData[i][1]);
        cloud->points[i].z = get_float(rawData[i][2]);
        cloud->points[i].intensity = get_float(rawData[i][3]);
    }

    pcl::io::savePCDFileASCII("../final_project_point_cloud.pcd", *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to final_project_point_cloud.pcd file." << std::endl;

}

int
main()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr segCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    std::cout << "Reading data from csv file..." << std::endl;
    std::vector<std::vector<std::string> > raw_data = ReadDataFromCSV("../final_project_point_cloud.csv");
    std::cout << "Writing data into pcd file..." << std::endl;
    pcdWriter(raw_data, cloud);
    raw_data.clear();


    return 0;
}

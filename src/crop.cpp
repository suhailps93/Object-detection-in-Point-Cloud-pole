#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
double xmin, ymin, zmin, xmax,ymax,zmax,threshold,radius;

typedef pcl::PointXYZ PointT;

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCDReader reader;//读
    pcl::ExtractIndices<PointT> extract;
    pcl::visualization::CloudViewer viewer("cloud viewer");//显示
    reader.read<pcl::PointXYZ>("../pointclouds/final_project_point_cloud.pcd", *cloud);//
            char c;
    while (true)
    {
        // cloud_filtered=cloud;

        cout << "Enter s:Save c:Crop t:threshold y:cylinder \n";
        c = getchar();
        if( c == 's' )
        {
          cout << "Saving frame ";
          pcl::io::savePCDFileASCII( "output.pcd", *cloud_filtered );
        }


        else if(c=='c')
        {

          cout << "Enter xmin xmax ymin ymax zmin zmax \n";
          cin >> xmin>> xmax>> ymin >>ymax>> zmin>> zmax;
          cout << " xmin= "<<xmin<< " xmax= "<<xmax<< " ymin= "<<ymin <<" ymax= "<<ymax<<" zmin= " << zmin<<" zmax= "<< zmax<< ".\n";
          pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("x");
            pass.setFilterLimits(xmin, xmax);
            pass.filter(*cloud_filtered);

            pass.setInputCloud(cloud_filtered);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(zmin, zmax);
            pass.filter(*cloud_filtered);


            pass.setInputCloud(cloud_filtered);
            pass.setFilterFieldName("y");
            pass.setFilterLimits(ymin, ymax);
            pass.filter(*cloud_filtered);

            viewer.showCloud(cloud_filtered);
        }


        else if(c=='t')
        {
          pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);
          cout << "Enter threshold \n";
          cin >> threshold;
          pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
          pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
          // Create the segmentation object
          pcl::SACSegmentation<pcl::PointXYZ> seg;
          // Optional
          seg.setOptimizeCoefficients (true);
          // Mandatory
          seg.setModelType (pcl::SACMODEL_PLANE);
          seg.setMethodType (pcl::SAC_RANSAC);
          seg.setDistanceThreshold (threshold);

          seg.setInputCloud (cloud);
          seg.segment (*inliers, *coefficients);
          // *cloud_filtered=*coefficients;

          extract.setInputCloud (cloud);
          extract.setIndices (inliers);
          extract.setNegative (true);

          pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
          extract.filter (*cloud_filtered);
          viewer.showCloud(cloud_filtered);
          cloud=cloud_filtered;

        }

        else if(c=='y')
        {
          pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);
          cout << "Enter threshold \n";
          cin >> threshold;

          cout << "Enter radius \n";
          cin >> radius;
          pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
          pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
          // Create the segmentation object
          pcl::SACSegmentation<pcl::PointXYZ> seg;
          // Optional
          seg.setOptimizeCoefficients (true);
          // Mandatory
          seg.setModelType (pcl::SACMODEL_CYLINDER);
          seg.setMethodType (pcl::SAC_RANSAC);
          seg.setDistanceThreshold (threshold);
          seg.setRadiusLimits (0, radius);

          seg.setInputCloud (cloud);
          seg.segment (*inliers, *coefficients);
          // *cloud_filtered=*coefficients;

          extract.setInputCloud (cloud);
          extract.setIndices (inliers);
          extract.setNegative (false);

          pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
          extract.filter (*cloud_filtered);
          viewer.showCloud(cloud_filtered);
          *cloud=*cloud_filtered;

        }



    }

}

#include "playtec/normal.h"

#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/search/pcl_search.h>

#include <iostream>
#include <vector>

namespace ltr_normal
{
void GetNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
               pcl::PointCloud<pcl::Normal>::Ptr normal, float vp[], float radius)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

    ne.setInputCloud(input);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(radius);
    ne.setViewPoint(vp[0], vp[1], vp[2]);
    ne.compute(*normal);
}

void GetPointNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                    pcl::PointCloud<pcl::PointNormal>::Ptr pnormal, float vp[], float radius)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

    ne.setInputCloud(input);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(radius);
    ne.setViewPoint(vp[0], vp[1], vp[2]);
    ne.compute(*normal);
    pcl::concatenateFields(*input, *normal, *pnormal);
}

void GetSurfaceNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                      pcl::PointCloud<pcl::Normal>::Ptr cloud_normal, float radius)
{
    // Define cloud pointers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_lower(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr lower_normal(new pcl::PointCloud<pcl::Normal>);

    // Get lower cloud points
    int cloud_size = cloud->points.size();
    std::stack<pcl::PointXYZ> cloud_stk;
    std::stack<int> index_stk;
    for (int i = 0; i < cloud_size; ++i)
    {
        if (cloud->points[i].z < -1.0)
        {
            cloud_stk.push(cloud->points[i]);
            index_stk.push(i);
        }
    }

    // Fill lower cloud
    cloud_lower->width = cloud_stk.size();
    cloud_lower->height = 1;
    cloud_lower->points.resize(cloud_lower->width * cloud_lower->height);

    int count = 0;
    while (!cloud_stk.empty())
    {
        cloud_lower->points[count] = cloud_stk.top();
        ++count;
        cloud_stk.pop();
    }

    // Get cloud & lower_cloud normals (PointCloud).
    float vp_upper[3] = {0.0f, 0.0f, -35.0f};
    GetNormal(cloud, cloud_normal, vp_upper, radius);

    float vp_lower[3] = {0.0f, 0.0f, 0.0f};
    GetNormal(cloud_lower, lower_normal, vp_lower, radius);

    // Refill lower_cloud normals in cloud normals.
    count = 0;
    while (!index_stk.empty())
    {
        cloud_normal->points[index_stk.top()] = lower_normal->points[count];
        index_stk.pop();
        ++count;
    }
}

};  // namespace ltr_normal
#include "playtec/surface.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/pcl_base.h>
#include <pcl/search/pcl_search.h>

namespace ltr_sfc
{
// GLOBAL VARIABLES
float minD = 0.1;
int kPoints = 1;
std::vector<int> pointID(kPoints);
std::vector<float> pointNKN(kPoints);
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
std::stack<pcl::PointXYZ> cloud_stack;

float dist3D(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) +
                (p1.y - p2.y) * (p1.y - p2.y) +
                (p1.z - p2.z) * (p1.z - p2.z));
}

void InterpolatePoints(pcl::PointXYZ p, pcl::PointXYZ q)
{
    int num_points = ceil(dist3D(p, q) / minD);
    pcl::PointXYZ inner_point;
    for (int i = 1; i < num_points; ++i)
    {
        inner_point.getArray3fMap() = p.getArray3fMap() +
                                      (q.getArray3fMap() - p.getArray3fMap()) * (float(i) / float(num_points));
        cloud_stack.push(inner_point);
    }
}
void kdtreeSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointXYZ searchPoint)
{
    if (tree->nearestKSearch(searchPoint, kPoints, pointID, pointNKN) > 0)
    {
        InterpolatePoints(searchPoint, cloud->points[pointID[0]]);
    }
}

void GetLatSurface(pcl::PointCloud<pcl::PointXYZ>::Ptr border_upper,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr border_lower,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
    // Initialize Output border setup
    int upper_points = border_upper->points.size();
    int lower_points = border_lower->points.size();

    // GENERATE SURFACE
    // Points from Upper - PointCloud from Lower
    tree->setInputCloud(border_lower);
    for (const pcl::PointXYZ& p : border_upper->points)
    {
        kdtreeSampling(border_lower, p);
    }
    tree->setInputCloud(border_upper);
    // Points from Lower - PointCloud from Upper
    for (const pcl::PointXYZ& p : border_lower->points)
    {
        kdtreeSampling(border_upper, p);
    }

    // Initialize Output surface setup
    int num_points = cloud_stack.size();
    output->width = num_points;
    output->height = 1;
    output->points.resize(output->width * output->height);
    int count = 0;
    while (!cloud_stack.empty())
    {
        output->points[count] = cloud_stack.top();
        cloud_stack.pop();
        ++count;
    }
}
}  // namespace ltr_sfc
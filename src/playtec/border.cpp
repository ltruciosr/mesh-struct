#include "playtec/border.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/pcl_search.h>

namespace ltr_border
{
// GLOBAL VARIABLES
pcl::PointXYZ p0;
float maxD;
float minD;
int kPoints = 1;
std::vector<int> pointID(kPoints);
std::vector<float> pointNKN(kPoints);

std::stack<pcl::PointXYZ> cloud_stk;
std::stack<pcl::PointXYZ> sample_stk;

pcl::PointCloud<pcl::PointXYZ>::Ptr input(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);

pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

pcl::PointXYZ NextToTop(std::stack<pcl::PointXYZ> &S)
{
    pcl::PointXYZ p = S.top();
    S.pop();
    pcl::PointXYZ res = S.top();
    S.push(p);
    return res;
}

float dist2D(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
    return (p1.x - p2.x) * (p1.x - p2.x) +
           (p1.y - p2.y) * (p1.y - p2.y);
}

float dist3D(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) +
                (p1.y - p2.y) * (p1.y - p2.y) +
                (p1.z - p2.z) * (p1.z - p2.z));
}

void InterpolatePoints(pcl::PointXYZ p, pcl::PointXYZ q)
{
    int num_points = ceil(dist3D(p, q) / minD);
    pcl::PointXYZ sample_point;
    for (int i = 1; i < num_points; ++i)
    {
        sample_point.getArray3fMap() = p.getArray3fMap() +
                                       (q.getArray3fMap() - p.getArray3fMap()) * (float(i) / float(num_points));
        sample_stk.push(sample_point);
    }
}

void kdtreeSampling(pcl::PointXYZ searchPoint)
{
    if (tree->nearestKSearch(searchPoint, kPoints, pointID, pointNKN) > 0)
    {
        // Check the distance to the nearest point
        if (dist3D(sample_stk.top(), input->points[pointID[0]]) > minD)
        {
            InterpolatePoints(sample_stk.top(), input->points[pointID[0]]);
        }
        sample_stk.push(input->points[pointID[0]]);
    }
}

void FindNearestPoints(pcl::PointXYZ p, pcl::PointXYZ q)
{
    int num_points = ceil(dist3D(p, q) / minD);
    pcl::PointXYZ search_point;
    //We add the first value for compare them.
    sample_stk.push(p);
    for (int i = 1; i < num_points; ++i)
    {
        search_point.getArray3fMap() = p.getArray3fMap() +
                                       (q.getArray3fMap() - p.getArray3fMap()) * (float(i) / float(num_points));
        kdtreeSampling(search_point);
    }
    if (dist3D(sample_stk.top(), q) > minD)
    {
        InterpolatePoints(sample_stk.top(), q);
    }
}

int orientation(pcl::PointXYZ p, pcl::PointXYZ q, pcl::PointXYZ r)
{
    float val = (q.y - p.y) * (r.x - q.x) -
                (q.x - p.x) * (r.y - q.y);

    // if (abs(val) < 0.001) return 0;    // colinear
    if (val == 0) return 0;    // colinear
    return (val > 0) ? 1 : 2;  // clock or counterclock wise
}

int compare(const void *vp1, const void *vp2)
{
    pcl::PointXYZ *p1 = (pcl::PointXYZ *)vp1;
    pcl::PointXYZ *p2 = (pcl::PointXYZ *)vp2;
    int o = orientation(p0, *p1, *p2);
    if (o == 0)
        return (dist2D(p0, *p2) >= dist2D(p0, *p1)) ? -1 : 1;

    return (o == 2) ? -1 : 1;
}

void process()
{
    int n = input->points.size();
    // Find the botton-most point
    float ymin = input->points[0].y, min = 0;
    for (int i = 1; i < n; i++)
    {
        float y = input->points[i].y;
        // Pick the bottom-most left point
        if ((y < ymin) || (ymin == y &&
                           input->points[i].x < input->points[min].x))
            ymin = input->points[i].y, min = i;
    }
    // Place the bottom-most point at first position
    std::swap(input->points[0], input->points[min]);

    // Sort the points about its clock-wise angle from p0
    p0 = input->points[0];
    std::qsort(&input->points[1], n - 1, sizeof(pcl::PointXYZ), compare);

    // Remove all but not the one that is farthest from p0
    int m = 1;  // Initialize size of modified array
    for (int i = 1; i < n; i++)
    {
        //when the angle of ith and (i+1)th elements are same, remove points
        while (i < n - 1 && orientation(p0, input->points[i],
                                        input->points[i + 1]) == 0)
            i++;
        input->points[m] = input->points[i];
        m++;  // Update size of modified array
    }

    // If modified array of points has less than 3 points,
    // convex hull is not possible
    if (m < 3) return;

    // Create an empty stack and push first three points to it.
    cloud_stk.push(input->points[0]);
    cloud_stk.push(input->points[1]);
    cloud_stk.push(input->points[2]);

    // Process remaining n-3 points
    for (int i = 3; i < m; i++)
    {
        // Keep removing top while the angle formed by
        // points next-to-top, top, and points[i] makes
        // a non-left turn
        while (orientation(NextToTop(cloud_stk), cloud_stk.top(), input->points[i]) != 2)
            cloud_stk.pop();
        cloud_stk.push(input->points[i]);
    }
    cloud_stk.push(input->points[0]);

    // Now stack has the output points, store content of the stack
    output->width = cloud_stk.size();
    output->height = 1;
    output->points.resize(output->width * output->height);

    //kdtree setup
    tree->setInputCloud(input);

    //Checking
    int count = 0;
    while (cloud_stk.size() > 1)
    {
        output->points[count] = cloud_stk.top();
        ++count;

        if (dist3D(NextToTop(cloud_stk), cloud_stk.top()) > maxD)
        {
            FindNearestPoints(NextToTop(cloud_stk), cloud_stk.top());
        }
        else if (dist3D(NextToTop(cloud_stk), cloud_stk.top()) > minD)
        {
            InterpolatePoints(NextToTop(cloud_stk), cloud_stk.top());
        }
        cloud_stk.pop();
    }
    output->points[count] = cloud_stk.top();
    cloud_stk.pop();

    //  Make Resample with Convex Border
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_resample(new pcl::PointCloud<pcl::PointXYZ>);
    output_resample->width = sample_stk.size();
    output_resample->height = 1;
    output_resample->points.resize(output_resample->width * output_resample->height);
    count = 0;
    while (!sample_stk.empty())
    {
        output_resample->points[count] = sample_stk.top();
        ++count;
        sample_stk.pop();
    }
    *output += *output_resample;
}

void GetBorderFromCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr border,
                        float min_D, float max_D)
{
    // Store input cloud
    input = cloud;
    output = border;
    minD = min_D;
    maxD = max_D;
    process();
}
}  // namespace ltr_border

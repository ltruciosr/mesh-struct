#include "playtec/volume.h"

#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace ltr_mesh
{
pcl::Normal GetNormalCentroid(pcl::Normal n1, pcl::Normal n2, pcl::Normal n3)
{
    pcl::Normal n;
    n.normal_x = (n1.normal_x + n2.normal_x + n3.normal_x) / 3;
    n.normal_y = (n1.normal_y + n2.normal_y + n3.normal_y) / 3;
    n.normal_z = (n1.normal_z + n2.normal_z + n3.normal_z) / 3;
    return n;
}

float determinant(pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3)
{
    return p1.x * p2.y * p3.z -
           p1.x * p2.z * p3.y -
           p1.y * p2.x * p3.z +
           p1.y * p2.z * p3.x +
           p1.z * p2.x * p3.y -
           p1.z * p2.y * p3.x;
}

float GetVolume(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                pcl::PointCloud<pcl::Normal>::Ptr normal,
                std::vector<pcl::Vertices> &vert)
{
    int num_faces = vert.size();
    int num_points = cloud->points.size();

    // Get triangle centroid parameters
    pcl::PointXYZ p1, p2, p3, pc;
    pcl::Normal n1, n2, n3, nc;

    float angle;
    float volume = 0;
    for (int i = 0; i < num_faces; ++i)
    {
        p1 = cloud->points[vert[i].vertices[0]];
        p2 = cloud->points[vert[i].vertices[1]];
        p3 = cloud->points[vert[i].vertices[2]];
        pc.getArray3fMap() = (p1.getArray3fMap() + p2.getArray3fMap() + p3.getArray3fMap()) / 3;

        n1 = normal->points[vert[i].vertices[0]];
        n2 = normal->points[vert[i].vertices[1]];
        n3 = normal->points[vert[i].vertices[2]];
        nc = GetNormalCentroid(n1, n2, n3);
        angle = pc.x * nc.normal_x + pc.y * nc.normal_y + pc.z * nc.normal_z;
        if (angle > 0)
            volume -= abs(determinant(p1, p2, p3));
        else
            volume += abs(determinant(p1, p2, p3));
    }
    volume /= 6;  // VOLUME = (TOTAL_VOLUME)/6;
    return volume;
}
};  // namespace ltr_mesh
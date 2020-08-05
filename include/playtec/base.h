#pragma once
#include <pcl/PolygonMesh.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "base.h"

namespace ltr_base
{
void GetBaseFromBorder(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr output,
                       float minZ = -2.0, float minD = 0.1);

void GetCloudFromMesh(pcl::PolygonMesh mesh,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr output);

}  // namespace ltr_base
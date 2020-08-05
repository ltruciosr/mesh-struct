#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "surface.h"

namespace ltr_sfc
{
void GetLatSurface(pcl::PointCloud<pcl::PointXYZ>::Ptr border_upper,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr border_lower,
                   pcl::PointCloud<pcl::PointXYZ>::Ptr output);
}
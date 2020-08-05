#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "border.h"

namespace ltr_border
{
void GetBorderFromCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr border,
                        float min_D = 0.2, float max_D = 2.0);
}
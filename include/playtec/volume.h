#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "volume.h"

namespace ltr_mesh
{
float GetVolume(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                pcl::PointCloud<pcl::Normal>::Ptr normal,
                std::vector<pcl::Vertices> &vert);
};  // namespace ltr_mesh
#pragma once
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "normal.h"

namespace ltr_normal
{
void GetNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
               pcl::PointCloud<pcl::Normal>::Ptr normal, float vp[], float radius = 1.0);

void GetPointNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                    pcl::PointCloud<pcl::PointNormal>::Ptr pnormal, float vp[], float radius = 1.0);

// Input is a surface pointcloud
void GetSurfaceNormal(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                      pcl::PointCloud<pcl::Normal>::Ptr cloud_normal, float radius);
};  // namespace ltr_normal
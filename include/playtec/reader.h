#pragma once

#include <pcl/Vertices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tinyply.h>

#include <string>

#include "reader.h"

namespace ltr_reader
{
int readPLYFile(const std::string& filepath,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                 pcl::PointCloud<pcl::Normal>::Ptr normal,
                 std::vector<pcl::Vertices>& pcl_vert,
                 const bool preload_into_memory = true);

int readCSVFile(const std::string& filepath,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr input);
}  // namespace ltr_reader
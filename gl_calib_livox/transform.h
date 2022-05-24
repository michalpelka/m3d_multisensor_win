#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <sophus/se3.hpp>
namespace calib_struct{
    pcl::PointCloud<pcl::PointXYZINormal> createTransformedPc(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& raw, Sophus::Vector6f& calib, float offset);

    std::vector<Sophus::Vector6f> initializeCalib(const std::string& fn);
    std::vector<Sophus::Vector6f> initializeCalib();

}
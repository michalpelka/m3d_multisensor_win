//
// Created by michal on 24.05.2022.
//
#include "transform.h"

pcl::PointCloud<pcl::PointXYZINormal> calib_struct::createTransformedPc(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& raw, Sophus::Vector6f& calib, float offset)
{
    pcl::PointCloud<pcl::PointXYZINormal> pc;
    const Eigen::Matrix4f calib_mat{ Sophus::SE3f::exp(calib).matrix() };
    pc.resize(raw->size());
    for (int i = 0; i < raw->size(); i++)
    {
        const auto raw_point = (*raw)[i];
        const float angle = raw_point.normal_x;
        const float s = sin(angle+offset);
        const float c = cos(angle+offset);
        Eigen::Matrix4f rot_angle;
        rot_angle << c, -s, 0, 0, s, c, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
        const Eigen::Vector4f p{ raw_point.getArray4fMap() };
        pc[i].getArray4fMap() = rot_angle.transpose() * calib_mat * p;
        pc[i].intensity = raw_point.intensity;
        pc[i].normal_x = angle;
    }
    return pc;
}


std::vector<Sophus::Vector6f> calib_struct::initializeCalib() {
    Eigen::Matrix4f calibration1 = Eigen::Matrix4f::Identity();
    calibration1.block<3, 3>(0, 0) = Eigen::AngleAxisf(-M_PI / 2.0, Eigen::Vector3f::UnitX()).toRotationMatrix();

    Eigen::Matrix4f calibration2 = Eigen::Matrix4f::Identity();
    calibration2.block<3, 3>(0, 0) = (Eigen::AngleAxisf((-14.5 * M_PI / 180.0), Eigen::Vector3f::UnitY()) *
                                      Eigen::AngleAxisf(M_PI / 2.0, Eigen::Vector3f::UnitX())).toRotationMatrix();

    Eigen::Matrix4f calibration3 = Eigen::Matrix4f::Identity();
    calibration3.block<3, 3>(0, 0) = Eigen::AngleAxisf((-0.0 * M_PI / 180.0), Eigen::Vector3f::UnitZ()).toRotationMatrix();

    // velodyne
    Eigen::Matrix4f calibration4 = Eigen::Matrix4f::Identity();
    calibration4 << 0,  0, 1, 0,
            0,  1, 0, 0,
            -1, 0, 0, 0,
            0, 0, 0, 1;

    return { Sophus::SE3f::fitToSE3(calibration2).log(),Sophus::SE3f::fitToSE3(calibration1).log(),Sophus::SE3f::fitToSE3(calibration4).log(),Sophus::SE3f::fitToSE3(calibration3).log()};
}

std::vector<Sophus::Vector6f> calib_struct::initializeCalib(const std::string& fn) {
    std::ifstream txt(fn);
    std::string line;
    std::vector<Sophus::Vector6f> r;
    while (std::getline(txt, line))
    {
        std::stringstream oss(line);
        Sophus::Vector6f v;
        for (int i = 0; i < 6; i++)
        {
            oss >> v[i];
        }
        r.emplace_back(v);
    };
    txt.close();
    return r;
}
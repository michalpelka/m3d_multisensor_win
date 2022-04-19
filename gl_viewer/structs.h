#pragma once
#include "GL/glwrapper.h"
#include <memory>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
namespace calib_struct{
	
    struct KeyFrame {

        KeyFrame(std::shared_ptr<float[]> data_p, int len, Eigen::Matrix4d mat, int laser_id);

        Eigen::Matrix4d mat;
        double timestamp;
        const std::shared_ptr<float[]> data;
        const int len;
        const int laser_id;
        VertexBuffer vb;
        VertexArray va;
    };

    void saveConfig(std::vector<Sophus::Vector6f>& extrinsic_calib_vec, const std::string& fn);

    std::shared_ptr<float[]> pclToBuffer(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, int& len, float scale);

    std::vector<Sophus::Vector6f> initializeCalib();

    std::vector<Sophus::Vector6f> initializeCalib(const std::string& fn);
    
    pcl::PointCloud<pcl::PointXYZI> createTransformedPc(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& raw, Sophus::Vector6f& calib);


    struct plane {
        Eigen::Matrix4f matrix{ Eigen::Matrix4f::Identity() };
        Eigen::Vector2f size{1,1};

        plane() {
            const Eigen::AngleAxis<float> aa(0.5f * M_PI, Eigen::Vector3f(1.0f, 0.0f, 0.0f));
            matrix.block<3, 3>(0, 0) = aa.matrix();
        }



        Eigen::Matrix4f getGizmo() const {
            return matrix;// Eigen::Matrix4f::Identity();// / *rotation;
        }

        void setGizmo(Eigen::Matrix4f& m)
        {
            matrix = m;
        }
    };

    void savePlanes(std::vector<plane>& planes, const std::string& fn);
    std::vector<plane> loadPlanes(const std::string& fn);

    Eigen::Matrix3d findCovariance(std::vector<Eigen::Vector4d> points, const Eigen::Vector4d avg);

    Eigen::Matrix3f findCovariance(std::vector<Eigen::Vector4f> points, const Eigen::Vector4f avg);

    template<typename T>
    T avgT(const std::vector<T>& data)
    {
        T ret = T::Zero();
        for (int i = 0; i < data.size(); i++)
        {
            ret += data[i];
        }
        return ret / data.size();
    }


}
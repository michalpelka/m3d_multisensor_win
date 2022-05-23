//
// Created by michal on 28.03.2021.
//
#pragma once

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <sophus/se3.hpp>
#include <ceres/ceres.h>

#include <Eigen/Dense>

#include <vector>
#include <iostream>


#include "nanoflann/nanoflann.hpp"
#include "nanoflann/KDTreeVectorOfVectorsAdaptor.h"


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "glm/glm.hpp"


namespace m3d_utils {

    class ladybug_camera_calibration{
    public:
        void loadConfig(const std::string &fn);
        void loadCfgFromString(const std::string &data);
        const std::vector<Eigen::Matrix4f> &getCameraExtrinsic() const;
        const std::vector<float> &getCameraFocal() const;
        const std::vector<Eigen::Vector2f> &getCameraCenter() const;

    private:
        static Eigen::Matrix4d makeTransformation( const double rotX, const double rotY, const double rotZ, const double transX, const double transY, const double transZ);
        std::vector<Eigen::Matrix4f> camera_extrinsic;
        std::vector<float> camera_focal;
        std::vector<Eigen::Vector2f> camera_center;


    };
    Eigen::Matrix4d loadMat(const std::string& fn);
    void saveMat(const std::string& fn, const Eigen::Matrix4d& mat);

    Sophus::Vector6d loadVec6(const std::string& fn);
    void saveVec6(const std::string& fn, const Sophus::Vector6d& mat);

    std::pair<double,double> findVariance(const std::vector<double> &data);

    struct TestPlane{
        glm::mat4 matrix {glm::mat4(1.0f)};
        TestPlane() = default;
        TestPlane (std::string T);
        boost::property_tree::ptree serialize() const;
    };


        Eigen::Affine3d orthogonize(const Eigen::Affine3d& p );
    Eigen::Matrix4d orthogonize(const Eigen::Matrix4d& p );

    std::vector<Eigen::Vector2d> detectMarkerInImage(const cv::Mat &input_image, const cv::Mat& marker);

//    Eigen::Vector2d projectPoint(const Eigen::Vector4d& p, const sensor_msgs::CameraInfo & ci);

    Eigen::Matrix3d findCovariance ( std::vector<Eigen::Vector4d> points ,const Eigen::Vector4d avg);
    Eigen::Matrix3d findCovariance ( std::vector<Eigen::Vector3d> points ,const Eigen::Vector3d avg);

    using pointcloud_t=std::vector<Eigen::Vector3d> ;
    using pointcloud_ptr=std::shared_ptr<pointcloud_t>;
    using my_kd_tree_t = KDTreeVectorOfVectorsAdaptor<pointcloud_t, double>;

    template<typename T>
    void saveVector( const std::string& fn, const T& vec)
    {
        boost::property_tree::ptree pt;
        for (int i =0; i < vec.size(); i++)
        {
            pt.put("vec."+std::to_string(i), vec[i]);
        }
        boost::property_tree::write_ini( fn, pt );
    }
    template<typename T>
    void loadVector( const std::string& fn, T& vec)
    {
        boost::property_tree::ptree pt;
        try {
            boost::property_tree::read_ini(fn, pt);
            for (int i =0; i < vec.size(); i++)
            {
                vec[i] = pt.get<double>("vec."+std::to_string(i));
            }
        }
        catch (const boost::property_tree::ptree_error &e) {
            std::cout << " error during loading INI :" << e.what() << std::endl;
        }
    }



    template<typename T>
    T avgT(const std::vector<T>& data)
    {
        T ret = T::Zero();
        for (int i = 0 ; i< data.size(); i++)
        {
            ret += data[i];
        }
        return ret / data.size();
    }




    template<typename T>
    Eigen::Matrix<T, 4, 4> getMatrixFromParams(const double *const params) {
        Eigen::Matrix<T, 6, 1> eigen_laser_params;
        eigen_laser_params << params[0], params[1], params[2], params[3], params[4], params[5];
        Sophus::SE3<T> TT = Sophus::SE3<T>::exp(eigen_laser_params);
        return TT.matrix();
    }


    void saveCFG(const std::vector<double> &laser1_config, const std::vector<double> &camera1_config);

    void tryLoadCFG(std::vector<double> &laser1_config, std::vector<double> &camera1_config);




}

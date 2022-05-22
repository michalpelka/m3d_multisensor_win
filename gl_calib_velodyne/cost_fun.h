//
// Created by michal on 21.10.2021.
//

#ifndef CALIB_TOOL_BIG_HEAD_COST_FUN_H
#define CALIB_TOOL_BIG_HEAD_COST_FUN_H
#include <sophus/se3.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <ceres/ceres.h>

class LocalParameterizationSE3 : public ceres::LocalParameterization {
// adopted from https://github.com/strasdat/Sophus/blob/master/test/ceres/local_parameterization_se3.hpp
public:
    virtual ~LocalParameterizationSE3() {}

    // SE3 plus operation for Ceres
    //
    //  T * exp(x)
    //
    virtual bool Plus(double const* T_raw, double const* delta_raw,
                      double* T_plus_delta_raw) const {
        Eigen::Map<Sophus::SE3d const> const T(T_raw);
        Eigen::Map<Sophus::Vector6d const> const delta(delta_raw);
        Eigen::Map<Sophus::SE3d> T_plus_delta(T_plus_delta_raw);
        T_plus_delta = T * Sophus::SE3d::exp(delta);
        return true;
    }

    // Jacobian of SE3 plus operation for Ceres
    //
    // Dx T * exp(x)  with  x=0
    //
    virtual bool ComputeJacobian(double const* T_raw,
                                 double* jacobian_raw) const {
        Eigen::Map<Sophus::SE3d const> T(T_raw);
        Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> jacobian(
                jacobian_raw);
        jacobian = T.Dx_this_mul_exp_x_at_0();
        return true;
    }

    virtual int GlobalSize() const { return Sophus::SE3d::num_parameters; }

    virtual int LocalSize() const { return Sophus::SE3d::DoF; }
};
namespace bigUnitCalib {
    template<typename T>
    Eigen::Matrix<T, 4, 4> getSE3OfLaser(const Eigen::Matrix<T, 4, 4> &local_calib, T angle) {
        const T sa = ceres::sin(angle);
        const T ca = ceres::cos(angle);
        Eigen::Matrix<T, 4, 4> angle_rot;
        angle_rot << ca, -sa, 0, 0,
                sa, ca, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1;
        // calib is calib.rotate(Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f::UnitY()));
        Eigen::Matrix<T, 4, 4> calib;
        calib << T(0),  T(0), T(1), T(0),
                 T(0),  T(1), T(0), T(0),
                 T(-1), T(0), T(0), T(0),
                 T(0),  T(0), T(0), T(1);
        return angle_rot * local_calib * calib;

    }

    struct OptimizePlaneNormalOnCameraImage{
        Eigen::Vector2f point ;
        const Eigen::Vector2f projectionCentrer;
        const double cameraFocalLen;
        Eigen::Matrix3d K;
        OptimizePlaneNormalOnCameraImage(Eigen::Vector2f point,
                                         const Eigen::Vector2f& projectionCentrer,
                                         double cameraFocLen) :
                point(point),projectionCentrer(projectionCentrer),cameraFocalLen(cameraFocLen)
                {
                    K = Eigen::Matrix3d::Zero();
                    K(0,0) = cameraFocalLen;
                    K(1,1) = cameraFocalLen;
                    K(2,0) = -projectionCentrer.x()*cameraFocalLen;
                    K(2,1) = -projectionCentrer.y()*cameraFocalLen;
                    K(2,2) = cameraFocalLen*cameraFocalLen;
                }

        template <typename T>
        bool operator()(const T* const plane_normal_raw,T* residuals) const {


            Eigen::Map<const Eigen::Matrix<T,3,1>>  plane_normal(plane_normal_raw);
            auto plane_normal_norm = plane_normal/plane_normal.norm();
            Eigen::Matrix<T,3,1> lc = K.cast<T>() * plane_normal_norm;
            //lc = lc / (ceres::sqrt(lc[0]*lc[0]+lc[1]*lc[1]));
            lc = lc / lc.norm();
            residuals[0]= lc[0]* T(point.y()) + lc[1] *T(point.x())+lc[2];
            return true;
        }

        static ceres::CostFunction* Create(Eigen::Vector2f point,
                                           const Eigen::Vector2f& projectionCentrer,
                                           double cameraFocLen) {
            return (new ceres::AutoDiffCostFunction<OptimizePlaneNormalOnCameraImage, 1, 3>(
                    new OptimizePlaneNormalOnCameraImage(point,projectionCentrer,cameraFocLen)));
//            return (new ceres::NumericDiffCostFunction<OptimizeLaserLocalPoseWithPoints, ceres::CENTRAL, 3, Sophus::SE3d ::num_parameters>(
//                    new OptimizeLaserLocalPoseWithPoints(point_left,point_right,angle_left,angle_right)));
        }
    };

    struct OptimizeLaserLocalPoseWithPoints{
        const Eigen::Vector4d point_left;
        const Eigen::Vector4d point_right;
        const double angle_left;
        const double angle_right;

        OptimizeLaserLocalPoseWithPoints(const Eigen::Vector3d& point_left_, const Eigen::Vector3d& point_right_, double angle_left,
                                    double angle_right) :
                                    point_left({point_left_.x(),point_left_.y(),point_left_.z(),1.0}),
                                    point_right({point_right_.x(),point_right_.y(),point_right_.z(),1.0}),
                                    angle_left(angle_left),
                                    angle_right(angle_right){}

        template <typename T>
        bool operator()(const T* const local_laser_se3,T* residuals) const {


            Eigen::Map<Sophus::SE3<T> const>  local_laser_SE3(local_laser_se3);

            const Eigen::Matrix<T,4,4> left_transformer = bigUnitCalib::getSE3OfLaser<T>(local_laser_SE3.matrix(), T(-angle_left));
            const Eigen::Matrix<T,4,4> right_transformer = bigUnitCalib::getSE3OfLaser<T>(local_laser_SE3.matrix(), T(-angle_right));

            const auto pt_left = left_transformer * point_left.cast<T>();
            const auto pt_right = right_transformer * point_right.cast<T>();


            residuals[0] = (pt_left.x()-pt_right.x());
            residuals[1] = (pt_left.y()-pt_right.y());
            residuals[2] = (pt_left.z()-pt_right.z());
            return true;
        }

        static ceres::CostFunction* Create(const Eigen::Vector3d& point_left, const Eigen::Vector3d& point_right, double angle_left,
                                           double angle_right) {
            return (new ceres::AutoDiffCostFunction<OptimizeLaserLocalPoseWithPoints, 3, Sophus::SE3d ::num_parameters>(
                    new OptimizeLaserLocalPoseWithPoints(point_left,point_right,angle_left,angle_right)));
//            return (new ceres::NumericDiffCostFunction<OptimizeLaserLocalPoseWithPoints, ceres::CENTRAL, 3, Sophus::SE3d ::num_parameters>(
//                    new OptimizeLaserLocalPoseWithPoints(point_left,point_right,angle_left,angle_right)));
        }
    };
    struct OptimizeLaserLocalPoseWithFix{
        const Eigen::Vector4d point_left;
        const Eigen::Vector4d point_fix;

        const double angle_left;

        OptimizeLaserLocalPoseWithFix(const Eigen::Vector3d& point_left_, const Eigen::Vector3d& point_fix_, double angle_left) :
                point_left({point_left_.x(),point_left_.y(),point_left_.z(),1.0}),
                point_fix({point_fix_.x(),point_fix_.y(),point_fix_.z(),1.0}),
                angle_left(angle_left){}

        template <typename T>
        bool operator()(const T* const local_laser_se3,T* residuals) const {


            Eigen::Map<Sophus::SE3<T> const>  local_laser_SE3(local_laser_se3);

            const Eigen::Matrix<T,4,4> left_transformer = bigUnitCalib::getSE3OfLaser<T>(local_laser_SE3.matrix(), T(-angle_left));

            const auto pt_left = left_transformer * point_left.cast<T>();


            residuals[0] = (pt_left.x()-T(point_fix.x()));
            residuals[1] = (pt_left.y()-T(point_fix.y()));
            residuals[2] = (pt_left.z()-T(point_fix.z()));
            return true;
        }

        static ceres::CostFunction* Create(const Eigen::Vector3d& point_left, const Eigen::Vector3d& point_fix, double angle_left) {
               return (new ceres::AutoDiffCostFunction<OptimizeLaserLocalPoseWithFix, 3, Sophus::SE3d ::num_parameters>(
                    new OptimizeLaserLocalPoseWithFix(point_left,point_fix,angle_left)));
//            return (new ceres::NumericDiffCostFunction<OptimizeLaserLocalPoseWithFix, ceres::CENTRAL, 3, Sophus::SE3d ::num_parameters>(
//                    new OptimizeLaserLocalPoseWithFix(point_left,point_fix,angle_left)));
        }
    };

    struct OptimizeLaser3DBlobs{
        pcl::PointCloud<pcl::PointXYZINormal> centroid_left_raw;
        pcl::PointCloud<pcl::PointXYZINormal> centroid_right_raw;

        OptimizeLaser3DBlobs(const pcl::PointCloud<pcl::PointXYZINormal>& centroid_left_raw_,
                                      const pcl::PointCloud<pcl::PointXYZINormal>& centroid_right_raw_) :
                centroid_left_raw(centroid_left_raw_),centroid_right_raw(centroid_right_raw_) {}

        template <typename T>
        bool operator()(const T* const local_laser_se3,T* residuals) const {


            Eigen::Map<Sophus::SE3<T> const>  local_laser_SE3(local_laser_se3);

            Eigen::Matrix<T, 1, 4> avg_left=Eigen::Matrix<T, 1, 4>::Zero();
            Eigen::Matrix<T, 1, 4> avg_right=Eigen::Matrix<T, 1, 4>::Zero();
            for (int i=0; i < centroid_left_raw.size(); i++)
            {
                Eigen::Matrix<T,1,4>t {T(centroid_left_raw[i].x),T(centroid_left_raw[i].y),T(centroid_left_raw[i].z),T(1.0)};
                T angle = T(-centroid_left_raw[i].normal_x);
                const Eigen::Matrix<T,4,4> transform = bigUnitCalib::getSE3OfLaser<T>(local_laser_SE3.matrix(), angle);
                const Eigen::Matrix<T, 1, 4> Pt1 = transform * t.transpose();
                avg_left += Pt1;
            }

            for (int i=0; i < centroid_right_raw.size(); i++)
            {
                Eigen::Matrix<T,1,4>t {T(centroid_right_raw[i].x),T(centroid_right_raw[i].y),T(centroid_right_raw[i].z),T(1.0)};
                T angle = T(-centroid_right_raw[i].normal_x);
                const Eigen::Matrix<T,4,4> transform = bigUnitCalib::getSE3OfLaser<T>(local_laser_SE3.matrix(), angle);
                const Eigen::Matrix<T, 1, 4> Pt1 = transform * t.transpose();
                avg_right += Pt1;
            }

            avg_left = avg_left / T(centroid_left_raw.size());
            avg_right = avg_right / T(centroid_right_raw.size());

            residuals[0] = (avg_right.x()-avg_left.x());
            residuals[1] = (avg_right.y()-avg_left.y());
            residuals[2] = (avg_right.z()-avg_left.z());
            return true;
        }

        static ceres::CostFunction* Create(const pcl::PointCloud<pcl::PointXYZINormal>& centroid_left_raw_,
                                           const pcl::PointCloud<pcl::PointXYZINormal>& centroid_right_raw_) {
            return (new ceres::AutoDiffCostFunction<OptimizeLaser3DBlobs, 3, Sophus::SE3d ::num_parameters>(
                    new OptimizeLaser3DBlobs(centroid_left_raw_,centroid_right_raw_)));
//            return (new ceres::NumericDiffCostFunction<OptimizeLaser3DBlobs, ceres::CENTRAL, 3, Sophus::SE3d ::num_parameters>(
//                    new OptimizeLaser3DBlobs(centroid_left_raw_,centroid_right_raw_)));
        }
    };

}
#endif //CALIB_TOOL_BIG_HEAD_COST_FUN_H

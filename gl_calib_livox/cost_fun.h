//
// Created by michal on 20.04.2022.
//

#ifndef M3D_MULTISENSOR_COST_FUN_H
#define M3D_MULTISENSOR_COST_FUN_H
#include <ceres/ceres.h>
#include <Sophus/sophus/se3.hpp>

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


class LocalParameterizationPlane : public ceres::LocalParameterization {
public:
    virtual ~LocalParameterizationPlane() {}

    bool Plus(const double* x,
              const double* delta,
              double* x_plus_delta) const {
        x_plus_delta[0] = x[0] + delta[0];
        x_plus_delta[1] = x[1] + delta[1];
        x_plus_delta[2] = x[2] + delta[2];
        x_plus_delta[3] = x[3] + delta[3];
        Eigen::Map<Eigen::Matrix<double, 3, 1>> x_plus_deltap (x_plus_delta);
        x_plus_deltap = x_plus_deltap / x_plus_deltap.norm();
        return true;
    }
    virtual bool ComputeJacobian(double const* T_raw,
                                 double* jacobian_raw) const {
        ceres::MatrixRef(jacobian_raw, 4, 4) = ceres::Matrix::Identity(4, 4);
        return true;
    }

    virtual int GlobalSize() const { return 4; }

    virtual int LocalSize() const { return 4; }
};

struct LockTranslation{

    LockTranslation(){}

    template <typename T>
    bool operator()(const T* const laser_se3,
                    T* residuals) const {
        Eigen::Map<Sophus::SE3<T> const> laser_SE3(laser_se3);
        residuals[0] = laser_SE3.translation().x();
        residuals[1] = laser_SE3.translation().y();
        residuals[2] = laser_SE3.translation().z();
        return true;
    }

    static ceres::CostFunction* Create() {
        return (new ceres::AutoDiffCostFunction<LockTranslation,  3, Sophus::SE3d::num_parameters>(
                new LockTranslation()));

    }
};

struct LaserSE3AgainstPlane{
    const Eigen::Vector4d raw_point;
    const double angle_rot;
    LaserSE3AgainstPlane(const Eigen::Vector4d& raw_point, double angle_rot ):
            raw_point(raw_point), angle_rot(angle_rot){
    }

    template <typename T>
    bool operator()(const T* const plane_abcd, const T* const laser_se3,
                    T* residuals) const {

        Eigen::Map<Sophus::SE3<T> const> laser_SE3(laser_se3);
        Eigen::Map<Eigen::Matrix<T,4,1> const> plane(plane_abcd);
        const Eigen::Matrix<T,4,1> pointT = raw_point.cast<T>();
        Eigen::Matrix<T,4,4> angle_rot_mat;
        const T sn = ceres::sin(T(angle_rot));
        const T cs = ceres::cos(T(angle_rot));
        angle_rot_mat << cs, -sn, T(0), T(0), sn, cs, T(0), T(0), T(0), T(0), T(1), T(0), T(0), T(0), T(0), T(1);
        const Eigen::Matrix<T,4,1> point_global =  angle_rot_mat.transpose() * laser_SE3.matrix() * raw_point;

        residuals[0] = plane.dot(point_global);

        return true;
    }

    static ceres::CostFunction* Create(const Eigen::Vector4d& raw_point, double angle_rot ) {
        return (new ceres::AutoDiffCostFunction<LaserSE3AgainstPlane,  1, 4, Sophus::SE3d::num_parameters>(
                new LaserSE3AgainstPlane(raw_point, angle_rot)));
//        return (new ceres::NumericDiffCostFunction<MirrorOprimizeABCDWithPose, ceres::CENTRAL, 3, 4, 4, 6, 6>(
//                new MirrorOprimizeABCDWithPose(point_1,point_target)));

//        return (new ceres::NumericDiffCostFunction<PlaneAligmentError, ceres::CENTRAL, 4, 6, 6>(
//                new PlaneAligmentError(plane_1,plane_2,pose_1,pose_2)));
    }
};


#endif //M3D_MULTISENSOR_COST_FUN_H

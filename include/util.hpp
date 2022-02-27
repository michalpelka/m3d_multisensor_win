#ifndef LIVOX_SYNCH_H
#define LIVOX_SYNCH_H
#include <Eigen/Dense>

namespace catopric_livox{

    template <typename T> Eigen::Matrix<T,4,1>getPlaneCoefFromSE3(const Eigen::Matrix<T,4,4>& SE3){
        const T a = -SE3(0,2);
        const T b = -SE3(1,2);
        const T c = -SE3(2,2);
        const T d = -SE3(0,3) * a - SE3(1,3) * b - SE3(2,3) * c;
        return Eigen::Matrix<T,4,1> {a,b,c,d};
    }

    template <typename T> Eigen::Matrix<T,3,1>getMirroredRay(const Eigen::Matrix<T,3,1>& dir, T ray_length, const Eigen::Matrix<T,4,1>& plane)
    {

        Eigen::Matrix<T,3,1> np {plane.x(), plane.y(), plane.z()};
        np= np /np.norm();
        Eigen::Matrix<T,3,1> ndir = dir / dir.norm();

        const T a = np.x() * ndir.x() + np.y() * ndir.y() + np.z() * ndir.z();
        const Eigen::Matrix<T,3,1> intersection = - ndir *(plane.w()/a);
        const Eigen::Matrix<T,3,1> rd= ndir - T(2.0)*(ndir.dot(np))*np;
        const T ll = ray_length - intersection.norm();
        return -intersection + rd * ll;
    }
}

#endif
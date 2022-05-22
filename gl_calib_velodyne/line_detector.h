//
// Created by michal on 03.07.2021.
//

#ifndef SRC_LINE_DETECTOR_H
#define SRC_LINE_DETECTOR_H
#include "utils.h"
#include <cv.h>


#include  <random>
#include  <iterator>
namespace spherical_calib_utils{
    std::vector<std::vector<Eigen::Vector2d>> loadPointsListsFromTXT(const std::string& fname);
    std::pair<Eigen::Vector3d,Eigen::Vector3d> getLineFromPCD(const std::string& fname);
    std::pair<Eigen::Vector3d,Eigen::Vector3d> getLineFromASC(const std::string& fname);

}

namespace line_detector {



    struct Setup{
        int blur{0};
        float canny_th1{50};
        float canny_th2{180};
        int bottom_point_ignore{300};
        double lm_minimum_cost{1e-5};
        double inlier_distance {1.1};
        double feature_minimum_relative_size{0.10};
        int ransac_iterations = 35;
        int ransac_sel_point = 8;
        float collapse_normal_vector_size{0.25};
        bool debug{true};
    };
    struct mergepolyline{
        Eigen::Vector2d initial;
        double cost;
        std::vector<Eigen::Vector2d> found_vars;
        bool merged{false};
        bool operator<(const mergepolyline &rhs) const {
            return cost < rhs.cost;
        }
    };

    template<typename Iter, typename RandomGenerator>
    Iter select_randomly(Iter start, Iter end, RandomGenerator& g) {
        std::uniform_int_distribution<> dis(0, std::distance(start, end) - 1);
        std::advance(start, dis(g));
        return start;
    }

    template<typename Iter>
    Iter select_randomly(Iter start, Iter end) {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        return select_randomly(start, end, gen);
    }

    struct CostFunctor2D{
        const Eigen::Vector2d spherical_coords;

        CostFunctor2D(const Eigen::Vector2d& spherical_coords) :
                spherical_coords(spherical_coords){
        }

        template <typename T>
        bool operator()(const T* const azimuth_inclination,
                        T* residuals) const {
            const T longitude = T(spherical_coords.x());
            const T azimuth = azimuth_inclination[0];
            const T inclination = azimuth_inclination[1];
            T lattidue_rad = ceres::atan(ceres::tan(inclination) * ceres::sin(longitude - azimuth));
            residuals[0] = lattidue_rad - T(spherical_coords.y());
            return true;
        }

        static ceres::CostFunction* Create(const Eigen::Vector2d& spherical_coords) {
            return (new ceres::AutoDiffCostFunction<CostFunctor2D, 1, 2>(
                    new CostFunctor2D(spherical_coords)));
//        return (new ceres::NumericDiffCostFunction<CostFunctor,ceres::CENTRAL, 2, 6>(
//                new CostFunctor(line_in_spherical_image, line_in_scene)));

        }
    };

    cv::Vec2f getShpericalFromLatLon(float alpha2, float omega2, int cols, int rows);
    Eigen::Vector2d getLatLonFromSpherical(int x, int y, int cols, int rows);
    cv::Mat& drawGreatCircle(cv::Mat &sphere_img, double azimuth, double inclination, cv::Vec3b color);
    int getGreatCircleScore(cv::Mat &sphere_img, double azimuth, double inclination);
    float inliersRatio(const std::vector<cv::Point>&countour, int rows, int cols, float max_distance, const mergepolyline &line);
    bool fitPoints(const std::vector<cv::Point>&countour, int rows, int cols, int num_random, double max_cost, mergepolyline&line);
    bool fitPoints(const std::vector<Eigen::Vector2d>&countour, int rows, int cols, int num_random, double max_cost, mergepolyline&line);
    std::vector<Eigen::Vector2d> findGreatCirclesInImage(const cv::Mat& inputRGB, const Setup& setup);
};


#endif //SRC_LINE_DETECTOR_H

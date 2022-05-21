//
// Created by michal on 03.07.2021.
//

#include "line_detector.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

std::vector<std::vector<Eigen::Vector2d>> spherical_calib_utils::loadPointsListsFromTXT(const std::string& fname)
{
    std::vector<std::vector<Eigen::Vector2d>> points_in_line;
    std::ifstream oss (fname);
    std::string   line;
    while(std::getline(oss, line))
    {
        std::vector<Eigen::Vector2d> vxy;
        std::stringstream   linestream(line);
        while (!linestream.eof())
        {
            Eigen::Vector2d  xy;
            linestream >> xy.y() >> xy.x();
            vxy.emplace_back(xy);
        }
        points_in_line.emplace_back(vxy);
    }
    return points_in_line;
}
std::pair<Eigen::Vector3d,Eigen::Vector3d> spherical_calib_utils::getLineFromPCD(const std::string& fname)
{
    pcl::PointCloud<pcl::PointXYZ> tcloud;
    pcl::io::loadPCDFile(fname, tcloud);
    std::vector<Eigen::Vector3d> cloud;
    for (const auto p: tcloud)
    {
        cloud.push_back(p.getArray3fMap().cast<double>());
    }
    const Eigen::Vector3d centroid = m3d_utils::avgT(cloud);
    const Eigen::Matrix3d covariance = m3d_utils::findCovariance(cloud, centroid);

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullV);
    Eigen::Matrix4d plane_rotation = Eigen::Matrix4d::Identity();
    plane_rotation.block<3,3>(0,0) = svd.matrixV();
    plane_rotation.block<3,1>(0,3) = centroid;
    const Eigen::Vector3d l = svd.matrixV().col(0);
    return {centroid +  2.0*l, centroid -  2.0*l};

//    Vector6d plucker1 = getPlucker(lines_3d[i].first,lines_3d[i].second);
//    Vector6d plucker2;
//    plucker2.block<3,1>(0,0) = centroid.cross(l);
//    plucker2.block<3,1>(3,0) = l;

}
std::pair<Eigen::Vector3d,Eigen::Vector3d> spherical_calib_utils::getLineFromASC(const std::string& fname)
{
    pcl::PointCloud<pcl::PointXYZ> tcloud;

    std::ifstream f(fname);
    std::string line;

    while (std::getline(f, line)) {
        std::replace_if(std::begin(line), std::end(line),[](std::string::value_type v) { return v==','; }, ' ');
        std::stringstream ss(line);
        pcl::PointXYZ p;
        ss >> p.x;
        ss >> p.y;
        ss >> p.z;
        tcloud.push_back(p);
    }
    f.close();

    std::vector<Eigen::Vector3d> cloud;
    for (const auto p: tcloud)
    {
        cloud.push_back(p.getArray3fMap().cast<double>());
    }
    const Eigen::Vector3d centroid = m3d_utils::avgT(cloud);
    const Eigen::Matrix3d covariance = m3d_utils::findCovariance(cloud, centroid);

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(covariance, Eigen::ComputeFullV);
    Eigen::Matrix4d plane_rotation = Eigen::Matrix4d::Identity();
    plane_rotation.block<3,3>(0,0) = svd.matrixV();
    plane_rotation.block<3,1>(0,3) = centroid;
    const Eigen::Vector3d l = svd.matrixV().col(0);
    return {centroid +  40.0*l, centroid -  40.0*l};

//    Vector6d plucker1 = getPlucker(lines_3d[i].first,lines_3d[i].second);
//    Vector6d plucker2;
//    plucker2.block<3,1>(0,0) = centroid.cross(l);
//    plucker2.block<3,1>(3,0) = l;

}

cv::Vec2f line_detector::getShpericalFromLatLon(float alpha2, float omega2, int cols, int rows)
{
    double xx2 = 1.0*cols * alpha2 /(2.0*M_PI) + cols/2;
    double yy2 = 1.0*(rows * (-omega2) /(M_PI)) + rows/2;
    return cv::Vec2f(xx2,yy2);
}
Eigen::Vector2d line_detector::getLatLonFromSpherical(int x, int y, int cols, int rows)
{
    double alpha = 2.0*M_PI * (x-cols/2) / cols;
    double omega = -M_PI * (y-rows/2) / rows;
    return Eigen::Vector2d(alpha,omega);
}
cv::Mat& line_detector::drawGreatCircle(cv::Mat &sphere_img, double azimuth, double inclination, cv::Vec3b color)
{
    for (float longitude =-M_PI; longitude < M_PI; longitude+=0.00005*M_PI)
    {
        float lattidue_rad = std::atan(std::tan(inclination) * std::sin(longitude - azimuth));
        cv::Vec2f xy = getShpericalFromLatLon(longitude, lattidue_rad, sphere_img.cols, sphere_img.rows);
        cv::drawMarker(sphere_img, cv::Vec2i(xy), color,cv::MARKER_SQUARE, 5 );
    }
    return sphere_img;
}

int line_detector::getGreatCircleScore(cv::Mat &sphere_img, double azimuth, double inclination)
{
    int i=0;
    for (float longitude =-M_PI; longitude < M_PI; longitude+=0.0001*M_PI)
    {
        float lattidue_rad = std::atan(std::tan(inclination) * std::sin(longitude - azimuth));
        cv::Vec2f xy = getShpericalFromLatLon(longitude, lattidue_rad, sphere_img.cols, sphere_img.rows);
        if(sphere_img.at<uint8_t>(xy[1], xy[0]) >0)
        {
            i+=1;
        }
    }
    return i;
}

float line_detector::inliersRatio(const std::vector<cv::Point>&countour, int rows, int cols, float max_distance, const mergepolyline&line)
{
    int inliers_count =0;
    for (const auto &pixel : countour)
    {
        auto spherical = getLatLonFromSpherical(pixel.x, pixel.y,cols, rows);
        const auto fun = CostFunctor2D(spherical);
        double err= 0;
        fun(line.initial.data(), &err);
        if (err < max_distance) inliers_count ++;
    }
    return 1.0f * inliers_count / countour.size();
}

bool line_detector::fitPoints(const std::vector<Eigen::Vector2d>&countour, int rows, int cols, int num_random, double max_cost, mergepolyline&line)
{
    ceres::Problem problem;
    Eigen::Vector2d azimuth_inclination;
    problem.AddParameterBlock(azimuth_inclination.data(),2);
    if (num_random > 0) {
        for (int j = 0; j < num_random; j++) {
            const auto pixel = *select_randomly(countour.begin(), countour.end());
            auto spherical = getLatLonFromSpherical(pixel.x(), pixel.y(), cols, rows);
            ceres::CostFunction *cost_function = CostFunctor2D::Create(spherical);
            ceres::LossFunction *loss = nullptr;
            problem.AddResidualBlock(cost_function, loss, azimuth_inclination.data());
        }
    }else
    {
        for (int j = 0; j < countour.size(); j++) {
            const auto pixel = countour[j];
            auto spherical = getLatLonFromSpherical(pixel.x(), pixel.y(), cols, rows);
            ceres::CostFunction *cost_function = CostFunctor2D::Create(spherical);
            ceres::LossFunction *loss = nullptr;
            problem.AddResidualBlock(cost_function, loss, azimuth_inclination.data());
        }
    }

    ceres::Solver::Options options;
    options.max_num_iterations = 50;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << "\n";

    line.initial = azimuth_inclination;
    line.cost = summary.final_cost;
    if (summary.termination_type == ceres::CONVERGENCE && summary.final_cost < max_cost) {

        return true;
    }
    return false;
}

bool line_detector::fitPoints(const std::vector<cv::Point>&countour, int rows, int cols, int num_random, double max_cost, mergepolyline&line)
{
    ceres::Problem problem;
    Eigen::Vector2d azimuth_inclination;
    problem.AddParameterBlock(azimuth_inclination.data(),2);
    if (num_random > 0) {
        for (int j = 0; j < num_random; j++) {
            const auto pixel = *select_randomly(countour.begin(), countour.end());
            auto spherical = getLatLonFromSpherical(pixel.x, pixel.y, cols, rows);
            ceres::CostFunction *cost_function = CostFunctor2D::Create(spherical);
            ceres::LossFunction *loss = nullptr;
            problem.AddResidualBlock(cost_function, loss, azimuth_inclination.data());
        }
    }else
    {
        for (int j = 0; j < countour.size(); j++) {
            const auto pixel = countour[j];
            auto spherical = getLatLonFromSpherical(pixel.x, pixel.y, cols, rows);
            ceres::CostFunction *cost_function = CostFunctor2D::Create(spherical);
            ceres::LossFunction *loss = nullptr;
            problem.AddResidualBlock(cost_function, loss, azimuth_inclination.data());
        }
    }

    ceres::Solver::Options options;
    options.max_num_iterations = 10;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    //std::cout << summary.FullReport() << "\n";

    line.initial = azimuth_inclination;
    line.cost = summary.final_cost;
    if (summary.termination_type == ceres::CONVERGENCE && summary.final_cost < max_cost) {

        return true;
    }
    return false;
}

std::vector<Eigen::Vector2d> line_detector::findGreatCirclesInImage(const cv::Mat& inputRGB, const Setup& setup){

    cv::Mat input_gray;
    cv::Mat input_image_small;

    cv::cvtColor(inputRGB,input_gray, cv::COLOR_BGR2GRAY );
    cv::GaussianBlur(input_gray,input_gray, cv::Size(setup.blur,setup.blur),1 );

    //canny edge detection
    cv::Mat input_gray_canny;
    cv::Canny(input_gray,input_gray_canny,setup.canny_th1,setup.canny_th2,3);

    // suppress bottom part of image
    cv::rectangle(input_gray_canny, cv::Rect(cv::Point2f(0,inputRGB.rows-setup.bottom_point_ignore),
                                             cv::Size2f(inputRGB.cols,setup.bottom_point_ignore)),
            cv::Scalar(0),-1 );
    cv::cvtColor(input_gray_canny,input_image_small, cv::COLOR_GRAY2BGR );

    // countour detection
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(input_gray_canny,contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

    // sort countour by length
    std::sort(contours.begin(),contours.end(), [](const auto &p, const auto &r){
        return p.size()>r.size();
    });

    // drop half of shortest contours
    contours.resize(contours.size()/2);

    static std::default_random_engine rng(0);
    std::uniform_int_distribution<int> dist(0, 256);

    cv::Mat drawingTotal = cv::Mat::zeros( input_gray_canny.size(), CV_8UC3 );

    if (setup.debug) {
        for (size_t i = 0; i < contours.size(); i++) {
            cv::Scalar color(1 * dist(rng), 1 * dist(rng), 1 * dist(rng));
            drawContours(drawingTotal, contours, (int) i, color, 2, cv::LINE_8, hierarchy, 0);
        }
        cv::imwrite("/tmp/dbg_drawingTotal.png", drawingTotal);
    }
    std::vector<line_detector::mergepolyline> polylines;

    for( size_t i = 0; i< contours.size(); i++ )
    {
        if (contours[i].size() < 20) continue;
        cv::Mat drawing = cv::Mat::zeros( input_gray_canny.size(), CV_8UC3 );
        cv::Scalar color(255,255,255);
        drawContours( drawing, contours, (int)i, color, 1, cv::LINE_8, hierarchy, 0 );

        // find relative size of countour
        int min_lat = std::numeric_limits<int>::max();
        int max_lat = std::numeric_limits<int>::min();
        int min_lon = std::numeric_limits<int>::max();
        int max_lon = std::numeric_limits<int>::min();
        for (int j =0; j < contours[i].size(); j++) {
            const auto pixel = contours[i][j];
            min_lat = std::min(pixel.x, min_lat);
            max_lat = std::max(pixel.x, max_lat);
            min_lon = std::min(pixel.y, min_lon);
            max_lon = std::max(pixel.y, max_lon);
            }
        if (max_lat-min_lat < input_gray_canny.cols * setup.feature_minimum_relative_size &&
        max_lon-min_lon < input_gray_canny.rows * setup.feature_minimum_relative_size ) continue;

        // RANSAC
        for (int tries =0; tries < setup.ransac_iterations; tries ++) {
        line_detector::mergepolyline polyline;
        if (fitPoints(contours[i], input_gray_canny.rows, input_gray_canny.cols,
                      setup.ransac_sel_point, setup.lm_minimum_cost, polyline)) {
            float ratio = inliersRatio(contours[i], input_gray_canny.rows, input_gray_canny.cols, setup.inlier_distance, polyline);
            //if (ratio>) {
                polyline.cost = 1.0-ratio;
                polylines.emplace_back(polyline);
            //}
            }
        }
    };

    //collapse
    std::sort(polylines.begin(),polylines.end());
//    for (auto & p : polylines)
//    {
//        std::cout << "->" << p.initial.transpose() << " " << p.merged <<std::endl;
//    }
    for (auto & p : polylines)
    {
        if (p.merged) continue;
        for (auto & c : polylines)
        {
            if (c.merged) continue;
            if (&p == &c) continue;
            const Eigen::Matrix3d transfer1 =(Eigen::AngleAxisd(- c.initial.x()+M_PI/2, Eigen::Vector3d::UnitZ())*
                                              Eigen::AngleAxisd( c.initial.y(), Eigen::Vector3d::UnitY())).matrix() ;


            const Eigen::Matrix3d transfer2 =(Eigen::AngleAxisd(- p.initial.x()+M_PI/2, Eigen::Vector3d::UnitZ())*
                                              Eigen::AngleAxisd( p.initial.y(), Eigen::Vector3d::UnitY())).matrix() ;

            const Eigen::Vector3d diff = transfer1.col(2).cross(transfer2.col(2));
            //std::cout << "diff.norm() " << diff.norm() << std::endl;
            if (diff.norm() < setup.collapse_normal_vector_size)
            {
                c.merged = true;
                p.found_vars.push_back(c.initial);
            }
        }
    }

    cv::Mat drawingColapsePoly = input_image_small.clone();
    cv::Mat drawingTotalPoly = input_image_small.clone();

    if (setup.debug) {
        for (auto &p : polylines) {
            cv::Vec3b color(1 * dist(rng), 1 * dist(rng), 1 * dist(rng));
            line_detector::drawGreatCircle(drawingColapsePoly, p.initial[0], p.initial[1], color);
            if (p.merged) continue;
            line_detector::drawGreatCircle(drawingTotalPoly, p.initial[0], p.initial[1], color);
        }
        cv::imwrite("/tmp/dbg_drawingTotalPoly_contours.png", drawingTotalPoly);
        cv::imwrite("/tmp/dbg_drawingColapsePoly_contours.png", drawingColapsePoly);
    }
    std::vector<Eigen::Vector2d> r;
    for (auto &p : polylines) {
        if (!p.merged)r.push_back(p.initial);
    }
    return r;
}
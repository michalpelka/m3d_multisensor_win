#include "structs.h"


calib_struct::KeyFrame::KeyFrame(std::shared_ptr<float[]> data_p, int len, Eigen::Matrix4d mat, int laser_id) :
    mat(mat), data(data_p), len(len), laser_id(laser_id),
    vb(data.get(), len * sizeof(float)),
    va()
{
    timestamp = data[4];
    VertexBufferLayout layoutPc;
    layoutPc.Push<float>(3);
    layoutPc.Push<float>(1); // angle
    layoutPc.Push<float>(1); // ts
    layoutPc.Push<float>(1); // intensity
    va.AddBuffer(vb, layoutPc);
}

void calib_struct::saveConfig(std::vector<Sophus::Vector6f>& extrinsic_calib_vec, const std::string& fn) {
    std::ofstream txt(fn);
    for (const auto& p : extrinsic_calib_vec) {
        txt << p[0] << " " << p[1] << " " << p[2] << " " << p[3] << " " << p[4] << " " << p[5] << std::endl;
    }
    txt.close();
}

std::shared_ptr<float[]> calib_struct::pclToBuffer(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, int& len, float scale)
{
    const int stride = 6;
    len = stride * cloud->size();
    std::shared_ptr<float[]> data(new float[len]);
    for (int i = 0; i < cloud->size(); i++) {
        data[stride * i + 0] = scale * (*cloud)[i].x;
        data[stride * i + 1] = scale * (*cloud)[i].y;
        data[stride * i + 2] = scale * (*cloud)[i].z;
        data[stride * i + 3] = (*cloud)[i].normal_x;
        data[stride * i + 4] = (*cloud)[i].normal_y;
        data[stride * i + 5] = (*cloud)[i].intensity;
    }
    return data;
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

void calib_struct::savePlanes(std::vector<plane>& planes, const std::string& fn) {
    std::ofstream txt(fn);
    for (const auto& plane : planes) {
        Sophus::Vector6f p = Sophus::SE3f::fitToSE3(plane.matrix).log();
        txt << p[0] << " " << p[1] << " " << p[2] << " " << p[3] << " " << p[4] << " " << p[5] <<" " << plane.size.x() << " " << plane.size.y() << std::endl;
    }
    txt.close();
}
std::vector<calib_struct::plane> calib_struct::loadPlanes(const std::string& fn) {
    std::ifstream txt(fn);
    std::string line;
    std::vector<calib_struct::plane> r;
    while (std::getline(txt, line))
    {
        std::stringstream oss(line);
        Sophus::Vector6f v;
        Eigen::Vector2f size;
        for (int i = 0; i < 6; i++)
        {
            oss >> v[i];
        }
        oss >> size.x();
        oss >> size.y();

        calib_struct::plane plane;
        plane.matrix = Sophus::SE3f::exp(v).matrix();
        plane.size = size;
        r.emplace_back(plane);
    };
    txt.close();
    return r;
}

Eigen::Matrix3d calib_struct::findCovariance(std::vector<Eigen::Vector4d> points, const Eigen::Vector4d avg)
{
    Eigen::Matrix3d covariance;
    for (int x = 0; x < 3; x++)
    {
        for (int y = 0; y < 3; y++)
        {
            double element = 0;
            for (const auto pp : points)
            {
                if ((avg - pp).norm() < 2) {
                    element += (pp(x) - avg(x)) * (pp(y) - avg(y));
                }
            }
            covariance(x, y) = element / (points.size() - 1);
        }
    };
    return covariance;
}


Eigen::Matrix3f calib_struct::findCovariance(std::vector<Eigen::Vector4f> points, const Eigen::Vector4f avg)
{
    Eigen::Matrix3f covariance;
    for (int x = 0; x < 3; x++)
    {
        for (int y = 0; y < 3; y++)
        {
            double element = 0;
            for (const auto pp : points)
            {
                if ((avg - pp).norm() < 2) {
                    element += (pp(x) - avg(x)) * (pp(y) - avg(y));
                }
            }
            covariance(x, y) = element / (points.size() - 1);
        }
    };
    return covariance;
}

Eigen::Matrix4f calib_struct::loadSE3Matrix(const std::string & fn){
    std::ifstream fs (fn);
    Eigen::Matrix4f mat;
    for (int i =0; i < 16; i++) {
        fs >> mat.data()[i];
    }
    return Sophus::SE3f::fitToSE3(mat.transpose()).matrix();
}

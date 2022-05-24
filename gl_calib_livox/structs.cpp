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

#include <memory>

#include <Eigen/Dense>
#include <sophus/se3.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "gl_calib_livox/transform.h"
#include <boost/program_options.hpp>
#include <random>
#include <boost/filesystem.hpp>
#include <boost/regex.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <static/ladybug_calib.hpp>
#define STB_IMAGE_IMPLEMENTATION
#include <3rd/stb_image.h>

#include <chrono>
#include <thread>

class ladybug_camera_calibration{
public:
    void loadCfgFromString(const std::string &data);
    const std::vector<Eigen::Matrix4f> &getCameraExtrinsic() const{return camera_extrinsic;}
    const std::vector<float> &getCameraFocal() const{return camera_focal;}
    const std::vector<Eigen::Vector2f> &getCameraCenter() const {return camera_center;}

private:
    static Eigen::Matrix4d makeTransformation( const double rotX, const double rotY, const double rotZ, const double transX, const double transY, const double transZ);
    std::vector<Eigen::Matrix4f> camera_extrinsic;
    std::vector<float> camera_focal;
    std::vector<Eigen::Vector2f> camera_center;
};

Eigen::Matrix4d ladybug_camera_calibration::makeTransformation( const double rotX, const double rotY, const double rotZ, const double transX, const double transY, const double transZ )
{
    Eigen::Matrix4d matrix;
    double cosX, sinX, cosY, sinY, cosZ, sinZ;

    cosX = cos( rotX );		sinX = sin( rotX );
    cosY = cos( rotY );		sinY = sin( rotY );
    cosZ = cos( rotZ );		sinZ = sin( rotZ );

    // translation portion of transform
    matrix(0,3) = transX;
    matrix(1,3) = transY;
    matrix(2,3) = transZ;

    // cz*cy;
    matrix(0,0) = cosZ * cosY;
    // cz*sy*sx - sz*cx;
    matrix(0,1) = cosZ * sinY * sinX - sinZ * cosX;
    // cz*sy*cx + sz*sx;
    matrix(0,2) = cosZ * sinY * cosX + sinZ * sinX;

    // sz*cy;
    matrix(1,0) = sinZ * cosY;
    // sz*sy*sx + cz*cx;
    matrix(1,1) = sinZ * sinY * sinX + cosZ * cosX;
    // sz*sy*cx - cz*sx;
    matrix(1,2) = sinZ * sinY * cosX - cosZ * sinX;

    //-sy;
    matrix(2,0) = -sinY;
    //cy*sx;
    matrix(2,1) = cosY * sinX;
    //cy*cx;
    matrix(2,2) = cosY * cosX;

    // bottom row, always the same
    matrix(3,0) = 0.0;
    matrix(3,1) = 0.0;
    matrix(3,2) = 0.0;
    matrix(3,3) = 1.0;
    return matrix;
}

void ladybug_camera_calibration::loadCfgFromString(const std::string &data){
    std::stringstream  ss(data);
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(ss, pt);
    this->camera_center.resize(6);
    this->camera_extrinsic.resize(6);
    this->camera_focal.resize(6);
    for (int i =0; i < 6; i++)
    {
        camera_center[i].x() = pt.get<float>("camera"+std::to_string(i)+"_CameraCenterX");
        camera_center[i].y() = pt.get<float>("camera"+std::to_string(i)+"_CameraCenterY");
        camera_focal[i] = pt.get<float>("camera"+std::to_string(i)+"_FocalLen");
        const float tx = pt.get<float>("camera"+std::to_string(i)+"_transX");
        const float ty = pt.get<float>("camera"+std::to_string(i)+"_transY");
        const float tz = pt.get<float>("camera"+std::to_string(i)+"_transZ");
        const float rx = pt.get<float>("camera"+std::to_string(i)+"_rotX");
        const float ry = pt.get<float>("camera"+std::to_string(i)+"_rotY");
        const float rz = pt.get<float>("camera"+std::to_string(i)+"_rotZ");
        camera_extrinsic[i] = makeTransformation(rx,ry,rz,tx,ty,tz).cast<float>();
    }
}

bool updateDrawingBufferWithColor(pcl::PointCloud<pcl::PointXYZRGBL>& color_pointcloud,
                                  const pcl::PointCloud<pcl::PointXYZI> &pc,
                                  const ladybug_camera_calibration &ladybug_calibration,
                                  const Eigen::Matrix4d& calibration_camera,
                                  const std::vector<std::unique_ptr<unsigned char>>& images,
                                  const Eigen::Vector2i& size,
                                  const int channels){
    color_pointcloud.clear();
    color_pointcloud.reserve(pc.size());

    for (const auto p: pc)
    {
        if (!images.empty()) {

            //Eigen::Matrix<double,1,4> pt_cam = camera_extrinsic_loc.inverse()*sphere_odometry.inverse()*pt.transpose();
            float distance_to_center_min = std::numeric_limits<float>::max();
            Eigen::Vector3i best_color;
            for (int camera_id = 0; camera_id < images.size(); camera_id++) {
                Eigen::Matrix<double, 1, 4> pt{p.x, p.y, p.z, 1.0};
                Eigen::Matrix<double, 4, 4> ladybug_loc_calib = ladybug_calibration.getCameraExtrinsic()[camera_id].cast<double>();
                Eigen::Matrix<double, 1, 4> pt_cam =
                        (calibration_camera * ladybug_loc_calib).inverse() * pt.transpose();

                float camera_FocalLen = ladybug_calibration.getCameraFocal()[camera_id];
                const auto &camera_center = ladybug_calibration.getCameraCenter()[camera_id];
                const double xx = camera_FocalLen * (pt_cam.x() / pt_cam.z());
                const double yy = camera_FocalLen * (pt_cam.y() / pt_cam.z());
                double distance_to_center = xx * xx + yy * yy;
                const double xx2 = xx + camera_center.x();
                const double yy2 = yy + camera_center.y();
                const unsigned char* img = images[camera_id].get();
                if (distance_to_center < distance_to_center_min && pt_cam.z() > 0 && xx2 > 0 && yy2 > 0 &&
                    xx2 < size.y() && yy2 < size.x()) {
                    distance_to_center_min = distance_to_center;
                    size_t loc = (int)yy2* size.y()*channels + (int)xx2*channels;

                    unsigned char r1 = img[loc];
                    unsigned char g1 = img[loc+1];
                    unsigned char b1 = img[loc+2];
                    best_color = Eigen::Vector3i{r1,g1,b1};

                }
            }
            if (distance_to_center_min != std::numeric_limits<float>::max()) {
                pcl::PointXYZRGBL rgb_p;
                rgb_p.getArray3fMap() = p.getArray3fMap();
                rgb_p.r = best_color[0];
                rgb_p.g = best_color[1];
                rgb_p.b = best_color[2];
                color_pointcloud.push_back(rgb_p);
            }
        }
        else {
            pcl::PointXYZRGBL rgb_p;
            rgb_p.getArray3fMap() = p.getArray3fMap();
            rgb_p.r = 255;
            rgb_p.g = 255;
            rgb_p.b = 255;
            color_pointcloud.push_back(rgb_p);
        }
    }
}


namespace po = boost::program_options;
const std::vector<float> encoder_offset{0,0,M_PI};
std::vector<Sophus::Vector6f> extrinsic_calib_vec;
int main(int argc, char** argv) {
    po::options_description description("Collapse data using calibration Usage");
    description.add_options()
            ("help,h", "Display this help message")
            ("input_data,f", po::value<std::string>(),"points to input data")
            ("input_dir,i", po::value<std::string>(),"input dir")
            ("output_dir,d", po::value<std::string>(),"output data")
            ("calibration_file,c",po::value<std::string>()->default_value("calibs.txt"), "calibration file")
            ("ladybug,l",po::value<bool>()->default_value(false), "use ladybug for texture")
            ("watch,w", po::value<bool>()->default_value(false), "actively watch input dir");
    std::cout << description << "\n";
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(description).run(), vm);
    po::notify(vm);
    const bool watch {vm["watch"].as<bool>()};

    if (vm.count("help")) {
        std::cout << description << "\n";
        return 1;
    }

    std::vector<std::string> files;

    if (vm.count("input_data"))
    {
        if (watch)
        {
            std::cout << "cannot watch file" << std::endl;
            std::abort();
        }
        files = {vm["input_data"].as<std::string>()};
    }
    do {
        if (vm.count("input_dir")) {
            boost::regex fn_regex("\\d{10}.json");
            const boost::filesystem::path dir_to_scan(vm["input_dir"].as<std::string>());
            boost::filesystem::directory_iterator end_itr;
            for (boost::filesystem::directory_iterator itr(dir_to_scan); itr != end_itr; ++itr) {
                if (is_regular_file(itr->path())) {
                    boost::smatch match;
                    const std::string current_file = itr->path().string();
                    if (boost::regex_search(current_file, match, fn_regex)) {
                        const auto scanned_file = dir_to_scan / itr->path().stem();
                        std::cout << "scanned_file " << scanned_file << std::endl;
                        files.emplace_back(scanned_file.string());
                    }
                }
            }
        }

        for (const auto &fn : files) {

            const std::string calibration_file = vm["calibration_file"].as<std::string>();
            const std::string output_dir = vm["output_dir"].as<std::string>();
            const boost::filesystem::path p(fn);
            const boost::filesystem::path p2(output_dir);
            const std::string target_pcd = (p2 / p.stem()).string() + ".pcd";
            const std::string target_pcd_color = (p2 / p.stem()).string() + "_color.pcd";
            const bool with_ladybug = vm["ladybug"].as<bool>();

            boost::filesystem::create_directory(output_dir);
            std::cout << "*********************************" << std::endl;
            std::cout << "        fn        : " << fn << std::endl;
            std::cout << " calibration_file : " << calibration_file << std::endl;
            std::cout << " output_dir       : " << output_dir << std::endl;
            std::cout << " target_pcd       : " << target_pcd << std::endl;
            std::cout << " with ladybug     : " << with_ladybug << std::endl;

            if (boost::filesystem::exists(target_pcd)) {
                std::cout << "target exists " << target_pcd << std::endl;
                continue;
            }

            extrinsic_calib_vec = calib_struct::initializeCalib(calibration_file);

            pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZINormal>);
            pcl::io::loadPCDFile<pcl::PointXYZINormal>(fn + "_pointcloud_raw_livox_1.pcd", *cloud2);
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZINormal>);
            pcl::io::loadPCDFile<pcl::PointXYZINormal>(fn + "_pointcloud_raw_livox_2.pcd", *cloud1);
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_velo(new pcl::PointCloud<pcl::PointXYZINormal>);
            pcl::io::loadPCDFile<pcl::PointXYZINormal>(fn + "_pointcloud_raw_velodyne.pcd", *cloud_velo);


            auto pcd1 = calib_struct::createTransformedPc(cloud1, extrinsic_calib_vec[0],
                                                          encoder_offset[0]).makeShared();
            auto pcd2 = calib_struct::createTransformedPc(cloud2, extrinsic_calib_vec[1],
                                                          encoder_offset[1]).makeShared();
            auto pcd3 = calib_struct::createTransformedPc(cloud_velo, extrinsic_calib_vec[2],
                                                          encoder_offset[2]).makeShared();
            const std::array<pcl::PointCloud<pcl::PointXYZINormal>::Ptr, 3> pcds = {pcd1, pcd2, pcd3};

            pcl::PointCloud<pcl::PointXYZI> output;
            std::default_random_engine generator;
            std::uniform_real_distribution<float> distribution(0.0, 800.0);

            Eigen::Affine3f rot2(Eigen::Affine3f::Identity());
            rot2.rotate(Eigen::AngleAxisf(-M_PI / 2, Eigen::Vector3f::UnitZ()));

            const std::vector<float> prob_mods = {1, 1, 10};
            for (int i = 0; i < prob_mods.size(); i++) {
                const auto pc = pcds[i];
                float prob_mod = prob_mods[i];
                output.reserve(output.size() + pc->size());
                for (const auto &p : *pc) {
                    pcl::PointXYZI tp;
                    Eigen::Vector3f ep = rot2 * p.getArray3fMap();
                    float distance = ep.norm();
                    float distance_random = distribution(generator);
                    if (true || distance * prob_mod > distance_random && distance > 1) {
                        tp.getArray3fMap() = ep;
                        tp.intensity = distance;
                        output.push_back(tp);
                    }
                }
            }


            if (with_ladybug) {
                std::cout << "processing ladybug" << std::endl;
                bool is_ok = true;
                std::vector<boost::filesystem::path> photos;
                std::vector<std::unique_ptr<unsigned char>> images;
                Eigen::Vector2i image_size;
                int image_channels;
                for (int i = 0; i < 6; i++) {
                    // load calib
                    photos.push_back(fn + "_ladybugPostProcess-rectified" + std::to_string(i) + "-0.jpg");
                    if (!boost::filesystem::exists(photos.back())) {
                        std::cout << "skiping ladybug, file not found " << photos.back() << std::endl;
                        is_ok = false;
                        break;
                    }
                }

                if (is_ok) {
                    for (int i = 0; i < 6; i++) {
                        //                      images.push_back(cv::imread());
                        int width, height, channels;
                        std::unique_ptr<unsigned char> ptr = std::unique_ptr<unsigned char>(
                                stbi_load(photos[i].string().c_str(), &width, &height, &channels, 0));
                        if (i == 0) {
                            image_size = Eigen::Vector2i{height, width};
                            image_channels = channels;
                        } else {
                            if (!(image_size.y() == width && image_size.x() == height)) {
                                is_ok = false;
                                std::cout << "image_size is not consitent " << image_size << std::endl;
                            }
                            if (channels != image_channels) {
                                is_ok = false;
                                std::cout << "image_channels is not consitent " << image_channels << std::endl;
                            }

                        }
                        images.emplace_back(std::move(ptr));

                    }
                    if (is_ok) {
                        ladybug_camera_calibration ladybug_calib;
                        ladybug_calib.loadCfgFromString(kLadybugINIData);
                        //load images

                        pcl::PointCloud<pcl::PointXYZRGBL> color_pointcloud;
                        Eigen::Matrix4d extrinsic_calib_ld = Sophus::SE3f::exp(
                                extrinsic_calib_vec.back()).inverse().matrix().cast<double>();
                        updateDrawingBufferWithColor(color_pointcloud, output, ladybug_calib, extrinsic_calib_ld,
                                                     images, image_size, image_channels);
                        std::cout << "processing ladybug - size " << color_pointcloud.size() << std::endl;
                        if (color_pointcloud.size() > 0) {
                            std::cout << "saving " << target_pcd_color << std::endl;
                            pcl::io::savePCDFileBinary(target_pcd_color, color_pointcloud);
                        }
                    }
                }

            }
            if (output.size() > 0) {
                pcl::io::savePCDFileBinary(target_pcd, output);
                if (boost::filesystem::exists(fn + "_odom.txt")) {
                    boost::filesystem::copy_file(fn + "_odom.txt", (p2 / p.stem()).string() + ".txt");
                }
            }
        }
    if (watch){
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(2000ms);
    }
    }while(watch);
};

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
            ("calibration_file,c",po::value<std::string>()->default_value("calibs.txt"), "calibration file");
    std::cout << description << "\n";
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(description).run(), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << description << "\n";
        return 1;
    }

    std::vector<std::string> files;

    if (vm.count("input_data"))
    {
        files = {vm["input_data"].as<std::string>()};
    }
    if (vm.count("input_dir"))
    {
        boost::regex fn_regex ("\\d{10}.json");
        const boost::filesystem::path dir_to_scan(vm["input_dir"].as<std::string>());
        boost::filesystem::directory_iterator end_itr;
        for (boost::filesystem::directory_iterator itr(dir_to_scan); itr != end_itr; ++itr)
        {
            if (is_regular_file(itr->path())) {
                boost::smatch match;
                const std::string current_file = itr->path().string();
                if (boost::regex_search(current_file, match, fn_regex)){
                    const auto scanned_file = dir_to_scan/itr->path().stem();
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

        std::cout << "*********************************" << std::endl;
        std::cout << "        fn        : " << fn << std::endl;
        std::cout << " calibration_file : " << calibration_file << std::endl;
        std::cout << " output_dir       : " << output_dir << std::endl;
        std::cout << " target_pcd       : " << target_pcd << std::endl;
        if (boost::filesystem::exists(target_pcd))
        {
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


        auto pcd1 = calib_struct::createTransformedPc(cloud1, extrinsic_calib_vec[0], encoder_offset[0]).makeShared();
        auto pcd2 = calib_struct::createTransformedPc(cloud2, extrinsic_calib_vec[1], encoder_offset[1]).makeShared();
        auto pcd3 = calib_struct::createTransformedPc(cloud_velo, extrinsic_calib_vec[2],
                                                      encoder_offset[2]).makeShared();
        const std::array<pcl::PointCloud<pcl::PointXYZINormal>::Ptr, 3> pcds = {pcd1, pcd2, pcd3};

        pcl::PointCloud<pcl::PointXYZI> output;
        std::default_random_engine generator;
        std::uniform_real_distribution<float> distribution(0.0, 1000.0);

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
                if (distance * prob_mod > distance_random && distance > 1) {
                    tp.getArray3fMap() = ep;
                    tp.intensity = distance;
                    output.push_back(tp);
                }
            }
        }
        if (target_pcd.size()>0) {
            pcl::io::savePCDFileBinary(target_pcd, output);
            boost::filesystem::copy_file(fn + "_odom.txt", (p2 / p.stem()).string() + ".txt");
        }
    }
};
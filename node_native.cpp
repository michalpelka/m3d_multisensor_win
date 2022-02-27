#include "mavlink_client_udp.h"
#include "livox_client.h"
#include "velodyne_client.h"

#include <boost/thread.hpp>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "livox/lds_lidar.h"
//#include "util.hpp"

//#include "xsens_client.h"
#include "point_to_point_source_to_target_rotated_mirror_tait_bryan_wc_jacobian.h"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "health_server.h"
#ifdef WITH_LADYBUG
#include "ladybug/ladybug_capture.h"
#endif

const std::string kVelodyne1 ="velodyne1" ;
const Eigen::Affine3f getRotLaserLocCalib(){
    Eigen::Matrix4f d;
    d <<0.999921, -0.00638093,   0.0107958, -0.00731472,
            0.00634553,    0.999974,  0.00331037,   0.0463718,
            -0.0108166, -0.00324161,    0.999936,   0.0222408,
            0,           0,           0 ,          1;
    return Eigen::Affine3f (d);
};

Eigen::Affine3f getVelodyne1(){
    Eigen::Affine3f rot0 (Eigen::Affine3f::Identity());
    rot0.rotate(Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f::UnitZ()));

    Eigen::Affine3f rot1 (Eigen::Affine3f::Identity());
    rot1.translate(Eigen::Vector3f (0, 0.003542, 0.072236));
    rot1.rotate(Eigen::Quaternionf(-0.509959, -0.391302, -0.466336,0.607746));
    return rot1*rot0;
}

Eigen::Affine3f getAngle(float angle){
    static Eigen::Affine3f staticTransform = getVelodyne1();
    Eigen::Affine3f rot2 (Eigen::Affine3f::Identity());
    rot2.rotate(Eigen::AngleAxisf(angle-5.53747-0.98*M_PI, Eigen::Vector3f::UnitZ()));
    return rot2*staticTransform*getRotLaserLocCalib();
}

Eigen::Affine3d getSE3Mirror(double angle){
    Eigen::Affine3d mirror_tf1 =Eigen::Affine3d::Identity();
    Eigen::Affine3d mirror_tf2 =Eigen::Affine3d::Identity();
    mirror_tf1.rotate(Eigen::AngleAxisd(angle-5.53747, Eigen::Vector3d::UnitX()));
    mirror_tf2.translation().x()=0.15;
    mirror_tf2.rotate(Eigen::AngleAxisd((-52.5*M_PI)/180.0, Eigen::Vector3d::UnitY()));
    return mirror_tf1*mirror_tf2;
}

class native_sync{
public:

    std::string produceReport()
    {
        boost::property_tree::ptree pt;

        pt.put("status.board.msgs", (int)client1->getEncoder_msgs_count());
        pt.put("status.board.angle", client1->getEncoderLast().angle);

        pt.put("status.hw1.livox_1.rejection_rate", livox_client_1->getRejectionRate());
        pt.put("status.hw1.livox_1.data_rate", livox_client_1->getDataRate());

        pt.put("status.hw1.livox_2.rejection_rate", livox_client_2->getRejectionRate());
        pt.put("status.hw1.livox_2.data_rate", livox_client_2->getDataRate());


        if (velo_client1) {
            pt.put("status.hw1.velodyne_1.data_rate", velo_client1->getDataRate());
            pt.put("status.hw1.velodyne_1.rejection_rate", velo_client1->getRejectionRate());
            pt.put("status.hw1.velodyne_1.jitter", velo_client1->getJitter());
        }

        const auto lidars_ptr = LdsLidar::GetInstance().getLidars();
        const uint32_t lidar_cnt = 32;
        for (uint32_t i = 0; i < lidar_cnt; i++) {
            if (lidars_ptr[i].handle == 32) break;
            const std::string id = std::to_string(i);
            pt.put("status.livox." + id + ".connecte_state", lidars_ptr[i].connect_state);
            pt.put("status.livox." + id + ".handle", lidars_ptr[i].handle);
            pt.put("status.livox." + id + ".info.broadcast_code", lidars_ptr[i].info.broadcast_code);
            pt.put("status.livox." + id + ".info.cmd_port", lidars_ptr[i].info.cmd_port);
            pt.put("status.livox." + id + ".info.data_port", lidars_ptr[i].info.data_port);
            pt.put("status.livox." + id + ".info.feature", lidars_ptr[i].info.feature);
            pt.put("status.livox." + id + ".info.firmware_version", lidars_ptr[i].info.firmware_version);
            pt.put("status.livox." + id + ".info.id", lidars_ptr[i].info.id);
            pt.put("status.livox." + id + ".info.sensor_port", lidars_ptr[i].info.sensor_port);
            pt.put("status.livox." + id + ".info.slot", lidars_ptr[i].info.slot);
            pt.put("status.livox." + id + ".info.state", lidars_ptr[i].info.state);
            pt.put("status.livox." + id + ".info.status.progress", lidars_ptr[i].info.status.progress);

            pt.put("status.livox." + id + ".info.status.status_code.error_code", lidars_ptr[i].info.status.status_code.error_code);
            //pt.put("status.livox." + id + ".info.status.status_code.hub_error_code", lidars_ptr[i].info.status.status_code.hub_error_code);
            pt.put("status.livox." + id + ".info.status.status_code.lidar_error_code.temp_status", lidars_ptr[i].info.status.status_code.lidar_error_code.temp_status);
            pt.put("status.livox." + id + ".info.status.status_code.lidar_error_code.volt_status", lidars_ptr[i].info.status.status_code.lidar_error_code.volt_status);
            pt.put("status.livox." + id + ".info.status.status_code.lidar_error_code.motor_status", lidars_ptr[i].info.status.status_code.lidar_error_code.motor_status);
            pt.put("status.livox." + id + ".info.status.status_code.lidar_error_code.dirty_warn", lidars_ptr[i].info.status.status_code.lidar_error_code.dirty_warn);
            pt.put("status.livox." + id + ".info.status.status_code.lidar_error_code.firmware_err", lidars_ptr[i].info.status.status_code.lidar_error_code.firmware_err);
            pt.put("status.livox." + id + ".info.status.status_code.lidar_error_code.pps_status", lidars_ptr[i].info.status.status_code.lidar_error_code.pps_status);
            pt.put("status.livox." + id + ".info.status.status_code.lidar_error_code.device_status", lidars_ptr[i].info.status.status_code.lidar_error_code.device_status);
            pt.put("status.livox." + id + ".info.status.status_code.lidar_error_code.fan_status", lidars_ptr[i].info.status.status_code.lidar_error_code.fan_status);
            pt.put("status.livox." + id + ".info.status.status_code.lidar_error_code.self_heating", lidars_ptr[i].info.status.status_code.lidar_error_code.self_heating);
            pt.put("status.livox." + id + ".info.status.status_code.lidar_error_code.ptp_status", lidars_ptr[i].info.status.status_code.lidar_error_code.ptp_status);
            pt.put("status.livox." + id + ".info.status.status_code.lidar_error_code.time_sync_status", lidars_ptr[i].info.status.status_code.lidar_error_code.time_sync_status);
            pt.put("status.livox." + id + ".info.status.status_code.lidar_error_code.system_status", lidars_ptr[i].info.status.status_code.lidar_error_code.system_status);
            pt.put("status.livox." + id + ".info.type", lidars_ptr[i].info.type);

        }


        {
            std::lock_guard<std::mutex> lck(tcp_client_status);
            pt.put("status.hw1.board.tt", tt_status_client_1);
        }
        std::stringstream oss;
        boost::property_tree::write_json(oss, pt);
        return oss.str();
    }
    void tcpThread(){
        while(true){
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            std::string loc_tt_cl1 = client1->sendTCPMessage("tt\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            //client1->sendCommand("mvel 500\n");
            //std::this_thread::sleep_for(std::chrono::milliseconds(50));

            std::lock_guard<std::mutex>lck(tcp_client_status);
            std::swap(tt_status_client_1, loc_tt_cl1);

        }
    }

    native_sync()
    {
        boost::asio::io_service io_service;

        client1 = std::make_shared<mavlink_client_udp>(io_service, "192.168.1.10", 14550);
        boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));


        std::vector<std::thread> velodyne_threads;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        int unit_rate1 = client1->getEncoder_msgs_count();

//        std::cout << "3d unit rate " << unit_rate1 << std::endl;
//        std::cout << "3d unit rate " << unit_rate1 << std::endl;
//
//        if (unit_rate1 < 100 || unit_rate2 < 100){
//            return;
//        }

      /*  const auto dataHandler_velo1 = [&](pcl::PointCloud<pcl::PointXYZINormal>& pc){
            if (pub_velo_1_raw.getNumSubscribers()>0) {
                for (int i = 0; i < pc.size(); i++) {
                    const float angle = -pc[i].normal_x;
                    const auto t = pc[i].getArray3fMap();
                    pc[i].getArray3fMap() = t;
                }
                pcl_conversions::toPCL(ros::Time::now(), pc.header.stamp);
                pc.header.frame_id = "m3d";
                pub_velo_1_raw.publish(pc);
            }

            if (pub_velo_1.getNumSubscribers()>0) {
                for (int i = 0; i < pc.size(); i++) {
                    const float angle = -pc[i].normal_x;
                    const auto t = pc[i].getArray3fMap();
                    pc[i].getArray3fMap() = getAngle(angle) * t;
                }
                pcl_conversions::toPCL(ros::Time::now(), pc.header.stamp);
                pc.header.frame_id = "m3d";
                pub_velo_1.publish(pc);
            }

        };*/
        // setup 

        //const auto dataHandler_velo1 = [&](pcl::PointCloud<pcl::PointXYZINormal>& pc){
        //    if (aggregation_deadline != std::chrono::time_point<std::chrono::system_clock>()) {
        //        if (std::chrono::system_clock::now() < aggregation_deadline)
        //        {
        //            aggregated_pointclouds[0] += pc;
        //        }
        //        else {
        //            if (!aggregated_pointclouds[0].empty())
        //            {
        //                using namespace std::chrono;
        //                int64_t timestamp = duration_cast<seconds>(aggregation_deadline.time_since_epoch()).count();
        //                pcl::io::savePCDFileBinary(std::to_string(timestamp) + "_pointcloud_raw_velodyne.pcd", aggregated_pointclouds[0]);
        //                aggregation_deadline = std::chrono::time_point<std::chrono::system_clock>();
        //                std::cout << "============================================" << std::endl;
        //                std::cout << "Done aggregating" << std::endl;
        //                std::cout << "============================================" << std::endl;
        //                aggregated_pointclouds[0].clear();
        //            }
        //        }
        //    }
        //};
        //velo_client1 = std::make_shared<velodyne_client>(client1, "0.0.0.0", 2370, "C:\\Users\\micha\\Desktop\\m3d_multisensor_win\\VLP-16.xml");

        //velo_client1->setHandler(dataHandler_velo1);

        std::vector<std::string> broadcast_code_strs;
        LdsLidar& read_lidar = LdsLidar::GetInstance();
        read_lidar.InitLdsLidar(broadcast_code_strs);
        read_lidar.setClient(client1);
        livox_client_1 = std::make_shared<livox_client>(client1, "1PQDH7600102371");
        livox_client_2 = std::make_shared<livox_client>(client1, "1PQDH7600102351");

//
        const auto dataHandler2 = [&](pcl::PointCloud<pcl::PointXYZINormal>& pc) {
            if (aggregation_deadline != std::chrono::time_point<std::chrono::system_clock>()) {
                if (std::chrono::system_clock::now() < aggregation_deadline)
                {
                    aggregated_pointclouds[1] += pc;
                }
                else {
                    if (!aggregated_pointclouds[1].empty())
                    {

                        using namespace std::chrono;
                        int64_t timestamp = duration_cast<seconds>(aggregation_deadline.time_since_epoch()).count();
                        pcl::io::savePCDFileBinary(std::to_string(timestamp) + "_pointcloud_raw_livox_1.pcd", aggregated_pointclouds[1]);
                        aggregation_deadline = std::chrono::time_point<std::chrono::system_clock>();
                        std::cout << "============================================" << std::endl;
                        std::cout << "Done aggregating" << std::endl;
                        std::cout << "============================================" << std::endl;
                        aggregated_pointclouds[1].clear();
                    }
                }
            }
        };
        const auto dataHandler3 = [&](pcl::PointCloud<pcl::PointXYZINormal>& pc) {
            if (aggregation_deadline != std::chrono::time_point<std::chrono::system_clock>()) {
                if (std::chrono::system_clock::now() < aggregation_deadline)
                {
                    aggregated_pointclouds[2] += pc;
                }
                else {
                    if (!aggregated_pointclouds[2].empty())
                    {

                        using namespace std::chrono;
                        int64_t timestamp = duration_cast<seconds>(aggregation_deadline.time_since_epoch()).count();
                        pcl::io::savePCDFileBinary(std::to_string(timestamp) + "_pointcloud_raw_livox_2.pcd", aggregated_pointclouds[2]);
                        aggregation_deadline = std::chrono::time_point<std::chrono::system_clock>();
                        std::cout << "============================================" << std::endl;
                        std::cout << "Done aggregating" << std::endl;
                        std::cout << "============================================" << std::endl;
                        aggregated_pointclouds[2].clear();
                    }
                }
            }
        };

        livox_client_1->setHandler(dataHandler2);
        livox_client_2->setHandler(dataHandler3);


        //const auto imu_data_handler = [&](double ts , Eigen::Vector3d acc , Eigen::Vector3d rate, Eigen::Quaterniond orientation){
        //    //q_glob =orientation;
        //    sensor_msgs::Imu imu_msg;
        //    imu_msg.orientation.x = orientation.x();
        //    imu_msg.orientation.y = orientation.y();
        //    imu_msg.orientation.z = orientation.z();
        //    imu_msg.orientation.w = orientation.w();

        //    imu_msg.angular_velocity.x = rate.x();
        //    imu_msg.angular_velocity.y = rate.y();
        //    imu_msg.angular_velocity.z = rate.z();

        //    imu_msg.linear_acceleration.x = acc.x();
        //    imu_msg.linear_acceleration.y = acc.y();
        //    imu_msg.linear_acceleration.z = acc.z();

        //    imu_msg.linear_acceleration_covariance[0] = 0.1;
        //    imu_msg.linear_acceleration_covariance[4] = 0.1;
        //    imu_msg.linear_acceleration_covariance[8] = 0.1;

        //    imu_msg.angular_velocity_covariance[0] = 0.1;
        //    imu_msg.angular_velocity_covariance[4] = 0.1;
        //    imu_msg.angular_velocity_covariance[8] = 0.1;

        //    imu_msg.orientation_covariance[0] = 0.1;
        //    imu_msg.orientation_covariance[4] = 0.1;
        //    imu_msg.orientation_covariance[8] = 0.1;

        //    imu_msg.header.frame_id="top_plate_link";
        //    imu_msg.header.stamp = ros::Time::now();
        //    pub_imu_wall.publish(imu_msg);

        //    imu_msg.header.stamp.fromSec(ts);
        //    pub_imu_hardware.publish(imu_msg);

        //};

        //xsens_client imu(client2);
        //if (imu.OpenImu())imu.setHandler(imu_data_handler);

        const auto status_string = [&](){
            return this->produceReport();
        };


        std::thread tcp_unit_comm_thread(&native_sync::tcpThread, this);
        health_server::setStatusHandler(status_string);
        std::thread http_thread(health_server::server_worker);
        const auto aggregate_data =[&](const std::string& t){
            using namespace std::chrono;
            auto tt = t;
            aggregation_deadline = system_clock::now()+std::chrono::seconds(std::atoi(tt.c_str()));
            
            int64_t timestamp = duration_cast<seconds>(aggregation_deadline.time_since_epoch()).count();

          /*  std::thread ladybug_th{ [=]() {
                captureLadybugImage(std::to_string(timestamp) + "_");
            } };
            ladybug_th.detach();*/

        };
        health_server::setTriggerHandler(aggregate_data, "scan");

#ifdef WITH_LADYBUG
        const auto take_photo = [&](const std::string& t) {
            int result = captureLadybugImage("");
            return std::to_string(result);
        };
        health_server::setTriggerHandler(take_photo, "ladybug");
#endif
        const auto reset1 =[&](const std::string& t){
            if (t=="1"){
                client1->sendCommand("RRR\n");
            }
        };
        health_server::setTriggerHandler(reset1, "resetBoard");

        for (;;) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

    }


private:
   

    std::shared_ptr<mavlink_client_udp> client1;

    std::mutex tcp_client_status;
    std::string tt_status_client_1;
    std::string tt_status_client_2;

    std::shared_ptr<velodyne_client> velo_client1;
    
    std::shared_ptr<livox_client> livox_client_1;
    std::shared_ptr<livox_client> livox_client_2;


    std::vector<double> velodyne_data_rate;
    std::chrono::time_point<std::chrono::system_clock> aggregation_deadline;
    std::array<pcl::PointCloud<pcl::PointXYZINormal>,3> aggregated_pointclouds;
};

int main(int argc, char** argv) {

    native_sync();
}

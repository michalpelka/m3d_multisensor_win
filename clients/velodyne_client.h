//
// Created by michal on 19.09.2021.
//

#ifndef SRC_VELODYNE_CLIENT_H
#define SRC_VELODYNE_CLIENT_H


#include "PacketDecoder.h"
#include "PacketDriver.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include <thread>
#include <mutex>
#include <vector>
#include <deque>

class mavlink_client_udp;
struct velodyne_chunk{
    std::vector<Eigen::Vector3f> points;
    std::vector<float> intensity;
    std::vector<int> laser_id;
    float angle;
    float error;
    double timestamp;
};


class velodyne_client {
public:
    velodyne_client(std::shared_ptr<mavlink_client_udp> encoder_client, const std::string& ip,
                    int udp, const std::string& xml_calibration);

    void setDone(){
        std::lock_guard<std::mutex> lck(mutex);
        is_done = true;
    }
    bool getDone(){
        std::lock_guard<std::mutex> lck(mutex);
        return is_done;
    }

    void setHandler(std::function<void(pcl::PointCloud<pcl::PointXYZINormal>&)>  handler)
    {
        data_handler = handler;
    }
    const float getDataRate() const {
        return data_rate;
    }

    const float getRejectionRate() const {
        return rejection_rate;
    }
    const float getJitter() const {
        return jitter;
    }
protected:
    void velodyne_listener_thread_worker();
    void velodyne_sync_thread_worker();
    std::thread listner_thread;
    std::thread synchronization_thread;
    std::mutex mutex;
    bool is_done{false};
    std::string xml_calib;
    std::string ip;
    int udp_port;
    std::deque<velodyne_chunk> data_chunks;
    std::mutex data_chunks_mutex;
    std::shared_ptr<mavlink_client_udp> client;
    std::function<void(pcl::PointCloud<pcl::PointXYZINormal>&)> data_handler;
    std::atomic<float> data_rate;
    std::atomic<float> rejection_rate;
    std::atomic<float> jitter;

};


#endif //SRC_VELODYNE_CLIENT_H

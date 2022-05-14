//
// Created by michal on 19.09.2021.
//

#ifndef SRC_LIVOX_CLIENT_H
#define SRC_LIVOX_CLIENT_H




#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include <thread>
#include <mutex>
#include <vector>
#include <deque>

#include "livox_sdk.h"

class mavlink_client_udp;

class livox_client{
public:
    livox_client(std::shared_ptr<mavlink_client_udp> encoder_client, const std::string& livox_serial);

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

    const float getRejectionRate() const {
        return rejection_rate;
    }

    float getDataRate() const {
        return data_rate;
    }

    /// Returns livox serial number
    const std::string& getLivoxSn() {
        return livox_serial;
    }

private:
    std::shared_ptr<mavlink_client_udp> client;
    const std::string livox_serial;
    void livox_sync_thread_worker();

    std::thread synchronization_thread;
    std::thread monitor_thread;
    std::mutex mutex;
    bool is_done{false};
    std::function<void(pcl::PointCloud<pcl::PointXYZINormal>&)> data_handler;
    std::atomic<float> rejection_rate;
    std::atomic<float> data_rate;
};

#endif //SRC_VELODYNE_CLIENT_H

#include "livox_client.h"
#include "mavlink_client_udp.h"
#include <iostream>
#include "lds_lidar.h"
livox_client::livox_client(std::shared_ptr<mavlink_client_udp> encoder_client, const std::string& livox_serial):
client(encoder_client), livox_serial(livox_serial), data_rate(-1)
{
    synchronization_thread = std::thread(&livox_client::livox_sync_thread_worker, this);
}

void livox_client::livox_sync_thread_worker()
{
    LdsLidar& read_lidar = LdsLidar::GetInstance();
    uint64_t points_all{0};
    uint64_t points_rejected{0};
    int rate = 0;
    auto p1 = std::chrono::system_clock::now();
    while(!getDone()){
        const auto p2 = std::chrono::system_clock::now();
        double duration = std::chrono::duration_cast<std::chrono::milliseconds>(p2-p1).count();
        if ( duration > 1000 )
        {
            //std::cout << "[livox " << livox_serial << "] RATE " << rate  << std::endl;
            data_rate.store(rate);
            rate =0;
            p1 = std::chrono::system_clock::now();
        }
        //find livox handle from SN, can be changed during operation
        int handle = -1;
        for (int i = 0; i < read_lidar.getLidarCount(); i++)
        {
            if (this->livox_serial.compare(read_lidar.getLidars()[i].info.broadcast_code) == 0)
            {
                handle = read_lidar.getLidars()[i].handle;
            }
            if (read_lidar.getLidars()[i].handle == read_lidar.getLidarCount())
                break;
        }
        if (handle == -1)
        {
            continue;
        }
        //std::this_thread::sleep_for(std::chrono::milliseconds(50));

        auto data_to_work = read_lidar.getLivoxSynch(handle).popBack(10);
        if (points_all> 1e3){
            double reject_rate =(100.0 * points_rejected) / points_all;
            //std::cout << "[livox "<< livox_serial <<"] rejection rate \t : " << reject_rate << "%" << std::endl;
            rejection_rate.store(reject_rate);
            points_rejected = 0;
            points_all =0;
        }

        pcl::PointCloud<pcl::PointXYZINormal> pc;
        for (const auto packet : data_to_work){

            const double packet_ts = packet.timestamp;
            encoder_with_timestamp encoder_reading;
            const double err = client->getEncoder(packet_ts, encoder_reading);
            points_all++;

            if (err < 0.001){
                pc.reserve(pc.size()+packet.datachunk.size());
                for (auto point : packet.datachunk){
                    pcl::PointXYZINormal p;
                    p.getArray3fMap() = point.getArray3fMap();
                    p.intensity = point.intensity;
                    p.normal_x = encoder_reading.angle;
                    p.normal_y = packet_ts-1.60000000e+09;
                    pc.push_back(p);
                    rate++;
                }

            }else{
                points_rejected++;
            }

        }
        if (pc.size() > 0 && data_handler) {

            //std::cout << "[livox " << livox_serial << "] Data handler with pointcloud size  " << pc.size() << std::endl;
            data_handler(pc);
        }
    }
}


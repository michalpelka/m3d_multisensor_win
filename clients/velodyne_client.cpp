
#include "velodyne_client.h"
#include "mavlink_client_udp.h"
#include <iostream>

velodyne_client::velodyne_client(std::shared_ptr<mavlink_client_udp> encoder_client, const std::string& ip,
                                 int udp, const std::string& xml_calibration):
client(encoder_client), ip(ip), udp_port(udp), xml_calib(xml_calibration),
data_rate(0),rejection_rate(0),jitter(0)
{
    listner_thread = std::thread(&velodyne_client::velodyne_listener_thread_worker, this);
    synchronization_thread = std::thread(&velodyne_client::velodyne_sync_thread_worker, this);
}

void velodyne_client::velodyne_listener_thread_worker(){
    std::unique_ptr<PacketDriver> driver;
    PacketDecoder decoder;
    decoder.SetCorrectionsFile(xml_calib);
    driver =std::make_unique<PacketDriver>(udp_port);
    std::string data;
    unsigned int dataLength;
    unsigned point_count = 0;
    auto p1 = std::chrono::system_clock::now();
    while (!getDone()){
        driver->GetPacket(&data, &dataLength);
        auto latest_frame = decoder.DecodePacket(&data, &dataLength);
        velodyne_chunk chunk;
        for (int i =0; i< latest_frame.x.size(); i++)
        {
//                fstream << point_count << "," << latest_frame.x[i]<<","
//                <<latest_frame.y[i]<<","<<latest_frame.z[i]<<"," <<int(latest_frame.intensity[i]) <<","
//                << int(latest_frame.laser_id[i]) <<","<< int(latest_frame.ms_from_top_of_hour[i]) <<"\n";
            chunk.points.emplace_back(Eigen::Vector3f(latest_frame.x[i],latest_frame.y[i],latest_frame.z[i]));
            chunk.intensity.emplace_back(latest_frame.intensity[i]);
            chunk.laser_id.emplace_back(latest_frame.laser_id[i]);
            int hours = std::floor(client->getEncoderLast().timestamp/ 3600.0);
            chunk.timestamp = latest_frame.ms_from_top_of_hour[i]* 1e-6 + 3600 * hours ;
            point_count++;
            if (chunk.points.size()>16)
            {
                std::unique_lock<std::mutex> lck(data_chunks_mutex);
                data_chunks.push_back(chunk);
                chunk = velodyne_chunk();
            }
        }
        const auto p2 = std::chrono::system_clock::now();
        double duration = std::chrono::duration_cast<std::chrono::milliseconds>(p2-p1).count();
        if (duration > 1000){
            std::cout << "[velo "<< udp_port<< " ] rate :" << 1.0*point_count << std::endl;
            data_rate.store(point_count);
            point_count = 0;
            p1 =p2;
        }
    }
}

void velodyne_client::velodyne_sync_thread_worker()
{
    uint32_t  message_count = 0;
    uint32_t  message_count_error = 0;
    auto p1 = std::chrono::system_clock::now();
    while(!getDone()){
        pcl::PointCloud<pcl::PointXYZINormal> pc;
        std::deque<velodyne_chunk> local_chunks;
        {
            std::unique_lock<std::mutex> lck(data_chunks_mutex);
            while (data_chunks.size()>100)
            {
                local_chunks.push_back(data_chunks.front());
                data_chunks.pop_front();
            }
        }

        for (int i =0; i< local_chunks.size();i++)
        {
            double ts = local_chunks[i].timestamp;
            encoder_with_timestamp encoder;
            double error = client->getEncoder(ts, encoder);
            local_chunks[i].angle = encoder.angle;

            if (error < 0.01){
                for (int j =0; j < local_chunks[i].points.size(); j++)
                {
                    pcl::PointXYZINormal pt;
                    //std::cout << local_chunks[i].points[j].cast<float>().transpose() << std::endl;
                    pt.getArray3fMap() = local_chunks[i].points[j];
                    pt.intensity = local_chunks[i].intensity[j];
                    pt.normal_x = encoder.angle;
                    pt.normal_y = ts-1.60000000e+09;
                    pc.push_back(pt);
                }
            }else{
                //std::cerr << "[sync ] buffer start" << client->getStartOfBufferSecs() << " : " << client->getEndOfBufferSecs() <<" query "<< ts << std::endl;
                message_count_error++;
            }
            message_count++;
        }
        if (data_handler) {
            data_handler(pc);
        }
//        //std::cout << pc.size() << std::endl;
//        pc.header.frame_id = "velodyne";
//        pub1.publish(pc);

        const auto p2 = std::chrono::system_clock::now();
        double duration = std::chrono::duration_cast<std::chrono::milliseconds>(p2-p1).count();
        if (duration>1000){
            std::unique_lock<std::mutex> lck(data_chunks_mutex);

            double laser_last_timestamp = data_chunks.size()?data_chunks.back().timestamp:0;
            const double start = client->getStartOfBufferSecs();
            const double end = client->getStartOfBufferSecs();
            //std::cout << "[sync ] encoder_buffer: "<< start << " - "<< end << std::endl;
            float rate =  message_count==0?-1:(100.0*message_count_error/message_count);
            std::cout << "[velo "+std::to_string(udp_port)+" ] rejection rate :  "<< rate <<"%"<< std::endl;
            //std::cout << "[sync ] jitter:         "<< laser_last_timestamp<<"\t"<<client->getEncoderLast().timestamp<< std::endl;
            rejection_rate.store(rate);
            jitter.store(laser_last_timestamp - client->getEncoderLast().timestamp);
            message_count_error = 0;
            message_count = 0;
            p1 =p2;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds (10));
    }
}

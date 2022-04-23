/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   mavlink_client.cpp
 * Author: michal
 *
 * Created on February 8, 2016, 10:53 PM
 */

#include "mavlink_client_udp.h"
using boost::asio::ip::udp;


void mavlink_client_udp::sendCommand(const std::string& msg){
    try {
        boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string(host_), 8888);

        boost::asio::ip::tcp::socket socket(io_service_);
        socket.connect(endpoint);
        boost::system::error_code error;
        socket.write_some(boost::asio::buffer(msg.data(), msg.size()), error);

    }catch (const std::exception &e) {
        std::cerr<< e.what();
    }
}

std::string mavlink_client_udp::sendTCPMessage(const std::string& msg){
    try {
        boost::asio::ip::tcp::endpoint endpoint(boost::asio::ip::address::from_string(host_), 8888);

        boost::asio::ip::tcp::socket socket(io_service_);
        socket.connect(endpoint);
        boost::system::error_code error;
        char data[1024];
        socket.read_some(boost::asio::buffer(data, 1024));
        socket.write_some(boost::asio::buffer(msg.data(), msg.size()), error);
        socket.read_some(boost::asio::buffer(data, 1024));
        return data;
    }catch (const std::exception &e) {
        return e.what();
    }
}
mavlink_client_udp::mavlink_client_udp(
				boost::asio::io_service& io_service,
                const std::string& host,
                const int port
        ) : io_service_(io_service), host_(host), socket_(io_service, udp::endpoint(udp::v4(), port)),
            timer_(io_service_, boost::posix_time::seconds(1))
		{

                endpoint_.address(boost::asio::ip::address::from_string("0.0.0.0"));
                endpoint_.port(port);
                socket_.async_receive_from(
                 boost::asio::buffer(data_, max_length), sender_endpoint_,
                 boost::bind(&mavlink_client_udp::handle_receive_from_udp, this,
                 boost::asio::placeholders::error,
                 boost::asio::placeholders::bytes_transferred));
                 timer_.async_wait( boost::bind(&mavlink_client_udp::clearMessagesCnt, this));
        }
void mavlink_client_udp::write_mavlink_msg(mavlink_message_t &message)
{
    uint8_t buffer[1024];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &message);
    try{
    	socket_.send_to(boost::asio::buffer(buffer, len), endpoint_);
    }
	catch (boost::system::system_error e)
	{
		std::cerr<<e.what();
	}

}
void mavlink_client_udp::handle_receive_from_udp (const boost::system::error_code& error,
        size_t bytes_recvd)
{
	endpoint_ = sender_endpoint_;
	this->handle_receive_from(data_, bytes_recvd);
	socket_.async_receive_from(
	                 boost::asio::buffer(data_, max_length), sender_endpoint_,
	                 boost::bind(&mavlink_client_udp::handle_receive_from_udp, this,
	                 boost::asio::placeholders::error,
	                 boost::asio::placeholders::bytes_transferred));

}

void mavlink_client_udp::handle_receive_from(char* data, size_t bytes_recvd)
{
	mavlink_message_t message;
	mavlink_status_t status;
	for (int i = 0; i < bytes_recvd; i++) {

		if (mavlink_parse_char(1, data[i], &message, &status)) {
			if (message.msgid == MAVLINK_MSG_ID_UNIT_ENCODERS)
			{
				mavlink_unit_encoders_t tt;
				mavlink_msg_unit_encoders_decode(&message,&tt);
				//std::cout << tt.encoder1 << std::endl;
				std::lock_guard<std::mutex> lck(mutex_);
				const double ts_sec = static_cast<double>(tt.ts)/1e6;
				encoder_with_timestamp ts{ts_sec, tt.encoder1 };
				if (!buffer.empty() && ts_sec < buffer.begin()->timestamp){
//					std::cerr << "Try to insert timestamp " << ts_sec << "\n";
//					std::cerr << "This timestamp is before buffer, so the change back in time occured\n";
//					std::cerr << "Invalidate buffer for encoder\n";
					buffer.clear();
					middle_handler_fired_at = 0;
				}
				double whole_sec = floor(ts_sec);
				double frac_sec = ts_sec - whole_sec;
				if (handler_pps_middle && middle_handler_fired_at < whole_sec && frac_sec > 0.5){
					handler_pps_middle(whole_sec, frac_sec);
					middle_handler_fired_at = whole_sec;
				}
				buffer.insert(buffer.end(),ts);
				if (buffer.size() > 500){
					buffer.erase(buffer.begin());
				}
				encoder_msgs_count++;
				const float delta_encoder1 = tt.encoder1 - last_encoder;
				last_encoder = tt.encoder1;
				if (std::abs(delta_encoder1) < 0.01) {
					rotated_radians += delta_encoder1;
				}
			}

			if (message.msgid == MAVLINK_MSG_ID_LD_CAMERA_STROBE)
			{
				mavlink_ld_camera_strobe_t t;
				mavlink_msg_ld_camera_strobe_decode(&message,&t);
				std::cout << "recieved strobe with given token :" << t.request_token <<"\t" <<"\n";
			}
			//std::cout << "MESSAGE ("<< (int) message.msgid <<") FROM " <<  (int) message.sysid <<"\t comp_id: "<< (int) message.compid << std::endl ;

		}

	}

}

unsigned int mavlink_client_udp::getEncoder_msgs_count() const {
	std::lock_guard<std::mutex> lck(mutex_);
	return encoder_msgs_rate;
}


double mavlink_client_udp::getStartOfBufferSecs() const{
	std::lock_guard<std::mutex> lck(mutex_);
	if (buffer.empty()) return -1;
	return buffer.begin()->timestamp;
}
double mavlink_client_udp::getEndOfBufferSecs() const{
	std::lock_guard<std::mutex> lck(mutex_);
	if (buffer.empty()) return -1;
	return buffer.rbegin()->timestamp;
}

double mavlink_client_udp::getEncoder(double ts, encoder_with_timestamp& ret) const{
	double err = std::numeric_limits<double>::max();
	std::lock_guard<std::mutex> lck(mutex_);
	ret.timestamp = ts;
	const auto it= std::upper_bound(buffer.begin(), buffer.end(), ret );
	if (it!= buffer.begin() && it != buffer.end()) {
		ret.timestamp = it->timestamp;
		ret.angle = it->angle;
		err = std::abs(ts - it->timestamp);
	}
	return err;
}

encoder_with_timestamp mavlink_client_udp::getEncoderLast() const{

	std::lock_guard<std::mutex> lck(mutex_);
	if(!buffer.empty()){
		return *buffer.rbegin();
	}
	encoder_with_timestamp ts;
	ts.angle = -1;
	return ts;
}
void mavlink_client_udp::clearMessagesCnt(){

    std::lock_guard<std::mutex> lck(mutex_);
    encoder_msgs_rate = encoder_msgs_count;
    encoder_msgs_count = 0;

    timer_.expires_at(timer_.expires_at() + boost::posix_time::seconds(1));
    timer_.async_wait( boost::bind(&mavlink_client_udp::clearMessagesCnt, this));
}

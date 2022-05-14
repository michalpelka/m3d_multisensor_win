#ifndef MAVLINK_CLIENT_UDP_H
#define MAVLINK_CLIENT_UDP_H
#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <thread>
#include <mutex>
#include <set>nc
#include "mavlink/3dunit/mavlink.h"

constexpr size_t kBufferMaxSize = 15000;

using boost::asio::ip::udp;
struct encoder_with_timestamp{
	double timestamp;
	float angle;

	friend bool operator<(const encoder_with_timestamp &lhs, const encoder_with_timestamp &rhs) {
		return lhs.timestamp < rhs.timestamp;
	}
};
class mavlink_client_udp
{
public:

    mavlink_client_udp(
		boost::asio::io_service& io_service, 
		const std::string& host, 
		const int port = 14450
	);
    void write_mavlink_msg(mavlink_message_t &message);

    std::string sendTCPMessage(const std::string& msg);
    void sendCommand(const std::string& msg);

private:
    udp::socket socket_;
    std::string host_;
    udp::endpoint endpoint_;
    udp::endpoint sender_endpoint_;
	boost::asio::io_service& io_service_;
    enum { max_length = 1024 };
    char data_[max_length];

    void handle_receive_from_udp (const boost::system::error_code& error,
            size_t bytes_recvd);
	void handle_receive_from(char* data, size_t bytes_recvd);

public:
	unsigned int getEncoder_msgs_count() const;
	double getStartOfBufferSecs() const;
	double getEndOfBufferSecs() const;
    double getEncoder(double ts, encoder_with_timestamp& ret) const;

	encoder_with_timestamp getEncoderLast() const;
	void setHandler_pps_change(const std::function<void(double, double)> &_handler_pps_change) {
		handler_pps_middle = _handler_pps_change;
	}

	double getRotatedRadians() {
		std::lock_guard<std::mutex> lck(mutex_);
		return rotated_radians; }

	void clearRotatedRadians() {
		std::lock_guard<std::mutex> lck(mutex_);
		rotated_radians = 0; 
	}

private:
	mutable std::mutex mutex_;
	unsigned int encoder_msgs_count = 0;
    unsigned int encoder_msgs_rate = 0;

    unsigned int middle_handler_fired_at =0;
	std::set<encoder_with_timestamp> buffer;

    boost::asio::deadline_timer timer_;
	double rotated_radians = 0;
	double last_encoder = 0;

private:
	std::function<void(double, double)> handler_pps_middle;
    void clearMessagesCnt();
	

};
#endif /* MAVLINK_CLIENT_H */



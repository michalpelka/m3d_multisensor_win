#pragma once
#include "mongoose.h"
#include "thread"
#include <string>
#include <mutex>
#include <atomic>
#include <chrono>
class robot_client {
public:
	robot_client(const std::string url);
	const std::string getReport() const{
		std::lock_guard<std::mutex> lck(mtx);
		std::string datacp(data);
		return data;
	};
private:
	static void client_fn(struct mg_connection* c, int ev, void* ev_data, void* fn_data);
	void robot_client_listener_thread_worker();
	std::string url;
	std::string data;
	std::thread listner_thread;
	mutable std::mutex mtx;
	std::atomic<bool> done;
	std::chrono::time_point<std::chrono::system_clock> transaction_started;
};
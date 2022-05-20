#pragma once

#include "thread"
#include <string>
#include <mutex>
#include <atomic>
#include <chrono>
#include <Boost-Serial-Port/BoostSerial.h>
class novatel_client {
public:
	novatel_client(const std::string portname, int baudrate);
	std::string getReport() const;
	bool startLogToFile(std::string filename);
	bool stopLogToFile();
	int getRate() {
		return msgs;
	}
private:
	void novatel_client_listener_thread_worker();
	std::string portname;
	int baudrate;
	std::string data;
	std::thread listner_thread;
	mutable std::mutex mtx;
	std::atomic<bool> done;
	std::string log_fn;
	std::unique_ptr<std::ofstream> log_fs;
	std::atomic<int> msgs;
};
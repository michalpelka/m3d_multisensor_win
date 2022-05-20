#include "novatel_client.h"
#include <iostream>
#include <fstream>
novatel_client::novatel_client(const std::string portname, int baudrate) :portname(portname),baudrate(baudrate) {
    listner_thread = std::thread(&novatel_client::novatel_client_listener_thread_worker, this);
}
std::string novatel_client::getReport()	const {
    std::lock_guard<std::mutex> lck(mtx);
    std::string datacp(data);
    return data;
};

bool novatel_client::startLogToFile(std::string filename) {
    std::lock_guard<std::mutex> lck(mtx);
    log_fs = std::make_unique<std::ofstream>(filename);
    return log_fs->good();
}
bool novatel_client::stopLogToFile() {
    std::lock_guard<std::mutex> lck(mtx);
    size_t size = log_fs->tellp();
    log_fs->close();
    log_fs = nullptr;
    if (size > 50) return true;
    return false;
}

void novatel_client::novatel_client_listener_thread_worker() {
    using namespace std::chrono_literals;
    BoostSerial serial;
    serial.open(portname, baudrate);
    serial.setTimeout(1000);
    while (true) {
        std::this_thread::sleep_for(250ms);
        const std::string pose = serial.readStringUntil('\n');
        std::lock_guard<std::mutex> lck(mtx);
        data = pose;
        msgs.fetch_add(1);
        if (log_fs && log_fs->good()) {
            using namespace std::chrono;
            milliseconds ms = duration_cast<milliseconds>(
                system_clock::now().time_since_epoch()
                );
            *log_fs << ms.count() <<":"<< data << std::endl;
        }
    }
}



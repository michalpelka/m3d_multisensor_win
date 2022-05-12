//
// Created by michal on 28.09.2021.
//

#ifndef SRC_HEALTH_SERVER_H
#define SRC_HEALTH_SERVER_H
#include <string>
#include <functional>
#include <map>
struct mg_connection;
namespace health_server {
    static std::function<std::string()> produce_status;
    static std::map<std::string,std::function<std::string(const std::string&)>> trig_handlers;
    void setStatusHandler(std::function<std::string()> hndl);
    void setTriggerHandler(std::function<std::string(const std::string&)> hndl, std::string trigger);

    static const char *s_listen_on = "http://0.0.0.0:8001";
    void fn(struct mg_connection *c, int ev, void *ev_data, void *fn_data);
    void server_worker();
};
namespace file_server {
    const std::string repo{ "data_root/" };

    static const char* s_listen_on = "http://0.0.0.0:8002";
    void fn(struct mg_connection* c, int ev, void* ev_data, void* fn_data);
    void server_worker();
};

#endif //SRC_HEALTH_SERVER_H

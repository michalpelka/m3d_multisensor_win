//
// Created by michal on 28.09.2021.
//

#include "health_server.h"
#include "mongoose.h"
#include <iostream>
#include "web/status.h"
#include "boost/filesystem.hpp"
#include <sstream>
void health_server::setStatusHandler(std::function<std::string()> hndl){
    produce_status = hndl;
}
void health_server::fn(struct mg_connection *c, int ev, void *ev_data, void *fn_data){
    if (ev == MG_EV_HTTP_MSG) {
        struct mg_http_message *hm = (struct mg_http_message *) ev_data;
        for (const auto & th : trig_handlers){
            const std::string &trig = th.first;
            const auto &fun = th.second;
            if (mg_http_match_uri(hm, std::string("/trig/"+trig).c_str()))
            {
                std::string query (hm->query.ptr, hm->query.len);
                if (fun){
                    fun(query);
                }
                mg_http_reply(c, 200, "Access-Control-Allow-Origin: *\n", "ok");
            }
        }
        if (mg_http_match_uri(hm, "/json/status")) {
            // Serve REST response
            std::string data = "{}";
            if (produce_status){
                data = produce_status();
            }
            mg_http_reply(c, 200, "Access-Control-Allow-Origin: *\n", data.c_str());
        }
        else {
            mg_http_reply(c, 200, "Access-Control-Allow-Origin: *\n", static_web_status_page);
        }
    }
    (void) fn_data;
}


void health_server::setTriggerHandler(std::function<void(const std::string&)> hndl, std::string trigger){
    trig_handlers[trigger] = hndl;
}

void health_server::server_worker(){
    struct mg_mgr mgr;                            // Event manager
    mg_mgr_init(&mgr);                            // Initialise event manager
    mg_http_listen(&mgr, s_listen_on, fn, NULL);  // Create HTTP listener
    for (;;) mg_mgr_poll(&mgr, 1000);             // Infinite event loop
    mg_mgr_free(&mgr);
}


void file_server::server_worker() {
    struct mg_mgr mgr;                            // Event manager
    mg_mgr_init(&mgr);                            // Initialise event manager
    mg_http_listen(&mgr, s_listen_on, fn, NULL);  // Create HTTP listener
    for (;;) mg_mgr_poll(&mgr, 1000);             // Infinite event loop
    mg_mgr_free(&mgr);
}

void file_server::fn(struct mg_connection* c, int ev, void* ev_data, void* fn_data) {
    if (ev == MG_EV_HTTP_MSG) {
        struct mg_http_message* hm = (struct mg_http_message*)ev_data;
        struct mg_http_serve_opts opts;
        
        if (mg_http_match_uri(hm, "/list_files")) {
            using namespace std;
            using namespace boost::filesystem;
            path p("data_root");
            directory_iterator end_itr;
            std::stringstream ss;
            for (directory_iterator itr(p); itr != end_itr; ++itr)
            {
                // If it's not a directory, list it. If you want to list directories too, just remove this check.
                if (is_regular_file(itr->path())) {
                    // assign current file name to current_file and echo it out to the console.
                    string current_file = itr->path().string();
                    ss << current_file << endl;
                }
            }
            mg_http_reply(c, 200, "Access-Control-Allow-Origin: *\n", ss.str().c_str());
        }else

        if (mg_http_match_uri(hm, "/index")) {
            using namespace std;
            ifstream file ("index.htm", 'r');
            std::stringstream buffer;
            buffer << file.rdbuf();
            mg_http_reply(c, 200, "Access-Control-Allow-Origin: *\n", buffer.str().c_str());
        }
        else
        {
            memset(&opts, 0, sizeof(opts));
            opts.root_dir = "data_root";
            mg_http_serve_dir(c, hm, &opts);
        }

    }
    (void)fn_data;
}
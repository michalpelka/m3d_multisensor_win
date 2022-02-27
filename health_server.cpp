//
// Created by michal on 28.09.2021.
//

#include "health_server.h"
#include "mongoose.h"
#include <iostream>
#include "web/status.h"

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

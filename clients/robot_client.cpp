#include "robot_client.h"
#include <iostream>
robot_client::robot_client(const std::string url) :url(url) { 
    listner_thread = std::thread(&robot_client::robot_client_listener_thread_worker, this);
}
std::string getReport();
void robot_client::robot_client_listener_thread_worker() {
    using namespace std::chrono_literals;
    while (true) {
        done = false;
        const char* s_url = url.c_str();
        struct mg_mgr mgr;
        mg_mgr_init(&mgr);                        // Init manager
        mg_http_connect(&mgr, s_url, robot_client::client_fn, (void*)this);  // Create client connection
        while(!done) mg_mgr_poll(&mgr, 1000);         // Event loop
        mg_mgr_free(&mgr);                        // Cleanup
        std::this_thread::sleep_for(500ms);
    }
}

void robot_client::client_fn(struct mg_connection* c, int ev, void* ev_data, void* fn_data) {
	robot_client* this_ptr = reinterpret_cast<robot_client*>(fn_data);
    const char* s_url = this_ptr->url.c_str();
    if (ev == MG_EV_CONNECT) {
        struct mg_str host = mg_url_host(s_url);
        // Send request
        mg_printf(c,
            "GET %s HTTP/1.0\r\n"
            "Host: %.*s\r\n"
            "\r\n",
            mg_url_uri(s_url), (int)host.len, host.ptr);
    } if (ev == MG_EV_HTTP_MSG) {
        struct mg_http_message* hm = (struct mg_http_message*)ev_data;
        /*printf("%.*s", (int)hm->message.len, hm->message.ptr);*/
        std::string body_str(hm->body.ptr, hm->body.len);
        this_ptr->done = true;
        std::lock_guard<std::mutex> lck(this_ptr->mtx);
        std::swap(this_ptr->data,body_str);
    }
    else if (ev == MG_EV_ERROR) {
        this_ptr->done = true;
    }
}

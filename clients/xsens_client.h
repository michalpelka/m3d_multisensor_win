//
// Created by michal on 24.09.2021.
//

#ifndef SRC_XSENS_CLIENT_H
#define SRC_XSENS_CLIENT_H
#include <memory>
#include <mavlink_client_udp.h>
#include <Eigen/Dense>
class XsControl;
class XsDataPacket;
class XsDevice;
class XsCallback;
class CallbackHandler;
class xsens_client {
private:
    XsControl* control{nullptr};
    std::shared_ptr<CallbackHandler> callback{nullptr};
    std::shared_ptr<mavlink_client_udp> client;
    std::function<void(double, Eigen::Vector3d, Eigen::Vector3d, Eigen::Quaterniond)> data_handler;
public:
    xsens_client( const std::shared_ptr<mavlink_client_udp>& client);
    bool OpenImu();
    void setHandler(std::function<void(double, Eigen::Vector3d, Eigen::Vector3d, Eigen::Quaterniond)> handler);

};


#endif //SRC_XSENS_CLIENT_H

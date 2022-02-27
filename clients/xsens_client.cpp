//
// Created by michal on 24.09.2021.
//

#include "xsens_client.h"
#include <xscontroller/xscontrol_def.h>
#include <xscontroller/xsdevice_def.h>
#include <xscontroller/xsscanner.h>
#include <xstypes/xsoutputconfigurationarray.h>
#include <xstypes/xsdatapacket.h>
#include <xstypes/xstime.h>
#include <xscommon/xsens_mutex.h>

#include <iostream>
#include <Eigen/Dense>
Journaller* gJournal = 0;
class CallbackHandler : public XsCallback
{
    std::shared_ptr<mavlink_client_udp> client;

public:
    CallbackHandler(const std::shared_ptr<mavlink_client_udp>& client):
            client(client)
    {
    }

    virtual ~CallbackHandler() throw()
    {
    }
    std::function<void(double, Eigen::Vector3d, Eigen::Vector3d, Eigen::Quaterniond)> data_handler;
protected:
    void onLiveDataAvailable(XsDevice*, const XsDataPacket* packet) override
    {
        const double ts_sec = client->getEncoderLast().timestamp;
        const double whole_sec = floor(ts_sec);
        const double frac_sec = ts_sec - whole_sec;
        if (frac_sec > 0.5 && last_fired_pps_check !=whole_sec ){
            last_fired_pps_check = whole_sec;
            if (whole_sec!=current_whole_pps)
            {
                current_whole_pps=whole_sec;
                std::cerr<<"[xsens] force pps update "<< whole_sec << ", " <<current_whole_pps << std::endl;
            }

        }
        bool sync_in = packet->status() & 0x200000;
        if (sync_in ) {
            lastXsensTick =  packet->sampleTime64();
            current_whole_pps++;
        }
        double xsens_timestamp = (1e-4)*(packet->sampleTime64() - lastXsensTick) + current_whole_pps;
        //std::cout <<std::to_string( xsens_timestamp )<< std::endl;
        if (data_handler)
        {
            if (packet->containsOrientation() ) {
                const XsQuaternion xsQuaternion = packet->orientationQuaternion();
                const XsVector xsRateOfTurn = packet->calibratedGyroscopeData();
                const XsVector xsAcceleration = packet->calibratedAcceleration();

                const Eigen::Quaterniond quaterniond{xsQuaternion.w(), xsQuaternion.x(), xsQuaternion.y(), xsQuaternion.z()};
                const Eigen::Vector3d rateOfTurn(xsRateOfTurn[0],xsRateOfTurn[1],xsRateOfTurn[2]);
                const Eigen::Vector3d acceleration(xsAcceleration[0],xsAcceleration[1],xsAcceleration[2]);
                data_handler(xsens_timestamp, acceleration, rateOfTurn,quaterniond);

            }
        }
        //std::cout << " -> "<< packet->packetCounter() <<" : " <<sync_in << " : " << (1e-4)*(packet->sampleTime64() - lastTick) <<std::endl;
    }
    uint64_t lastXsensTick = 0;
    double current_whole_pps =0;
    double last_fired_pps_check =0;
};

xsens_client::xsens_client( const std::shared_ptr<mavlink_client_udp>& client ):
client(client){
    callback = nullptr;
}

bool xsens_client::OpenImu(){

    if (control == nullptr)
    {
        control = XsControl::construct();
    }
    assert(control != 0);
    auto handleError = [&](std::string errorString)
    {
        control->destruct();
        std::cout << errorString << std::endl;
        return false;
    };
    XsPortInfoArray portInfoArray = XsScanner::scanPorts();
    XsPortInfo mtPort;
    for (auto const &portInfo : portInfoArray)
    {
        if (portInfo.deviceId().isMti() || portInfo.deviceId().isMtig())
        {
            mtPort = portInfo;
            break;
        }
    }
    std::cout << "Opening port..." << std::endl;
    if (!control->openPort(mtPort.portName().toStdString(), mtPort.baudrate()))
        return handleError("Could not open port. Aborting.");

    // Get the device object
    XsDevice* device = control->device(mtPort.deviceId());
    assert(device != 0);

    std::cout << "Device: " << device->productCode().toStdString() << ", with ID: " << device->deviceId().toString() << " opened." << std::endl;

    // Create and attach callback handler to device
    callback = std::make_shared<CallbackHandler>(client);
    device->addCallbackHandler(callback.get());
    // Put the device into configuration mode before configuring the device
    std::cout << "Putting device into configuration mode..." << std::endl;
    if (!device->gotoConfig())
        return handleError("Could not put device into configuration mode. Aborting.");

    std::cout << "Configuring the device..." << std::endl;

    // Important for Public XDA!
    // Call this function if you want to record a mtb file:
    device->readEmtsAndDeviceConfiguration();

    XsOutputConfigurationArray configArray;
    configArray.push_back(XsOutputConfiguration(XDI_PacketCounter, 200));
    configArray.push_back(XsOutputConfiguration(XDI_SampleTimeFine, 200));
    configArray.push_back(XsOutputConfiguration(XDI_SampleTimeCoarse, 200));
    configArray.push_back(XsOutputConfiguration(XDI_Acceleration, 200));
    configArray.push_back(XsOutputConfiguration(XDI_RateOfTurn, 200));
    configArray.push_back(XsOutputConfiguration(XDI_StatusWord, 200));
    configArray.push_back(XsOutputConfiguration(XDI_Quaternion, 200));

    if (!device->setOutputConfiguration(configArray))
        return handleError("Could not configure MTi device. Aborting.");

    std::cout << "Putting device into measurement mode..." << std::endl;
    if (!device->gotoMeasurement())
        return handleError("Could not put device into measurement mode. Aborting.");
    return true;
}
void xsens_client::setHandler(std::function<void(double, Eigen::Vector3d, Eigen::Vector3d, Eigen::Quaterniond)> handler)
{
    if (callback) {
        callback->data_handler = handler;
    }
}
//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "lds_lidar.h"
#include "mavlink_client_udp.h"

#include <stdio.h>
#include <string.h>
#include <thread>
#include <memory>


/** Const varible ------------------------------------------------------------------------------- */
/** User add broadcast code here */
static const char* local_broadcast_code_list[] = {
  "000000000000001",
};

/** For callback use only */
LdsLidar* g_lidars = nullptr;

/** Lds lidar function ---------------------------------------------------------------------------*/
LdsLidar::LdsLidar() {
  auto_connect_mode_ = true;
  whitelist_count_   = 0;
  is_initialized_    = false;

  lidar_count_       = 0;
  memset(broadcast_code_whitelist_, 0, sizeof(broadcast_code_whitelist_));

  memset(lidars_, 0, sizeof(lidars_));
  for (uint32_t i=0; i<kMaxLidarCount; i++) {
    lidars_[i].handle = kMaxLidarCount; 
    /** Unallocated state */
    lidars_[i].connect_state = kConnectStateOff;
  }
  for (int i =0; i < kMaxLidarCount; i++){
     global_timestamp[i] = -1;
  }
}

LdsLidar::~LdsLidar() {
}

int LdsLidar::InitLdsLidar(std::vector<std::string>& broadcast_code_strs) {

  if (is_initialized_) {
    printf("[livox] LiDAR data source is already inited!\n");
    return -1;
  }

  if (!Init()) {
    Uninit();
    printf("[livox] Livox-SDK init fail!\n");
    return -1;
  }

  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  printf("[livox] Livox SDK version %d.%d.%d\n", _sdkversion.major, _sdkversion.minor, _sdkversion.patch);

  SetBroadcastCallback(LdsLidar::OnDeviceBroadcast);
  SetDeviceStateUpdateCallback(LdsLidar::OnDeviceChange);

  /** Add commandline input broadcast code */
  for (auto input_str : broadcast_code_strs) {
    LdsLidar::AddBroadcastCodeToWhitelist(input_str.c_str());
  }

  /** Add local broadcast code */
  LdsLidar::AddLocalBroadcastCode();

  if (whitelist_count_) {
    LdsLidar::DisableAutoConnectMode();
    printf("[livox] Disable auto connect mode!\n");

    printf("[livox] List all broadcast code in whiltelist:\n");
    for (uint32_t i=0; i<whitelist_count_; i++) {
      printf("[livox] %s\n", broadcast_code_whitelist_[i]);
    }
  } else {
    LdsLidar::EnableAutoConnectMode();
    printf("[livox] No broadcast code was added to whitelist, swith to automatic connection mode!\n");
  }

  /** Start livox sdk to receive lidar data */
  if (!Start()) {
    Uninit();
    printf("[livox] Livox-SDK init fail!\n");
    return -1;
  }

  /** Add here, only for callback use */
  if (g_lidars == nullptr) {
    g_lidars = this;
  }
  is_initialized_= true;
  printf("Livox-SDK init success!\n");

  return 0;
}


int LdsLidar::DeInitLdsLidar(void) {

  if (!is_initialized_) {
    printf("[livox] LiDAR data source is not exit");
    return -1;
  }

  Uninit();
  printf("Livox SDK Deinit completely!\n");

  return 0;
}


/** Static function in LdsLidar for callback or event process ------------------------------------*/

/** Receiving point cloud data from Livox LiDAR. */
void LdsLidar::GetLidarDataCb(uint8_t handle, LivoxEthPacket *data,
                              uint32_t data_num, void *client_data) {
  using namespace std;

  LdsLidar* lidar_this = static_cast<LdsLidar *>(client_data);
  LivoxEthPacket* eth_packet = data;

  if (!data || !data_num || (handle >= kMaxLidarCount)) {
    return;
  }

  if (eth_packet) {
    lidar_this->data_recveive_count_[handle] ++;

    uint64_t cur_timestamp = *((uint64_t *)(data->timestamp));

    double ts_sec = cur_timestamp /1e9;
    lidar_this->lidars_[handle].last_timestamp = ts_sec;
    if (data->data_type == kCartesian || data->data_type == kExtendCartesian)
    {
        double global_timestamp_t = lidar_this->global_timestamp[handle];
        if (ts_sec > 1.0)
        {
            std::cout << "[Strange PPS on livox " << int(handle) << "]\t" << ts_sec << std::endl;
        }
        if (lidar_this->last_second_[handle] > 0.9 && ts_sec < 0.1)
        {
            global_timestamp_t = global_timestamp_t + 1;
            lidar_this->global_timestamp[handle].store(global_timestamp_t);
            std::cout << "PPS on tunrover on Livox " << int(handle) << std::endl;
            //std::cout <<  "PPS on livox " << int(handle) <<"\t" << lidar_this->last_second_[handle] << " " << ts_sec << std::endl;
        }
        lidar_this->last_second_[handle] = ts_sec;

        if (data->data_type == kCartesian) {
            //printf("[livox] receive packet count %d %d\n", handle, lidar_this->data_recveive_count_[handle]);
            livox_synch::packet new_packet;
            new_packet.livox_handle = handle;
            new_packet.timestamp = global_timestamp_t + ts_sec;
            new_packet.datachunk.resize(data_num);
            for (int i = 0; i < data_num; i++) {
                LivoxRawPoint* p_point_data_f = (LivoxRawPoint*)data->data;
                LivoxRawPoint* p_point_data = p_point_data_f + i;
                new_packet.datachunk[i].x = p_point_data->x * 0.001;
                new_packet.datachunk[i].y = p_point_data->y * 0.001;
                new_packet.datachunk[i].z = p_point_data->z * 0.001;
                new_packet.datachunk[i].intensity = p_point_data->reflectivity;
            }
            std::lock_guard<std::mutex> lck(lidar_this->m_buffer[handle].buffer_lock);
            lidar_this->m_buffer[handle].buffer.push_back(new_packet);
            if (lidar_this->m_buffer[handle].buffer.size() > 1000) {
                //std::cout <<" 1: "<< std::to_string(lidar_this->m_buffer.buffer.front().timestamp) << "\t" << std::to_string(lidar_this->m_buffer.buffer.back().timestamp) << std::endl;
                lidar_this->m_buffer[handle].buffer.pop_front();
            }
        }
        if (data->data_type == kExtendCartesian) {
            //printf("[livox] receive packet count %d %d\n", handle, lidar_this->data_recveive_count_[handle]);
            livox_synch::packet new_packet;
            new_packet.livox_handle = handle;
            new_packet.timestamp = global_timestamp_t + ts_sec;
            new_packet.datachunk.resize(data_num);
            for (int i = 0; i < data_num; i++) {
                LivoxExtendRawPoint* p_point_data_f = (LivoxExtendRawPoint*)data->data;
                LivoxExtendRawPoint* p_point_data = p_point_data_f + i;
                new_packet.datachunk[i].x = p_point_data->x * 0.001;
                new_packet.datachunk[i].y = p_point_data->y * 0.001;
                new_packet.datachunk[i].z = p_point_data->z * 0.001;
                new_packet.datachunk[i].intensity = p_point_data->reflectivity;
            }
            std::lock_guard<std::mutex> lck(lidar_this->m_buffer[handle].buffer_lock);
            lidar_this->m_buffer[handle].buffer.push_back(new_packet);
            if (lidar_this->m_buffer[handle].buffer.size() > 1000) {
                //std::cout <<" 1: "<< std::to_string(lidar_this->m_buffer.buffer.front().timestamp) << "\t" << std::to_string(lidar_this->m_buffer.buffer.back().timestamp) << std::endl;
                lidar_this->m_buffer[handle].buffer.pop_front();
            }
        }
    }

      /** Parsing the timestamp and the point cloud data. */

//      if(data ->data_type == kCartesian) {
//        LivoxRawPoint *p_point_data = (LivoxRawPoint *)data->data;
//      }else if ( data ->data_type == kSpherical) {
//        LivoxSpherPoint *p_point_data = (LivoxSpherPoint *)data->data;
//      }else if ( data ->data_type == kExtendCartesian) {
//        LivoxExtendRawPoint *p_point_data = (LivoxExtendRawPoint *)data->data;
//      }else if ( data ->data_type == kExtendSpherical) {
//        LivoxExtendSpherPoint *p_point_data = (LivoxExtendSpherPoint *)data->data;
//      }else if ( data ->data_type == kDualExtendCartesian) {
//        LivoxDualExtendRawPoint *p_point_data = (LivoxDualExtendRawPoint *)data->data;
//      }else if ( data ->data_type == kDualExtendSpherical) {
//        LivoxDualExtendSpherPoint *p_point_data = (LivoxDualExtendSpherPoint *)data->data;
//      }else if ( data ->data_type == kImu) {
//        LivoxImuPoint *p_point_data = (LivoxImuPoint *)data->data;
//      }else if ( data ->data_type == kTripleExtendCartesian) {
//        LivoxTripleExtendRawPoint *p_point_data = (LivoxTripleExtendRawPoint *)data->data;
//      }else if ( data ->data_type == kTripleExtendSpherical) {
//        LivoxTripleExtendSpherPoint *p_point_data = (LivoxTripleExtendSpherPoint *)data->data;
//      }

  }
}

void LdsLidar::OnDeviceBroadcast(const BroadcastDeviceInfo *info) {
  if (info == nullptr) {
    return;
  }

  if (info->dev_type == kDeviceTypeHub) {
    printf("[livox] In lidar mode, couldn't connect a hub : %s\n", info->broadcast_code);
    return;
  }

  if (g_lidars->IsAutoConnectMode()) {
    printf("[livox] In automatic connection mode, will connect %s\n", info->broadcast_code);
  } else {
    if (!g_lidars->FindInWhitelist(info->broadcast_code)) {
      printf("[livox] Not in the whitelist, please add %s to if want to connect!\n",\
             info->broadcast_code);
      return;
    }
  }

  livox_status result = kStatusFailure;
  uint8_t handle = 0;
  result = AddLidarToConnect(info->broadcast_code, &handle);
  if (result == kStatusSuccess && handle < kMaxLidarCount) {
    SetDataCallback(handle, LdsLidar::GetLidarDataCb, (void *)g_lidars);
    LidarDevice* p_lidar = &(g_lidars->lidars_[handle]);
    g_lidars->lidar_count_++;
    p_lidar->handle = handle;
    p_lidar->connect_state = kConnectStateOff;
    p_lidar->config.enable_fan = true;
    p_lidar->config.return_mode = kStrongestReturn;
    p_lidar->config.coordinate = kCoordinateCartesian;
    p_lidar->config.imu_rate = kImuFreq200Hz;
  } else {
    printf("Add lidar to connect is failed : %d %d \n", result, handle);
  }
}

/** Callback function of changing of device state. */
void LdsLidar::OnDeviceChange(const DeviceInfo *info, DeviceEvent type) {
  if (info == nullptr) {
    return;
  }

  uint8_t handle = info->handle;
  if (handle >= kMaxLidarCount) {
    return;
  }

  LidarDevice* p_lidar = &(g_lidars->lidars_[handle]);
  if (type == kEventConnect) {
    QueryDeviceInformation(handle, DeviceInformationCb, g_lidars);
    if (p_lidar->connect_state == kConnectStateOff) {
      p_lidar->connect_state = kConnectStateOn;
      p_lidar->info = *info;
    }
    printf("[livox][WARNING] Lidar sn: [%s] Connect!!!\n", info->broadcast_code);
  } else if (type == kEventDisconnect) {
    p_lidar->connect_state = kConnectStateOff;
    printf("[livox][WARNING] Lidar sn: [%s] Disconnect!!!\n", info->broadcast_code);
  } else if (type == kEventStateChange) {
    p_lidar->info = *info;
    printf("[livox][WARNING] Lidar sn: [%s] StateChange!!!\n", info->broadcast_code);
  }

  if (p_lidar->connect_state == kConnectStateOn) {
    printf("[livox]Device Working State %d\n", p_lidar->info.state);
    if (p_lidar->info.state == kLidarStateInit) {
      printf("[livox]Device State Change Progress %u\n", p_lidar->info.status.progress);
    } else {
      printf("[livox]Device State Error Code 0X%08x\n", p_lidar->info.status.status_code.error_code);
    }
    printf("[livox]Device feature %d\n", p_lidar->info.feature);
    SetErrorMessageCallback(handle, LdsLidar::LidarErrorStatusCb);

    /** Config lidar parameter */
    if (p_lidar->info.state == kLidarStateNormal) {
      if (p_lidar->config.coordinate != 0) {
        SetSphericalCoordinate(handle, LdsLidar::SetCoordinateCb, g_lidars);
      } else {
        SetCartesianCoordinate(handle, LdsLidar::SetCoordinateCb, g_lidars);
      }
      p_lidar->config.set_bits |= kConfigCoordinate;

      if (kDeviceTypeLidarMid40 != info->type) {
        LidarSetPointCloudReturnMode(handle, (PointCloudReturnMode)(p_lidar->config.return_mode),\
                                     LdsLidar::SetPointCloudReturnModeCb, g_lidars);
        p_lidar->config.set_bits |= kConfigReturnMode;
      }

      if (kDeviceTypeLidarMid40 != info->type && kDeviceTypeLidarMid70 != info->type) {
        LidarSetImuPushFrequency(handle, (ImuFreq)(p_lidar->config.imu_rate),\
                                 LdsLidar::SetImuRatePushFrequencyCb, g_lidars);
        p_lidar->config.set_bits |= kConfigImuRate;
      }

      p_lidar->connect_state = kConnectStateConfig;
    }
  }
}

/** Query the firmware version of Livox LiDAR. */
void LdsLidar::DeviceInformationCb(livox_status status, uint8_t handle, \
                         DeviceInformationResponse *ack, void *client_data) {
  if (status != kStatusSuccess) {
    printf("[livox]Device Query Informations Failed : %d\n", status);
  }
  if (ack) {
    printf("[livox]firm ver: %d.%d.%d.%d\n",
           ack->firmware_version[0],
           ack->firmware_version[1],
           ack->firmware_version[2],
           ack->firmware_version[3]);
  }
}

/** Callback function of Lidar error message. */
void LdsLidar::LidarErrorStatusCb(livox_status status, uint8_t handle, ErrorMessage *message) {
  static uint32_t error_message_count = 0;
  if (message != NULL) {
    ++error_message_count;
    if (0 == (error_message_count % 100)) {
      printf("[livox]handle: %u\n", handle);
      printf("[livox]temp_status : %u\n", message->lidar_error_code.temp_status);
      printf("[livox]volt_status : %u\n", message->lidar_error_code.volt_status);
      printf("[livox]motor_status : %u\n", message->lidar_error_code.motor_status);
      printf("[livox]dirty_warn : %u\n", message->lidar_error_code.dirty_warn);
      printf("[livox]firmware_err : %u\n", message->lidar_error_code.firmware_err);
      printf("[livox]pps_status : %u\n", message->lidar_error_code.device_status);
      printf("[livox]fan_status : %u\n", message->lidar_error_code.fan_status);
      printf("[livox]self_heating : %u\n", message->lidar_error_code.self_heating);
      printf("[livox]ptp_status : %u\n", message->lidar_error_code.ptp_status);
      printf("[livox]time_sync_status : %u\n", message->lidar_error_code.time_sync_status);
      printf("[livox]system_status : %u\n", message->lidar_error_code.system_status);
    }
  }
}

void LdsLidar::ControlFanCb(livox_status status, uint8_t handle, \
                           uint8_t response, void *client_data) {

}

void LdsLidar::SetPointCloudReturnModeCb(livox_status status, uint8_t handle, \
                                         uint8_t response, void *client_data) {
  LdsLidar* lds_lidar = static_cast<LdsLidar *>(client_data);

  if (handle >= kMaxLidarCount) {
    return;
  }
  LidarDevice* p_lidar = &(lds_lidar->lidars_[handle]);

  if (status == kStatusSuccess) {
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigReturnMode));
    printf("Set return mode success!\n");

    if (!p_lidar->config.set_bits) {
      LidarStartSampling(handle, LdsLidar::StartSampleCb, lds_lidar);
      p_lidar->connect_state = kConnectStateSampling;
    }
  } else {
    LidarSetPointCloudReturnMode(handle, (PointCloudReturnMode)(p_lidar->config.return_mode),\
                                 LdsLidar::SetPointCloudReturnModeCb, lds_lidar);
    printf("[livox]Set return mode fail, try again!\n");
  }
}

void LdsLidar::SetCoordinateCb(livox_status status, uint8_t handle, \
                               uint8_t response, void *client_data) {
  LdsLidar* lds_lidar = static_cast<LdsLidar *>(client_data);

  if (handle >= kMaxLidarCount) {
    return;
  }
  LidarDevice* p_lidar = &(lds_lidar->lidars_[handle]);

  if (status == kStatusSuccess) {
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigCoordinate));
    printf("[livox]Set coordinate success!\n");

    if (!p_lidar->config.set_bits) {
       LidarStartSampling(handle, LdsLidar::StartSampleCb, lds_lidar);
       p_lidar->connect_state = kConnectStateSampling;
    }
  } else {
    if (p_lidar->config.coordinate != 0) {
      SetSphericalCoordinate(handle, LdsLidar::SetCoordinateCb, lds_lidar);
    } else {
      SetCartesianCoordinate(handle, LdsLidar::SetCoordinateCb, lds_lidar);
    }

    printf("[livox]Set coordinate fail, try again!\n");
  }
}

void LdsLidar::SetImuRatePushFrequencyCb(livox_status status, uint8_t handle, \
                                         uint8_t response, void *client_data) {
  LdsLidar* lds_lidar = static_cast<LdsLidar *>(client_data);

  if (handle >= kMaxLidarCount) {
    return;
  }
  LidarDevice* p_lidar = &(lds_lidar->lidars_[handle]);

  if (status == kStatusSuccess) {
    p_lidar->config.set_bits &= ~((uint32_t)(kConfigImuRate));
    printf("Set imu rate success!\n");

    if (!p_lidar->config.set_bits) {
      LidarStartSampling(handle, LdsLidar::StartSampleCb, lds_lidar);
      p_lidar->connect_state = kConnectStateSampling;
    }

  } else {
    LidarSetImuPushFrequency(handle, (ImuFreq)(p_lidar->config.imu_rate),\
                             LdsLidar::SetImuRatePushFrequencyCb, lds_lidar);
    printf("Set imu rate fail, try again!\n");
  }
}

/** Callback function of starting sampling. */
void LdsLidar::StartSampleCb(livox_status status, uint8_t handle, \
                             uint8_t response, void *client_data) {
  LdsLidar* lds_lidar = static_cast<LdsLidar *>(client_data);

  if (handle >= kMaxLidarCount) {
    return;
  }

  LidarDevice* p_lidar = &(lds_lidar->lidars_[handle]);
  if (status == kStatusSuccess) {
    if (response != 0) {
      p_lidar->connect_state = kConnectStateOn;
      printf("Lidar start sample fail : state[%d] handle[%d] res[%d]\n", \
             status, handle, response);
    } else {
      printf("Lidar start sample success\n");
   }
  } else if (status == kStatusTimeout) {
    p_lidar->connect_state = kConnectStateOn;
    printf("Lidar start sample timeout : state[%d] handle[%d] res[%d]\n", \
           status, handle, response);
  }
}

/** Callback function of stopping sampling. */
void LdsLidar::StopSampleCb(livox_status status, uint8_t handle, \
                            uint8_t response, void *client_data) {
}

/** Add broadcast code to whitelist */
int LdsLidar::AddBroadcastCodeToWhitelist(const char* bd_code) {
  if (!bd_code || (strlen(bd_code) > kBroadcastCodeSize) || \
      (whitelist_count_ >= kMaxLidarCount)) {
    return -1;
  }

  if (LdsLidar::FindInWhitelist(bd_code)) {
    printf("%s is alrealy exist!\n", bd_code);
    return -1;
  }

  strcpy(broadcast_code_whitelist_[whitelist_count_], bd_code);
  ++whitelist_count_;

  return 0;
}

void LdsLidar::AddLocalBroadcastCode(void) {
  for (size_t i=0; i<sizeof(local_broadcast_code_list)/sizeof(intptr_t); ++i) {
    std::string invalid_bd = "000000000";
    printf("Local broadcast code : %s\n", local_broadcast_code_list[i]);
    if ((kBroadcastCodeSize == strlen(local_broadcast_code_list[i]) + 1) && \
        (nullptr == strstr(local_broadcast_code_list[i], invalid_bd.c_str()))) {
      LdsLidar::AddBroadcastCodeToWhitelist(local_broadcast_code_list[i]);
    } else {
      printf("Invalid local broadcast code : %s\n", local_broadcast_code_list[i]);
    }
  }
}

bool LdsLidar::FindInWhitelist(const char* bd_code) {
  if (!bd_code) {
    return false;
  }

  for (uint32_t i=0; i<whitelist_count_; i++) {
    if (strncmp(bd_code, broadcast_code_whitelist_[i], kBroadcastCodeSize) == 0) {
      return true;
    }
  }

  return false;
}


void LdsLidar::setClient(std::shared_ptr<mavlink_client_udp> &client) {
    LdsLidar::client = client;
    const auto handler_pps_middle = [&](double whole, double frac){
        //for (auto &p : livox_reciver_buffers) {
        for (int i = 0; i < kMaxLidarCount; i++)
        {
            if (lidars_[i].handle == kMaxLidarCount) break;
            if (whole != this->global_timestamp[i]) {
                std::cerr << "[pps mismatch livox  " << std::to_string(i) << "] : " << lidars_[i].last_timestamp << "  " << whole << "." << frac << std::endl;
            }
            this->global_timestamp[i].store(whole);
        }
        // }
    };
    client->setHandler_pps_change(handler_pps_middle);
}

double livox_synch::getLastTs()
{
    std::lock_guard<std::mutex> lck(buffer_lock);
    if (buffer.empty()) return -1;
    return buffer.back().timestamp;
}
std::deque<livox_synch::packet> livox_synch::popBack(int toKeep) {
    std::deque<packet> packet;

    std::lock_guard<std::mutex> lck(buffer_lock);
    if (buffer.size() < toKeep + 1) return packet;
    const auto lidars_ptr = LdsLidar::GetInstance().getLidars();

    while (buffer.size() > toKeep)
    {
        packet.push_front(buffer.front());
        buffer.pop_front();
    }
    //packet.insert(packet.begin(), std::make_move_iterator(start), std::make_move_iterator(stop));
    //std::cout << "buffer " << buffer.size() << std::endl;
    return packet;
}
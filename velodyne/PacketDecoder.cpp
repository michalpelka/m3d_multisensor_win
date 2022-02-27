// Velodyne HDL Packet Decoder
// Nick Rypkema (rypkema@mit.edu), MIT 2017
// shared library to decode a Velodyne packet - largely repurposed from vtkVelodyneHDLReader
#define _USE_MATH_DEFINES
#include <cmath>
#include <stdint.h>
#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>

#include "PacketDecoder.h"

PacketDecoder::PacketDecoder()
{
  InitTables();
  LoadHDL32Corrections();
}

PacketDecoder::~PacketDecoder()
{

}


PacketDecoder::HDLFrame PacketDecoder::DecodePacket(std::string* data, unsigned int* data_length)
{
  const unsigned char* data_char = reinterpret_cast<const unsigned char*>(data->c_str());
  return ProcessHDLPacket(const_cast<unsigned char*>(data_char), *data_length);
}

PacketDecoder::HDLFrame PacketDecoder::ProcessHDLPacket(unsigned char *data, unsigned int data_length)
{
  PacketDecoder::HDLFrame frame;
  if (data_length != 1206) {
    std::cout << "PacketDecoder: Warning, data packet is not 1206 bytes" << std::endl;
    return frame ;
  }

  HDLDataPacket* dataPacket = reinterpret_cast<HDLDataPacket *>(data);

  for (int i = 0; i < HDL_FIRING_PER_PKT; ++i) {
    HDLFiringData firingData = dataPacket->firingData[i];
    int offset = (firingData.blockIdentifier == BLOCK_0_TO_31) ? 0 : 32;

//    if (firingData.rotationalPosition < _last_azimuth) {
//      SplitFrame();
//    }

    _last_azimuth = firingData.rotationalPosition;

    for (int j = 0; j < HDL_LASER_PER_FIRING; j++) {
      unsigned char laserId = static_cast<unsigned char>(j + offset);
      if (firingData.laserReturns[j].distance != 0.0) {
        PushFiringData(laserId, firingData.rotationalPosition, dataPacket->gpsTimestamp, firingData.laserReturns[j], laser_corrections_[j + offset],frame);
      }
    }
  }
  return frame;
}

void PacketDecoder::PushFiringData(unsigned char laserId, unsigned short azimuth, unsigned int timestamp,
                                   HDLLaserReturn laserReturn, HDLLaserCorrection correction, HDLFrame &frame)
{
  double cosAzimuth, sinAzimuth;
  if (correction.azimuthCorrection == 0) {
    cosAzimuth = cos_lookup_table_[azimuth];
    sinAzimuth = sin_lookup_table_[azimuth];
  } else {
    double azimuthInRadians = HDL_Grabber_toRadians((static_cast<double> (azimuth) / 100.0) - correction.azimuthCorrection);
    cosAzimuth = std::cos(azimuthInRadians);
    sinAzimuth = std::sin(azimuthInRadians);
  }

  double distanceM = laserReturn.distance * 0.002 + correction.distanceCorrection;
  double xyDistance = distanceM * correction.cosVertCorrection - correction.sinVertOffsetCorrection;

  double x = (xyDistance * sinAzimuth - correction.horizontalOffsetCorrection * cosAzimuth);
  double y = (xyDistance * cosAzimuth + correction.horizontalOffsetCorrection * sinAzimuth);
  double z = (distanceM * correction.sinVertCorrection + correction.cosVertOffsetCorrection);
  unsigned char intensity = laserReturn.intensity;

  frame.x.push_back(x);
  frame.y.push_back(y);
  frame.z.push_back(z);
  frame.intensity.push_back(intensity);
  frame.laser_id.push_back(laserId);
  frame.azimuth.push_back(azimuth);
  frame.distance.push_back(distanceM);
  frame.ms_from_top_of_hour.push_back(timestamp);
}

void PacketDecoder::SetCorrectionsFile(const std::string& corrections_file)
{
  if (corrections_file == _corrections_file) {
    return;
  }

  if (corrections_file.length()) {
    LoadCorrectionsFile(corrections_file);
  } else {
    LoadHDL32Corrections();
  }
  _corrections_file = corrections_file;
}

void PacketDecoder::InitTables()
{
  if (cos_lookup_table_ == NULL && sin_lookup_table_ == NULL) {
    cos_lookup_table_ = static_cast<double *> (malloc (HDL_NUM_ROT_ANGLES * sizeof (*cos_lookup_table_)));
    sin_lookup_table_ = static_cast<double *> (malloc (HDL_NUM_ROT_ANGLES * sizeof (*sin_lookup_table_)));
    for (unsigned int i = 0; i < HDL_NUM_ROT_ANGLES; i++) {
      double rad = HDL_Grabber_toRadians(i / 100.0);
      cos_lookup_table_[i] = std::cos(rad);
      sin_lookup_table_[i] = std::sin(rad);
    }
  }
}

void PacketDecoder::LoadCorrectionsFile(const std::string& correctionsFile)
{

  boost::property_tree::ptree pt;
  try {
    read_xml(correctionsFile, pt, boost::property_tree::xml_parser::trim_whitespace);
  } catch (boost::exception const&) {
    std::cout << "PacketDecoder: Error reading calibration file - " << correctionsFile << std::endl;
    return;
  }

  BOOST_FOREACH (boost::property_tree::ptree::value_type &v, pt.get_child("boost_serialization.DB.points_")) {
    if (v.first == "item") {
      boost::property_tree::ptree points = v.second;
      BOOST_FOREACH (boost::property_tree::ptree::value_type &px, points) {
        if (px.first == "px") {
          boost::property_tree::ptree calibrationData = px.second;
          int index = -1;
          double azimuth = 0;
          double vertCorrection = 0;
          double distCorrection = 0;
          double vertOffsetCorrection = 0;
          double horizOffsetCorrection = 0;

          BOOST_FOREACH (boost::property_tree::ptree::value_type &item, calibrationData) {
            if (item.first == "id_")
              index = atoi(item.second.data().c_str());
            if (item.first == "rotCorrection_")
              azimuth = atof(item.second.data().c_str());
            if (item.first == "vertCorrection_")
              vertCorrection = atof(item.second.data().c_str());
            if (item.first == "distCorrection_")
              distCorrection = atof(item.second.data().c_str());
            if (item.first == "vertOffsetCorrection_")
              vertOffsetCorrection = atof(item.second.data().c_str());
            if (item.first == "horizOffsetCorrection_")
              horizOffsetCorrection = atof(item.second.data().c_str());
          }
          if (index != -1) {
            laser_corrections_[index].azimuthCorrection = azimuth;
            laser_corrections_[index].verticalCorrection = vertCorrection;
            laser_corrections_[index].distanceCorrection = distCorrection / 100.0;
            laser_corrections_[index].verticalOffsetCorrection = vertOffsetCorrection / 100.0;
            laser_corrections_[index].horizontalOffsetCorrection = horizOffsetCorrection / 100.0;

            laser_corrections_[index].cosVertCorrection = std::cos (HDL_Grabber_toRadians(laser_corrections_[index].verticalCorrection));
            laser_corrections_[index].sinVertCorrection = std::sin (HDL_Grabber_toRadians(laser_corrections_[index].verticalCorrection));
          }
        }
      }
    }
  }

  SetCorrectionsCommon();
}

void PacketDecoder::LoadHDL32Corrections()
{
  double hdl32VerticalCorrections[] = {
    -30.67, -9.3299999, -29.33, -8, -28,
    -6.6700001, -26.67, -5.3299999, -25.33, -4, -24, -2.6700001, -22.67,
    -1.33, -21.33, 0, -20, 1.33, -18.67, 2.6700001, -17.33, 4, -16, 5.3299999,
    -14.67, 6.6700001, -13.33, 8, -12, 9.3299999, -10.67, 10.67 };

  for (int i = 0; i < HDL_LASER_PER_FIRING; i++) {
    laser_corrections_[i].azimuthCorrection = 0.0;
    laser_corrections_[i].distanceCorrection = 0.0;
    laser_corrections_[i].horizontalOffsetCorrection = 0.0;
    laser_corrections_[i].verticalOffsetCorrection = 0.0;
    laser_corrections_[i].verticalCorrection = hdl32VerticalCorrections[i];
    laser_corrections_[i].sinVertCorrection = std::sin(HDL_Grabber_toRadians(hdl32VerticalCorrections[i]));
    laser_corrections_[i].cosVertCorrection = std::cos(HDL_Grabber_toRadians(hdl32VerticalCorrections[i]));
  }

  for (int i = HDL_LASER_PER_FIRING; i < HDL_MAX_NUM_LASERS; i++) {
    laser_corrections_[i].azimuthCorrection = 0.0;
    laser_corrections_[i].distanceCorrection = 0.0;
    laser_corrections_[i].horizontalOffsetCorrection = 0.0;
    laser_corrections_[i].verticalOffsetCorrection = 0.0;
    laser_corrections_[i].verticalCorrection = 0.0;
    laser_corrections_[i].sinVertCorrection = 0.0;
    laser_corrections_[i].cosVertCorrection = 1.0;
  }

  SetCorrectionsCommon();
}

void PacketDecoder::SetCorrectionsCommon()
{
  for (int i = 0; i < HDL_MAX_NUM_LASERS; i++) {
    HDLLaserCorrection correction = laser_corrections_[i];
    laser_corrections_[i].sinVertOffsetCorrection = correction.verticalOffsetCorrection
                                       * correction.sinVertCorrection;
    laser_corrections_[i].cosVertOffsetCorrection = correction.verticalOffsetCorrection
                                       * correction.cosVertCorrection;
  }
}


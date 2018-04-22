#pragma once
#include <pylon/PylonIncludes.h>
#include <pylon/gige/BaslerGigEInstantCamera.h>
#include <image_transport/image_transport.h>
#include <camera_pylon/CameraConfig.h>
#include <ros/ros.h>
struct global_s
{
  ros::NodeHandle*                 phNode=nullptr;
  image_transport::ImageTransport* imageTransporter=nullptr;
  Pylon::CBaslerGigEInstantCamera* Camera=nullptr;
  image_transport::Publisher       pub;
  Pylon::CImageFormatConverter*    fc=nullptr;
  std::string                      encoding="";
  int                              opencv_format=0;
  bool                             isInit=false;
};

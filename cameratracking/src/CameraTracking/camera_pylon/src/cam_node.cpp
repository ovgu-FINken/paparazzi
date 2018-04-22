// Grab_UsingGrabLoopThread.cpp
/*
 This sample illustrates how to grab and process images using the grab loop thread
 provided by the Instant Camera class.
 */

// Include files to use the PYLON API.
#include <pylon/PylonIncludes.h>
#include <unistd.h>
#include <sys/time.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// Include files used by samples.
#include <pylon/ImageFormatConverter.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

#include <pylon/gige/BaslerGigEInstantCamera.h>
#include <config.h>
#include <camera_pylon/CameraConfig.h>

#include <boost/algorithm/string.hpp>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>    // std::max
#include <memory>

using namespace cv;
using namespace Basler_GigECameraParams;
using namespace Pylon;
using namespace std;
using Config = camera_pylon::CameraConfig;

long getTimeDiff(timeval begin, timeval end) {
	long seconds = end.tv_sec - begin.tv_sec;
	long useconds = end.tv_usec - begin.tv_usec;
	useconds += seconds * 1000000L;
	return useconds;
}

class CSampleImageEventHandler: public CImageEventHandler {
private:
	global_s& global;
public:
	CSampleImageEventHandler(global_s& global) : global(global) {}
	virtual void OnImageGrabbed(CInstantCamera& camera, const CGrabResultPtr& ptrGrabResult) {

		if (ptrGrabResult->GrabSucceeded()) {
			CPylonImage image;
			global.fc->Convert(image, ptrGrabResult);
			Mat cv_img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), global.opencv_format, (uint8_t*) image.GetBuffer());
			sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), global.encoding, cv_img).toImageMsg();
			global.pub.publish(msg);
			/*if (global.display) {
				Size size(700, 700); //the dst image size,e.g.100x100
				resize(cv_img, cv_img, size); //resize image
				imshow("CV_Image", cv_img);
				waitKey(1);
			}
			else
				destroyWindow("CV_Image");*/

		}
	}

};

namespace camera_pylon {
class Camera : public nodelet::Nodelet {
	private:
		static unsigned int camNum;
		CGrabResultPtr ptrGrabResult;
		unique_ptr<dynamic_reconfigure::Server<Config>> reconfigureServer;
		global_s global;
		void setDefault() {
			global.fc = new Pylon::CImageFormatConverter();
			global.Camera->OffsetX.SetValue(0);
			global.Camera->OffsetY.SetValue(0);
			global.Camera->Width.SetValue(1600);
			global.Camera->Height.SetValue(1200);
			if (GenApi::IsAvailable(global.Camera->PixelFormat.GetEntry(PixelFormat_Mono8)))
				global.Camera->PixelFormat.SetValue(PixelFormat_Mono8);
			global.Camera->GainAuto.SetValue(GainAuto_Continuous);
			global.Camera->ExposureAuto.SetValue(ExposureAuto_Off);
			if (GenApi::IsWritable(global.Camera->ExposureTimeRaw))
				global.Camera->ExposureTimeRaw.SetValue(1000);
			global.Camera->ShutterMode.SetValue(ShutterMode_Rolling);
			global.Camera->AcquisitionFrameRateEnable.SetValue(true);
			global.Camera->AcquisitionFrameRateAbs.SetValue(50);
			global.Camera->GammaEnable.SetValue(false);
			global.Camera->LightSourceSelector.SetValue( LightSourceSelector_Off);
			global.fc->OutputPixelFormat = PixelType_Mono8;
			global.opencv_format = CV_8UC1;
			global.encoding = "mono8";

		}
	public:
		void reconfigure(Config &config, uint32_t level) {
			ostringstream os;
			global.Camera->StopGrabbing();
			//ExposureTime
			if (config.ExposureAuto == camera_pylon::Camera_Continuous) {
				global.Camera->ExposureAuto.SetValue(ExposureAuto_Continuous);
			}
			if (config.ExposureAuto == camera_pylon::Camera_Once) {
				global.Camera->ExposureAuto.SetValue(ExposureAuto_Once);
			}
			if (config.ExposureAuto == camera_pylon::Camera_Off_) {
				global.Camera->ExposureAuto.SetValue(ExposureAuto_Off);
				if (GenApi::IsWritable(global.Camera->ExposureTimeRaw))
					global.Camera->ExposureTimeRaw.SetValue(config.ExposureTimeAbs);
			}
			os << "Auto Exposure   : " << global.Camera->ExposureAuto.ToString() << " (" << global.Camera->ExposureTimeRaw.GetValue() << ")" << endl;
			config.ExposureTimeAbs = global.Camera->ExposureTimeRaw.GetValue();


			//Gain
			if (GenApi::IsWritable(global.Camera->GainRaw))
				global.Camera->GainRaw.SetValue(config.Gain);
			if (config.GainAuto == camera_pylon::Camera_Off_) {
				global.Camera->GainAuto.SetValue(GainAuto_Off);
			}

			if (config.GainAuto == camera_pylon::Camera_Once) {
				global.Camera->GainRaw.SetValue(GainAuto_Once);
			}
			if (config.GainAuto == camera_pylon::Camera_Continuous) {
				global.Camera->GainAuto.SetValue(GainAuto_Continuous);
			}
			os << "Auto Gain       : " << global.Camera->GainAuto.ToString() << " (" << global.Camera->GainRaw.GetValue() << ")" << endl;
			config.Gain = global.Camera->GainRaw.GetValue();

			//AcquisitionMode
			if (config.AcquisitionMode == camera_pylon::Camera_Continuous_) {
				global.Camera->AcquisitionMode.SetValue(AcquisitionMode_Continuous);
			}
			if (config.AcquisitionMode == camera_pylon::Camera_MultiFrame) {
				global.Camera->AcquisitionMode.SetValue(AcquisitionMode_MultiFrame);
			}
			if (config.AcquisitionMode == camera_pylon::Camera_SingleFrame) {
				global.Camera->AcquisitionMode.SetValue(AcquisitionMode_SingleFrame);
			}
			os << "AcquisitionMode : " << global.Camera->AcquisitionMode.ToString() << endl;
			if (global.Camera->AcquisitionMode.GetValue() == AcquisitionMode_Continuous)
				config.AcquisitionMode = camera_pylon::Camera_Continuous_;
			if (global.Camera->AcquisitionMode.GetValue() == AcquisitionMode_MultiFrame)
				config.AcquisitionMode = camera_pylon::Camera_MultiFrame;
			if (global.Camera->AcquisitionMode.GetValue() == AcquisitionMode_SingleFrame)
				config.AcquisitionMode = camera_pylon::Camera_SingleFrame;

			//FrameRate
			global.Camera->AcquisitionFrameRateEnable.SetValue(config.FixedFrameRate);
			if (global.Camera->AcquisitionFrameRateEnable.GetValue()) {
				global.Camera->AcquisitionFrameRateAbs.SetValue(config.FrameRate);
			}
			config.FrameRate = global.Camera->AcquisitionFrameRateAbs.GetValue();
			config.FixedFrameRate = global.Camera->AcquisitionFrameRateEnable.GetValue();
			os << "Frame Rate      : " << (global.Camera->AcquisitionFrameRateEnable.GetValue() ? "On" : "off") << " ("
					<< global.Camera->AcquisitionFrameRateAbs.GetValue() << ")" << endl;

			//PixelFormat

			if (config.PixelFormat == camera_pylon::Camera_Bayer_RG8) {
				global.Camera->PixelFormat.SetValue(PixelFormat_BayerRG8);
				global.fc->OutputPixelFormat = PixelType_BGR8packed;
				global.opencv_format = CV_8UC3;
				global.encoding = "bgr8";
			}
			if (config.PixelFormat == camera_pylon::Camera_YUV_422) {
				global.Camera->PixelFormat.SetValue(PixelFormat_YUV422Packed);
				global.fc->OutputPixelFormat = PixelType_BGR8packed;
				global.opencv_format = CV_8UC3;
				global.encoding = "bgr8";
			}
			if (config.PixelFormat == camera_pylon::Camera_Mono_8) {
				global.Camera->PixelFormat.SetValue(PixelFormat_Mono8);
				global.fc->OutputPixelFormat = PixelType_Mono8;
				global.opencv_format = CV_8UC1;
				global.encoding = "mono8";

			}
			os << "PixelFormat     : " << global.Camera->PixelFormat.ToString() << " (" << global.Camera->PixelFormat.GetValue() << ")" << endl;
			//ShutterMode
			if (config.ShutterMode == camera_pylon::Camera_Rolling) {
				global.Camera->ShutterMode.SetValue(ShutterMode_Rolling);
			}
			if (config.ShutterMode == camera_pylon::Camera_Global) {
				global.Camera->ShutterMode.SetValue(ShutterMode_Global);
			}
			os << "Shutter Mode    : " << global.Camera->ShutterMode.ToString() << endl;

			//ROI
			if(config.Height%2)
				config.Height--;
			if(config.Width%2)
				config.Width--;
			if(config.OffsetX%2)
				config.OffsetX--;
			if(config.OffsetY%2)
				config.OffsetY--;


			global.Camera->Width.SetValue(std::min((int)(global.Camera->Width.GetMax()-config.OffsetX),config.Width));
			config.Width=global.Camera->Width.GetValue();
			global.Camera->Height.SetValue(std::min((int)(global.Camera->Height.GetMax()-config.OffsetY),config.Height));
			config.Height=global.Camera->Height.GetValue();

			global.Camera->OffsetX.SetValue(std::min((int)(global.Camera->OffsetX.GetMax()),config.OffsetX));
			config.OffsetX=global.Camera->OffsetX.GetValue();
			global.Camera->OffsetY.SetValue(std::min((int)(global.Camera->OffsetY.GetMax()),config.OffsetY));
			config.OffsetY=global.Camera->OffsetY.GetValue();

			//White Balance
			global.Camera->BalanceRatioSelector.SetValue(BalanceRatioSelector_Red);
			global.Camera->BalanceRatioAbs.SetValue(config.WhiteBalanceRatioRed);
			os << global.Camera->BalanceRatioSelector.ToString() << ": "
			   << global.Camera->BalanceRatioAbs.ToString() << endl;
			global.Camera->BalanceRatioSelector.SetValue(BalanceRatioSelector_Blue);
			global.Camera->BalanceRatioAbs.SetValue(config.WhiteBalanceRatioBlue);
			os << global.Camera->BalanceRatioSelector.ToString() << ": "
			   << global.Camera->BalanceRatioAbs.ToString() << endl;
			global.Camera->BalanceRatioSelector.SetValue(BalanceRatioSelector_Green);
			global.Camera->BalanceRatioAbs.SetValue(config.WhiteBalanceRatioGreen);
			os << global.Camera->BalanceRatioSelector.ToString() << ": "
			   << global.Camera->BalanceRatioAbs.ToString() << endl;

			//Restart Aquisition
			global.Camera->StartGrabbing(GrabStrategy_LatestImageOnly, GrabLoop_ProvidedByInstantCamera);
			NODELET_INFO_STREAM(os.str());

		}
		virtual void onInit() {
			ros::NodeHandle nh=getNodeHandle();
			Pylon::PylonInitialize();
			global.isInit = true;
			string serial;
			ros::param::get("~serial", serial);
			if (!serial.empty()) {
				NODELET_INFO_STREAM("Camera("<<serial <<") streaming on camera/"<<serial);
				global.phNode = &nh;
				CDeviceInfo camInfo=CDeviceInfo().SetSerialNumber(serial.c_str());
				global.Camera = new CBaslerGigEInstantCamera(CTlFactory::GetInstance().CreateDevice(camInfo));
				global.imageTransporter = new image_transport::ImageTransport(*global.phNode);
				global.pub = global.imageTransporter->advertise("camera/"+serial, 1);
			}
			else {
				NODELET_INFO_STREAM("Camera("<<camNum <<") streaming on camera/"<<camNum);
				DeviceInfoList_t devices;
				CTlFactory& f=CTlFactory::GetInstance();
				f.EnumerateDevices(devices);
				CDeviceInfo& device = devices.at(camNum++);
				global.phNode = &nh;
				global.Camera = new CBaslerGigEInstantCamera(CTlFactory::GetInstance().CreateDevice(device));
				global.imageTransporter = new image_transport::ImageTransport(*global.phNode);
				global.pub = global.imageTransporter->advertise("camera/"+to_string(camNum-1), 1);
			}
			global.Camera->Open();
			ostringstream os;
			// Get camera device information.
			os << "Camera Device Information" << endl << "=========================" << endl;
			os << "Vendor : " << global.Camera->DeviceVendorName.GetValue() << endl;
			os << "Model  : " << global.Camera->DeviceModelName.GetValue() << endl;
			os << "Firmware version : " << global.Camera->DeviceFirmwareVersion.GetValue() << endl << endl;
			setDefault();
			// Camera settings.
			os << "Set Camera default Settings" << endl << "======================" << endl;
			os << "OffsetX       : " << global.Camera->OffsetX.GetValue() << endl;
			os << "OffsetY       : " << global.Camera->OffsetY.GetValue() << endl;
			os << "Width         : " << global.Camera->Width.GetValue() << endl;
			os << "Height        : " << global.Camera->Height.GetValue() << endl;
			os << "PixelFormat   : " << global.Camera->PixelFormat.ToString() << " (" << global.Camera->PixelFormat.GetValue() << ")" << endl;
			os << "Auto Exposure : " << global.Camera->ExposureAuto.ToString() << " (" << global.Camera->ExposureTimeRaw.GetValue() << ")" << endl;
			os << "Auto Gain : " << global.Camera->GainAuto.ToString() << " (" << global.Camera->GainRaw.GetValue() << ")" << endl;
			os << "Shutter Mode  : " << global.Camera->ShutterMode.ToString() << endl;
			os << "Frame Rate    : " << (global.Camera->AcquisitionFrameRateEnable.GetValue() ? "On" : "off") << " ("
					<< global.Camera->AcquisitionFrameRateAbs.GetValue() << ")" << endl;
			NODELET_INFO_STREAM(os.str());
			ros::Duration(1.0).sleep();

			try {

				global.Camera->RegisterConfiguration(new CAcquireContinuousConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);
				global.Camera->RegisterImageEventHandler(new CSampleImageEventHandler(global), RegistrationMode_Append, Cleanup_Delete);
				global.Camera->MaxNumBuffer = 10;
				global.Camera->StartGrabbing(GrabStrategy_LatestImageOnly, GrabLoop_ProvidedByInstantCamera);
				reconfigureServer.reset(new dynamic_reconfigure::Server<Config>(getPrivateNodeHandle()));
				dynamic_reconfigure::Server<Config>::CallbackType reconfigureCallback;
				reconfigureCallback = boost::bind(&Camera::reconfigure, this, _1, _2);
				reconfigureServer->setCallback(reconfigureCallback);
				//doing ROS-things
			} catch (GenICam::TimeoutException &e) {
				NODELET_ERROR_STREAM("An exception occurred." << endl << e.GetDescription());
			}
		}
		virtual ~Camera() {
			if(global.isInit) {
				global.Camera->StopGrabbing();
				Pylon::PylonTerminate();
			}
		}
};

unsigned int Camera::camNum = 0;
}

PLUGINLIB_EXPORT_CLASS(camera_pylon::Camera, nodelet::Nodelet) 

/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2018-06-06 22:00
#
# Filename:		capture.h
#
# Description: 
#
===============================================*/
#include <iostream>
#include <sys/time.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/features/integral_image_normal.h>
//#include <OpenNI.h>
//#include <libfreenect2/libfreenect2.hpp>
//#include <libfreenect2/frame_listener_impl.h>
//#include <libfreenect2/registration.h>
//#include <libfreenect2/packet_pipeline.h>
//#include <libfreenect2/config.h>
//#include <map>
#include "types.h"

namespace ulysses
{
	class Capture
	{
	public:

		Capture()
		{
			remove("data/rgb.txt");
			remove("data/depth.txt");
			remove("data/groundtruth.txt");
		}

		~Capture() {}

		bool initialize();
		void close();
		
		void loadScan(Scan *scan, IntrinsicParam cam);
		void capture(cv::Mat color, cv::Mat depth);


	private:

		std::ofstream fp_rgb, fp_dep, fp_gt;
		cv::Mat img_depth, img_rgb;
		double time_stamp;

//		int deviceId_;
//		libfreenect2::Freenect2* freenect2_;
//		libfreenect2::SyncMultiFrameListener* listener_;//color, listener_ir_depth;
//		libfreenect2::Freenect2Device* dev_;
//		libfreenect2::Registration * reg_;
//		libfreenect2::Freenect2Device::IrCameraParams params;
//
//		template<class K, class V>
//		inline V uValue(const std::map<K, V> & m, const K & key, const V & defaultValue = V())
//		{
//			V v = defaultValue;
//			typename std::map<K, V>::const_iterator i = m.find(key);
//			if(i != m.end())
//			{
//				v = i->second;
//			}
//			return v;
//		}
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////

//		openni::Device mDevice;
//		openni::Status status;  
//		openni::VideoStream mDepthStream;  
//		openni::VideoStream mColorStream;
//
//		openni::VideoFrameRef mColorFrame;
//		openni::VideoFrameRef mDepthFrame;
	};

	class Collect
	{
	public:

		Collect()
		{
			remove("data/rgb.txt");
			remove("data/depth.txt");
			remove("data/groundtruth.txt");
		}

		~Collect() {}

		bool initialize();
		
		void collect(cv::Mat color, cv::Mat depth);


	private:

		std::ofstream fp_rgb, fp_dep, fp_gt;
		cv::Mat img_depth, img_rgb;
		double time_stamp;

	};


}

/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-06-08 09:41
#
# Filename:		data_reading.h
#
# Description: 
#
===============================================*/
#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <pcl-1.8/pcl/io/io.h>
#include <pcl-1.8/pcl/io/file_io.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <eigen3/Eigen/src/Core/DenseBase.h>
#include <pcl-1.8/pcl/features/integral_image_normal.h>
#include <pcl-1.8/pcl/features/normal_3d.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl-1.8/pcl/visualization/cloud_viewer.h>
#include "types.h"

// + foldername
//   + rgb
//   + depth
//   - rgb.txt
//   - depth.txt
//
namespace ulysses
{
	class DataReading
	{
	public:

		DataReading()
		{
//			Width=960;//640;//
//			Height=540;//480;//
			first_frame=true;
		}

		~DataReading() 
		{
			fp_rgb.close();
			fp_depth.close();
			fp_gt.close();
		}

		void setDebug(bool d) {debug=d;}

		void setPath(std::string p) {path=p;}

		void setSampleInterval(double delta_t) {sample_interval=delta_t;}

//		void setNormalEstimation(pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA,
//								 pcl::Normal>::NormalEstimationMethod method, 
//								 float MaxDepthChangeFactor, float NormalSmoothingSize)
//		{
//			normal_estimate_integral.setNormalEstimationMethod (method);
//			normal_estimate_integral.setMaxDepthChangeFactor(MaxDepthChangeFactor);
//			normal_estimate_integral.setNormalSmoothingSize(NormalSmoothingSize);
//		}

		double getTime() {return timestamp_depth;}

		bool isEOF() {return fp_rgb.eof() || fp_depth.eof();}

		bool loadScan(Scan *scan, IntrinsicParam cam);

		// scan is allocated before this function is called;
		bool loadScan_once(Scan *scan, IntrinsicParam cam, double time, bool gt=false);

		// Initialize
		// + input
		//   - time_start: read the image sequence from time_start.
		// + function
		//   - open rgb.txt and depth.txt;
		//   - set timestamp to the start time;
		void Initialize(double time_start = 0);
		void readData(Scan *scan, IntrinsicParam cam);
//		void read(Scan *scan, IntrinsicParam cam,std::string rgb, std::string dep)
//		{
//			filename_rgb=rgb;
//			filename_depth=dep;
//			readData(scan,cam);
//		}


	private:


		// integral image normal estimation method;
		pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimate_integral;

		// camera intrinsic parameters;
//		double fx,fy,cx,cy,factor,dx,dy;
		// width and height of the image;
//		int Width,Height;
		// the time interval that sample the sequence;
		double sample_interval;
		// path of the root folder that locate the image sequence;
		std::string path;
		// file stream of rgb.txt and depth.txt;
		std::ifstream fp_rgb,fp_depth, fp_gt;
		// timestamp controlling the sample of the sequence;
		double timestamp_rgb, timestamp_depth, timestamp, timestamp_gt;
		// filename_rgb - rgb/*.png
		// filename_depth - depth/*.png
		std::string filename_rgb, filename_depth;

		double tx,ty,tz,qx,qy,qz,qw;
		bool first_frame;
		Transform Tg;

		bool debug;
	};
}

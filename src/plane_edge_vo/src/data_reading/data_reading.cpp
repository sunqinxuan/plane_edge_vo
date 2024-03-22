/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-06-08 09:41
#
# Filename:		data_reading.cpp
#
# Description: 
#
===============================================*/
#include "data_reading.h"

namespace ulysses
{

	void DataReading::Initialize(double time_start)
	{
		std::string file_rgb=path+"/rgb.txt";
		std::string file_depth=path+"/depth.txt";
		std::string file_gt=path+"/groundtruth.txt";
		// open the file rgb.txt and depth.txt;
		fp_rgb.open(file_rgb.c_str(),std::ios::in);
		fp_depth.open(file_depth.c_str(),std::ios::in);
		fp_gt.open(file_gt.c_str(),std::ios::in);
		std::string tmp_cache;

		// skip the comments started by character "#";
		// this implementation sucks;
		// needs modification;
		for(int i=0;i<9;i++)
		{
			fp_rgb>>tmp_cache;
			fp_depth>>tmp_cache;
		}
		for(int i=0;i<16;i++)
		{
			fp_gt>>tmp_cache;
		}
		if(time_start!=0)
		{
			// if the time_start is set by users,
			// then set the timestamp with the user-defined value;
			timestamp=time_start;
		}
		else
		{
			// if not, set the timestamp_rgb and timestamp_depth
			// to the first time stamp of the sequence;
			fp_rgb>>timestamp_rgb>>filename_rgb;
			fp_depth>>timestamp_depth>>filename_depth;
			// set timestamp to the larger one (subsequent one);
			if(timestamp_rgb>timestamp_depth)
				timestamp=timestamp_rgb;
			else
				timestamp=timestamp_depth;
		}

		if(debug)
		{
			std::cout<<"sample image sequence "<<path<<" from "<<std::fixed<<timestamp<<std::endl;
		}
	}

	bool DataReading::loadScan(Scan *scan, IntrinsicParam cam)
	{
		do
		{
			fp_rgb>>timestamp_rgb>>filename_rgb;
			if(fp_rgb.eof())
				return false;
		}while(timestamp_rgb<timestamp);
		do
		{
			fp_depth>>timestamp_depth>>filename_depth;
			if(fp_depth.eof())
				return false;
		}while(timestamp_depth<timestamp);
		do
		{
			fp_gt>>timestamp_gt>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
			if(fp_gt.eof())
				return false;
		}while(timestamp_gt<timestamp);

		if(first_frame)
		{
			Eigen::Quaterniond quat_1(qw,qx,qy,qz);
			Eigen::Vector3d vec3d_1(tx,ty,tz);
			Tg=Transform(quat_1,vec3d_1);
			first_frame=false;
		}

		if(debug)
		{
			std::cout<<"timestamp: "<<std::fixed<<timestamp<<std::endl;
			std::cout<<"sample rgb image at "<<std::fixed<<timestamp_rgb<<","<<filename_rgb<<std::endl;
			std::cout<<"sample depth image at "<<std::fixed<<timestamp_depth<<","<<filename_depth<<std::endl;
			std::cout<<"sample groundtruth at "<<std::fixed<<timestamp_gt<<","<<tx<<" "<<ty<<" "<<tz<<", "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<std::endl;
		}

		timestamp+=sample_interval;

//		scan=new Scan;
		readData(scan,cam);

		return true;
	}

	bool DataReading::loadScan_once(Scan *scan, IntrinsicParam cam, double time, bool gt)
	{
		std::string file_rgb=path+"/rgb.txt";
		std::string file_depth=path+"/depth.txt";
		std::string file_gt=path+"/groundtruth.txt";
		// open the file rgb.txt and depth.txt;
		fp_rgb.open(file_rgb.c_str(),std::ios::in);
		fp_depth.open(file_depth.c_str(),std::ios::in);
		fp_gt.open(file_gt.c_str(),std::ios::in);
		std::string tmp_cache;

		// skip the comments started by character "#";
		// this implementation sucks;
		// needs modification;
		for(int i=0;i<9;i++)
		{
			fp_rgb>>tmp_cache;
			fp_depth>>tmp_cache;
		}
		for(int i=0;i<16;i++)
		{
			fp_gt>>tmp_cache;
		}

		timestamp=time;
		do
		{
			fp_rgb>>timestamp_rgb>>filename_rgb;
			if(fp_rgb.eof())
				return false;
		}while(timestamp_rgb<=timestamp);
		do
		{
			fp_depth>>timestamp_depth>>filename_depth;
			if(fp_depth.eof())
				return false;
		}while(timestamp_depth<=timestamp);
		do
		{
			fp_gt>>timestamp_gt>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
			if(fp_gt.eof())
				return false;
		}while(timestamp_gt<=timestamp);

//		std::cout<<"timestamp: "<<std::fixed<<timestamp<<std::endl;
//		std::cout<<"sample rgb image at "<<std::fixed<<timestamp_rgb<<","<<filename_rgb<<std::endl;
//		std::cout<<"sample depth image at "<<std::fixed<<timestamp_depth<<","<<filename_depth<<std::endl;
//		std::cout<<"sample groundtruth at "<<std::fixed<<timestamp_gt<<","<<tx<<" "<<ty<<" "<<tz<<", "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<std::endl;

		if(gt)
		{
			Transform Tcg(Eigen::Quaterniond(qw,qx,qy,qz), Eigen::Vector3d(tx,ty,tz));
			Tcg.inverse();
			scan->Tcg_gt=Tcg;
			if(fabs(timestamp_gt-timestamp)>0.1)
				scan->Tcg_gt.setIdentity();
		}
		else
		{
			readData(scan,cam);
		}
		
		fp_rgb.close();
		fp_depth.close();
		fp_gt.close();
		return true;
	}

	void DataReading::readData(Scan *scan, IntrinsicParam cam)
	{
//		std::cout<<"readData "<<std::endl;
		scan->point_cloud=pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
		scan->normal_cloud=pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
		scan->pixel_cloud=pcl::PointCloud<pcl::PointXY>::Ptr (new pcl::PointCloud<pcl::PointXY>);

//		std::cout<<"timestamp "<<timestamp_depth<<std::endl;
		scan->time_stamp=timestamp_depth;
		cv::Mat rgb_image,depth_image;
		uint8_t *depth_ptr,*rgb_ptr;
		pcl::PointXYZRGBA point_tmp;
		unsigned short *depth_tmp_ptr=new unsigned short;
		pcl::PointXY tmp_pointxy;
		// full path of the current rgb and depth image;
//		std::cout<<"filename_rgb\t"<<path<<"\t"<<filename_rgb<<std::endl;
//		std::cout<<"filename_depth\t"<<path<<"\t"<<filename_depth<<std::endl;
		std::string filename_rgb_full=path+"/"+filename_rgb;
		std::string filename_depth_full=path+"/"+filename_depth;
//		std::cout<<"filename_rgb_full "<<filename_rgb_full<<std::endl;
//		std::cout<<"filename_depth_full "<<filename_depth_full<<std::endl;
		// load the rgb and depth image to cv::Mat;
		// the depth_image is stored as CV_8UC2;
		scan->img_rgb=cv::imread(filename_rgb_full);
		scan->img_depth=cv::imread(filename_depth_full,-1);
//		cv::imshow("rgb",scan->img_rgb);
//		cv::waitKey(0);
//		cv::imshow("depth",scan->img_depth);
//		cv::waitKey(0);

		// pointer to the Mat data;
		rgb_ptr=scan->img_rgb.data;
		depth_ptr=scan->img_depth.data;
		// clear the pointcloud;
		// the allocated memory does not release;
		// the newly pushed elements cover the old ones;
		scan->point_cloud->clear();
		scan->normal_cloud->clear();
		scan->pixel_cloud->clear();
		// generate the point_cloud;
		for(int i=0;i<scan->img_rgb.rows;i++)
		{
			for(int j=0;j<scan->img_rgb.cols;j++)
			{
				// 3 channels for one pixel in rgb image;
				point_tmp.b=*rgb_ptr;
				rgb_ptr++;
				point_tmp.g=*rgb_ptr;
				rgb_ptr++;
				point_tmp.r=*rgb_ptr;
				rgb_ptr++;
				// 2 channels for one pixel in depth image;
				memcpy(depth_tmp_ptr,depth_ptr,2);
				depth_ptr+=2;
				if(j<300 || j>=800 || i<80 || i>=500)
					continue;
				// transformation from pixel coordinate to the camera coordinate;
				// wrong results if considering length of the pixel;
				point_tmp.z=*depth_tmp_ptr/cam.factor;
				point_tmp.x=(j-cam.cx)*point_tmp.z/cam.fx;
				point_tmp.y=(i-cam.cy)*point_tmp.z/cam.fy;
				scan->point_cloud->push_back(point_tmp);
			}
		}
//		std::cout<<"point cloud "<<scan->point_cloud->size()<<std::endl;
		delete depth_tmp_ptr;
		// organize the point_cloud for the normal estimation;
		scan->point_cloud->width=500;//cam.width;
		scan->point_cloud->height=420;//cam.height;
//		scan->point_cloud->width=cam.width;
//		scan->point_cloud->height=cam.height;
		// generate the normal_cloud;
		normal_estimate_integral.setInputCloud(scan->point_cloud);
		normal_estimate_integral.compute (*scan->normal_cloud);
		// generate the pixel_cloud;
		for(int v=0;v<scan->point_cloud->height;v++)
		{
			for(int u=0;u<scan->point_cloud->width;u++)
			{
				tmp_pointxy.x=u;
				tmp_pointxy.y=v;
				scan->pixel_cloud->push_back(tmp_pointxy);
			}
		}
//		std::cout<<"pixel cloud "<<scan->pixel_cloud->size()<<std::endl;

//		Eigen::Quaterniond quat(qw,qx,qy,qz);
//		Eigen::Vector3d vec3d(tx,ty,tz);
//		Transform Tc(quat,vec3d);
		Transform Tcg(Eigen::Quaterniond(qw,qx,qy,qz), Eigen::Vector3d(tx,ty,tz));
		Tcg.inverse();
		scan->Tcg_gt=Tcg;

//		scan->Tcg_gt=Tc.inv()*Tg;
	}
}

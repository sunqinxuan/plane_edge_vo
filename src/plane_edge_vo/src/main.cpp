/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2018-06-27 16:14
#
# Filename:		main.cpp
#
# Description: 
#
===============================================*/

#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/visualization/cloud_viewer.h>
#include <pcl-1.8/pcl/features/integral_image_normal.h>
#include <pcl-1.8/pcl/registration/transforms.h>
#include <sys/time.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <boost/make_shared.hpp>
#include <boost/thread/thread.hpp>
#include <eigen3/unsupported/Eigen/NonLinearOptimization>
#include <eigen3/Eigen/StdVector>
#include "data_reading.h"
//#include "plane_segmentation.h"
#include "plane_feature_matching.h"
#include "pose_estimation.h"
//#include "plane_map_update.h"
#include "plane_param_estimation.h"
#include "plane_extraction.h"
#include "edge_point_extraction.h"
#include "loop_closing.h"
#include "pose_graph_optimization.h"
#include <pcl-1.8/pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl-1.8/pcl/segmentation/planar_region.h>
#include <pcl-1.8/pcl/features/organized_edge_detection.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.8/pcl/visualization/cloud_viewer.h>
#include <pcl-1.8/pcl/filters/voxel_grid.h>
#include <limits>
#include "display.h"
#include "traj_puzzle.h"
#include "capture.h"
#include "plane_fusing.h"

#include <stdlib.h>
#include <sstream>
#include <string>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>
//
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
//#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>

#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Bool.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <kinect2_bridge/kinect2_definitions.h>

using namespace ulysses;

class Receiver
{
public:
	enum Mode
	{
		IMAGE = 0,
		CLOUD,//=1
		BOTH,//=2
		DEBUG,//=3
		ONLINE,//=4
		COLLECT//=5
	};

private:
	std::mutex lock;

	const std::string topicColor, topicDepth;
	const bool useExact, useCompressed;

	bool updateImage, updateCloud;
	bool save;
	bool running;
	size_t frame;
	const size_t queueSize;

	cv::Mat color, depth;
	cv::Mat cameraMatrixColor, cameraMatrixDepth;
	cv::Mat lookupX, lookupY;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ExactSyncPolicy;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximateSyncPolicy;

	ros::NodeHandle nh;
	ros::NodeHandle nh_flag;
	ros::AsyncSpinner spinner;
	image_transport::ImageTransport it;
	image_transport::SubscriberFilter *subImageColor, *subImageDepth;
	message_filters::Subscriber<sensor_msgs::CameraInfo> *subCameraInfoColor, *subCameraInfoDepth;
	ros::Subscriber sub;

	message_filters::Synchronizer<ExactSyncPolicy> *syncExact;
	message_filters::Synchronizer<ApproximateSyncPolicy> *syncApproximate;

	std::thread imageViewerThread;
	Mode mode;

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;
	pcl::PCDWriter writer;
	std::ostringstream oss;
	std::vector<int> params;

	ulysses::Collect collect;

	bool debug;
	int pln_fitting_method;
	bool usePln, usePt;
	bool useWeight;
	double alpha, thres_weight;
	int max_icp, max_lm;
	int min_inliers;
	int occluding, curvature, canny;
	int total_frames;
	std::string sequence_name;
	double time_start;
	double time_interval;

	std::ofstream fp_flag;

public:
	Receiver(const std::string &topicColor, const std::string &topicDepth, const bool useExact, const bool useCompressed)
		: topicColor(topicColor), topicDepth(topicDepth), useExact(useExact), useCompressed(useCompressed),
		  updateImage(false), updateCloud(false), save(false), running(false), frame(0), queueSize(5),
		  nh("~"), spinner(0), it(nh), mode(ONLINE),
		  debug(false), pln_fitting_method(1), usePln(true), usePt(true), useWeight(true),
		  alpha(1), thres_weight(0.01), max_icp(10), max_lm(10), min_inliers(10000),
		  occluding(2), curvature(0), canny(0), total_frames(100),
		  sequence_name("/home/zgz/sun/catkin_ws/devel/lib/plane_edge_vo/data_collect"),
	      time_start(0), time_interval(0.1)
	{
		cameraMatrixColor = cv::Mat::zeros(3, 3, CV_64F);
		cameraMatrixDepth = cv::Mat::zeros(3, 3, CV_64F);
		params.push_back(cv::IMWRITE_JPEG_QUALITY);
		params.push_back(100);
		params.push_back(cv::IMWRITE_PNG_COMPRESSION);
		params.push_back(1);
		params.push_back(cv::IMWRITE_PNG_STRATEGY);
		params.push_back(cv::IMWRITE_PNG_STRATEGY_RLE);
		params.push_back(0);
	}

	void setDebug(bool d) {debug=d;}
	void setPlnFitMethod(int m) {pln_fitting_method=m;}
	void setMotionCoeff(bool pln, bool pt, bool weight, double a, double th, int icp, int lm)
	{
		usePln=pln;
		usePt=pt;
		useWeight=weight;
		alpha=a;
		thres_weight=th;
		max_icp=icp;
		max_lm=lm;
	}
	void setInliers(int i) {min_inliers=i;}
	void setEdgeType(int occ, int cur, int can)
	{
		occluding=occ;
		curvature=cur;
		canny=can;
	}
	void setTotalFrames(int f) {total_frames=f;}
	void setDataReadCoeff(std::string name, double start, double interval)
	{
		sequence_name=name;
		time_start=start;
		time_interval=interval;
	}

	~Receiver()
	{
	}

	void run(const Mode mode)
	{
		start(mode);
		stop();
	}

private:
	void flag_callback(const std_msgs::Int64::ConstPtr& flag)
	{
		fp_flag.open("flag.txt",std::ios::app);
		fp_flag<<flag->data<<std::endl;
		if(flag->data>4000000)
			running=false;
		fp_flag.close();
	}
	void start(const Mode mode)
	{
		this->mode = mode;
		running = true;

		std::string topicCameraInfoColor = topicColor.substr(0, topicColor.rfind('/')) + "/camera_info";
		std::string topicCameraInfoDepth = topicDepth.substr(0, topicDepth.rfind('/')) + "/camera_info";

		image_transport::TransportHints hints(useCompressed ? "compressed" : "raw");
		subImageColor = new image_transport::SubscriberFilter(it, topicColor, queueSize, hints);
		subImageDepth = new image_transport::SubscriberFilter(it, topicDepth, queueSize, hints);
		subCameraInfoColor = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoColor, queueSize);
		subCameraInfoDepth = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh, topicCameraInfoDepth, queueSize);
		sub=nh_flag.subscribe("flag",1000,(boost::function <void(const std_msgs::Int64::ConstPtr&)>)boost::bind(&Receiver::flag_callback, this, _1 ));

		collect.initialize();

		if(useExact)
		{
			syncExact = new message_filters::Synchronizer<ExactSyncPolicy>(ExactSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
			syncExact->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
		}
		else
		{
			syncApproximate = new message_filters::Synchronizer<ApproximateSyncPolicy>(ApproximateSyncPolicy(queueSize), *subImageColor, *subImageDepth, *subCameraInfoColor, *subCameraInfoDepth);
			syncApproximate->registerCallback(boost::bind(&Receiver::callback, this, _1, _2, _3, _4));
		}

		spinner.start();

		std::chrono::milliseconds duration(1);
		while(!updateImage || !updateCloud)
		{
			if(!ros::ok())
			{
				return;
			}
			std::this_thread::sleep_for(duration);
		}

		cloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
		cloud->height = color.rows;
		cloud->width = color.cols;
		cloud->is_dense = false;
		cloud->points.resize(cloud->height * cloud->width);
		createLookup(this->color.cols, this->color.rows);

		plane_edge_vo(mode);

//		switch(mode)
//		{
//		case CLOUD:
//			cloudViewer();
//			break;
//		case IMAGE:
//			imageViewer();
//			break;
//		case BOTH:
//			imageViewerThread = std::thread(&Receiver::imageViewer, this);
//			cloudViewer();
//			break;
//		}
	}

	void stop()
	{
		spinner.stop();

		if(useExact)
		{
			delete syncExact;
		}
		else
		{
			delete syncApproximate;
		}

		delete subImageColor;
		delete subImageDepth;
		delete subCameraInfoColor;
		delete subCameraInfoDepth;

		running = false;
		if(mode == BOTH)
		{
			imageViewerThread.join();
		}
	}

	void callback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::Image::ConstPtr imageDepth,
				  const sensor_msgs::CameraInfo::ConstPtr cameraInfoColor, const sensor_msgs::CameraInfo::ConstPtr cameraInfoDepth)
	{
		cv::Mat color, depth;

		readCameraInfo(cameraInfoColor, cameraMatrixColor);
		readCameraInfo(cameraInfoDepth, cameraMatrixDepth);
		readImage(imageColor, color);
		readImage(imageDepth, depth);

		// IR image input
		if(color.type() == CV_16U)
		{
			cv::Mat tmp;
			color.convertTo(tmp, CV_8U, 0.02);
			cv::cvtColor(tmp, color, CV_GRAY2BGR);
		}

		lock.lock();
		this->color = color;
		this->depth = depth;
		updateImage = true;
		updateCloud = true;
		if(running)
			collect.collect(color,depth);
		lock.unlock();
	}

	void imageViewer()
	{
		cv::Mat color, depth, depthDisp, combined;
		std::chrono::time_point<std::chrono::high_resolution_clock> start, now;
		double fps = 0;
		size_t frameCount = 0;
		std::ostringstream oss;
		const cv::Point pos(5, 15);
		const cv::Scalar colorText = CV_RGB(255, 255, 255);
		const double sizeText = 0.5;
		const int lineText = 1;
		const int font = cv::FONT_HERSHEY_SIMPLEX;

		cv::namedWindow("Image Viewer");
		oss << "starting...";

		start = std::chrono::high_resolution_clock::now();
		for(; running && ros::ok();)
		{
			if(updateImage)
			{
				lock.lock();
				color = this->color;
				depth = this->depth;
				updateImage = false;
				lock.unlock();

				++frameCount;
				now = std::chrono::high_resolution_clock::now();
				double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() / 1000.0;
				if(elapsed >= 1.0)
				{
					fps = frameCount / elapsed;
					oss.str("");
					oss << "fps: " << fps << " ( " << elapsed / frameCount * 1000.0 << " ms)";
					start = now;
					frameCount = 0;
				}

				dispDepth(depth, depthDisp, 12000.0f);
				combine(color, depthDisp, combined);
				//combined = color;

				cv::putText(combined, oss.str(), pos, font, sizeText, colorText, lineText, CV_AA);
				cv::imshow("Image Viewer", combined);
			}

			int key = cv::waitKey(1);
			switch(key & 0xFF)
			{
			case 27:
			case 'q':
				running = false;
				break;
			case ' ':
			case 's':
				running = false;
				if(mode == IMAGE)
				{
					createCloud(depth, color, cloud);
					saveCloudAndImages(cloud, color, depth, depthDisp);
				}
				else
				{
					save = true;
				}
				break;
			}
		}
		cv::destroyAllWindows();
		cv::waitKey(100);
	}

	void plane_edge_vo(const Mode mode)
	{
		this->mode=mode;
		debug=(this->mode==DEBUG);

		ulysses::Scan *scan_ref;
		ulysses::Scan *scan_cur;
		ulysses::IntrinsicParam cam;

		std::vector<ulysses::PlanePair> matched_planes;
		Transform Tcr_align_planes, Tgc, Tcr;

		ulysses::PlaneParamEstimation plane_fitting;
		plane_fitting.setDebug(debug);
		plane_fitting.setPlnFittingMethod(pln_fitting_method);

		ulysses::PlaneFeatureMatching pfm;
		pfm.setDebug(debug);

		ulysses::PoseEstimation pe;
		pe.setDebug(debug);
		pe.usePlnPt(usePln,usePt);
		pe.useEdgeWeight(useWeight);
		pe.setAlpha(alpha);
		pe.setThresWeight(thres_weight);
		pe.setMaxIterationICP(max_icp);
		pe.setMaxIterationLM(max_lm);

		ulysses::PlaneExtraction extract;
		extract.setDebug(debug);
		extract.setMinPlaneSize(min_inliers);
		extract.allocBottomGrid();

		ulysses::EdgePointExtraction edge_ext;
		edge_ext.setDebug(debug);
		int edge_type=occluding|curvature|canny;
		edge_ext.setEdgeType(edge_type);
	//	std::cout<<"edge type - "<<edge_type<<std::endl;

		ulysses::Capture cap;
		cap.initialize();
//		std::cout<<"online mode"<<std::endl;
//		if(!cap.initialize())
//		{
//			std::cerr<<"openni initialization failed"<<std::endl;
//			return 0;
//		}

		DataReading *data_reading=new ulysses::DataReading();
		data_reading->setPath(sequence_name);
		data_reading->setDebug(false);
		data_reading->setSampleInterval(time_interval);

		scan_ref=0;


//		ulysses::LoopClosing lc;
//		lc.setDebug(debug);
//		lc.setDeltaTime(delta_time);
//		lc.setDeltaAngle(delta_angle);
//		lc.setDeltaDist(delta_dist);
//		ulysses::Map *map=new ulysses::Map;

//		ulysses::PoseGraphOptimization pgo;
//		pgo.setDebug(debug);

//		ulysses::PlaneFusing pf;
//		pf.setDebug(debug);

//		boost::shared_ptr<pcl::visualization::PCLVisualizer> vis (new pcl::visualization::PCLVisualizer ("3D Viewer"));
//		vis->setBackgroundColor (0, 0, 0);
//		vis->initCameraParameters ();
//		vis->registerKeyboardCallback (keyboardEventOccurred, (void*)vis.get ());

		cv::Mat color, depth;
		pcl::visualization::PCLVisualizer::Ptr visualizer(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
//		visualizer=pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer("Cloud Viewer"));
		const std::string cloudName = "rendered";

		lock.lock();
		color = this->color;
		depth = this->depth;
		updateCloud = false;
		lock.unlock();

//		createCloud(depth, color, cloud);

		visualizer->addPointCloud(cloud, cloudName);
		visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloudName);
		visualizer->initCameraParameters();
		visualizer->setBackgroundColor(0, 0, 0);
//		visualizer->setPosition(mode == BOTH ? color.cols : 0, 0);
		visualizer->setPosition( 0, 0);
		visualizer->setSize(color.cols, color.rows);
		visualizer->setShowFPS(true);
		visualizer->setCameraPosition(0, 0, 0, 0, -1, 0);
		visualizer->registerKeyboardCallback(&Receiver::keyboardEvent, *this);
//		visualizer->registerKeyboardCallback(&Receiver::keyboardEvent, (void*)visualizer.get());
//		visualizer->registerKeyboardCallback (keyboardEventOccurred, (void*)vis.get());

		ofstream fp;
		fp.open("traj.txt",std::ios::out);
		ofstream fp_time;
		fp_time.open("time.txt",std::ios::out);

		int first=0;
		int filenum = first;
		timeval start, end;
		timeval start_all, end_all;
		double timeused, timeused_all;
		bool stopedCollect=false;

		data_reading->Initialize(time_start);
		for(; !data_reading->isEOF() && ros::ok() ;)
		{
			if(!running && !stopedCollect)
			{
//				visualizer->spin();
				stopedCollect=true;
			}
			if(updateCloud)
			{
//				lock.lock();
//				color = this->color;
//				depth = this->depth;
//				updateCloud = false;
//				lock.unlock();

				scan_cur=new Scan;
				scan_cur->id=filenum;

				lock.lock();
				if(!data_reading->loadScan(scan_cur,cam))
				{
					std::cerr<<"DataReading::loadScan failure!"<<std::endl;
					break;
				}
				lock.unlock();

				gettimeofday(&start_all,NULL);

				/////// std::cout<<std::endl<<"***************** frame "<<filenum<<" ******************"<<std::endl;
//				std::cout<<"mode="<<this->mode<<std::endl;

//				if(this->mode==COLLECT)
//				{
//					cap.capture(color,depth);
//					filenum++;
//					continue;
//				}

//				gettimeofday(&start,NULL);
//				cap.capture(color,depth);
//				gettimeofday(&end,NULL);
//				timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
//				std::cout<<filenum<<"\tcapture - "<<timeused<<std::endl;
//
//				gettimeofday(&start,NULL);
//				cap.loadScan(scan_cur,cam);
//				//cap.close();
//				gettimeofday(&end,NULL);
//				timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
//				std::cout<<filenum<<"\tloadScan - "<<timeused<<std::endl;

//				if (!visualizer->updatePointCloud (scan_cur->point_cloud, "scan"))
//					visualizer->addPointCloud (scan_cur->point_cloud, "scan");
//				visualizer->spin();

//				writer.write("scan.pcd",*scan_cur->point_cloud);
				gettimeofday(&start,NULL);
				extract.loadPoints(scan_cur);
				extract.extractPlanes(scan_cur);
				gettimeofday(&end,NULL);
				timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
				/////// std::cout<<filenum<<"\textract plane - "<<timeused<<" "<<scan_cur->observed_planes.size()<<std::endl;

				gettimeofday(&start,NULL);
				for(size_t i=0;i<scan_cur->observed_planes.size();i++)
					plane_fitting.estimatePlaneParams(scan_cur->observed_planes[i],cam);
				gettimeofday(&end,NULL);
				timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
				/////// std::cout<<filenum<<"\tplane fitting - "<<timeused<<std::endl;

				gettimeofday(&start,NULL);
				edge_ext.extractEdgePoints(scan_cur);
				gettimeofday(&end,NULL);
				timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
				/////// std::cout<<filenum<<"\textract edge - "<<timeused<<std::endl;

//				displayPlanes(scan_cur,visualizer);
//				visualizer->spin();
//				displayEdgePoints(scan_cur,visualizer);
//				visualizer->spin();

				if(scan_ref!=0)
				{
					scan_cur->scan_ref=scan_ref;

					gettimeofday(&start,NULL);
					pfm.match(scan_ref->observed_planes, scan_cur->observed_planes, scan_cur->plane_matches);
					gettimeofday(&end,NULL);
					timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
					/////// std::cout<<filenum<<"\tplane match - "<<timeused<<std::endl;

					gettimeofday(&start,NULL);
					Tcr.setIdentity();
					pe.alignScans(scan_cur,scan_ref,Tcr);
					scan_cur->Tcr=Tcr;
					scan_cur->Tcg=scan_cur->Tcr*scan_ref->Tcg;
					gettimeofday(&end,NULL);
					timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
					/////// std::cout<<filenum<<"\talign scan - "<<timeused<<std::endl;
					//std::cout<<filenum<<"\tedge-points - "<<scan_cur->point_matches.size()<<std::endl;

//					displayScans_2scans(scan_cur,scan_ref,Transform(),visualizer);
//					visualizer->spin();
//					displayScans_2scans(scan_cur,scan_ref,Tcr,visualizer);
//					visualizer->spin();

					delete scan_ref;
				}
				else
				{
					scan_cur->Tcg.setIdentity();
				}

	//			gettimeofday(&end,NULL);
	//			timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
	//			std::cout<<filenum<<"\t"<<timeused<<std::endl;

				display_addScan(scan_cur,visualizer);
				display_addCameraPose(scan_cur,visualizer);
//				vis->spinOnce(20);

//				if(filenum==0)
//				{
//					visualizer->spin();
//				}

				Tgc=scan_cur->Tcg.inv();
				fp<<std::fixed<<scan_cur->time_stamp<<" "<<Tgc.t.transpose()<<" "<<Tgc.Quaternion().transpose()<<std::endl;
				scan_ref=scan_cur;
				filenum++;

				gettimeofday(&end_all,NULL);
				timeused_all=(1000000*(end_all.tv_sec-start_all.tv_sec)+end_all.tv_usec-start_all.tv_usec)/1000;
				/////// std::cout<<std::endl<<"time used - "<<timeused_all<<std::endl;
				fp_time<<std::fixed<<scan_cur->time_stamp<<" "<<timeused_all<<std::endl;

//				createCloud(depth, color, cloud);
//				visualizer->updatePointCloud(cloud, cloudName);
			}
//			std::cout<<"running "<<running<<std::endl;
//			std::cout<<"ros::ok "<<ros::ok()<<std::endl;
//			std::cout<<"filenum "<<filenum<<std::endl;
//			if(save)
//			{
//				save = false;
//				cv::Mat depthDisp;
//				dispDepth(depth, depthDisp, 12000.0f);
//				saveCloudAndImages(cloud, color, depth, depthDisp);
//			}
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
//			visualizer->spinOnce(10);
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////
		}
		visualizer->spin();
		visualizer->close();
//		cap.close()
		fp.close();
		fp_time.close();
	}

	void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void *)
	{
//		pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
//		if (event.getKeySym () == "a" && event.keyDown ())
//		{
//		   viewer->close();
//		}
		if(event.keyUp())
		{
			switch(event.getKeyCode())
			{
			case 27:
			case 'q':
				running = false;
				break;
			case ' ':
			case 's':
				save = true;
				break;
			}
		}
	}

	void readImage(const sensor_msgs::Image::ConstPtr msgImage, cv::Mat &image) const
	{
		cv_bridge::CvImageConstPtr pCvImage;
		pCvImage = cv_bridge::toCvShare(msgImage, msgImage->encoding);
		pCvImage->image.copyTo(image);
	}

	void readCameraInfo(const sensor_msgs::CameraInfo::ConstPtr cameraInfo, cv::Mat &cameraMatrix) const
	{
		double *itC = cameraMatrix.ptr<double>(0, 0);
		for(size_t i = 0; i < 9; ++i, ++itC)
		{
			*itC = cameraInfo->K[i];
		}
	}

	void dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue)
	{
		cv::Mat tmp = cv::Mat(in.rows, in.cols, CV_8U);
		const uint32_t maxInt = 255;

		#pragma omp parallel for
		for(int r = 0; r < in.rows; ++r)
		{
			const uint16_t *itI = in.ptr<uint16_t>(r);
			uint8_t *itO = tmp.ptr<uint8_t>(r);

			for(int c = 0; c < in.cols; ++c, ++itI, ++itO)
			{
				*itO = (uint8_t)std::min((*itI * maxInt / maxValue), 255.0f);
			}
		}

		cv::applyColorMap(tmp, out, cv::COLORMAP_JET);
	}

	void combine(const cv::Mat &inC, const cv::Mat &inD, cv::Mat &out)
	{
		out = cv::Mat(inC.rows, inC.cols, CV_8UC3);

		#pragma omp parallel for
		for(int r = 0; r < inC.rows; ++r)
		{
			const cv::Vec3b
			*itC = inC.ptr<cv::Vec3b>(r),
			*itD = inD.ptr<cv::Vec3b>(r);
			cv::Vec3b *itO = out.ptr<cv::Vec3b>(r);

			for(int c = 0; c < inC.cols; ++c, ++itC, ++itD, ++itO)
			{
				itO->val[0] = (itC->val[0] + itD->val[0]) >> 1;
				itO->val[1] = (itC->val[1] + itD->val[1]) >> 1;
				itO->val[2] = (itC->val[2] + itD->val[2]) >> 1;
			}
		}
	}

	void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) const
	{
		const float badPoint = std::numeric_limits<float>::quiet_NaN();

		#pragma omp parallel for
		for(int r = 0; r < depth.rows; ++r)
		{
			pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
			const uint16_t *itD = depth.ptr<uint16_t>(r);
			const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
			const float y = lookupY.at<float>(0, r);
			const float *itX = lookupX.ptr<float>();

			for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX)
			{
				register const float depthValue = *itD / 1000.0f;
				// Check for invalid measurements
				if(*itD == 0)
				{
					// not valid
					itP->x = itP->y = itP->z = badPoint;
					itP->rgba = 0;
					continue;
				}
				itP->z = depthValue;
				itP->x = *itX * depthValue;
				itP->y = y * depthValue;
				itP->b = itC->val[0];
				itP->g = itC->val[1];
				itP->r = itC->val[2];
				itP->a = 255;
			}
		}
	}

	void saveCloudAndImages(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, const cv::Mat &color, const cv::Mat &depth, const cv::Mat &depthColored)
	{
		oss.str("");
		oss << "./" << std::setfill('0') << std::setw(4) << frame;
		const std::string baseName = oss.str();
		const std::string cloudName = baseName + "_cloud.pcd";
		const std::string colorName = baseName + "_color.jpg";
		const std::string depthName = baseName + "_depth.png";
		const std::string depthColoredName = baseName + "_depth_colored.png";

		OUT_INFO("saving cloud: " << cloudName);
		writer.writeBinary(cloudName, *cloud);
		OUT_INFO("saving color: " << colorName);
		cv::imwrite(colorName, color, params);
		OUT_INFO("saving depth: " << depthName);
		cv::imwrite(depthName, depth, params);
		OUT_INFO("saving depth: " << depthColoredName);
		cv::imwrite(depthColoredName, depthColored, params);
		OUT_INFO("saving complete!");
		++frame;
	}

	void createLookup(size_t width, size_t height)
	{
		const float fx = 1.0f / cameraMatrixColor.at<double>(0, 0);
		const float fy = 1.0f / cameraMatrixColor.at<double>(1, 1);
		const float cx = cameraMatrixColor.at<double>(0, 2);
		const float cy = cameraMatrixColor.at<double>(1, 2);
		float *it;

		lookupY = cv::Mat(1, height, CV_32F);
		it = lookupY.ptr<float>();
		for(size_t r = 0; r < height; ++r, ++it)
		{
			*it = (r - cy) * fy;
		}

		lookupX = cv::Mat(1, width, CV_32F);
		it = lookupX.ptr<float>();
		for(size_t c = 0; c < width; ++c, ++it)
		{
			*it = (c - cx) * fx;
		}
	}

//	void RosAriaNode::cmdvel_cb( const geometry_msgs::TwistConstPtr &msg)
//	{
//	  veltime = ros::Time::now();
//	  ROS_INFO( "new speed: [%0.2f,%0.2f](%0.3f)", msg->linear.x*1e3, msg->angular.z, veltime.toSec() );
//
//	  robot->lock();
//	  robot->setVel(msg->linear.x*1e3);
//	  if(robot->hasLatVel())
//		robot->setLatVel(msg->linear.y*1e3);
//	  robot->setRotVel(msg->angular.z*180/M_PI);
//	  robot->unlock();
//	  ROS_DEBUG("RosAria: sent vels to to aria (time %f): x vel %f mm/s, y vel %f mm/s, ang vel %f deg/s", veltime.toSec(),
//		(double) msg->linear.x * 1e3, (double) msg->linear.y * 1.3, (double) msg->angular.z * 180/M_PI);
//	}
};

int main (int argc, char *argv[])
{
	// some default settings;
//	std::string sequence_name="";
	std::string sequence_name="/home/zgz/sun/catkin_ws/devel/lib/plane_edge_vo/data_collect";
	double time_start=0;
	double time_interval=0.2;
	int min_inliers=10000;
	bool usePln=true, usePt=true;
	int pln_fitting_method=1;
//	double m_fp=2.85e-3, sigma_u=0.5, sigma_v=0.5;
//	int vis_every_n_frames=10;
	int total_frames=100;
	double alpha=1, thres_weight=0.01;
	int max_icp=10, max_lm=10;
	int occluding=2, curvature=0, canny=0;
	bool useWeight=true;
//	double delta_time=2, delta_angle=6.0, delta_dist=0.1;
//	std::string traj_path="/home/sun/traj/";
//	enum Mode {DEBUG, VIS_SCAN, CLOSE_LOOP, CLOSE_LOOP_FILE, TRAJ_PUZZLE, RELEASE, ONLINE, COLLECT, VIEW} mode;
//	int key_frame=1;
	Receiver::Mode mode = Receiver::ONLINE;

	for(int i=1;i<argc;i++)
	{
		if(strcmp(argv[i],"-mode")==0)
		{
			if(strcmp(argv[i+1],"debug")==0)
				mode=Receiver::DEBUG;
			if(strcmp(argv[i+1],"online")==0)
				mode=Receiver::ONLINE;
			if(strcmp(argv[i+1],"collect")==0)
				mode=Receiver::COLLECT;
		}
		if(strcmp(argv[i],"-ds")==0) {sequence_name=argv[i+1];}
		if(strcmp(argv[i],"-st")==0) {time_start=atof(argv[i+1]);}
		if(strcmp(argv[i],"-ti")==0) {time_interval=atof(argv[i+1]);}
		if(strcmp(argv[i],"-mi")==0) {min_inliers=atoi(argv[i+1]);}
		if(strcmp(argv[i],"-icp")==0) {max_icp=atoi(argv[i+1]);}
		if(strcmp(argv[i],"-lm")==0) {max_lm=atoi(argv[i+1]);}
		if(strcmp(argv[i],"-pln")==0)
		{
			if(strcmp(argv[i+1],"1")==0)
				usePln=true;
			if(strcmp(argv[i+1],"0")==0)
				usePln=false;
		}
		if(strcmp(argv[i],"-pt")==0)
		{
			if(strcmp(argv[i+1],"1")==0)
				usePt=true;
			if(strcmp(argv[i+1],"0")==0)
				usePt=false;
		}
		if(strcmp(argv[i],"-plnfit")==0) {pln_fitting_method=atoi(argv[i+1]);}
//		if(strcmp(argv[i],"-vis")==0) {vis_every_n_frames=atoi(argv[i+1]);}
		if(strcmp(argv[i],"-frames")==0) {total_frames=atoi(argv[i+1]);}
		if(strcmp(argv[i],"-alpha")==0) {alpha=atof(argv[i+1]);}
		if(strcmp(argv[i],"-thres_weight")==0) {thres_weight=atof(argv[i+1]);}
		if(strcmp(argv[i],"-occluding")==0)
		{
			if(strcmp(argv[i+1],"1")==0)
				occluding=2;
			if(strcmp(argv[i+1],"0")==0)
				occluding=0;
		}
		if(strcmp(argv[i],"-curvature")==0)
		{
			if(strcmp(argv[i+1],"1")==0)
				curvature=8;
			if(strcmp(argv[i+1],"0")==0)
				curvature=0;
		}
		if(strcmp(argv[i],"-canny")==0)
		{
			if(strcmp(argv[i+1],"1")==0)
				canny=16;
			if(strcmp(argv[i+1],"0")==0)
				canny=0;
		}
		if(strcmp(argv[i],"-useWeight")==0)
		{
			if(strcmp(argv[i+1],"1")==0)
				useWeight=true;
			if(strcmp(argv[i+1],"0")==0)
				useWeight=false;
		}
//		if(strcmp(argv[i],"-traj_path")==0) {traj_path=argv[i+1];}
//		if(strcmp(argv[i],"-delta_time")==0) {delta_time=atof(argv[i+1]);}
//		if(strcmp(argv[i],"-delta_angle")==0) {delta_angle=atof(argv[i+1]);}
//		if(strcmp(argv[i],"-delta_dist")==0) {delta_dist=atof(argv[i+1]);}
//		if(strcmp(argv[i],"-key_frame")==0) {key_frame=atoi(argv[i+1]);}
	}

	
	ros::init(argc, argv, "kinect2_viewer", ros::init_options::AnonymousName);

	if(!ros::ok())
	{
		return 0;
	}

	std::string ns = K2_DEFAULT_NS;
	std::string topicColor = K2_TOPIC_QHD K2_TOPIC_IMAGE_COLOR K2_TOPIC_IMAGE_RECT;
	std::string topicDepth = K2_TOPIC_QHD K2_TOPIC_IMAGE_DEPTH K2_TOPIC_IMAGE_RECT;
	bool useExact = true;
	bool useCompressed = false;

	topicColor = "/" + ns + topicColor;
	topicDepth = "/" + ns + topicDepth;
	OUT_INFO("topic color: " FG_CYAN << topicColor << NO_COLOR);
	OUT_INFO("topic depth: " FG_CYAN << topicDepth << NO_COLOR);


	Receiver receiver(topicColor, topicDepth, useExact, useCompressed);
	receiver.setDebug(mode==Receiver::DEBUG);
	receiver.setPlnFitMethod(pln_fitting_method);
	receiver.setMotionCoeff(usePln,usePt,useWeight,alpha,thres_weight,max_icp,max_lm);
	receiver.setInliers(min_inliers);
	receiver.setEdgeType(occluding,curvature,canny);
	receiver.setTotalFrames(total_frames);
	receiver.setDataReadCoeff(sequence_name, time_start, time_interval);

	OUT_INFO("starting receiver...");
	receiver.run(mode);

//
////	if(mode==ONLINE || mode==COLLECT)
//	{
//
//		while(filenum<total_frames)
//		{
//		}
//		//vis->spin();
//	}
//
	ros::shutdown();
	return 0;
}

/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2018-06-06 22:00
#
# Filename:		capture.cpp
#
# Description: 
#
===============================================*/
#include "capture.h"

namespace ulysses
{
	bool Capture::initialize()
	{
//		freenect2_ = new libfreenect2::Freenect2();
//		listener_ = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
////		listener_color = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color);
////		listener_ir_depth = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
//
//		libfreenect2::PacketPipeline* pipeline;
//		pipeline = new libfreenect2::CpuPacketPipeline();
//
//		std::cout << ">>> opening default device ..." << std::endl;
//		dev_ = freenect2_->openDefaultDevice(pipeline);
//		pipeline = 0;
//
//		if(dev_)
//		{
//			std::cout << ">>> Had got default device " << std::endl;
//			libfreenect2::Freenect2Device::Config config;
//			config.EnableBilateralFilter = true;
//			config.EnableEdgeAwareFilter = true;
//			config.MinDepth = 0.3f;
//			config.MaxDepth = 12.0f;
//			dev_->setConfiguration(config);
//
//			dev_->setColorFrameListener(listener_);
//			dev_->setIrAndDepthFrameListener(listener_);
////			dev_->setColorFrameListener(listener_color);
////			dev_->setIrAndDepthFrameListener(listener_ir_depth);
//
//			dev_->start();
//
//			std::cout << ">>> CameraFreenect2: device serial: " << dev_->getSerialNumber() << std::endl;
//			std::cout << ">>> CameraFreenect2: device firmware: " << dev_->getFirmwareVersion() << std::endl;
//
//			//default registration params
//			libfreenect2::Freenect2Device::IrCameraParams depthParams = dev_->getIrCameraParams();
//			libfreenect2::Freenect2Device::ColorCameraParams colorParams = dev_->getColorCameraParams();
//			reg_ = new libfreenect2::Registration(depthParams, colorParams);
//			libfreenect2::Freenect2Device::IrCameraParams params = dev_->getIrCameraParams();
//			//std::cout<<"initial params "<<params.fx<<" "<<params.fy<<" "<<params.cx<<" "<<params.cy<<std::endl;
//
//		}
//		////////////////////////////////////////////////////////////////////////////////////////////
//		////////////////////////////////////////////////////////////////////////////////////////////
//		////////////////////////////////////////////////////////////////////////////////////////////
//		////////////////////////////////////////////////////////////////////////////////////////////

//		if(openni::OpenNI::initialize() != openni::STATUS_OK )
//		{
//			std::cerr<<"OpenNI Initial Error: "<<openni::OpenNI::getExtendedError()<<std::endl;
//			return false;
//		}
//
//		if( mDevice.open( openni::ANY_DEVICE ) != openni::STATUS_OK )
//		{
//			std::cerr<<"Can't Open Device: "<<openni::OpenNI::getExtendedError()<<std::endl;
//			return false;
//		}
//
//		if( mDevice.hasSensor( openni::SENSOR_DEPTH ) )  
//		{
//			if( mDepthStream.create( mDevice, openni::SENSOR_DEPTH ) == openni::STATUS_OK )  
//			{
//				openni::VideoMode mMode;  
//				mMode.setResolution( 640, 480 );  
//				mMode.setFps( 30 );  
//				mMode.setPixelFormat( openni::PIXEL_FORMAT_DEPTH_1_MM );  
//				if( mDepthStream.setVideoMode( mMode) != openni::STATUS_OK )  
//				{  
//					std::cout << "Can't apply VideoMode: "<< openni::OpenNI::getExtendedError() << std::endl;  
//				}  
//			}
//			else  
//			{
//				std::cerr<<"Can't create depth stream on device: "<<openni::OpenNI::getExtendedError()<<std::endl;  
//				return false;  
//			}
//		}
//		else  
//		{  
//			std::cerr << "ERROR: This device does not have depth sensor" << std::endl;  
//			return false;  
//		}  
//
//		if( mDevice.hasSensor( openni::SENSOR_COLOR ) )
//		{
//			if( mColorStream.create( mDevice, openni::SENSOR_COLOR ) == openni::STATUS_OK )  
//			{
//				openni::VideoMode mMode;  
//				mMode.setResolution( 640, 480 );  
//				mMode.setFps( 30 );  
//				mMode.setPixelFormat( openni::PIXEL_FORMAT_RGB888 );  
//				if( mColorStream.setVideoMode( mMode) != openni::STATUS_OK )  
//				{  
//					std::cout << "Can't apply VideoMode: "<< openni::OpenNI::getExtendedError() << std::endl;  
//				}  
//				if( mDevice.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR ) )  
//				{
//					mDevice.setImageRegistrationMode( openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR );  
//				}
//			}  
//			else  
//			{  
//				std::cerr << "Can't create color stream on device: "<< openni::OpenNI::getExtendedError() << std::endl;
//				return false;
//			}
//		}
//
//		if( mDevice.isImageRegistrationModeSupported( openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR ) )  
//		{  
//			status=mDevice.setImageRegistrationMode( openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR );  
//		}
		
		fp_rgb.open("data/rgb.txt",std::ios::app);
		fp_dep.open("data/depth.txt",std::ios::app);
		fp_gt.open("data/groundtruth.txt",std::ios::app);
		fp_dep<<"# depth maps"<<std::endl<<"# file: 'xxx.bag'"<<std::endl<<"# timestamp filename"<<std::endl;
		fp_rgb<<"# color maps"<<std::endl<<"# file: 'xxx.bag'"<<std::endl<<"# timestamp filename"<<std::endl;
		fp_gt<<"# ground truth trajectory"<<std::endl<<"# file: 'rgbd_dataset_freiburg1_desk.bag'"<<std::endl<<"# timestamp tx ty tz qx qy qz qw"<<std::endl;
		fp_rgb.close();
		fp_dep.close();
		fp_gt.close();
		return true;
	}

	void Capture::close()
	{
//		delete freenect2_;
//		delete listener_;
//
//		delete listener_color;
//		delete listener_ir_depth;
//		////////////////////////////////////////////////////////////////////////////////////////////
//		////////////////////////////////////////////////////////////////////////////////////////////
//		////////////////////////////////////////////////////////////////////////////////////////////
//		////////////////////////////////////////////////////////////////////////////////////////////
//		mDepthStream.destroy();  
//		mColorStream.destroy();  
//		mDevice.close();  
//		openni::OpenNI::shutdown(); 
	}

	void Capture::loadScan(Scan *scan, IntrinsicParam cam)
	{
//		timeval time;
//		fp_rgb.open("data/rgb.txt",std::ios::app);
//		fp_dep.open("data/depth.txt",std::ios::app);
//		fp_gt.open("data/groundtruth.txt",std::ios::app);
//		char name[50];
//
//		if(dev_ && listener_)
//		{
//			libfreenect2::FrameMap frames;
//			libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
//
//			if(!listener_->waitForNewFrame(frames,100))
//			{
//				std::cout << "*** CameraFreenect2: Failed to get frames!" << std::endl;
//			}
//			else
//			{
//				libfreenect2::Frame* rgbFrame = 0;
//				libfreenect2::Frame* irFrame = 0;
//				libfreenect2::Frame* depthFrame = 0;
//
//				rgbFrame = uValue(frames, libfreenect2::Frame::Color, (libfreenect2::Frame*)0);
//				depthFrame = uValue(frames, libfreenect2::Frame::Depth, (libfreenect2::Frame*)0);
//				reg_->apply(rgbFrame, depthFrame, &undistorted, &registered);
//
//				if(rgbFrame && depthFrame)
//				{
////					img_rgb=cv::Mat(registered.height, registered.width, CV_8UC4, registered.data);
//					cv::Mat(registered->height, registered->width, CV_8UC4, registered->data).convertTo(img_rgb, CV_8U3, 1);
//					gettimeofday(&time,NULL);
//					time_stamp=time.tv_sec+time.tv_usec/1.0e6;
//					fp_rgb<<std::fixed<<time_stamp<<" rgb/"<<std::fixed<<time_stamp<<".png"<<std::endl;
//					sprintf(name,"data/rgb/%lf.png",time_stamp);
//					cv::imwrite(name,img_rgb);
////					img_rgb=cv::imread(name);
//
//					cv::Mat(depthFrame->height, depthFrame->width, CV_32FC1, depthFrame->data).convertTo(img_depth, CV_16U, 1);
//					gettimeofday(&time,NULL);
//					time_stamp=time.tv_sec+time.tv_usec/1.0e6;
//					fp_dep<<std::fixed<<time_stamp<<" depth/"<<std::fixed<<time_stamp<<".png"<<std::endl;
//					sprintf(name,"data/depth/%lf.png",time_stamp);
//					cv::imwrite(name,img_depth);
////					img_depth=cv::imread(name,-1);
//				}
//				listener_->release(frames);
//			}
//		}
//
//		Transform Tgc;
//		fp_gt<<std::fixed<<time_stamp<<" "<<Tgc.t.transpose()<<" "<<Tgc.Quaternion().transpose()<<std::endl;
//
//		fp_rgb.close();
//		fp_dep.close();
//		fp_gt.close();
		//////////////////////////////////////////////////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////////////////////////////

//		std::ofstream fp;
//		fp.open("capture.txt",std::ios::out);
		//std::cout<<"loadScan params "<<params.fx<<" "<<params.fy<<" "<<params.cx<<" "<<params.cy<<std::endl;
//		cam.fx=367.933;//params.fx;
//		cam.fy=367.933;//params.fy;
//		cam.cx=254.169;//params.cx;
//		cam.cy=204.267;//params.cy;
//		cam.width=512;
//		cam.height=424;
//		cam.factor=1000.0;

		scan->point_cloud=pcl::PointCloud<pcl::PointXYZRGBA>::Ptr (new pcl::PointCloud<pcl::PointXYZRGBA>);
		scan->normal_cloud=pcl::PointCloud<pcl::Normal>::Ptr (new pcl::PointCloud<pcl::Normal>);
		scan->pixel_cloud=pcl::PointCloud<pcl::PointXY>::Ptr (new pcl::PointCloud<pcl::PointXY>);

		cv::Mat rgb_image,depth_image;
		uint8_t *depth_ptr,*rgb_ptr;
		pcl::PointXYZRGBA point_tmp;
		unsigned short *depth_tmp_ptr=new unsigned short;
		pcl::PointXY tmp_pointxy;

		scan->point_cloud->clear();
		scan->normal_cloud->clear();
		scan->pixel_cloud->clear();
		
		//capture();

//		cv::imwrite("rgb.png",img_rgb);
//		cv::imwrite("depth.png",img_depth);
//		std::cout<<"rgb channels "<<img_rgb.channels()<<" "<<img_rgb.depth()<<" "<<img_rgb.elemSize()<<" "<<img_rgb.elemSize1()<<std::endl;
//		std::cout<<"depth channels "<<img_depth.channels()<<" "<<img_depth.depth()<<" "<<img_depth.elemSize()<<" "<<img_depth.elemSize1()<<std::endl;

		scan->time_stamp=time_stamp;
		scan->img_rgb=img_rgb;
		scan->img_depth=img_depth;

//		std::cout<<"scan "<<std::endl;
//		std::cout<<"rgb channels "<<scan->img_rgb.channels()<<" "<<scan->img_rgb.depth()<<" "<<scan->img_rgb.elemSize()<<" "<<scan->img_rgb.elemSize1()<<std::endl;
//		std::cout<<"depth channels "<<scan->img_depth.channels()<<" "<<scan->img_depth.depth()<<" "<<scan->img_depth.elemSize()<<" "<<scan->img_depth.elemSize1()<<std::endl;

		rgb_ptr=scan->img_rgb.data;
		depth_ptr=scan->img_depth.data;
//		rgb_ptr=img_rgb.data;
//		depth_ptr=img_depth.data;
		for(int i=0;i<scan->img_rgb.rows;i++)
		{
			for(int j=0;j<scan->img_rgb.cols;j++)
			{
				//rgb_ptr++;
				point_tmp.b=*rgb_ptr;
				rgb_ptr++;
				point_tmp.g=*rgb_ptr;
				rgb_ptr++;
				point_tmp.r=*rgb_ptr;
//				rgb_ptr+=2;
				rgb_ptr++;
				memcpy(depth_tmp_ptr,depth_ptr,2);
				depth_ptr+=2;
				if(j<300 || j>=800 || i<80 || i>=500)
					continue;
				point_tmp.z=*depth_tmp_ptr/cam.factor;
				point_tmp.x=(j-cam.cx)*point_tmp.z/cam.fx;
				point_tmp.y=(i-cam.cy)*point_tmp.z/cam.fy;
//				fp<<(int)point_tmp.r<<"\t"<<(int)point_tmp.g<<"\t"<<(int)point_tmp.b<<"\t";
//				fp<<point_tmp.x<<"\t"<<point_tmp.y<<"\t"<<point_tmp.z<<std::endl;
				scan->point_cloud->push_back(point_tmp);
			}
//			fp<<std::endl;
		}
		delete depth_tmp_ptr;
		scan->point_cloud->width=500;//cam.width;
		scan->point_cloud->height=420;//cam.height;

		pcl::IntegralImageNormalEstimation<pcl::PointXYZRGBA, pcl::Normal> normal_estimate_integral;
//		normal_estimate_integral.setNormalEstimationMethod (method);
//		normal_estimate_integral.setMaxDepthChangeFactor(MaxDepthChangeFactor);
//		normal_estimate_integral.setNormalSmoothingSize(NormalSmoothingSize);
		normal_estimate_integral.setInputCloud(scan->point_cloud);
		normal_estimate_integral.compute (*scan->normal_cloud);
		for(int v=0;v<scan->point_cloud->height;v++)
		{
			for(int u=0;u<scan->point_cloud->width;u++)
			{
				tmp_pointxy.x=u;
				tmp_pointxy.y=v;
				scan->pixel_cloud->push_back(tmp_pointxy);
			}
		}

//		fp.close();
	}


	void Capture::capture(cv::Mat color, cv::Mat depth)
	{
		timeval time;
		fp_rgb.open("data/rgb.txt",std::ios::app);
		fp_dep.open("data/depth.txt",std::ios::app);
		fp_gt.open("data/groundtruth.txt",std::ios::app);
		char name[50];
		bool getFrame;

		timeval start, end;
		double timeused;

		img_rgb=color;
		img_depth=depth;

//		if(dev_ && listener_)
////		if(dev_ && listener_color && listener_ir_depth)
//		{
//			gettimeofday(&start,NULL);
//			libfreenect2::FrameMap frames;
//			libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
//
//			getFrame=true;
//			listener_->waitForNewFrame(frames);
////			listener_color->waitForNewFrame(frames);
////			listener_ir_depth->waitForNewFrame(frames);
//
//			gettimeofday(&end,NULL);
//			timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
//			std::cout<<"\tcapture\tgetFrame - "<<timeused<<std::endl;
//
//			if(getFrame)
//			{
//				libfreenect2::Frame* rgbFrame = 0;
//				libfreenect2::Frame* irFrame = 0;
//				libfreenect2::Frame* depthFrame = 0;
//
//				rgbFrame = uValue(frames, libfreenect2::Frame::Color, (libfreenect2::Frame*)0);
//				depthFrame = uValue(frames, libfreenect2::Frame::Depth, (libfreenect2::Frame*)0);
//
////				listener_->onNewFrame(libfreenect2::Frame::Color, rgbFrame);
////				listener_->onNewFrame(libfreenect2::Frame::Depth, depthFrame);
//
//				reg_->apply(rgbFrame, depthFrame, &undistorted, &registered);
//
//				if(rgbFrame && depthFrame)
//				{
//					//std::cout<<"rgb "<<rgbFrame->format<<", "<<rgbFrame->height<<", "<<rgbFrame->width<<", "<<rgbFrame->bytes_per_pixel<<std::endl;
//					//std::cout<<"depth "<<depthFrame->format<<", "<<depthFrame->height<<", "<<depthFrame->width<<", "<<depthFrame->bytes_per_pixel<<std::endl;
//					cv::Mat rgb(registered.height, registered.width, CV_8UC4, registered.data);
//					std::vector<cv::Mat> channels;
//					cv::split(rgb,channels);
//					channels[3].deallocate();
//					channels.pop_back();
//					merge(channels,img_rgb);
//					rgb.deallocate();
//					for(size_t i=0;i<channels.size();i++)
//					{
//						channels[i].deallocate();
//					}
////					img_rgb=cv::Mat(registered.height, registered.width, CV_8UC4, registered.data);
					gettimeofday(&time,NULL);
					time_stamp=time.tv_sec+time.tv_usec/1.0e6;
					fp_rgb<<std::fixed<<time_stamp<<" rgb/"<<std::fixed<<time_stamp<<".png"<<std::endl;
					sprintf(name,"data/rgb/%lf.png",time_stamp);
					cv::imwrite(name,img_rgb);
//					//img_rgb=cv::imread(name);
//
//					cv::Mat(depthFrame->height, depthFrame->width, CV_32FC1, depthFrame->data).convertTo(img_depth, CV_16U, 1);
					gettimeofday(&time,NULL);                                                                    
					time_stamp=time.tv_sec+time.tv_usec/1.0e6;
					fp_dep<<std::fixed<<time_stamp<<" depth/"<<std::fixed<<time_stamp<<".png"<<std::endl;
					//img_depth=depth;
					sprintf(name,"data/depth/%lf.png",time_stamp);
					cv::imwrite(name,img_depth);
//					//img_depth=cv::imread(name,-1);
//				}
//
//				listener_->release(frames);
//			}
//		}
//		////////////////////////////////////////////////////////////////////////////////////////////
//		////////////////////////////////////////////////////////////////////////////////////////////
//		////////////////////////////////////////////////////////////////////////////////////////////
//		////////////////////////////////////////////////////////////////////////////////////////////

//		mDepthStream.start();
//		mColorStream.start();
//
//		if( mColorStream.isValid() )  
//		{
//			if(mColorStream.readFrame(&mColorFrame) ==openni::STATUS_OK)
//			{
//				gettimeofday(&time,NULL);
//				time_stamp=time.tv_sec+time.tv_usec/1.0e6;
//				fp_rgb<<std::fixed<<time_stamp<<" rgb/"<<std::fixed<<time_stamp<<".png"<<std::endl;
//				const cv::Mat mImageRGB(mColorFrame.getHeight(),mColorFrame.getWidth(),
//										CV_8UC3,(void*)mColorFrame.getData());
//				cv::cvtColor(mImageRGB,img_rgb,CV_RGB2BGR);
//				sprintf(name,"data/rgb/%lf.png",time_stamp);
//				cv::imwrite(name,img_rgb);
//			}
//		}
//
//		if( mDepthStream.readFrame( &mDepthFrame ) == openni::STATUS_OK )  
//		{  
//			gettimeofday(&time,NULL);
//			time_stamp=time.tv_sec+time.tv_usec/1.0e6;
//			fp_dep<<std::fixed<<time_stamp<<" depth/"<<std::fixed<<time_stamp<<".png"<<std::endl;
//			const cv::Mat mImageDepth(mDepthFrame.getHeight(),mDepthFrame.getWidth(),
//									  CV_16UC1,(void*)mDepthFrame.getData());
//			img_depth=mImageDepth;
//			sprintf(name,"data/depth/%lf.png",time_stamp);
//			cv::imwrite(name,img_depth);
//		}

		Transform Tgc;
		fp_gt<<std::fixed<<time_stamp<<" "<<Tgc.t.transpose()<<" "<<Tgc.Quaternion().transpose()<<std::endl;

		fp_rgb.close();
		fp_dep.close();
		fp_gt.close();
	}

	bool Collect::initialize()
	{
		fp_rgb.open("data_collect/rgb.txt",std::ios::app);
		fp_dep.open("data_collect/depth.txt",std::ios::app);
		fp_gt.open("data_collect/groundtruth.txt",std::ios::app);
		fp_dep<<"# depth maps"<<std::endl<<"# file: 'xxx.bag'"<<std::endl<<"# timestamp filename"<<std::endl;
		fp_rgb<<"# color maps"<<std::endl<<"# file: 'xxx.bag'"<<std::endl<<"# timestamp filename"<<std::endl;
		fp_gt<<"# ground truth trajectory"<<std::endl<<"# file: 'rgbd_dataset_freiburg1_desk.bag'"<<std::endl<<"# timestamp tx ty tz qx qy qz qw"<<std::endl;
		fp_rgb.close();
		fp_dep.close();
		fp_gt.close();
		return true;
	}

	void Collect::collect(cv::Mat color, cv::Mat depth)
	{
		timeval time;
		fp_rgb.open("data_collect/rgb.txt",std::ios::app);
		fp_dep.open("data_collect/depth.txt",std::ios::app);
		fp_gt.open("data_collect/groundtruth.txt",std::ios::app);
		char name[50];

		timeval start, end;
		double timeused;

		img_rgb=color;
		img_depth=depth;

		gettimeofday(&time,NULL);
		time_stamp=time.tv_sec+time.tv_usec/1.0e6;
		fp_rgb<<std::fixed<<time_stamp<<" rgb/"<<std::fixed<<time_stamp<<".png"<<std::endl;
		sprintf(name,"data_collect/rgb/%lf.png",time_stamp);
		cv::imwrite(name,img_rgb);

		gettimeofday(&time,NULL);                                                                    
		time_stamp=time.tv_sec+time.tv_usec/1.0e6;
		fp_dep<<std::fixed<<time_stamp<<" depth/"<<std::fixed<<time_stamp<<".png"<<std::endl;
		//img_depth=depth;
		sprintf(name,"data_collect/depth/%lf.png",time_stamp);
		cv::imwrite(name,img_depth);

		Transform Tgc;
		fp_gt<<std::fixed<<time_stamp<<" "<<Tgc.t.transpose()<<" "<<Tgc.Quaternion().transpose()<<std::endl;

		fp_rgb.close();
		fp_dep.close();
		fp_gt.close();
	}
}

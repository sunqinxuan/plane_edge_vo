/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2017-10-18 20:12
#
# Filename: edge_point_extraction.h
#
# Description: 
#
===============================================*/

#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <sys/time.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <boost/make_shared.hpp>
#include <boost/thread/thread.hpp>
#include <pcl-1.8/pcl/features/organized_edge_detection.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.8/pcl/segmentation/rgb_plane_coefficient_comparator.h>
#include <pcl-1.8/pcl/common/centroid.h>
#include <pcl-1.8/pcl/common/eigen.h>
#include <eigen3/Eigen/Eigenvalues>
#include <limits>
#include "ANN/ANN.h"
#include "types.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace ulysses
{
	class EdgePointExtraction : public pcl::OrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>
	{
	public:
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_NAN_BOUNDARY;
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_OCCLUDING;
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_OCCLUDED;
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_HIGH_CURVATURE;
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_RGB_CANNY;	

		EdgePointExtraction()
		{
			float th_dd = 0.05f;
			int max_search = 50;
			// edge detection;
			setDepthDisconThreshold (th_dd);
			setMaxSearchNeighbors (max_search);
			setEdgeType (EDGELABEL_OCCLUDING); // EDGELABEL_HIGH_CURVATURE | EDGELABEL_OCCLUDING | EDGELABEL_OCCLUDED
			remove("extract_EdgePoints.txt");
		}

		~EdgePointExtraction() {}

		void setDebug(bool d) {debug=d;}

		void extractEdgePoints(Scan *scan);

	private:
		
		bool debug;

		// radius=0.1m;
		static constexpr double sqRad_ANN=0.01;
		// K=20;
		static const int K_ANN=20;

	};
}


/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-10-17 22:13
#
# Filename:		plane_param_estimation.h
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
#include <float.h>
#include <eigen3/Eigen/Eigenvalues>
#include <boost/make_shared.hpp>
#include <boost/thread/thread.hpp>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.8/pcl/common/centroid.h>
#include <pcl-1.8/pcl/common/eigen.h>
#include <limits>
#include "types.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace ulysses
{
	class PlaneParamEstimation
	{
	public:

		PlaneParamEstimation() 
		{
			remove("plane_param_estimation.txt");
			pln_fitting_method=0;
		}
		~PlaneParamEstimation() {}

		void setDebug(bool d) {debug=d;}
		void setPlnFittingMethod(int i) {pln_fitting_method=i;}

		void estimatePlaneParams(Plane *plane, IntrinsicParam cam);

	private:
		
		bool debug;
		std::ofstream fp;
		int pln_fitting_method;

		void compute_point_weight(Point &point, Eigen::Vector3d n, double d, IntrinsicParam cam);

		void compute_plane_centroid(Plane *plane);

		void compute_plane_cov_inv(Plane *plane);

		void compute_scatter_matrix(Plane *plane);
	};
}


/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-11-02 09:24
#
# Filename:		pose_estimation.h
#
# Description: 
#
===============================================*/
#pragma once
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <sys/time.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <boost/make_shared.hpp>
#include <boost/thread/thread.hpp>
#include <pcl-1.8/pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl-1.8/pcl/segmentation/planar_region.h>
#include <pcl-1.8/pcl/features/organized_edge_detection.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.8/pcl/registration/transforms.h>
#include <pcl-1.8/pcl/filters/voxel_grid.h>
#include <limits>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <eigen3/Eigen/Eigenvalues>
#include "ANN/ANN.h"
#include "types.h"

namespace ulysses
{
	// MeasurementPlane
	// - two planes observed in current and reference frames;
	class MeasurementPlane
	{
	public:
		// pln = [n^T,d]^T;
		Eigen::Vector4d pln_cur, pln_ref;

		Eigen::Vector4d transform_pln_ref(g2o::SE3Quat Tcr)
		{
			Eigen::Vector4d pln_ref_trans;
			Eigen::Matrix3d Rcr=Tcr.rotation().toRotationMatrix();
			Eigen::Vector3d tcr=Tcr.translation();
			pln_ref_trans.block<3,1>(0,0)=Rcr*pln_ref.block<3,1>(0,0);
			pln_ref_trans(3)=pln_ref(3)-pln_ref_trans.block<3,1>(0,0).transpose()*tcr;
			return pln_ref_trans;
		}
	};

	// MeasurementPoint
	// - two edge points observed in current and reference frames;
	class MeasurementPoint
	{
	public:
		Eigen::Vector3d pt_cur, pt_ref;

		Eigen::Vector3d transform_pt_ref(g2o::SE3Quat Tcr)
		{
			return Tcr*pt_ref;	
		}
	};

	// Edge_Plane
	// - binary edge;
	// - vertices
	//   - vertices[0]: camera pose of current frame;
	//   - vertices[1]: camera pose of reference frame;
	// - measurement: MeasurementPlane;
	// - error = [c_pi-Tcr(r_pi)]^T*Omega*[c_pi-Tcr(r_pi)];
	class Edge_Plane: public g2o::BaseBinaryEdge< 4, MeasurementPlane, g2o::VertexSE3Expmap, g2o::VertexSE3Expmap>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		Edge_Plane() 
		{
			debug=false;
		}
		
		virtual void computeError();

//		virtual void linearizeOplus();

		virtual bool read ( std::istream& in ) {}
		virtual bool write ( std::ostream& out ) const {}
		
		void setDebug(bool d) {debug=d;}


	private:

		bool debug;
	};

	// Edge_EdgePoint
	// - binary edge;
	// - vertices
	//   - vertices[0]: camera pose of current frame;
	//   - vertices[1]: camera pose of reference frame;
	// - measurement: MeasurementPoint;
	// - error = [c_pi-Tcr(r_pi)]^T*Omega*[c_pi-Tcr(r_pi)];
	class Edge_EdgePoint: public g2o::BaseBinaryEdge< 3, MeasurementPoint, g2o::VertexSE3Expmap, g2o::VertexSE3Expmap>
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		Edge_EdgePoint() 
		{
			debug=false;
		}
		
		virtual void computeError();

//		virtual void linearizeOplus();

		virtual bool read ( std::istream& in ) {}
		virtual bool write ( std::ostream& out ) const {}
		
		void setDebug(bool d) {debug=d;}


	private:

		bool debug;
	};

	class PoseEstimation
	{
	public:
		enum ConstraintCase { DoF_6, DoF_5, DoF_3, none };

		PoseEstimation() 
		{
			max_iter_icp=10;
			max_iter_g2o=10;
			alpha=1;
			Weight_p=1;
			thres_weight=0.01;
			remove("alignPlanes.txt");
			remove("alignScans.txt");
//			remove("alpha.txt");
//			remove("gradient.txt");
			debug=false;
			usePln=true;
			usePt=true;
			useWeight=true;
		}

		~PoseEstimation() {}
		
		void setDebug(bool d) {debug=d;}
		void usePlnPt(bool pln, bool pt) {usePln=pln; usePt=pt;}
		void useEdgeWeight(bool w) {useWeight=w;}

		ConstraintCase constraint_case(std::vector<PlanePair> matched_planes);

		Transform alignPlanes(std::vector<PlanePair> matched_planes);

		// alignScans
		// - align two scans using planes and edges;
		// - initial guess is scan->Tcr;
		double alignScans(Scan *scan, Scan *scan_ref, Transform &Tcr);

		void setMaxIterationICP(int i) {max_iter_icp=i;}
		void setMaxIterationLM(int i) {max_iter_g2o=i;}

		void setAlpha(double a) {alpha=a;}
		void setThresWeight(double t) {thres_weight=t;}

		Eigen::Vector3d getHSingulars() {return H_singularValues;}

	private:
		
		bool debug;
		ofstream fp;

		bool usePln, usePt;
		bool useWeight;

		int max_iter_icp;
		int max_iter_g2o;
		double thres_weight;

		static const int MAX_PLNS=10;

		Eigen::Matrix3d H, H_svd_U, H_svd_V;
		Eigen::Vector3d H_singularValues;
		ConstraintCase _case;

		double alpha;
		// largest sqrt eigenvalue of Psi_pi;
		double lambda_pi_1;
		// corresponding eigenvector;
		Eigen::Matrix<double,6,1> q_pi_1;
		// index of the largest eigenvalue;
		int idx_1;
		// all the eigenvalues and eigenvectors of Psi_pi;
		Eigen::Matrix<double,6,1> Lambda_pi;
		Eigen::Matrix<double,6,6> Q_pi;
		double Weight_p;

//		void addCameraPose(g2o::SparseOptimizer *optimizer, Transform Tcr);
		double optimize(Scan *scan, Transform &Tcr);

		void confirmPlaneCorrespondence(Scan *scan_cur, Transform &Tcr);

		// buildCorrespondence
		// - build correspondences between scan_cur and scan_ref;
		// - and store them in the scan_cur.plane_matches;
		void buildCorrespondence(Scan *scan_cur, Scan *scan_ref, Transform Tcr, Transform Tcr_pre);

		// planeDist
		// - distance between two planes;
		// - return [pln_cur-Tcr(pln_ref)]^T*Cov_cur_inv*[pln_cur-Tcr(pln_ref)];
//		double planeDist(Plane *plane_cur, Plane *plane_ref, Transform Tcr);

		void compute_Psi_pi(Scan *scan, Transform Tcr);
		
		void compute_EdgePoint_weight(Scan *scan, Transform Tcr);

//		Eigen::Matrix4d transform_Pln_cov(Eigen::Matrix4d cov, Transform Tcr);

	};
}


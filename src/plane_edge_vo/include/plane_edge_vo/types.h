/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2018-02-03 11:21
#
# Filename: types.h
#
# Description: 
#
===============================================*/


#pragma once
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <vector>
#include <math.h>
#include <list>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Eigenvalues>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/point_cloud.h>
#include <pcl-1.8/pcl/filters/extract_indices.h>
#include <pcl-1.8/pcl/segmentation/planar_region.h>
#include <pcl-1.8/pcl/common/eigen.h>
#include <opencv2/core/core.hpp>

namespace ulysses
{
	// IntrinsicParam 
	struct IntrinsicParam
	{
		IntrinsicParam() // {fx=525;fy=525;cx=240;cy=320;}
		{
			fx=540.686;
			fy=540.686;
			cx=479.75;
			cy=269.75;
			width=960;
			height=540;
			//fx=367.933;//params.fx;
			//fy=367.933;//params.fy;
			//cx=254.169;//params.cx;
			//cy=204.267;//params.cy;
			//width=512;
			//height=424;
			factor=1000.0;

//			fx=517.3;//591.1;//567.6;//580.8;//525;
//			fy=516.5;//590.1;//570.2;//581.8;//525;
//			cx=318.6;//331;//324.7;//308.8;//319.5;
//			cy=255.3;//234;//250.1;//253;//239.5;
			m_fp=2.85e-3; //m^-1
			sigma_disparity=0.5; //pixel
			sigma_u=0.5; //pixel
			sigma_v=0.5; //pixel
//			factor=5000.0;
//			width=640;
//			height=480;
		}
		IntrinsicParam(double fx_, double fy_, double cx_, double cy_) {fx=fx_;fy=fy_;cx=cx_;cy=cy_;}
//		IntrinsicParam(const IntrinsicParam int_param) {fx=int_param.fx;fy=int_param.fy;cx=imt_param.cx;cy=int_param.cy;}

		double fx,fy,cx,cy;
		double m_fp;
		double sigma_disparity, sigma_u, sigma_v;
		double factor;
		int width, height;

		// getMatrix
		// - K =
		// - |fx 0  cx|
		//   |0  fy cy|
		//   |0  0  1 |
		Eigen::Matrix3d getMatrix()
		{
			Eigen::Matrix3d mat;
			mat.setIdentity();
			mat(0,0)=fx;
			mat(1,1)=fy;
			mat(0,2)=cx;
			mat(1,2)=cy;
			return mat;
		}

		// project
		// - project the point in 3d camera frame into the pixel frame;
		// - u=Kp;
		// - u is the homogeneous coordinates;
		// - u=[col,row,1]^T;
		Eigen::Vector3d project(Eigen::Vector3d p)
		{
			Eigen::Vector3d tmp,u;
			tmp=getMatrix()*p/p(2);
			u(0)=tmp(0);
			u(1)=tmp(1);
			u(2)=1;
			//u(0) = p[0]*fx/p[2] + cx;
			//u(1) = p[1]*fy/p[2] + cy;
			return u;
		}
	};

	struct Plane;

	struct Point
	{
		// xyz in meters;
		Eigen::Vector3d xyz, normal;

		// coordinate in the PPS for the local plane parameters (after Rotation_PCA);
		Eigen::Vector3d pps;
		void Cartesian2PPS(Eigen::Matrix3d Rotation_PCA=Eigen::Matrix3d::Identity())
		{
			Eigen::Vector3d tmp_vec3d;
			tmp_vec3d=Rotation_PCA*normal;
			if(tmp_vec3d(2)>1.0-1e-20)
			{
				tmp_vec3d(2)=1.0;
			}
			if(tmp_vec3d(2)<-1.0+1e-20)
			{
				tmp_vec3d(2)=-1.0;
			}
			pps(0)=acos(tmp_vec3d(2));
//			if(pps(0)>M_PI)
//			{
//				pps(0)=M_PI*2-pps(0);
//			}
			pps(1)=atan2(tmp_vec3d(1),tmp_vec3d(0));
			pps(2)=-normal.dot(xyz);
		}

		// rgb \in [0,1]^3;
		Eigen::Vector3d rgb;

		// u,v: pixel coordinate;
		int u,v; // u<480, v<640

		// cov of the point;
		// measurement uncertainty;
		Eigen::Matrix3d cov, cov_inv; 
		void compute_cov(IntrinsicParam& cam)
		{
			double sigma_d=cam.m_fp*cam.sigma_disparity*xyz(2)*xyz(2); //m
			double d_fu=xyz(2)/cam.fx;
			double d_fv=xyz(2)/cam.fy;
			double x_d=xyz(0)/xyz(2);
			double y_d=xyz(1)/xyz(2);
			cov(0,0)=d_fu*d_fu*cam.sigma_u*cam.sigma_u+x_d*x_d*sigma_d*sigma_d;
			cov(0,1)=x_d*y_d*sigma_d*sigma_d;
			cov(0,2)=x_d*sigma_d*sigma_d;
			cov(1,0)=cov(0,1);
			cov(1,1)=d_fv*d_fv*cam.sigma_v*cam.sigma_v+y_d*y_d*sigma_d*sigma_d;
			cov(1,2)=y_d*sigma_d*sigma_d;
			cov(2,0)=cov(0,2);
			cov(2,1)=cov(1,2);
			cov(2,2)=sigma_d*sigma_d;
			cov_inv=cov.inverse();
		}

		// weight in the plane fitting;
		// weight = pln_n^T * cov_inv * pln_n;
		// weight_angle = [p_pi/(pln_n^T*p_pi)]^T * cov_inv * [p_pi/(pln_n^T*p_pi)];
		double weight;

	};

	struct Transform
	{
		Transform()
		{
			R=Eigen::Matrix3d::Identity();
			t=Eigen::Vector3d::Zero();
		}

		Transform(const Transform& T)
		{
			R=T.R;
			t=T.t;
			time_stamp=T.time_stamp;
		}

		Transform(Eigen::Matrix3d R_, Eigen::Vector3d t_)
		{
			R=R_;
			t=t_;
		}

		Transform(Eigen::Quaterniond Q_, Eigen::Vector3d t_)
		{
			R=Q_.toRotationMatrix();
			t=t_;
		}

		Eigen::Vector3d t;
		Eigen::Matrix3d R;
		double time_stamp;

//		IntrinsicParam intrinsic;

//		std::vector<Plane*> ptr_planes;

		// eular angle: Z-Y-X
		Eigen::Vector3d Eulars() {return R.eulerAngles(2,1,0);}

		Eigen::Quaterniond Quat() {return Eigen::Quaterniond(R);}
		Eigen::Vector4d Quaternion()
		{
			Eigen::Quaterniond q=Quat();
			Eigen::Vector4d vec;
			vec.block<3,1>(0,0)=q.vec();
			vec(3)=q.w();
			return vec;
		}

		void inverse()
		{
			Eigen::Matrix3d R_tmp;
			Eigen::Vector3d t_tmp;
			R_tmp=R.transpose();
			t_tmp=-R.transpose()*t;
			R=R_tmp;
			t=t_tmp;
		}

		Transform inv()
		{
			Eigen::Matrix3d R_tmp;
			Eigen::Vector3d t_tmp;
			R_tmp=R.transpose();
			t_tmp=-R.transpose()*t;
			Transform T(R_tmp,t_tmp);
			T.time_stamp=time_stamp;
			return T;
		}

		void setIdentity()
		{
			R=Eigen::Matrix3d::Identity();
			t=Eigen::Vector3d::Zero();
		}

		Transform operator* (const Transform& tr2) const
		{
			Transform result;
			result.t=t+R*tr2.t;
			result.R=R*tr2.R;
			return result;
		}

		void leftMultiply(Eigen::Isometry3d T)
		{
			Eigen::Matrix3d R_tmp;
			Eigen::Vector3d t_tmp;
			R_tmp=T.rotation()*R;
			t_tmp=T.rotation()*t+T.translation();
			R=R_tmp;
			t=t_tmp;
		}

		Eigen::Vector3d transformPoint(Eigen::Vector3d p)
		{
			return R*p+t;
		}

		Eigen::Vector4d transformPlane(Eigen::Vector4d pln)
		{
			Eigen::Vector4d pln_trans;
			pln_trans.block<3,1>(0,0)=R*pln.block<3,1>(0,0);
			pln_trans(3)=pln(3)-pln_trans.block<3,1>(0,0).transpose()*t;
			return pln_trans;
		}

		Eigen::Matrix4f getMatrix4f()
		{
			Eigen::Matrix4f transform;
			transform.setIdentity();
			for(int i=0;i<3;i++)
			{
				transform(i,3)=(float)t(i);
				for(int j=0;j<3;j++)
				{
					transform(i,j)=(float)R(i,j);
				}
			}
			return transform;
		}
	};

	struct EdgePoint
	{
		EdgePoint() {isEdge=false;}
		~EdgePoint()
		{
			std::vector<EdgePoint*> tmp;
			tmp.swap(neighbors);
		}

		Eigen::Vector3d xyz;
		bool isEdge;
		
		Eigen::Matrix3d cov;
		std::vector<EdgePoint*> neighbors;
	};

	struct EdgePointPair
	{
		EdgePointPair() {}

		EdgePointPair(EdgePoint *c, EdgePoint *r)
		{
			cur=c;
			ref=r;
		}

		EdgePoint *cur;
		EdgePoint *ref;

		double weight;
		// gradient = {\partial Jpk}/{\partial \xi};
		Eigen::Matrix<double,6,1> gradient;
		Eigen::Matrix<double,6,6> Psi_pk;
		double sq_lambda_pk;
		Eigen::Matrix<double,6,1> v_pk;

		Eigen::Matrix3d computeCov(Transform Tcr)
		{
			return Tcr.R*ref->cov*Tcr.R.transpose()+cur->cov;
//			return cur->cov;
		}

		double pointDist(Transform Tcr)
		{
			Eigen::Vector3d delta=cur->xyz-Tcr.transformPoint(ref->xyz);
			Eigen::Matrix<double,1,1> dist=delta.transpose()*computeCov(Tcr).inverse()*delta;
			return dist(0,0);
		}
	};

	struct PlaneLM
	{
		Eigen::Vector3d n;
		double d;
		int id;
	};

	struct Plane
	{
		Eigen::Vector3d normal;
		double d;

		PlaneLM *plane_landmark;

		// cov_inv = hessian
		//         = | H_nn H_nd |
		//           | H_dn H_dd |
		Eigen::Matrix4d cov_inv; // Hessian matrix;
		Eigen::Matrix4d cov;

		Eigen::Vector3d centroid;

		Eigen::Matrix3d scatter_matrix;

		std::vector<Point> points;

		// avg_rgb, cov_rgb: computed from the points
		Eigen::Vector3d avg_rgb,avg_xyz;
		Eigen::Matrix3d cov_rgb,cov_xyz;

		int index;

		void clear()
		{
			std::vector<Point> tmp;
			tmp.swap(points);
		}

		~Plane()
		{
			std::vector<Point> tmp;
			tmp.swap(points);
		}

		double similarity_color(Plane *p)
		{
			// color similarity;
			// Bhattachryya distance bwteen the color distribution of two planes;
			Eigen::Matrix3d C=(cov_rgb+p->cov_rgb)/2;
			Eigen::Vector3d m=avg_rgb-p->avg_rgb;
			Eigen::Matrix<double,1,1> M_dist=m.transpose()*C.inverse()*m/8;
			double s_col=M_dist(0,0)+0.5*log(C.determinant()/sqrt(cov_rgb.determinant()*p->cov_rgb.determinant()));
			return s_col;
		}

		double similarity_angle(Plane *p)
		{
			double cos_angle=normal.transpose()*p->normal;
			if(cos_angle>0.9999)
				cos_angle=0.9999;
			double angle=acos(cos_angle);
			return angle;
		}

		double similarity_dist(Plane *p)
		{
			double dist=fabs(d-p->d);
			return dist;
		}

	};

	struct PlanePair
	{
		PlanePair(){}

		PlanePair(Plane *r, Plane *c)
		{
			ref=r;
			cur=c;
		}

		Plane *ref;
		Plane *cur;

		Eigen::Matrix4d computeCov(Transform Tcr)
		{
			Eigen::Matrix4d cov;
			Eigen::Matrix4d trans;
			trans.setZero();
			trans.block<3,3>(0,0)=-Tcr.R;
			trans.block<1,3>(3,0)=Tcr.t.transpose()*Tcr.R;
			trans(3,3)=-1;
			cov=trans*ref->cov*trans.transpose();
			//cov=cov+cur->cov;
			return cov;
		}

		double planeDist(Transform Tcr)
		{
			Eigen::Vector4d pln_cur, pln_ref;
			pln_cur.block<3,1>(0,0)=cur->normal;
			pln_cur(3)=cur->d;
			pln_ref.block<3,1>(0,0)=ref->normal;
			pln_ref(3)=ref->d;
			pln_ref=Tcr.transformPlane(pln_ref);
			Eigen::Matrix<double,1,1> dist;
			pln_cur=pln_cur-pln_ref;
//			dist=pln_cur.transpose()*computeCov(Tcr).inverse()*pln_cur;
			dist=pln_cur.transpose()*cur->cov_inv*pln_cur;
			return dist(0,0);

		}
	};

	struct Scan
	{
		Scan *scan_ref;

		// transform from global frame to current frame;
		Transform Tcg; 
		Transform Tcg_gt;
		Transform Tcr;
		double time_stamp;
		int id;

		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr point_cloud;
		pcl::PointCloud<pcl::Normal>::Ptr normal_cloud;
		pcl::PointCloud<pcl::PointXY>::Ptr pixel_cloud;
		cv::Mat img_rgb, img_depth;
		
		Eigen::Matrix3d Rotation_PCA;

		// plane features;
		std::vector<Plane*> observed_planes;
		std::vector<PlanePair> plane_matches;

		// Psi_pi = \sum(d_Jpln_i/d_xi)(d_Jpln_i/d_xi)^T;
		Eigen::Matrix<double,6,6> Psi_pi;
		
		// edge points;
		std::vector<EdgePoint*> edge_points;
		std::vector<EdgePointPair> point_matches;

		void clear()
		{
			pcl::PointCloud<pcl::PointXYZRGBA> pt;
			pcl::PointCloud<pcl::Normal> n;
			pcl::PointCloud<pcl::PointXY> pl;
			point_cloud->swap(pt);
			normal_cloud->swap(n);
			pixel_cloud->swap(pl);
			img_rgb.release();
			img_depth.release();
			for(size_t i=0;i<observed_planes.size();i++)
				delete observed_planes[i];
			std::vector<Plane*> tmp_pln;
			tmp_pln.swap(observed_planes);
			std::vector<PlanePair> tmp_pln_pair;
			tmp_pln_pair.swap(plane_matches);
			for(size_t i=0;i<edge_points.size();i++)
				delete edge_points[i];
			std::vector<EdgePoint*> tmp_edge;
			tmp_edge.swap(edge_points);
			std::vector<EdgePointPair> tmp_matches;
			tmp_matches.swap(point_matches);
		}

		~Scan()
		{
			img_rgb.release();
			img_depth.release();
			for(size_t i=0;i<observed_planes.size();i++)
				delete observed_planes[i];
			std::vector<Plane*> tmp_pln;
			tmp_pln.swap(observed_planes);
			std::vector<PlanePair> tmp_pln_pair;
			tmp_pln_pair.swap(plane_matches);
			for(size_t i=0;i<edge_points.size();i++)
				delete edge_points[i];
			std::vector<EdgePoint*> tmp_edge;
			tmp_edge.swap(edge_points);
			std::vector<EdgePointPair> tmp_matches;
			tmp_matches.swap(point_matches);
		}

	};

	struct LoopClosure
	{
		LoopClosure(Scan *cur, Scan *ref)
		{
			scan_cur=cur;
			scan_ref=ref;
			delta_time=scan_cur->time_stamp-scan_ref->time_stamp;
			Tcr=cur->Tcg*ref->Tcg.inv();
			match_score=0;
		}

		Scan *scan_cur, *scan_ref;
		Transform Tcr;
		double delta_time;
		double match_score;

		bool operator < (const LoopClosure &m)const
		{
			return delta_time < m.delta_time;
		}
	};

	struct Map
	{
		std::vector<Scan*> seq;

		std::vector<LoopClosure> loop_closure;

		std::vector<PlaneLM*> planes;

		void addScan(Scan *s)
		{
			pcl::PointCloud<pcl::PointXYZRGBA> pt;
			pcl::PointCloud<pcl::Normal> n;
			pcl::PointCloud<pcl::PointXY> pl;
			s->point_cloud->swap(pt);
			s->normal_cloud->swap(n);
			s->pixel_cloud->swap(pl);
			s->img_rgb.release();
			s->img_depth.release();
			for(size_t i=0;i<s->observed_planes.size();i++)
			{
				s->observed_planes[i]->clear();
			}
			seq.push_back(s);
		}

	};

}

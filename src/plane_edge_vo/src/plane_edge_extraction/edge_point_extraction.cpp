/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2017-10-18 20:13
#
# Filename: edge_point_extraction.cpp
#
# Description: 
#
===============================================*/

#include "edge_point_extraction.h"
#include <pcl-1.8/pcl/filters/extract_indices.h>

namespace ulysses
{
	void EdgePointExtraction::extractEdgePoints(Scan *scan)
	{
		ofstream fp;
		fp.open("extract_EdgePoints.txt",std::ios::app);
		if(debug)
			fp<<std::endl<<"******************************************************************"<<std::endl;

		// edge
		pcl::PointCloud<pcl::Label>::Ptr edge_label_cloud (new pcl::PointCloud<pcl::Label>);
		std::vector<pcl::PointIndices> edge_indices;

		// for edge detection;
		// change the invalid depth in scan->point_cloud from zero to infinite;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);
		*cloud_tmp=*scan->point_cloud;
		for(size_t i=0;i<cloud_tmp->height;i++)
		{
			for(size_t j=0;j<cloud_tmp->width;j++)
			{
				double dep=cloud_tmp->points[cloud_tmp->width*i+j].z;
				if(std::abs(dep)<1e-4)
				{
					cloud_tmp->points[cloud_tmp->width*i+j].z=std::numeric_limits<double>::max();
				}
			}
		}

		// edge detection;
		if (getEdgeType () & EDGELABEL_HIGH_CURVATURE)
		{
			pcl::OrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>::setInputNormals(scan->normal_cloud);
		}
		pcl::OrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>::setInputCloud(cloud_tmp);
		pcl::OrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>::compute(*edge_label_cloud, edge_indices);
		if(debug)
		{
			fp<<"organized edge detection "<<std::endl;
			fp<<"\tEDGELABEL_NAN_BOUNDARY - "<<edge_indices[0].indices.size()<<std::endl;
			fp<<"\tEDGELABEL_OCCLUDING - "<<edge_indices[1].indices.size()<<std::endl;
			fp<<"\tEDGELABEL_OCCLUDED - "<<edge_indices[2].indices.size()<<std::endl;
			fp<<"\tEDGELABEL_HIGH_CURVATURE - "<<edge_indices[3].indices.size()<<std::endl;
			fp<<"\tEDGELABEL_RGB_CANNY - "<<edge_indices[4].indices.size()<<std::endl;
		}

		// scan->edge_points;
		// fitting local line segment of each edge point;
		ANNkd_tree *kdtree;
		ANNpoint query_point=annAllocPt(3);
		ANNidxArray index=new ANNidx[K_ANN];
		ANNdistArray distance=new ANNdist[K_ANN];
		// edge_indices[1] - occluding points;
		// edge_indices[2] - occluded points;
		ANNpointArray edge_points=annAllocPts(edge_indices[1].indices.size()+edge_indices[3].indices.size()+edge_indices[4].indices.size(),3);
		scan->edge_points.resize(edge_indices[1].indices.size()+edge_indices[3].indices.size()+edge_indices[4].indices.size());
		if(debug)
		{
			fp<<"occluding points"<<std::endl;
		}
		for(size_t i=0;i<edge_indices[1].indices.size();i++)
		{
			int idx=edge_indices[1].indices[i];
			edge_points[i][0]=scan->point_cloud->at(idx).x;
			edge_points[i][1]=scan->point_cloud->at(idx).y;
			edge_points[i][2]=scan->point_cloud->at(idx).z;
			// fill scan->edge_points;
			scan->edge_points[i]=new EdgePoint;
			scan->edge_points[i]->xyz(0)=scan->point_cloud->at(idx).x;
			scan->edge_points[i]->xyz(1)=scan->point_cloud->at(idx).y;
			scan->edge_points[i]->xyz(2)=scan->point_cloud->at(idx).z;
			if(debug)
			{
				fp<<"\t"<<i<<" - "<<scan->edge_points[i]->xyz.transpose()<<std::endl;
			}
		}
		if(debug)
		{
			fp<<"high curvature points"<<std::endl;
		}
		for(size_t i=0;i<edge_indices[3].indices.size();i++)
		{
			int idx=edge_indices[3].indices[i];
			int j=i+edge_indices[1].indices.size();
			edge_points[j][0]=scan->point_cloud->at(idx).x;
			edge_points[j][1]=scan->point_cloud->at(idx).y;
			edge_points[j][2]=scan->point_cloud->at(idx).z;
			// fill scan->edge_points;
			scan->edge_points[j]=new EdgePoint;
			scan->edge_points[j]->xyz(0)=scan->point_cloud->at(idx).x;
			scan->edge_points[j]->xyz(1)=scan->point_cloud->at(idx).y;
			scan->edge_points[j]->xyz(2)=scan->point_cloud->at(idx).z;
			if(debug)
			{
				fp<<"\t"<<j<<" - "<<scan->edge_points[j]->xyz.transpose()<<std::endl;
			}
		}
		if(debug)
		{
			fp<<"canny points"<<std::endl;
		}
		for(size_t i=0;i<edge_indices[4].indices.size();i++)
		{
			int idx=edge_indices[4].indices[i];
			int j=i+edge_indices[1].indices.size()+edge_indices[3].indices.size();
			edge_points[j][0]=scan->point_cloud->at(idx).x;
			edge_points[j][1]=scan->point_cloud->at(idx).y;
			edge_points[j][2]=scan->point_cloud->at(idx).z;
			// fill scan->edge_points;
			scan->edge_points[j]=new EdgePoint;
			scan->edge_points[j]->xyz(0)=scan->point_cloud->at(idx).x;
			scan->edge_points[j]->xyz(1)=scan->point_cloud->at(idx).y;
			scan->edge_points[j]->xyz(2)=scan->point_cloud->at(idx).z;
			if(debug)
			{
				fp<<"\t"<<j<<" - "<<scan->edge_points[j]->xyz.transpose()<<std::endl;
			}
		}
		// build the kd-tree using the occluding edge points;
		kdtree=new ANNkd_tree(edge_points,scan->edge_points.size(),3);
		// for each occluding edge point;
		// search for the nearest neighbor in the occluding edge points;
		for(size_t i=0;i<scan->edge_points.size();i++)
		{
			query_point[0]=scan->edge_points[i]->xyz(0);
			query_point[1]=scan->edge_points[i]->xyz(1);
			query_point[2]=scan->edge_points[i]->xyz(2);
			int points_in_radius=kdtree->annkFRSearch(query_point,sqRad_ANN,K_ANN,index,distance);
			//	ANNpoint q, // query point
			//	ANNdist sqRad, // squared radius
			//	int k = 0, // number of near neighbors to return
			//	ANNidxArray nn_idx = NULL, // nearest neighbor array (modified)
			//	ANNdistArray dd = NULL, // dist to near neighbors (modified)
			//	double eps = 0.0); // error bound
			if(points_in_radius>7)
			{
				scan->edge_points[i]->isEdge=true;
				for(size_t j=0;j<K_ANN;j++)
				{
					if(index[j]==ANN_NULL_IDX)
						continue;
					scan->edge_points[i]->neighbors.push_back(scan->edge_points[index[j]]);
				}
				Eigen::Vector3d mean=Eigen::Vector3d::Zero();
				for(size_t j=0;j<scan->edge_points[i]->neighbors.size();j++)
				{
					mean+=scan->edge_points[i]->neighbors[j]->xyz;
				}
				mean=mean/scan->edge_points[i]->neighbors.size();
				scan->edge_points[i]->cov.setZero();
				for(size_t j=0;j<scan->edge_points[i]->neighbors.size();j++)
				{
					Eigen::Vector3d vec3d=scan->edge_points[i]->neighbors[j]->xyz-mean;
					scan->edge_points[i]->cov+=vec3d*vec3d.transpose();
				}
				scan->edge_points[i]->cov=scan->edge_points[i]->cov/(scan->edge_points[i]->neighbors.size()-1);
				if(scan->edge_points[i]->cov.determinant()<1e-20)
					scan->edge_points[i]->isEdge=false;
			}
		}
		if(debug)
		{
			fp<<"extracted edge Points - "<<scan->edge_points.size()<<std::endl;
			for(size_t i=0;i<scan->edge_points.size();i++)
			{
				fp<<i<<" - "<<scan->edge_points[i]->xyz.transpose();
				if(scan->edge_points[i]->isEdge)
				{
					fp<<"\tneighbors - "<<scan->edge_points[i]->neighbors.size()<<std::endl;
					fp<<"\tcov - "<<std::endl<<scan->edge_points[i]->cov<<std::endl;
				}
			}
		}
		annDeallocPt(query_point);
		annDeallocPts(edge_points);
		delete kdtree;
		delete index;
		delete distance;
		fp.close();
	}
}


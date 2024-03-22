/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2017-07-19 16:13
#
# Filename: display.h
#
# Description: 
#
===============================================*/
#include "pose_estimation.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl-1.8/pcl/filters/voxel_grid.h>

using namespace ulysses;
typedef pcl::PointXYZRGBA PointT;

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
	if (event.getKeySym () == "q" && event.keyDown ())
	{
	   viewer->close();
	}
}

// displayPlanes
// - display planes from one scan in different colors;
void displayPlanes(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
{
	char id[20];
	unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
	unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
	unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};

	vis->removeAllPointClouds();
//	vis->removeAllShapes();
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane (new pcl::PointCloud<pcl::PointXYZRGBA>);

	// add raw scan data;
	sprintf(id,"scan");
	if (!vis->updatePointCloud (scan->point_cloud, id))
		vis->addPointCloud (scan->point_cloud, id);

	for(size_t i=0;i<scan->observed_planes.size();i++)
	{
		sprintf(id,"plane%d",i);
		plane->resize(scan->observed_planes[i]->points.size());
		for(size_t j=0;j<plane->size();j++)
		{
			plane->at(j).x=scan->observed_planes[i]->points[j].xyz[0];
			plane->at(j).y=scan->observed_planes[i]->points[j].xyz[1];
			plane->at(j).z=scan->observed_planes[i]->points[j].xyz[2];
		}
		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color1 (plane, red[i%12], grn[i%12], blu[i%12]);
		if (!vis->updatePointCloud (plane, color1, id))
			vis->addPointCloud (plane, color1, id);
	}
}

// displayPlanes_2scans
// - display planes from 2 scans;
// - different color for each scan;
// - scan_ref is transformed by Tcr;
void displayPlanes_2scans(Scan *scan, Scan *scan_ref, Transform Tcr, 
							    boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
{
	char id[20];
	unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
	unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
	unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};

	vis->removeAllPointClouds();
//	vis->removeAllShapes();
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane (new pcl::PointCloud<pcl::PointXYZRGBA>);

	for(size_t i=0;i<scan->observed_planes.size();i++)
	{
		sprintf(id,"plane_cur%d",i);
		plane->resize(scan->observed_planes[i]->points.size());
		for(size_t j=0;j<plane->size();j++)
		{
			plane->at(j).x=scan->observed_planes[i]->points[j].xyz[0];
			plane->at(j).y=scan->observed_planes[i]->points[j].xyz[1];
			plane->at(j).z=scan->observed_planes[i]->points[j].xyz[2];
		}
		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color1 (plane, red[0], grn[0], blu[0]);
		if (!vis->updatePointCloud (plane, color1, id))
			vis->addPointCloud (plane, color1, id);
	}
	for(size_t i=0;i<scan_ref->observed_planes.size();i++)
	{
		sprintf(id,"plane_ref%d",i);
		plane->resize(scan_ref->observed_planes[i]->points.size());
		for(size_t j=0;j<plane->size();j++)
		{
			plane->at(j).x=scan_ref->observed_planes[i]->points[j].xyz[0];
			plane->at(j).y=scan_ref->observed_planes[i]->points[j].xyz[1];
			plane->at(j).z=scan_ref->observed_planes[i]->points[j].xyz[2];
		}
		pcl::transformPointCloud(*plane,*plane,Tcr.getMatrix4f());
		pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color1 (plane, red[2], grn[2], blu[2]);
		if (!vis->updatePointCloud (plane, color1, id))
			vis->addPointCloud (plane, color1, id);
	}
}

// displayScans_2scans
// - display raw scan data from 2 scans;
// - different color for each scan;
// - scan_ref is transformed by Tcr;
void displayScans_2scans(Scan *scan, Scan *scan_ref, Transform Tcr, 
						 boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
{
	char id[20];
	unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
	unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
	unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};

	vis->removeAllPointClouds();
//	vis->removeAllShapes();
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane (new pcl::PointCloud<pcl::PointXYZRGBA>);

	sprintf(id,"scan_ref");
	pcl::transformPointCloud(*scan_ref->point_cloud,*plane,Tcr.getMatrix4f());
	pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color1 (plane, red[2], grn[2], blu[2]);
	if (!vis->updatePointCloud (plane, color1, id))
		vis->addPointCloud (plane, color1, id);

	sprintf(id,"scan_cur");
	pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color2 (scan->point_cloud, red[0], grn[0], blu[0]);
	if (!vis->updatePointCloud (scan->point_cloud, color2, id))
		vis->addPointCloud (scan->point_cloud, color2, id);

//	pcl::PointXYZRGBA pt1,pt2;
//	for(size_t i=0;i<scan->edge_points.size();i++)
//	{
//		if(scan->edge_points[i]->cov.determinant()<1e-20 || scan->point_matches[i]==0)
//			continue;
//		pt1.x=scan->edge_points[i]->xyz(0);
//		pt1.y=scan->edge_points[i]->xyz(1);
//		pt1.z=scan->edge_points[i]->xyz(2);
//		pt2.x=scan->point_matches[i]->xyz(0);
//		pt2.y=scan->point_matches[i]->xyz(1);
//		pt2.z=scan->point_matches[i]->xyz(2);
//		sprintf(id,"dir%d",i);
//		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,red[1], grn[1], blu[1],id);
//	}
}

// display_addScan
// - add one scan to vis after calling this function;
// - the scan is transformed to global frame by scan->Tcg;
void display_addScan(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
{
	char id[20];
	unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
	unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
	unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};

//	vis->removeAllPointClouds();
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane (new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);

	Transform Tgc=scan->Tcg.inv();
	//std::cout<<"addScan "<<Tgc.t.transpose()<<" "<<Tgc.Quaternion().transpose()<<std::endl;
	pcl::transformPointCloud(*scan->point_cloud,*plane,Tgc.getMatrix4f());
//	int count=0;
	for(size_t i=0;i<plane->height;i++)
	{
		for(size_t j=0;j<plane->width;j++)
		{
//			std::cout<<i<<" "<<i%10<<" "<<j<<" "<<j%10<<std::endl;
			if(i%4==0&&j%4==0)
			{
				plane_filtered->push_back(plane->at(j,i));
//				count++;
			}
		}
	}
//	std::cout<<"count "<<count<<std::endl;
//	std::cout<<"plane "<<plane->size()<<std::endl;
//	pcl::VoxelGrid<pcl::PointXYZRGBA> filter;
//	filter.setInputCloud(plane);
//	filter.setLeafSize(0.01,0.01,0.01);
//	filter.filter(*plane);

	sprintf(id,"scan%d", scan->id);
	if (!vis->updatePointCloud (plane_filtered, id))
		vis->addPointCloud (plane_filtered, id);
//	if (!vis->updatePointCloud (plane_filtered, id))
//		vis->addPointCloud (plane_filtered, id);
}

// displayEdgePoints
// - display edge points in one scan;
void displayEdgePoints(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
{
	char id[20];
	unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
	unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
	unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};

//	vis->removeAllPointClouds();
//	vis->removeAllShapes();
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edge (new pcl::PointCloud<pcl::PointXYZRGBA>);
	// add edge points to vis;
	int count=0;
	edge->resize(scan->edge_points.size());
	for(size_t i=0;i<scan->edge_points.size();i++)
	{
		edge->at(i).x=scan->edge_points[i]->xyz(0);
		edge->at(i).y=scan->edge_points[i]->xyz(1);
		edge->at(i).z=scan->edge_points[i]->xyz(2);
	}
	sprintf(id,"edgePoints");
	pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color_edge (edge, red[13], grn[13], blu[13]);//white
	if (!vis->updatePointCloud (edge, color_edge, id))
		vis->addPointCloud (edge, color_edge, id);
	vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, id);
}

void displayMatchedEdgePoints(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
{
	char id[20];
	unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
	unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
	unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};

//	vis->removeAllPointClouds();
//	vis->removeAllShapes();
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr edge (new pcl::PointCloud<pcl::PointXYZRGBA>);

//	// add raw scan data;
//	sprintf(id,"scan");
//	if (!vis->updatePointCloud (scan->point_cloud, id))
//		vis->addPointCloud (scan->point_cloud, id);

	// add edge points to vis;
	int count=0;
	edge->resize(scan->point_matches.size());
//	ofstream fp;
//	fp.open("display.txt",std::ios::out);
	for(size_t i=0;i<scan->point_matches.size();i++)
	{
		edge->at(i).x=scan->point_matches[i].cur->xyz(0);
		edge->at(i).y=scan->point_matches[i].cur->xyz(1);
		edge->at(i).z=scan->point_matches[i].cur->xyz(2);
//		fp<<scan->edge_points[i]->xyz.transpose()<<std::endl;
//		// find the principle direction of edge_point->cov;
//		if(!scan->edge_points[i]->isEdge)
//			continue;
//		count++;
//		Eigen::EigenSolver<Eigen::Matrix3d> es(scan->edge_points[i]->cov);
//		Eigen::Matrix<std::complex<double>,3,1> eigenvalues=es.eigenvalues();
//		Eigen::Matrix<std::complex<double>,3,3> eigenvectors=es.eigenvectors();
//		double ev_max=DBL_MIN;
//		size_t ev_max_idx=-1;
//		for(size_t j=0;j<3;j++)
//		{
//			if(eigenvalues(j).real()>ev_max)
//			{
//				ev_max=eigenvalues(j).real();
//				ev_max_idx=j;
//			}
//		}
//		ev_max=sqrt(ev_max);
//		Eigen::Vector3d dir;
//		dir(0)=eigenvectors(0,ev_max_idx).real();
//		dir(1)=eigenvectors(1,ev_max_idx).real();
//		dir(2)=eigenvectors(2,ev_max_idx).real();
//		// draw a red line in the principle direction;
//		pcl::PointXYZRGBA pt1,pt2;
//		pt1=edge->at(i);
//		pt2.x=pt1.x+dir(0)*ev_max;
//		pt2.y=pt1.y+dir(1)*ev_max;
//		pt2.z=pt1.z+dir(2)*ev_max;
//		sprintf(id,"dir%d",i);
//		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,red[0], grn[0], blu[0],id);
	}
//	fp.close();
//	std::cout<<"points on edge - "<<count<<std::endl;
	sprintf(id,"edgePoints");
	pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color_edge (edge, red[13], grn[13], blu[13]);//white
	if (!vis->updatePointCloud (edge, color_edge, id))
		vis->addPointCloud (edge, color_edge, id);
	vis->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, id);
}



void displayTraj(Map *map, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
{
//	vis->removeAllPointClouds();
	vis->removeAllShapes();
	char id[20];
	pcl::PointXYZRGBA pt1,pt2;
	double scale=0.1;
	Transform Tg0=map->seq[0]->Tcg_gt.inv();
	for(size_t i=0;i<map->seq.size();i++)
	{
		Transform Tgc=map->seq[i]->Tcg.inv();
		Eigen::Vector3d x=Tgc.R.block<3,1>(0,0);
		Eigen::Vector3d y=Tgc.R.block<3,1>(0,1);
		Eigen::Vector3d z=Tgc.R.block<3,1>(0,2);
		pt1.x=Tgc.t(0);
		pt1.y=Tgc.t(1);
		pt1.z=Tgc.t(2);
		pt2.x=pt1.x+z(0)*scale;
		pt2.y=pt1.y+z(1)*scale;
		pt2.z=pt1.z+z(2)*scale;
		sprintf(id,"%dz",map->seq[i]->id);
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,0,255,id);
		if(i>0)
		{
			Transform Tgc_pre=map->seq[i]->scan_ref->Tcg.inv();
			pt2.x=Tgc_pre.t(0);
			pt2.y=Tgc_pre.t(1);
			pt2.z=Tgc_pre.t(2);
			sprintf(id,"%dtraj",map->seq[i]->id);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,255,255,id);
		}

		Tgc=Tg0.inv()*map->seq[i]->Tcg_gt.inv();
		if(map->seq[i]->Tcg_gt.t.norm()==0 || map->seq[i]->scan_ref->Tcg_gt.t.norm()==0)
			continue;
//		std::cout<<map->seq[i]->Tcg_gt.t.transpose()<<std::endl;
		pt1.x=Tgc.t(0);
		pt1.y=Tgc.t(1);
		pt1.z=Tgc.t(2);
		if(i>0)
		{
			Transform Tgc_pre=Tg0.inv()*map->seq[i]->scan_ref->Tcg_gt.inv();
			pt2.x=Tgc_pre.t(0);
			pt2.y=Tgc_pre.t(1);
			pt2.z=Tgc_pre.t(2);
			sprintf(id,"%dtraj_gt",map->seq[i]->id);
			vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,255,0,id);
		}
	}
	for(size_t i=0;i<map->loop_closure.size();i++)
	{
		if(map->loop_closure[i].match_score==-1)
			continue;
//		if(map->loop_closure[i].delta_time<10)
//			continue;
//		if(map->loop_closure[i].Tcr.t.norm()>0.5 || acos((map->loop_closure[i].Tcr.R.trace()-1.0)/2.0)>0.3)
//			continue;
		Transform Tgc=map->loop_closure[i].scan_cur->Tcg.inv();
		Transform Tgr=map->loop_closure[i].scan_ref->Tcg.inv();
		pt1.x=Tgc.t(0);
		pt1.y=Tgc.t(1);
		pt1.z=Tgc.t(2);
		pt2.x=Tgr.t(0);
		pt2.y=Tgr.t(1);
		pt2.z=Tgr.t(2);
		sprintf(id,"%dlc",i);
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,0,0,id);
	}
	vis->spin();
	for(size_t i=0;i<map->loop_closure.size();i++)
	{
		if(map->loop_closure[i].match_score==-1)
			continue;
		sprintf(id,"%dlc",i);
		vis->removeShape(id);
	}
	for(size_t i=0;i<map->loop_closure.size();i++)
	{
		if(map->loop_closure[i].match_score==-1)
			continue;
//		if(map->loop_closure[i].delta_time<10)
//			continue;
		if(map->loop_closure[i].Tcr.t.norm()>0.2 || acos((map->loop_closure[i].Tcr.R.trace()-1.0)/2.0)>0.2)
			continue;
		Transform Tgc=map->loop_closure[i].scan_cur->Tcg.inv();
		Transform Tgr=map->loop_closure[i].scan_ref->Tcg.inv();
		pt1.x=Tgc.t(0);
		pt1.y=Tgc.t(1);
		pt1.z=Tgc.t(2);
		pt2.x=Tgr.t(0);
		pt2.y=Tgr.t(1);
		pt2.z=Tgr.t(2);
		sprintf(id,"%dlc",i);
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,0,0,id);
	}
}


void display_addCameraPose(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
{
//	vis->removeAllPointClouds();
//	vis->removeAllShapes();
	char id[20];
	pcl::PointXYZRGBA pt1,pt2;
	double scale=0.1;
//	for(size_t i=0;i<map->seq.size();i++)
	{
		Transform Tgc=scan->Tcg.inv();
		Eigen::Vector3d x=Tgc.R.block<3,1>(0,0);
		Eigen::Vector3d y=Tgc.R.block<3,1>(0,1);
		Eigen::Vector3d z=Tgc.R.block<3,1>(0,2);
		pt1.x=Tgc.t(0);
		pt1.y=Tgc.t(1);
		pt1.z=Tgc.t(2);
		// x - green
		pt2.x=pt1.x+x(0)*scale;
		pt2.y=pt1.y+x(1)*scale;
		pt2.z=pt1.z+x(2)*scale;
		sprintf(id,"%dx",scan->id);
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,255,0,id);
		// y - blue
		pt2.x=pt1.x+y(0)*scale;
		pt2.y=pt1.y+y(1)*scale;
		pt2.z=pt1.z+y(2)*scale;
		sprintf(id,"%dy",scan->id);
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,0,255,id);
		// z - red
		pt2.x=pt1.x+z(0)*scale;
		pt2.y=pt1.y+z(1)*scale;
		pt2.z=pt1.z+z(2)*scale;
		sprintf(id,"%dz",scan->id);
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,0,0,id);
	}
}


void display_LoopClosure(LoopClosure loop_closure, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
{
	char id[20];
	unsigned char red [14] = {255,   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
	unsigned char grn [14] = {  0, 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
	unsigned char blu [14] = {  0,   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};

	vis->removeAllPointClouds();
	vis->removeAllShapes();
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr plane (new pcl::PointCloud<pcl::PointXYZRGBA>);

	Scan *scan=loop_closure.scan_cur;
	Scan *scan_ref=loop_closure.scan_ref;

	sprintf(id,"cur");
	plane->resize(scan->edge_points.size());
	for(size_t i=0;i<scan->edge_points.size();i++)
	{
		plane->at(i).x=scan->edge_points[i]->xyz[0];
		plane->at(i).y=scan->edge_points[i]->xyz[1];
		plane->at(i).z=scan->edge_points[i]->xyz[2];
	}
	pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color1 (plane, red[0], grn[0], blu[0]);
	if (!vis->updatePointCloud (plane, color1, id))
		vis->addPointCloud (plane, color1, id);

	sprintf(id,"ref");
	plane->resize(scan_ref->edge_points.size());
	for(size_t i=0;i<scan_ref->edge_points.size();i++)
	{
		plane->at(i).x=scan_ref->edge_points[i]->xyz[0];
		plane->at(i).y=scan_ref->edge_points[i]->xyz[1];
		plane->at(i).z=scan_ref->edge_points[i]->xyz[2];
	}
	pcl::transformPointCloud(*plane,*plane,loop_closure.Tcr.getMatrix4f());
	pcl::visualization::PointCloudColorHandlerCustom <pcl::PointXYZRGBA> color2 (plane, red[2], grn[2], blu[2]);
	if (!vis->updatePointCloud (plane, color2, id))
		vis->addPointCloud (plane, color2, id);
}



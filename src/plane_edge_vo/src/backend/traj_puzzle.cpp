/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-10-29 15:45
#
# Filename:		traj_puzzle.cpp
#
# Description: 
#
===============================================*/
#include "traj_puzzle.h"

namespace ulysses
{

	void TrajPuzzle::readTrajFiles()
	{
		struct dirent *ptr;    
		DIR *dir;
		dir=opendir(traj_path.c_str());
		std::cout<<traj_path<<std::endl;
		traj_file_list.clear();
		std::string traj_name;
//		std::regex reg_ex("^(traj)(\\_)(\\d+)(\\.)?(\\d+)?(\\_)(\\d+)(\\.(txt))$");
//		std::regex reg_ex("^(traj)(\\_)(\\d+)(\\.)?(\\d+)?(\\.(txt))$");
		std::regex reg_ex("^(traj)(\\_)(\\d+)(\\.(txt))$");
		while((ptr=readdir(dir))!=NULL)
		{
			if(ptr->d_name[0] == '.')
				continue;
			traj_name=ptr->d_name;
			std::cout<<"traj_name - "<<traj_name<<std::endl;
			if(std::regex_match(traj_name, reg_ex))
				traj_file_list.push_back(TrajFile(traj_path,traj_name));
		}
		closedir(dir);
		if(traj_file_list.size()==0)
		{
			std::cerr<<"no traj file!"<<std::endl;
			return;
		}
		std::sort(traj_file_list.begin(), traj_file_list.end());

		{
			std::cout<<"read files"<<std::endl;
			for(size_t i=0;i<traj_file_list.size();i++)
			{
				std::cout<<"\t"<<traj_file_list[i].file_name<<", "<<std::fixed<<traj_file_list[i].start_time<<std::endl<<std::endl;
			}
		}

		double tx,ty,tz,qx,qy,qz,qw,time;
		traj_file_list[0].fp.open(traj_file_list[0].file_name,std::ios::in);
		while(true)
		{
			traj_file_list[0].fp>>time>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
			if(traj_file_list[0].fp.eof()) break;
//			std::cout<<std::fixed<<time<<" "<<tx<<" "<<ty<<" "<<tx<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<std::endl;
			Transform Tcg(Eigen::Quaterniond(qw,qx,qy,qz), Eigen::Vector3d(tx,ty,tz));
			Tcg.inverse();
			Tcg.time_stamp=time;
			trajectory.push_back(Tcg);
		}
//		std::cout<<trajectory.size()<<std::endl;
		traj_file_list[0].fp.close();
		for(size_t i=1;i<traj_file_list.size();i++)
		{
			traj_file_list[i].fp.open(traj_file_list[i].file_name,std::ios::in);
			traj_file_list[i].fp>>time>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
//			std::cout<<traj_file_list[i].file_name<<std::endl;
//			std::cout<<"\t"<<std::fixed<<time<<" "<<tx<<" "<<ty<<" "<<tx<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<std::endl;
//			Transform Tg;
			std::vector<double> tmp_list;
			for(size_t j=trajectory.size()-1;j>=0;j--)
			{
//				std::cout<<"\t"<<j<<std::endl;
				double tmp=time-trajectory[j].time_stamp;
				tmp_list.push_back(fabs(tmp));
				if(tmp>0)
				{
//					Tg=trajectory[j];
//					trajectory.resize(j+1);
//					std::cout<<"\t\t"<<j<<std::endl;
//					std::cout<<"\t\t"<<std::fixed<<trajectory[j].time_stamp<<std::endl;
//					std::cout<<"\t\t"<<trajectory.size()<<std::endl;
					break;
				}
			}
			double tmp_min=DBL_MAX;
			int index=-1;
			for(size_t j=0;j<tmp_list.size();j++)
			{
				if(tmp_list[j]<tmp_min)
				{
					tmp_min=tmp_list[j];
					index=j;
				}
			}
			trajectory.resize(trajectory.size()-index);
			Transform T0G=trajectory[trajectory.size()-1];
//			T0G.inverse();
			Transform T0g(Eigen::Quaterniond(qw,qx,qy,qz), Eigen::Vector3d(tx,ty,tz));
			T0g.inverse();
//			do
//			{
//				traj_file_list[i].fp>>time>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
//			}
//			while();

			while(!traj_file_list[i].fp.eof())
			{
				traj_file_list[i].fp>>time>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
//				std::cout<<"\t"<<std::fixed<<time<<" "<<tx<<" "<<ty<<" "<<tx<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<std::endl;
				Transform Tcg(Eigen::Quaterniond(qw,qx,qy,qz), Eigen::Vector3d(tx,ty,tz));
				Tcg.inverse();
				Tcg=Tcg*T0g.inv()*T0G;
//				Tcg.inverse();
				Tcg.time_stamp=time;
				trajectory.push_back(Tcg);
			}
			traj_file_list[i].fp.close();
		}

		fp.open("traj.txt",std::ios::out);
		for(size_t i=0;i<trajectory.size();i++)
		{
			fp<<std::fixed<<trajectory[i].time_stamp<<" "<<trajectory[i].inv().t.transpose()<<" "<<trajectory[i].inv().Quaternion().transpose()<<std::endl;
		}
		fp.close();
	}

	void TrajPuzzle::readTraj2Map(int n)
	{
		std::ifstream fp_traj;
		fp_traj.open("traj.txt",std::ios::in);
		double tx,ty,tz,qx,qy,qz,qw,time;
		int i=0;
		while(true)
		{
			fp_traj>>time>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
			if(fp_traj.eof()) break;
//			std::cout<<i<<"\t"<<n<<"\t"<<i%n<<std::endl;
			if(i%n==0)
			{
				Transform Tcg(Eigen::Quaterniond(qw,qx,qy,qz), Eigen::Vector3d(tx,ty,tz));
				Tcg.inverse();
				Tcg.time_stamp=time;
				trajectory.push_back(Tcg);
			}
			i++;
		}
		fp_traj.close();
	}
}


/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2017-10-29 15:45
#
# Filename:		traj_puzzle.h
#
# Description: 
#
===============================================*/
#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <dirent.h>
#include <regex>
#include <limits>
//#include <eigen3/Eigen/src/Core/DenseBase.h>
#include "types.h"

namespace ulysses
{
	struct TrajFile
	{
		TrajFile(std::string path, std::string file)
		{
			file_name=path+file;
			fp.open(file_name,std::ios::in);
			fp>>start_time;
			fp.close();
		}

		std::string file_name;
		std::ifstream fp;
		double start_time;

		bool operator < (const TrajFile &m) const
		{
			return start_time < m.start_time;
		}
	};
	
	class TrajPuzzle
	{
	public:

		TrajPuzzle(const std::string path): traj_path(path) {}
		TrajPuzzle() {}

		~TrajPuzzle() {}

		Transform Traj(size_t i) {return trajectory[i];}
		size_t TrajLength() {return trajectory.size();}

		void readTrajFiles();

		void readTraj2Map(int n);

	private:

		std::ofstream fp;

		const std::string traj_path;
		std::vector<TrajFile> traj_file_list;
		std::vector<Transform> trajectory;
	};
}

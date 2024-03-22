/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2018-03-01 21:03
#
# Filename: plane_fusing.cpp
#
# Description: 
#
===============================================*/

#include "plane_fusing.h"

namespace ulysses
{
	void PlaneFusing::fusePlanes(Map *map, Scan *scan)
	{
		fp.open("fuseplanes.txt",std::ios::app);
		fp<<"************************fusePlanes***********************************"<<std::endl;
		for(size_t i=0;i<scan->observed_planes.size();i++)
		{
			bool new_landmark=true;
			for(size_t j=0;j<scan->plane_matches.size();j++)
			{
				if(scan->plane_matches[j].cur==scan->observed_planes[i])
				{
					//fusing...
					//update: scan->plane_matches[j].ref->plane_landmark
					scan->plane_matches[j].cur->plane_landmark=scan->plane_matches[j].ref->plane_landmark;
					new_landmark=false;
					break;
				}
			}
			if(new_landmark)
			{
				//add new landmark;
				scan->observed_planes[i]->plane_landmark=new PlaneLM;
				scan->observed_planes[i]->plane_landmark->id=map->planes.size();
				scan->observed_planes[i]->plane_landmark->n=scan->observed_planes[i]->normal;
				scan->observed_planes[i]->plane_landmark->d=scan->observed_planes[i]->d;
				map->planes.push_back(scan->observed_planes[i]->plane_landmark);
			}
		}
		if(debug)
		{
			fp<<"plane landmarks in the map"<<std::endl;
			for(size_t i=0;i<map->planes.size();i++)
			{
				fp<<"\t"<<map->planes[i]<<std::endl;
			}
			fp<<"observed planes in the current scan"<<std::endl;
			for(size_t i=0;i<scan->observed_planes.size();i++)
			{
				fp<<"\t"<<scan->observed_planes[i]<<"\t"<<scan->observed_planes[i]->plane_landmark<<std::endl;
			}
			fp<<"matched planes"<<std::endl;
			for(size_t i=0;i<scan->plane_matches.size();i++)
			{
				fp<<"\t"<<scan->plane_matches[i].cur<<"\t"<<scan->plane_matches[i].ref<<std::endl;
			}
		}
		fp.close();
	}
}

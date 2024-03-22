/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified: 2018-03-01 21:03
#
# Filename: plane_fusing.h
#
# Description: 
#
===============================================*/

#include "types.h"

namespace ulysses
{
	class PlaneFusing
	{
	public:
		PlaneFusing() 
		{
			debug=false;
//			remove("fuseplanes.txt");
		}

		void setDebug(bool d) {debug=d;}

		void fusePlanes(Map *map, Scan *scan);

	private:

		bool debug;
		std::ofstream fp;
	};
}

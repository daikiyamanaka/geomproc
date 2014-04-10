#ifndef VORONOI3D_H__
#define VORONOI3D_H__
 
#include <eigen3/Eigen/Core>
#include "Volume.h"

namespace geom{


template <class T> void computeVoronoiDiagram(const std::vector<Eigen::Vector3d> &sites, Volume<T> *vol);
	
}
 
#endif /* end of include guard: VORONOI3D_H__ */

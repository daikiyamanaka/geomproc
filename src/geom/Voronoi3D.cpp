
#include "Voronoi3D.h"

namespace geom{


template <class T> void  computeVoronoiDiagram(const std::vector<Eigen::Vector3d> &sites, Volume<T> *vol){

	assert(vol != NULL);

	if(sites.size() ==0){
		return;
	}


	Eigen::Vector3f minC = vol->getMinCorner();
	Eigen::Vector3f pitch = vol->getPitch();
	Eigen::Vector3i grid_size = vol->getGridSize();

	for(int k=0; k<grid_size[2]; k++){
		for(int j=0; j<grid_size[1]; j++){
			for(int i=0; i<grid_size[0]; i++){
				double x = minC[0] + (i+0.5)*pitch[0];
				double y = minC[1] + (j+0.5)*pitch[1];
				double z = minC[2] + (k+0.5)*pitch[2];
				Eigen::Vector3d p(x, y, z);
				double min_dist = (sites[0]-p).norm();
				int min_id = 0;
				for(int l=1; l<sites.size(); l++){
					double dist = (sites[l]-p).norm();
					if(min_dist > dist){
						min_dist = dist;
						min_id = l;
					}
				}
				vol->setValue(i, j, k, min_id);
			}
		}
	}
};

}

template void geom::computeVoronoiDiagram<int>(const std::vector<Eigen::Vector3d> &sites, Volume<int> *vol);


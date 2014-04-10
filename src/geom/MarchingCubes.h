#include <eigen3/Eigen/Core>

#include "Volume.h"
#include "TriangleMesh.h"

#define epsilon 1.0E-5

#define ABS(x) (x < 0 ? -(x) : (x))

typedef struct {
   Eigen::Vector3d p[8];
   Eigen::Vector3d n[8];
   double val[8];
} GRIDCELL;

typedef struct {
   Eigen::Vector3d p[3];         /* Vertices */
   Eigen::Vector3d c;            /* Centroid */
   Eigen::Vector3d n[3];         /* Normal   */
   int index[3];
} TRIANGLE;
	

//template <class T> void polygonizeCube(GRIDCELL grid, T iso, ){
	
//}

template <class T> Eigen::Vector3d interpolateVertex(double &isolevel, Eigen::Vector3d &p1, Eigen::Vector3d &p2, T v1, T v2){
  double mu;
  Eigen::Vector3d p;
  
  double val1 = (double)v1;
  double val2 = (double)v2;

  if (ABS(isolevel-v1) < 0.00001)
  	return(p1);
  if (ABS(isolevel-v2) < 0.00001)
  	return(p2);
  if (ABS(v1-v2) < 0.00001)
    return(p1);

  mu = (isolevel-val1) / (val2-val1);
  p[0] = p1[0] + mu * (p2[0] - p1[0]);
  p[1] = p1[1] + mu * (p2[1] - p1[1]);
  p[2] = p1[2] + mu * (p2[2] - p1[2]);  
}

template <class T> void marchingcubes(Volume<T> *vol, TriangleMesh &mesh, double iso = 0){

	Eigen::Vector3f pitch = vol->getPitch();
	Eigen::Vector3i grid_size = vol->getGridSize();
	Eigen::Vector3f minC = vol->getMinCorner();

	std::vector<Eigen::Vector3d> vertices;
	std::vector<std::vector<int> > faces;

  for(int k=0; k<grid_size[2]; k++){
  	for(int j=0; j<grid_size[1]; j++){
  		for(int i=0; i<grid_size[0]; i++){

  		}
  	}    	
  }		
}

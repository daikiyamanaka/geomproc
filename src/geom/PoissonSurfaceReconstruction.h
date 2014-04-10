#ifndef POISSONSURFACERECONSTRUCTION_H__
#define POISSONSURFACERECONSTRUCTION_H__
 
#include <eigen3/Eigen/Core>

#include "TriangleMesh.h"

class PoissonSurfaceReconstruction
{
public:
	PoissonSurfaceReconstruction();
	~PoissonSurfaceReconstruction();

	void reconstruct(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &normals, TriangleMesh &mesh);
	void setCriteria(double angle_criteria, double radius_criteria, double distance_criteria);

private:	
	double angle_criteria;
	double radius_criteria;
	double distance_criteria;
};
 
#endif /* end of include guard: POISSONSURFACERECONSTRUCTION_H__ */

#ifndef SIGNEDDISTANCEFIELD_H__
#define SIGNEDDISTANCEFIELD_H__

#include "Volume.h"
#include "TriangleMesh.h"

// is this flag is defined, function->compute_implicit_function() is can be called //
#define CGAL_EIGEN3_ENABLED

#include <CGAL/trace.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/Surface_mesh_default_triangulation_3.h>
#include <CGAL/make_surface_mesh.h>
#include <CGAL/Implicit_surface_3.h>
#include <CGAL/IO/output_surface_facets_to_polyhedron.h>
#include <CGAL/Poisson_reconstruction_function.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>
#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/compute_average_spacing.h>

template <class T> class SignedDistanceField
{
public:
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT FT;
typedef Kernel::Point_3 Point;
typedef CGAL::Point_with_normal_3<Kernel> Point_with_normal;
typedef Kernel::Sphere_3 Sphere;
typedef std::vector<Point_with_normal> PointList;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef CGAL::Poisson_reconstruction_function<Kernel> Poisson_reconstruction_function;
typedef CGAL::Surface_mesh_default_triangulation_3 STr;
typedef CGAL::Surface_mesh_complex_2_in_triangulation_3<STr> C2t3;
typedef CGAL::Implicit_surface_3<Kernel, Poisson_reconstruction_function> Surface_3;
typedef CGAL::Vector_3<Kernel> Vector;
typedef Polyhedron::Halfedge_around_facet_circulator Halfedge_facet_circulator;

public:
	~SignedDistanceField();

	SignedDistanceField(const std::vector<Eigen::Vector3d> &points, const std::vector<Eigen::Vector3d> &normals);

	//T operator()(const int ix, const int iy, const int iz);

	T operator()(const double x, const double y, const double z);

	void rasterize(Volume<T> *vol);

	bool isIn(const double x, const double y, const double z);

	bool isIn(const Eigen::Vector3d p);

	void getBBox(Eigen::Vector3d &min, Eigen::Vector3d &max);

private:
	void computeBBox(const std::vector<Eigen::Vector3d> &points);
  Poisson_reconstruction_function *function_;
  Eigen::Vector3d min_, max_, center_;
};

#endif /* end of include guard: SIGNEDDISTANCEFIELD_H__ */

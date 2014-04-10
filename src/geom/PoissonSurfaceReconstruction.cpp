#include "PoissonSurfaceReconstruction.h"

#define CGAL_EIGEN3_ENABLED

#include <iostream>
#include <vector>
#include <fstream>

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

// Types
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

PoissonSurfaceReconstruction::PoissonSurfaceReconstruction(){
	angle_criteria = 20.0;
	radius_criteria = 30;
	distance_criteria = 0.375;	
}

PoissonSurfaceReconstruction::~PoissonSurfaceReconstruction(){

}

void PoissonSurfaceReconstruction::reconstruct(std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector3d> &normals, TriangleMesh &mesh){

	assert(points.size() == normals.size());

  std::cout << "creating points with normal..." << std::endl;
  std::vector<Point_with_normal> points_with_normal;
  points_with_normal.resize((int)points.size());
  for(int i=0; i<(int)points.size(); i++){
    Vector vec(normals[i][0], normals[i][1], normals[i][2]);
    //Point_with_normal pwn(points[i][0], points[i][1], points[i][2], vec);
    //points_with_normal[i]  = pwn;
    points_with_normal[i] = Point_with_normal(points[i][0], points[i][1], points[i][2], vec);
  }

  std::cout << "constructing poisson reconstruction function..." << std::endl;
  Poisson_reconstruction_function function(points_with_normal.begin(), points_with_normal.end(),
                                           CGAL::make_normal_of_point_with_normal_pmap(PointList::value_type()));

  std::cout << "computing implicit function..." << std::endl;
  if( ! function.compute_implicit_function() ) {
  	std::cout << "compute implicit function is failure" << std::endl;
  	return;
  }
  	//return EXIT_FAILURE;

  // Computes average spacing
  std::cout << "compute average spacing..." << std::endl;
  FT average_spacing = CGAL::compute_average_spacing(points_with_normal.begin(), points_with_normal.end(),
                                                       6 /* knn = 1 ring */);
  // Gets one point inside the implicit surface
  // and computes implicit function bounding sphere radius.
  Point inner_point = function.get_inner_point();
  Sphere bsphere = function.bounding_sphere();
  FT radius = std::sqrt(bsphere.squared_radius());

  // Defines the implicit surface: requires defining a
  // conservative bounding sphere centered at inner point.
  FT sm_sphere_radius = 5.0 * radius;
  FT sm_dichotomy_error = distance_criteria*average_spacing/1000.0; // Dichotomy error must be << sm_distance
  //FT sm_dichotomy_error = distance_criteria*average_spacing/10.0; // Dichotomy error must be << sm_distance
  std::cout << "reconstructed surface" << std::endl;
  Surface_3 reconstructed_surface(function,
                    Sphere(inner_point,sm_sphere_radius*sm_sphere_radius),
                    sm_dichotomy_error/sm_sphere_radius);

  // Defines surface mesh generation criteria    
  CGAL::Surface_mesh_default_criteria_3<STr> criteria(angle_criteria,  // Min triangle angle (degrees)
                                                        radius_criteria*average_spacing,  // Max triangle size
                                                        distance_criteria*average_spacing); // Approximation error
  
  std::cout << "generating surface mesh..." << std::endl;
  // Generates surface mesh with manifold option
  STr tr; // 3D Delaunay triangulation for surface mesh generation
  C2t3 c2t3(tr); // 2D complex in 3D Delaunay triangulation
  CGAL::make_surface_mesh(c2t3,                                 // reconstructed mesh
                            reconstructed_surface,                              // implicit surface
                            criteria,                             // meshing criteria
                            CGAL::Manifold_tag());  // require manifold mesh

  if(tr.number_of_vertices() == 0){
  	std::cout << "surface mesh generation is failed" << std::endl;
  	return;
  }

  Polyhedron surface;
  CGAL::output_surface_facets_to_polyhedron(c2t3, surface);

  // convert CGAL::surface to TriangleMesh //
  std::cout << "converting CGA::surface to TriangleMesh..." << std::endl;
	std::vector<Eigen::Vector3d> pts;
	std::vector<std::vector<int> > faces;
	pts.resize(surface.size_of_vertices());
	faces.resize(surface.size_of_facets());

	Polyhedron::Point_iterator pit;
	int index = 0;
	for(pit=surface.points_begin(); pit!=surface.points_end(); ++pit){
		pts[index][0] = pit->x();
		pts[index][1] = pit->y();
		pts[index][2] = pit->z();				
		index ++;
	}
	index = 0;
	Polyhedron::Face_iterator fit;
	for(fit=surface.facets_begin(); fit!=surface.facets_end(); ++fit){
		std::vector<int > face(3);
		Halfedge_facet_circulator j = fit->facet_begin();
		int f_index = 0;
		do {
			face[f_index] = std::distance(surface.vertices_begin(), j->vertex());
			f_index++;
    } while ( ++j != fit->facet_begin());    

    faces[index] = face;
		index++;
	}

	mesh.createFromFaceVertex(pts, faces);  
}

void PoissonSurfaceReconstruction::setCriteria(double _angle_criteria, double _radius_criteria, double _distance_criteria){
	angle_criteria = _angle_criteria;
	radius_criteria = _radius_criteria;
	distance_criteria = _distance_criteria;
}


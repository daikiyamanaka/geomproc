#ifndef MESHSIMPLIFICATION_H__
#define MESHSIMPLIFICATION_H__
 
#include "TriangleMesh.h"
//#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Surface_mesh_simplification/HalfedgeGraph_Polyhedron_3.h> 
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Count_ratio_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Midpoint_and_length.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>

//typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Polyhedron_3<Kernel> Surface;
typedef CGAL::Polyhedron_3<Kernel> Polyhedron;
typedef Polyhedron::HalfedgeDS HalfedgeDS;
typedef Polyhedron::Halfedge_around_facet_circulator Halfedge_facet_circulator;
typedef CGAL::Point_3<Kernel> Point;

template<class HDS>
class PolyhedronBuilder : public CGAL::Modifier_base<HDS> {
public:
 std::vector<Eigen::Vector3d> &points;
 std::vector<std::vector<int> >    &faces;
    PolyhedronBuilder( std::vector<Eigen::Vector3d> &_points, std::vector<std::vector<int> > &_faces ) : points(_points), faces(_faces) {}
    void operator()( HDS& hds) {
  typedef typename HDS::Vertex   Vertex;
        typedef typename Vertex::Point Point;
 
  // create a cgal incremental builder
        CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);
        B.begin_surface( points.size(), faces.size());
   
  // add the polyhedron vertices
  for( int i=0; i<(int)points.size(); i++ ){
   B.add_vertex( Point( points[i][0], points[i][1], points[i][2]) );
  }
   
  // add the polyhedron triangles
  for( int i=0; i<(int)faces.size(); i++){
   B.begin_facet();
   B.add_vertex_to_facet( faces[i][0]);
   B.add_vertex_to_facet( faces[i][1]);
   B.add_vertex_to_facet( faces[i][2]);
   B.end_facet();
  }
   
  // finish up the surface
  B.end_surface();
  }
};  

void simplifyMesh(TriangleMesh &src, TriangleMesh &dst);
 
#endif /* end of include guard: MESHSIMPLIFICATION_H__ */

#include "MeshSimplification.h"

void simplifyMesh(TriangleMesh &src, TriangleMesh &dst){
	namespace SMS = CGAL::Surface_mesh_simplification; 

	Surface surface;
  PolyhedronBuilder<HalfedgeDS> builder(src.points, src.facets);
	surface.delegate(builder);


	// --- before mesh simplification, to maintain mesh quality high poisson surface reconstruction is applied ---


	// This is a stop predicate (defines when the algorithm terminates).
	// In this example, the simplification stops when the number of undirected edges
	// left in the surface drops below the specified number (1000) 
	// SMS::Count_stop_predicate<Surface> stop(3000);
   //SMS::Count_ratio_stop_predicate<Surface> stop(0.1);
	SMS::Count_stop_predicate<Surface> stop(100);	
  // This the actual call to the simplification algorithm.
  // The surface and stop conditions are mandatory arguments. 
  // The index maps are needed because the vertices and edges 
  // of this surface lack an "id()" field.
  int r = SMS::edge_collapse
             (surface
             ,stop
             ,CGAL::vertex_index_map(boost::get(CGAL::vertex_external_index, surface))
             .edge_index_map(boost::get(CGAL::edge_external_index, surface))
             .get_cost(CGAL::Surface_mesh_simplification::Edge_length_cost<Polyhedron>())
             .get_placement(CGAL::Surface_mesh_simplification::Midpoint_placement<Polyhedron>())
             //.get_cost(CGAL::Surface_mesh_simplification::LindstromTurk_cost<Polyhedron>())            
             //.get_placement(CGAL::Surface_mesh_simplification::LindstromTurk_placement<Polyhedron>())
             );
  std::cout << "\nFinished...\n" << r << " edges removed.\n"
           << (surface.size_of_halfedges()/2) << " final edges.\n" ;

	std::vector<Eigen::Vector3d> points;
	std::vector<std::vector<int> > faces;
	points.resize(surface.size_of_vertices());
	faces.resize(surface.size_of_facets());

	Polyhedron::Point_iterator pit;
	int index = 0;
	for(pit=surface.points_begin(); pit!=surface.points_end(); ++pit){
		points[index][0] = pit->x();
		points[index][1] = pit->y();
		points[index][2] = pit->z();				
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

	dst.createFromFaceVertex(points, faces);
}


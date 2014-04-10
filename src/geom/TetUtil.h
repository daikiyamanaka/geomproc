#include <iostream>
#include <vector>
#include <sstream>

#include <eigen3/Eigen/Core>

#include "../../external/tetgen/tetgen.h"

void tetrahedralize(const std::vector<Eigen::Vector3d> &src_points,
                    const std::string switches,
                    tetgenio &out
	                  );

void mesh2tet(const std::vector<Eigen::Vector3d> &src_points,
              const std::vector<std::vector<int> > &src_faces,
              std::vector<Eigen::Vector3d > &dst_points,
              std::vector<std::vector<int> > &dst_tets,
              std::vector<std::vector<int> > &dst_ntets,
              const std::string switches
	            );

void mesh2tet(const std::vector<Eigen::Vector3d> &src_points,
              const std::vector<std::vector<int> > &src_faces,
	            const std::vector<Eigen::Vector3d> &src_inner_poinst,              
              std::vector<Eigen::Vector3d > &dst_points,
              std::vector<std::vector<int> > &dst_tets,
              std::vector<std::vector<int> > &dst_ntets,
              std::vector<std::pair<int, int> > &tet_edges,
              std::vector<int> &dst_edge_marker, 
              const std::string switches
	            );

void refinement(std::vector<Eigen::Vector3d> &points,
	              std::vector<std::vector<int> > &tets,
	              std::vector<std::vector<int> > &ntets);

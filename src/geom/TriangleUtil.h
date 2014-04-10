#ifndef TRIANGLEUTIL_H__
#define TRIANGLEUTIL_H__

#include <iostream>
#include "../../external/triangle/triangle.h"
#include "Polyline.h"
#include "TriangleMesh.h"


namespace triangle_util{

//template <class T> void reDelaunay(T &mesh, std::string switches);
//void reDelaunay(Shape2D &mesh, std::vector<Eigen::Vector2i> &boundary_edges, std::string switches = std::string("pnze"));

void boundary2mesh(PolyLine &boundary, TriangleMesh &mesh, std::string switches);

void boundary2mesh(PolyLine &boundary, 
                   std::vector<Eigen::Vector3d> &points, 
                   TriangleMesh &mesh,
                   std::vector<int> &index_mapper,
                   std::string switches = std::string("pnze")
                   );

/*
void genMeshWithConstraints(std::vector<Eigen::Vector3d> &points, 
                            std::vector<Eigen::Vector2i> &edges,
                            TriangleMesh &mesh,
                            std::string switches
                            );

void genDualMeshWithConstraints(std::vector<Eigen::Vector3d> &points, 
                                std::vector<Eigen::Vector2i> &edges,
                                std::vector<Eigen::Vector3d> &dual_points,
                                std::vector<Eigen::Vector2i> &dual_edges);
*/                                
}

#endif /* end of include guard: TRIANGLEUTIL_H__ */

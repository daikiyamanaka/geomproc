#ifndef MESHUTIL_H__
#define MESHUTIL_H__

#include "TriangleMesh.h"

namespace utility{

void readPLY(std::string filename, std::vector<Eigen::Vector3d> &points, std::vector<std::vector<int> > &faces);

void writePLY(std::string basename, std::vector<Eigen::Vector3d> &points, std::vector<std::vector<int> > &faces);

void offsetWithNormal(const TriangleMesh &mesh, TriangleMesh &offsetted_mesh, const double d);

void divideByPlanes(TriangleMesh &mesh, std::vector<int> &face_segment_ids, const std::vector<Eigen::Vector4d> &planes);

}

#endif /* end of include guard: MESHUTIL_H__ */


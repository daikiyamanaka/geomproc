#ifndef UTILITY_H__
#define UTILITY_H__

#include <vector>
#include <eigen3/Eigen/Dense>

namespace utility{

void getDivisionPlanes(const Eigen::Vector3d &v, const Eigen::Vector3d &p, const int num_of_segments, std::vector<Eigen::Vector4d> &planes);

void divideByPlanes(const std::vector<Eigen::Vector3d> &points, std::vector<int> &point_segment_ids, const std::vector<Eigen::Vector4d> &planes);

void rodriguesRotation(Eigen::Matrix3d &mat, const Eigen::Vector3d &axis, const float &angle);

void pts2BBox(std::vector<Eigen::Vector3d> &points, Eigen::Vector3d &min, Eigen::Vector3d &max, double ratio);

double distPts2Seg(const Eigen::Vector3d &p, const Eigen::Vector3d &e0, const Eigen::Vector3d &e1);

double distPts2Line(const Eigen::Vector3d &p, const Eigen::Vector3d &x, const Eigen::Vector3d &v);

bool intersect(const Eigen::Vector3d& center, 
               const double &r,
               const Eigen::Vector3d& _p0, 
               const Eigen::Vector3d& _p1,
               std::vector<Eigen::Vector3d> &intersections,
               double intersection_threshold
               );
 
Eigen::Vector3d circumcenter(Eigen::Vector3d &a, Eigen::Vector3d &b, Eigen::Vector3d &c);

Eigen::Vector3d centroid(Eigen::Vector3d &a, Eigen::Vector3d &b, Eigen::Vector3d &c);

}

#endif /* end of include guard: UTILITY_H__ */

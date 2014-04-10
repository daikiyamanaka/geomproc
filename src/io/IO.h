#ifndef IO_H__
#define IO_H__
 
#include <string>
#include <vector>
#include <iostream>
#include <eigen3/Eigen/Core>

namespace IO{

  // Output //
	void savePoints(std::string filename, std::vector<Eigen::Vector3d> &points);

  void savePoints(std::string filename, std::vector<Eigen::Vector3d> &points, std::vector<double> &values);

  void savePolyLine(std::string filename, std::vector<Eigen::Vector3d> &points, std::vector<std::vector<int> > &edges);

  void savePolyLine(std::string filename, std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector2i > &edges);

  void saveTetAsVTK(const std::string filename, 
                    const std::vector<Eigen::Vector3d > &points, 
                    const std::vector<std::vector<int> >&tets);

  void writePLY(std::string basename, std::vector<Eigen::Vector3d> &points, std::vector<std::vector<int> > &faces);  

  // Input //
  void loadPoints(const std::string filename, std::vector<Eigen::Vector3d> &points);

  void loadPolyLine(const std::string filename, std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector2i> &edges);  
  
  void readPLY(std::string filename, std::vector<Eigen::Vector3d> &points, std::vector<std::vector<int> > &faces);

  void loadTetAsVtk(const std::string filename,
  	                std::vector<Eigen::Vector3d > &points,
  	                std::vector<std::vector<int> > &tets);
  
}
 
#endif /* end of include guard: IO_H__ */

#include "Truss.h"
#include <iostream>

const Truss::UnitCircle Truss::unit_circle = Truss::UnitCircle();

void saveTrussStructure(const std::string filename, std::vector<Truss> &truss_structure){
  std::ofstream ofs;
  ofs.open(filename.c_str());

  int num_of_points = 0; 
  int num_of_faces = 0;  
  std::vector<int> offsets;

  //  initialize //
  for(int i=0; i<truss_structure.size(); i++){
    const std::vector<Eigen::Vector3d> &points = truss_structure[i].getPoints();
    const std::vector<std::vector<int> > &faces = truss_structure[i].getFaces();
    offsets.push_back(num_of_points);
    num_of_points += points.size();
    num_of_faces += faces.size();
  }

  //  header //
  ofs << "ply" << std::endl;
  ofs << "format ascii 1.0" << std::endl;
  ofs << "comment VCGLIB generated" << std::endl;
  ofs << "element vertex " <<  num_of_points << std::endl;
  ofs << "property float x" << std::endl;
  ofs << "property float y" << std::endl;
  ofs << "property float z" << std::endl;
  ofs << "element face " << num_of_faces << std::endl;
  ofs << "property list uchar int vertex_indices" << std::endl;
  ofs << "end_header" << std::endl;

  // truss points //
  for(int i=0; i<(int)truss_structure.size(); i++){
    const std::vector<Eigen::Vector3d> &points = truss_structure[i].getPoints();
    for(int j=0; j<(int)points.size(); j++){      
      ofs << points[j][0] << " " << points[j][1] << " " << points[j][2] << std::endl;
    }
  }

  // truss faces //
  for(int i=0; i<truss_structure.size(); i++){
    const std::vector<std::vector<int> > &faces = truss_structure[i].getFaces();
    int offset = offsets[i];
    for(int j=0; j<(int)faces.size(); j++){
      ofs << 3 << " " << faces[j][0]+offset << " " << faces[j][1]+offset << " " << faces[j][2]+offset << std::endl;
    }
  }
  ofs.close();  
}

void rodriguesRotation(Eigen::Matrix3d &mat, Eigen::Vector3d axis, double angle){
	//mat.resize(3, 3);
	mat(0, 0) = cos(angle) + axis[0]*axis[0]*(1-cos(angle));
	mat(0, 1) = axis[0]*axis[1]*(1-cos(angle)) - axis[2]*sin(angle);
	mat(0, 2) = axis[1]*sin(angle) + axis[0]*axis[2]*(1-cos(angle));

	mat(1, 0) = axis[2]*sin(angle) + axis[0]*axis[1]*(1-cos(angle));
	mat(1, 1) = cos(angle) + pow(axis[1], 2)*(1-cos(angle));
	mat(1, 2) = -1*axis[0]*sin(angle) + axis[1]*axis[2]*(1-cos(angle));

	mat(2, 0) = -1*axis[1]*sin(angle) + axis[0]*axis[2]*(1-cos(angle));
	mat(2, 1) = axis[0]*sin(angle) + axis[1]*axis[2]*(1-cos(angle));
	mat(2, 2) = cos(angle) + pow(axis[2], 2)*(1-cos(angle));
}

Truss::Truss(Eigen::Vector3d s, Eigen::Vector3d t, double d): s_(s), t_(t), d_(d)
{	
	meshing();
}

Truss::~Truss()
{

}

void Truss::save(std::ofstream &ofs)
{

}

const std::vector<Eigen::Vector3d>& Truss::getPoints(){
	return points_;
}
const std::vector<std::vector<int> >& Truss::getFaces(){
	return faces_;
}

const std::vector<Eigen::Vector3d>& Truss::getNormals(){
	return normals_;
}

const Eigen::Vector3d& Truss::getNormal(int i){
	return normals_[i];
}

const Eigen::Vector3d& Truss::getSource(){
	return s_;
}

const Eigen::Vector3d& Truss::getTarget(){
	return t_;
}

void Truss::meshing()
{

	points_.resize(0);
	faces_.resize(0);

	double length = (t_-s_).norm();
	Eigen::Vector3d n0 = Eigen::Vector3d(0, 0, 1);
	Eigen::Vector3d n1 = t_-s_;
	Eigen::Vector3d axis = n0.cross(n1);
	axis.normalize();
	double theta = acos(n0.dot(n1)/(n1.norm()));
	Eigen::Matrix3d R;
	rodriguesRotation(R, axis, theta);
	Eigen::Vector3d tz = Eigen::Vector3d(0, 0, length);
	int n = UnitCircle::circle_division_num;

	for(int i=0; i<n; i++){
		Eigen::Vector3d u = unit_circle.points[i]*d_;
		Eigen::Vector3d p0 = R*u + s_;
		Eigen::Vector3d p1 = R*(u+tz) + s_;
		points_.push_back(p0);
		points_.push_back(p1);
  }	
  int point_size = 2*n;
  for(int i=0; i<point_size; i+=2){
  	std::vector<int> face(3);
  	face[0] = i; 
  	face[1] = (i+2)%point_size; 
  	face[2] = (i+1)%point_size;
  	faces_.push_back(face);
  	face[0] = (i+2)%point_size; 
  	face[1] = (i+3)%point_size; 
  	face[2] = (i+1)%point_size;
  	faces_.push_back(face);  	
  }

  // compute normals //
  normals_.resize(faces_.size());
  for(int i=0; i<faces_.size(); i++){
  	normals_[i] = (points_[faces_[i][1]]-points_[faces_[i][0]]).cross(points_[faces_[i][2]] - points_[faces_[i][0]]);
  	normals_[i].normalize();
  	//std::cout << normals_[i] << std::endl;
  }
}



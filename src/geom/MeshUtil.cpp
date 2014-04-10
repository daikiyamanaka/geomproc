#include "MeshUtil.h"
#include <fstream>

namespace utility{

void readPLY(std::string filename, std::vector<Eigen::Vector3d> &points, std::vector<std::vector<int> > &faces){
//void readPoint(std::string filename, PointSet *ps){
	std::ifstream ifs;
	ifs.open(filename.c_str());

	int vertex_number, face_number;
	while(1){
		std::string s_tmp;
		std::getline(ifs, s_tmp);
		//ifs >> s_tmp;
		//std::cout << s_tmp << std::endl;
		std::stringstream ss(s_tmp);
		std::string str;
		ss >> str;
		//std::cout << str << std::endl;
		if(str == "element"){
			ss >> str;
			if(str == "vertex"){
				ss >> vertex_number;
			}
			else if(str == "face"){
				ss >> face_number;
			}
		}
		if(str == "end_header"){
			break;
		}
	}

	std::string s_tmp;
	// --- read vertex ---
	for(int i=0; i<vertex_number; i++){
		std::getline(ifs, s_tmp);
		std::stringstream ss(s_tmp);
		float x, y, z;
		ss >> x;
		ss >> y;
		ss >> z;
		//std::cout << "x : " << x << " y : " << y << " z : " << z << std::endl;
		//std::vector<float> point;
		//point.push_back(x);
		//point.push_back(y);
		//point.push_back(z);
		points.push_back(Eigen::Vector3d(x, y, z));
		//constrained_point.push_back(Point(x, y, z));
		//vertices.push_back(i);
	}
	// --- read faces
	for(int i=0; i<face_number; i++){
		std::getline(ifs, s_tmp);
		std::stringstream ss(s_tmp);
		int v_index;
		ss >> v_index; //skip point number
		std::vector<int> face;
		for(int j=0; j<3; j++){
			ss >> v_index;
			face.push_back(v_index);
		}
		//std::cout << "face " << i << " " << face.at(0) << ", " << face.at(1) << " ," << face.at(2) << std::endl;
		faces.push_back(face);
	}
}

void writePLY(std::string basename, std::vector<Eigen::Vector3d> &points, std::vector<std::vector<int> > &faces){

	std::ofstream ofs;
	ofs.open((basename + ".ply").c_str());
	// --- header ---
	ofs << "ply" << std::endl;
	ofs << "format ascii 1.0" << std::endl;
	ofs << "comment VCGLIB generated" << std::endl;
	ofs << "element vertex " <<  points.size() << std::endl;
	ofs << "property float x" << std::endl;
	ofs << "property float y" << std::endl;
	ofs << "property float z" << std::endl;
	ofs << "element face " << faces.size() << std::endl;
	ofs << "property list uchar int vertex_indices" << std::endl;
	ofs << "end_header" << std::endl;

	for(int i=0; i<(int)points.size(); i++){
		ofs << points[i][0] << " " << points[i][1] << " " << points[i][2] << std::endl;
	}
	for(int i=0; i<(int)faces.size(); i++){
		ofs << faces[i].size() << " ";
		for(int j=0; j<(int)faces[i].size()-1; j++){
			ofs << faces[i][j] << " ";
		}
		ofs << faces[i][faces[i].size()-1] << std::endl;
	}
	ofs.close();
}

void offsetWithNormal(const TriangleMesh &mesh, TriangleMesh &offsetted_mesh, const double d){

  std::vector<Eigen::Vector3d> points = mesh.points;
  std::vector<std::vector<int> > faces = mesh.facets;

  for(int i=0; i<(int)points.size(); i++){
  	Eigen::Vector3d normal = mesh.v_normals[i];
  	points[i][0] -= d*normal[0];
  	points[i][1] -= d*normal[1];
  	points[i][2] -= d*normal[2];
  }

  offsetted_mesh.createFromFaceVertex(points, faces);
};

void divideByPlanes(TriangleMesh &mesh, std::vector<int> &face_segment_ids, const std::vector<Eigen::Vector4d> &planes)
{
	Eigen::Vector3d p0, p1, p;
	double d0, d1;

	int num_of_faces = mesh.get_number_of_faces();
	int num_of_planes = planes.size();

	face_segment_ids.resize(num_of_faces);

  // augment planes //
  std::vector<Eigen::Vector4d> augment_planes = planes;
  for(int i=0; i<num_of_planes; i++){
    Eigen::Vector4d plane = planes[i];
    augment_planes.push_back(Eigen::Vector4d(-plane[0], -plane[1], -plane[2], plane[3]));
  }	

  for(int l=0; l<(int)augment_planes.size(); l++){
    int l2 = (l+1)%augment_planes.size();
    p0 = Eigen::Vector3d(augment_planes[l][0], augment_planes[l][1], augment_planes[l][2]);
    p1 = Eigen::Vector3d(augment_planes[l2][0], augment_planes[l2][1], augment_planes[l2][2]);
		d0 = augment_planes[l][3];
		d1 = augment_planes[l2][3];
		for(int i=0; i<num_of_faces; i++){
			p = mesh.centroids[i];
			double v0 = p0.dot(p)+d0;
			double v1 = p1.dot(p)+d1;
			if(v0>0 && v1<0){
				face_segment_ids[i] = l;
			}
		}
	}
}

}

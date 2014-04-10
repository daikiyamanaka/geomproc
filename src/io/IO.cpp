#include "IO.h"
#include <sstream>
#include <fstream>
namespace IO{

void savePoints(std::string filename, std::vector<Eigen::Vector3d> &points){
	std::ofstream ofs;
	ofs.open(filename.c_str());
	for(int i=0; i<(int)points.size(); i++){
		ofs << points[i][0] << " " << points[i][1] << " " << points[i][2] << std::endl;
	}
	ofs.close();	
}

void savePoints(std::string filename, std::vector<Eigen::Vector3d> &points, std::vector<double> &values){
	std::ofstream ofs;
	ofs.open(filename.c_str());
	for(int i=0; i<(int)points.size(); i++){
		ofs << points[i][0] << " " << points[i][1] << " " << points[i][2] << std::endl;
	}

	for(int i=0; i<(int)points.size(); i++){
		ofs << values[i] << std::endl;
	}
	ofs.close();
}

void savePolyLine(std::string filename, std::vector<Eigen::Vector3d> &points, std::vector<std::vector<int> > &edges){
	std::ofstream ofs;
	ofs.open(filename.c_str());

  ofs << "ply" << std::endl;
  ofs << "format ascii 1.0" << std::endl;   
  ofs << "element vertex "<< (int)points.size() << std::endl;
  ofs << "property float x" << std::endl;   
  ofs << "property float y" << std::endl;   
  ofs << "property float z" << std::endl;   
  ofs << "element edge " << (int)edges.size() << std::endl;   
  ofs << "property int vertex1" << std::endl;
  ofs << "property int vertex2" << std::endl;   
  ofs << "end_header" << std::endl;   

	for(int i=0; i<(int)points.size(); i++){
		ofs << points[i][0] << " " << points[i][1] << " " << points[i][2] << std::endl;
	}

	for(int i=0; i<(int)edges.size(); i++){
		ofs << edges[i][0] << " " << edges[i][1] << std::endl;
	}

	ofs.close();	
}

void savePolyLine(std::string filename, std::vector<Eigen::Vector3d> &points, std::vector<Eigen::Vector2i > &edges){
	std::ofstream ofs;
	ofs.open(filename.c_str());

  ofs << "ply" << std::endl;
  ofs << "format ascii 1.0" << std::endl;   
  ofs << "element vertex "<< (int)points.size() << std::endl;
  ofs << "property float x" << std::endl;   
  ofs << "property float y" << std::endl;   
  ofs << "property float z" << std::endl;   
  ofs << "element edge " << (int)edges.size() << std::endl;   
  ofs << "property int vertex1" << std::endl;
  ofs << "property int vertex2" << std::endl;   
  ofs << "end_header" << std::endl;   

	for(int i=0; i<(int)points.size(); i++){
		ofs << points[i][0] << " " << points[i][1] << " " << points[i][2] << std::endl;
	}
	for(int i=0; i<(int)edges.size(); i++){
		ofs << edges[i][0] << " " << edges[i][1] << std::endl;
	}
	ofs.close();		
}

void loadPoints(const std::string filename, std::vector<Eigen::Vector3d> &points){
  std::ifstream ifs;
  ifs.open(filename.c_str());
  points.resize(0);
  while(1){
  	std::string line;  	  	
  	std::getline(ifs, line);
  	std::stringstream ss(line);
  	if(ifs.fail()) break;  	
  	double x, y, z;  	
  	try{
  		ss >> x;
  		ss >> y;
  		ss >> z;
  		points.push_back(Eigen::Vector3d(x, y, z));
  	}catch(std::exception e){
  		std::cerr << e.what() << std::endl;
  		break;
  	}
  }
  ifs.close();
}

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

void saveTetAsVTK(const std::string filename, const std::vector<Eigen::Vector3d > &points, const std::vector<std::vector<int> >&tets){
  int num_of_points = points.size();
  int num_of_tets = tets.size();
  std::ofstream ofs;
  ofs.open(filename.c_str());

  //Header //
  ofs << "# vtk DataFile Version 3.0" << std::endl;

  //Title //
  ofs << "Tetrahedral Mesh" << std::endl;

  //Data type //
  ofs << "ASCII" << std::endl;

  //Geometry, Topology //
  ofs << "DATASET UNSTRUCTURED_GRID" << std::endl;
  ofs << "POINTS " << num_of_points << " float" << std::endl;
  for(int i=0; i< num_of_points; i++){
    ofs << points[i][0] << " ";
    ofs << points[i][1] << " ";
    ofs << points[i][2] << std::endl;
  }
  ofs << "CELLS " << num_of_tets << " "<< num_of_tets*5 << std::endl;
  for(int i=0; i<num_of_tets; i++){
    ofs << "4 ";
    ofs << tets[i][0] << " ";
    ofs << tets[i][1] << " ";
    ofs << tets[i][2] << " ";
    ofs << tets[i][3] << std::endl;
  }
  ofs << "CELL_TYPES " << num_of_tets << std::endl;
  for(int i=0; i<num_of_tets; i++){
    ofs << "10" << std::endl;
  }

  //Dataset attributes
  /*
  ofs << std::endl;
  ofs << "CELL_DATA "<< num_of_tets << std::endl;
  ofs << "SCALARS cell_scalars float 1" << std::endl;
  ofs << "LOOKUP_TABLE default" << std::endl;
  for(int i=0; i<tetio.number; i++){
    ofs << float(ct[i]) << std::endl;
  }
*/  
  ofs.close();
}

void loadTetAsVtk(const std::string filename,
                  std::vector<Eigen::Vector3d > &points,
                  std::vector<std::vector<int> > &tets)
{
  std::ifstream ifs(filename.c_str());
  std::string line;
  int num_of_points;
  int num_of_tets;
  int count = 0;

  // load points //
  while(getline(ifs, line)){
    std::stringstream ss(line);
    std::string s;
    ss >> s;
    if(s == "POINTS"){
      ss >> num_of_points;
      break;        
    }
  }      
  count=0;
  points.resize(num_of_points);
  while(getline(ifs, line)){
    double x, y, z;
    std::stringstream ss(line);    
    ss >> x;
    ss >> y;
    ss >> z;
    Eigen::Vector3d p(x, y, z);
    points[count] = p;
    count++;
    if(count == num_of_points){
      break;
    }
  }

  // load tets //  
  while(getline(ifs, line)){
    std::stringstream ss(line);
    std::string s;
    ss >> s;
    if(s == "CELLS"){
      ss >> num_of_tets;
      break;        
    }
  }      
  count=0;
  tets.resize(num_of_tets);
  while(getline(ifs, line)){
    int cell_point_num;
    std::vector<int> tet(4);
    std::stringstream ss(line);  
    ss >> cell_point_num;
    ss >> tet[0];
    ss >> tet[1];
    ss >> tet[2];
    ss >> tet[3];    
    tets[count] = tet;
    count++;
    if(count == num_of_tets){
      break;
    }
  }  

}

}

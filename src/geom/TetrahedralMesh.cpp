#include <sstream>

#include "TetrahedralMesh.h"
#include "../io/IO.h"
#include "../geom/TetUtil.h"

TetrahedralMesh::TetrahedralMesh(){

}

TetrahedralMesh::~TetrahedralMesh(){

}

void TetrahedralMesh::load(const std::string filename){
	IO::loadTetAsVtk(filename, points_, tets_);
	calcNeighbors();
	init();
}

void TetrahedralMesh::save(const std::string filename){
	IO::saveTetAsVTK(filename, points_, tets_);
}

void TetrahedralMesh::createFrom(const TriangleMesh &mesh, const TetgenParams params=TetgenParams()){
	std::stringstream switches;
	switches << "pzn";
	if(params.min_volume >= 0){
		switches << "a" << params.min_volume;
	}
	if(params.quality >= 0){
		switches << "q" << params.quality;
	}
	mesh2tet(mesh.points,
		       mesh.facets,
		       points_,
		       tets_,
		       ntets_,
		       switches.str());	
	init();
}

void TetrahedralMesh::createFrom(const std::vector<Eigen::Vector3d> &points, 
	                               const std::vector<std::vector<int> > &tets, 
	                               const TetgenParams params)
{
	std::copy(points.begin(), points.end(), points_.begin());
	std::copy(tets.begin(), tets.end(), tets_.begin());
	init();
}

void TetrahedralMesh::getBBox(Eigen::Vector3d &min, Eigen::Vector3d &max){
	min = min_;
	max = max_;
}

const Eigen::Vector3d &TetrahedralMesh::getPoint(int index){	
	return points_[index];
}
const std::vector<int> &TetrahedralMesh::getTetrahedron(int index){
	return tets_[index];
}
const std::vector<int> &TetrahedralMesh::getNeighborTetrahedron(int index){
	return ntets_[index];
}

const std::vector<Eigen::Vector3d> &TetrahedralMesh::getPoints(){
	return points_;
}

const std::vector<std::vector<int> > &TetrahedralMesh::getTetrahedra(){
	return tets_;
}

const std::vector<std::vector<int> > & TetrahedralMesh::getNeighborTetrahedra(){
	return ntets_;
}

const Eigen::Vector3d &TetrahedralMesh::getFaceNormal(int t_id, int f_id){
	return face_normals_[t_id][f_id];
}

void TetrahedralMesh::init(){
	computeBBox();
	computeFaceNormals();
}

void TetrahedralMesh::calcNeighbors(){

	//static int faces[4][3] = {{0,1,3},{1,0,2},{3,2,0},{2,3,1}};

    int i;
    int num_of_vertices = points_.size();
    int num_of_tet = tets_.size();
    int *deg = new int[num_of_vertices];
    for(i=0; i<num_of_vertices; i++)
      deg[i] = 0;
    for(i=0; i<num_of_tet; i++){
      std::vector<int> t = tets_[i];
      for(int j=0; j<4; j++)
        deg[t[j]]++;
    }
    
    int **link = new int*[num_of_vertices];
    for(i=0; i<num_of_vertices; i++)
      if(deg[i] != 0)
        link[i] = new int[deg[i]];
    
    for(i=0; i<num_of_vertices; i++)
      deg[i] = 0;
    for(i=0; i<num_of_tet; i++){
      std::vector<int> t = tets_[i];      
      for(int j=0; j<4; j++)
        link[t[j]][deg[t[j]]++] = i;
    }
    
    ntets_.resize(num_of_tet);
    for(i=0; i<num_of_tet; i++){
    	std::vector<int> ntet(4);
      for(int j=0; j<4; j++){
        ntet[j] = -1;
      }
      ntets_[i] = ntet;
    }
    
    //Too many loops but it still works in O(vertexN)
    for(i=0; i<num_of_vertices; i++){
      int d = deg[i];
      int* l = link[i];
      for(int j=0; j<d; j++){
      	std::vector<int> t = tets_[l[j]];        
        for(int k=0; k<4; k++){
          if(ntets_[l[j]][k] >= 0)
            continue;
          
          for(int m=0; m<d; m++){
            if(m == j)
              continue;
            std::vector<int> t1 = tets_[l[m]];                    
            for(int s=0; s<4; s++){
              for(int x=0; x<3; x++)
                if(t[faces[k][2]] == t1[faces[s][x]] &&
                   t[faces[k][1]] == t1[faces[s][(x+1)%3]] &&
                   t[faces[k][0]] == t1[faces[s][(x+2)%3]] ){
                  ntets_[l[j]][k] = l[m];
                  ntets_[l[m]][s] = l[j];
                  break;
                }
              if(ntets_[l[j]][k] >= 0)
                break;
            }
            if(ntets_[l[j]][k] >= 0)
              break;
          }
        }
      }
    }
    
    for(i=0; i<num_of_vertices; i++)
      if(deg[i] != 0)
        delete[] link[i];
    delete[] deg;
    delete[] link;	
}

void TetrahedralMesh::computeBBox(){
  min_ = max_ = points_[0];
  for(int i=1; i<(int)points_.size(); i++){
    for(int j=0; j<3; j++){
    	if(points_[i][j] < min_[j]){
    		min_[j] = points_[i][j];
      }
      if(points_[i][j] > max_[j]){
      	max_[j] = points_[i][j];
      }
    }
  }
}

void TetrahedralMesh::computeFaceNormals(){
	std::vector<Eigen::Vector3d> tri(3);
	std::vector<Eigen::Vector3d> normals(4);
	face_normals_.resize(tets_.size());
	for(int i=0; i<(int)tets_.size(); i++){
		for(int j=0; j<4; j++){
			tri[0] = points_[tets_[i][faces[j][0]]];
      tri[1] = points_[tets_[i][faces[j][1]]];
      tri[2] = points_[tets_[i][faces[j][2]]];			
      Eigen::Vector3d n = (tri[1]-tri[0]).cross(tri[2]-tri[0]);
      n.normalize();
      normals[j] = n;
		}
		face_normals_[i] = normals;
	}
}

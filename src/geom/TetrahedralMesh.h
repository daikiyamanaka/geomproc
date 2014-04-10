#include <string>

#include "TriangleMesh.h"


class TetrahedralMesh
{
public:
	TetrahedralMesh();
	~TetrahedralMesh();

  struct TetgenParams{
  	TetgenParams(){min_volume=-1; quality=-1;};
  	double min_volume;
  	double quality;
  };

  const int faces[4][3] = {{1, 2, 3}, {0, 3, 2}, {0, 1, 3}, {0, 2, 1}};	

	//----------------
	// IO
	//----------------
	void load(const std::string filename);
	void save(const std::string filename);

	//----------------
	// Creation
	//----------------	
	void createFrom(const TriangleMesh &mesh, const TetgenParams params);
	void createFrom(const std::vector<Eigen::Vector3d> &points, 
		              const std::vector<std::vector<int> > &tets, 
		              const TetgenParams params);

	//------------------
	// Access Functions
	//------------------
 	void getBBox(Eigen::Vector3d &min, Eigen::Vector3d &max);
 	const Eigen::Vector3d & getPoint(int index);
 	const std::vector<int> & getTetrahedron(int index);
 	const std::vector<int> & getNeighborTetrahedron(int index);
 	const std::vector<Eigen::Vector3d> & getPoints();
 	const std::vector<std::vector<int> > & getTetrahedra();
 	const std::vector<std::vector<int> > & getNeighborTetrahedra();
 	const Eigen::Vector3d &getFaceNormal(int t_id, int f_id);

	//-----------
	// Modifier
	//-----------

private:
	void init();
	void calcNeighbors();	
	void computeBBox();
	void computeFaceNormals();

private:
	std::vector<Eigen::Vector3d> points_;
	std::vector<std::vector<int> > tets_;	
	std::vector<std::vector<int> > ntets_;
	std::vector<std::vector<Eigen::Vector3d> > face_normals_;
	double volume_;
	Eigen::Vector3d min_, max_;
};

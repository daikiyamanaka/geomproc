#ifndef TRUSS_H__
#define TRUSS_H__

#include <vector>
#include <fstream>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

/**
 * @brief Truss class 
 */
class Truss
{
public:
	class UnitCircle{
		public:
		const static int circle_division_num = 15;
		UnitCircle(){
			double d_theta = 2*M_PI/(double)circle_division_num;
			for(int i=0; i<circle_division_num; i++){
				double theta = d_theta*((double)i+0.5);
				double x = cos(theta);
				double y = sin(theta);
				points.push_back(Eigen::Vector3d(x, y, 0));
			}			
		};
		std::vector<Eigen::Vector3d> points;
	};

public:	
	const static double offset;	
	const static UnitCircle unit_circle;
public:
	Truss(){};
	Truss(Eigen::Vector3d s, Eigen::Vector3d t, double d);
	~Truss();

	Truss(const Truss& rt){
		d_ = rt.d_;
		s_ = rt.s_;
		t_ = rt.t_;
		points_ = rt.points_;
		faces_ = rt.faces_;
		normals_ = rt.normals_;
	};

	//-------
	// IO
	//-------	
	void save(std::ofstream &ofs);

	//-----------------
	// Access Function
	//-----------------
	const std::vector<Eigen::Vector3d>& getPoints();
	const std::vector<std::vector<int> >& getFaces();	
	const std::vector<Eigen::Vector3d>& getNormals();
	const Eigen::Vector3d& getNormal(int i);
	const Eigen::Vector3d& getSource();
	const Eigen::Vector3d& getTarget();

private:	
	void meshing();
	/**
	 * @brief truss radius
	 */
	double d_;
	Eigen::Vector3d s_, t_;
	std::vector<Eigen::Vector3d> points_;
	std::vector<std::vector<int> > faces_;
	std::vector<Eigen::Vector3d> normals_;
};

void saveTrussStructure(const std::string filename, std::vector<Truss> &truss_structure);

#endif /* end of include guard: TRUSS_H__ */

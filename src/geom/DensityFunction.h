#ifndef DENSITYFUNCTION_H__
#define DENSITYFUNCTION_H__

#include <boost/function.hpp>
#include "../geom/Volume.h"

/**
 * @brief Density Function
 * @details abstract class for density function
 */
class DensityFunction
{
public:
	DensityFunction(){};
	~DensityFunction(){};
	virtual double operator()(const double x, const double y, const double z) {
		return 1.0;
	};
};

/**
 * @brief Continueous Density Function
 * @details set functor as continuous function
 */
class ContinuousDensityFunction: public DensityFunction
{
public:
	ContinuousDensityFunction(){
	};

	~ContinuousDensityFunction(){

	};

	void setFunc(boost::function<double(double, double, double)> func){
		func_ = func;
	}

	double operator()(const double x, const double y, const double z){
		return func_(x, y, z);	
	};

	private:
		boost::function<double(double, double, double)> func_;
};

/**
 * @brief Discrete Density Function
 * @details density value is calculated by grid assigned value 
 */
class DiscreteDensityFunction: public DensityFunction
{

public:
	DiscreteDensityFunction(Eigen::Vector3d pitch, Eigen::Vector3i grid_size, Eigen::Vector3d minC);
	~DiscreteDensityFunction();

	//--------
	// IO
	//--------
	void save(std::string filename);
	void saveInside(std::string filename);

	//-------------
	// Density
	//-------------
	double operator()(const int i, const int j, const int k);
  double operator()(const double x, const double y, const double z);
  double operator()(const Eigen::Vector3d &p);

	//-------------
	// Modifier
	//-------------
  void setFuncVal(const int ix, const int iy, const int iz, const double val);
  void setInterpolant(Volume<float>::INTERPOLANT interpolant);
  void fill(float val);

	//-----------------
	// Access Function
	//-----------------
  Eigen::Vector3d calcPosition(const int ix, const int iy, const int iz);
  Eigen::Vector3d getPitch();
  Eigen::Vector3i getGridSize();
  Eigen::Vector3d getMinCorner();    

	//-------------
	// Predicates
	//-------------
  bool include(const double x, const double y, const double z);
  bool include(const Eigen::Vector3d &p);	
  bool isValid(const int ix, const int iy, const int iz);  
  bool isValid(const double x, const double y, const double z);
  bool isValid(const Eigen::Vector3d &p);

private:

	Eigen::Vector3d pitch_;
	Eigen::Vector3i grid_size_;
	Eigen::Vector3d minC_;	

	Volume<float> *val_;
	Volume<int> *label_;
};

#endif /* end of include guard: DENSITYFUNCTION_H__ */

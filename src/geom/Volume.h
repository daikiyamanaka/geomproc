/*
 * Volume.hpp
 *
 *  Created on: May 23, 2013
 *      Author: daikiyamanaka
 */

#ifndef VOLUME_HPP_
#define VOLUME_HPP_

#include <eigen3/Eigen/Core>
#include "ArrayUtil.h"

template <class T>
class Volume {

public:
 	typedef enum INTERPOLANT{
		NN = 0,
		LINEAR = 1,
		BILINEAR = 2
	} INTERPOLANT;	

public:


	Volume(Eigen::Vector3f pitch, Eigen::Vector3i grid_size, Eigen::Vector3f minC);
	virtual ~Volume();

	void save(std::string filename);

	//-----------------
	// Modifier
	//-----------------
	void fill(T value);

	void fillRange(const std::pair<int, int> range_x,
	               const std::pair<int, int> range_y,
	               const std::pair<int, int> range_z,
	               T value);

	void setValue(const int x, const int y, const int z, const T val);

    void setInterpolant(INTERPOLANT interpolant);	

	//-----------------
	// Access Function
	//-----------------
    T operator()(const int &ix, const int &iy, const int &iz);

    T operator()(const double &x, const double &y, const double &z);

	T ***getValue();	

	T getValue(const int &x, const int &y, const int &z);

	T getValue(const double &x, const double &y, const double &z);

	Eigen::Vector3i getIndex(const Eigen::Vector3d &p);

	Eigen::Vector3i getIndex(const double &x, const double &y, const double &z);

	Eigen::Vector3d getPosition(const int &i, const int &j, const int &k);

	Eigen::Vector3f getPitch();

	float getPitchX();

	float getPitchY();

	float getPitchZ();

	Eigen::Vector3i getGridSize();

	int getGridSizeX();

	int getGridSizeY();

	int getGridSizeZ();

	Eigen::Vector3f getMinCorner();

private:
	// method for function interpolation //
	T interpolate(const double x, const double y, const double z);

	T interpolateNN(const double x, const double y, const double z);

	T interpolateLinear(const double x, const double y, const double z);

	T interpolateBiLinear(const double x, const double y, const double z);

	T ***value;

	Eigen::Vector3f pitch;

	Eigen::Vector3f size;

	Eigen::Vector3i grid_size;
	
	Eigen::Vector3f minC;

	INTERPOLANT interpolant_;	

};


#endif /* VOLUME_HPP_ */

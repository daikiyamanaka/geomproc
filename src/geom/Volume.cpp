/*
 * Volume.cpp
 *
 *  Created on: May 23, 2013
 *      Author: daikiyamanaka
 */

#include "Volume.h"


static const int grid_id[8][3] = {{0, 0, 0}, {0, 0, 1}, 
                                  {1, 0, 0}, {1, 1, 0}, 
                                  {0, 0, 1}, {1, 0, 1}, 
                                  {0, 1, 1}, {1, 1, 1}};

template<class T> Volume<T>::Volume(Eigen::Vector3f pitch, Eigen::Vector3i grid_size, Eigen::Vector3f minC): minC(minC), pitch(pitch), grid_size(grid_size)
{
	size = Eigen::Vector3f(
			pitch[0]*(float)grid_size[0],
			pitch[1]*(float)grid_size[1],
			pitch[2]*(float)grid_size[2]
			);

	value = (T***)malloc((int)grid_size[0] * sizeof(T**));
	value[0] = (T**)malloc((int)grid_size[0] * (int)grid_size[1] * sizeof(T*));
	value[0][0] = (T*)malloc((int)grid_size[0] * (int)grid_size[1] * (int)grid_size[2] * sizeof(T));
	allocate3D(value, (int)grid_size[0], (int)grid_size[1], (int)grid_size[2], (T)1);
	interpolant_ = NN;
}

template<class T> Volume<T>::~Volume() {
	// TODO Auto-generated destructor stub
	delete(value[0][0]);
	delete(value[0]);
	delete(value);		
}

template<class T> void Volume<T>::fill(T v){
	for(int k=0; k<grid_size[2]; k++){
		for(int j=0; j<grid_size[1]; j++){
			for(int i=0; i<grid_size[0]; i++){
				value[i][j][k] = v;
			}
		}
	}
}

template <class T> void Volume<T>::fillRange(const std::pair<int, int> range_x,
                                             const std::pair<int, int> range_y,
		                                         const std::pair<int, int> range_z,
		                                         T v)
{
	assert(range_x.first <= range_x.second);
	assert(range_y.first <= range_y.second);
	assert(range_z.first <= range_z.second);		

	for(int k=range_z.first; k<range_z.second; k++){
		for(int j=range_y.first; j<range_y.second; j++){
			for(int i=range_x.first; i<range_x.second; i++){
				value[i][j][k] = v;
			}
		}
	}
}

template<class T>T Volume<T>::operator()(const int &ix, const int &iy, const int &iz){
	return getValue(ix, iy, iz);
}

template<class T> T Volume<T>::operator()(const double &x, const double &y, const double &z){
	return getValue(x, y, z);
}

template<class T> T ***Volume<T>::getValue(){
	return value;
}

template<class T>T Volume<T>::getValue(const int &i, const int &j, const int &k){
	return value[i][j][k];
}

template<class T> T Volume<T>::getValue(const double &x, const double &y, const double &z){
	
	return interpolate(x, y, z);
}

template<class T>void Volume<T>::setValue(const int x, const int y, const int z, const T val){
	value[x][y][z] = val;
}

template <class T> void Volume<T>::setInterpolant(INTERPOLANT interpolant){
	interpolant_ = interpolant;
}

template <class T> Eigen::Vector3i Volume<T>::getIndex(const Eigen::Vector3d &p){
	return getIndex(p[0], p[1], p[2]);
}

template <class T> Eigen::Vector3i Volume<T>::getIndex(const double &x, const double &y, const double &z){
   int ix = (int)((x-minC[0])/pitch[0]);
   int iy = (int)((y-minC[1])/pitch[1]);
   int iz = (int)((z-minC[2])/pitch[2]);

   ix = (ix < 0)? -1: ix;
   iy = (iy < 0)? -1: iy;
   iz = (iz < 0)? -1: iz; 

   ix = (ix >= grid_size[0])? -1 : ix;
   iy = (iy >= grid_size[1])? -1 : iy;
   iz = (iz >= grid_size[2])? -1 : iz;

   return Eigen::Vector3i(ix, iy, iz);
}

template <class T> Eigen::Vector3d Volume<T>::getPosition(const int &i, const int &j, const int &k){
	double x = minC[0]+pitch[0]*(i+0.5);
	double y = minC[1]+pitch[1]*(j+0.5);
	double z = minC[2]+pitch[2]*(k+0.5);
	return Eigen::Vector3d(x, y, z);
}

template<class T> Eigen::Vector3f Volume<T>::getPitch(){
	return pitch;
}

template<class T> float Volume<T>::getPitchX(){
	return pitch[0];
}

template<class T> float Volume<T>::getPitchY(){
	return pitch[1];
}

template<class T>float Volume<T>::getPitchZ(){
	return pitch[2];
}

template<class T> Eigen::Vector3i Volume<T>::getGridSize(){
	return grid_size;
}

template<class T> int Volume<T>::getGridSizeX(){
	return grid_size[0];
}

template<class T> int Volume<T>::getGridSizeY(){
	return grid_size[1];
}

template<class T> int Volume<T>::getGridSizeZ(){
	return grid_size[2];
}

template<class T> Eigen::Vector3f Volume<T>::getMinCorner(){
	return minC;
}

template<class T> void Volume<T>::save(std::string filename){
	save3D(value, grid_size[0], grid_size[1], grid_size[2], filename);
}

template<class T>	T Volume<T>::interpolate(const double x, const double y, const double z){
	switch(interpolant_){
		case NN:
		  return interpolateNN(x, y, z);
		  break;
		case LINEAR:
		  return interpolateLinear(x, y, z);
		  break;
		case BILINEAR:
		  return interpolateBiLinear(x, y, z);
		  break;
		default:
		  return interpolateNN(x, y, z);
		  break;
	}
}
/**
 * @brief Interpolation
 * @details return nearest neighbor value
 */
template<class T>	T Volume<T>::interpolateNN(const double x, const double y, const double z){
   int ix = (int)((x-minC[0])/pitch[0]);
   int iy = (int)((y-minC[1])/pitch[1]);
   int iz = (int)((z-minC[2])/pitch[2]);
   ix = (ix >= grid_size[0])? grid_size[0]-1 : ix;
   ix = (ix < 0)? 0: ix;
   iy = (iy >= grid_size[1])? grid_size[1]-1 : iy;
   iy = (iy < 0)? 0: iy;  	
   iz = (iz >= grid_size[2])? grid_size[2]-1 : iz;
   iz = (iz < 0)? 0: iz; 
   return getValue(ix, iy, iz);
}

/**
 * @brief Linear interpolation
 * @details calc linear interpolation value by 8 grid points
 */
template<class T>	T Volume<T>::interpolateLinear(const double x, const double y, const double z){

  int ix = (int)((x-minC[0])/pitch[0]);
  int iy = (int)((y-minC[1])/pitch[1]);
  int iz = (int)((z-minC[2])/pitch[2]);

  double wx = (x-minC[0]-ix*pitch[0])/pitch[0];
  double wy = (y-minC[1]-iy*pitch[1])/pitch[1];
  double wz = (z-minC[2]-iz*pitch[2])/pitch[2];

  std::vector<T> values(8);

  for(int i=0; i<8; i++){
  	int x_id = ix+grid_id[i][0];
  	int y_id = iy+grid_id[i][1];
  	int z_id = iz+grid_id[i][2];  	  	
  	x_id = (x_id >= grid_size[0])? grid_size[0]-1 : x_id;
  	x_id = (x_id < 0)? 0: x_id;
  	y_id = (y_id >= grid_size[1])? grid_size[1]-1 : y_id;
  	y_id = (y_id < 0)? 0: y_id;  	
  	z_id = (z_id >= grid_size[2])? grid_size[2]-1 : z_id;
  	z_id = (z_id < 0)? 0: z_id;
  	values[i] = getValue(x_id, y_id, z_id);
  }

  // interpolation along widh x direction //
  for(int i=0; i<4; i++){
  	values[i] = (1-wx)*values[i] + wx*values[i+4];
  }

  // interpolation along widh y direction //  
  values[0] = (1-wy)*values[0] + wy*values[2];
  values[1] = (1-wy)*values[1] + wy*values[3];

  // interpolation along widh z direction //  
  return (1-wz)*values[0]+wz*values[1];
}

template<class T>	T Volume<T>::interpolateBiLinear(const double x, const double y, const double z){
	
}

template class Volume<int>;
template class Volume<float>;
template class Volume<double>;

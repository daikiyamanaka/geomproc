/*
 * gridUtil.hpp
 *
 *  Created on: May 20, 2013
 *      Author: daikiyamanaka
 */

#ifndef GRIDUTIL_HPP_
#define GRIDUTIL_HPP_

#include <iostream>
#include <fstream>
#include <string>
#include <eigen3/Eigen/Sparse>

template <class X> void allocate3D(X ***p, int sx, int sy, int sz, X value);
template <class T> void save3D(T ***p, int sx, int sy, int sz, std::string filename);

#endif /* GRIDUTIL_HPP_ */

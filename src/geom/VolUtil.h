#ifndef VOLUTIL_H__
#define VOLUTIL_H__

#include <eigen3/Eigen/Core>
#include "Volume.h" 

namespace utility{

template <class T> void divideByPlanes(Volume<T> *vol, const std::vector<Eigen::Vector4d> &planes);
	
}

#endif /* end of include guard: VOLUTIL_H__ */

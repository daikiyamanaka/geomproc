#include "SignedDistanceField.h"

template <class T> SignedDistanceField<T>::~SignedDistanceField(){		
		if(function_ != NULL){
			delete(function_);
		}
	}

template <class T> SignedDistanceField<T>::SignedDistanceField(const std::vector<Eigen::Vector3d> &points, const std::vector<Eigen::Vector3d> &normals){
		assert(points.size() == normals.size());
    std::vector<Point_with_normal> points_with_normal;
    points_with_normal.resize(points.size());

    // create point_with_normal //
    for(int i=0; i<(int)points.size(); i++){
    	points_with_normal[i] = Point_with_normal(points[i][0], points[i][1], points[i][2], 
    		                                 Vector(normals[i][0], normals[i][1], normals[i][2]));    	
    }

    // reconstruct implicit function //
    function_ = new Poisson_reconstruction_function(points_with_normal.begin(), points_with_normal.end(),
                                           CGAL::make_normal_of_point_with_normal_pmap(PointList::value_type()));

    if(!function_->compute_implicit_function() ) {
    	std::cout << "compute implicit function is failure" << std::endl;
    }
    computeBBox(points);
	}

/*
template <class T>	T SignedDistanceField<T>::operator()(const int ix, const int iy, const int iz){

	}
*/
template <class T>	T SignedDistanceField<T>::operator()(const double x, const double y, const double z){	
		Point p(x, y, z);
		return (*function_)(p);
	}

template <class T>	void SignedDistanceField<T>::rasterize(Volume<T> *vol){
		Eigen::Vector3f minC = vol->getMinCorner();
		Eigen::Vector3f pitch = vol->getPitch();
		Eigen::Vector3i grid_size = vol->getGridSize();

		for(int k=0; k<grid_size[2]; k++){
			for(int j=0; j<grid_size[1]; j++){
				for(int i=0; i<grid_size[0]; i++){
					double x = (i+0.5)*pitch[0]+minC[0];
					double y = (j+0.5)*pitch[1]+minC[1];
					double z = (k+0.5)*pitch[2]+minC[2];
					double val = (*function_)(Point(x, y, z));
					vol->setValue(i, j, k, val);
				}
			}
		}
	}

/**
 * @brief isIn
 * @details in-side is negative
 */
template <class T>	bool SignedDistanceField<T>::isIn(const double x, const double y, const double z){
	Point p(x, y, z);
	if((*function_)(p) < 0){
		return true;
	}
	return false;
}

template <class T> bool SignedDistanceField<T>::isIn(const Eigen::Vector3d p){
	return isIn(p[0], p[1], p[2]);
}

template <class T>	void SignedDistanceField<T>::getBBox(Eigen::Vector3d &min, Eigen::Vector3d &max){
		min = min_;
		max = max_;
	}
template <class T>	void SignedDistanceField<T>::computeBBox(const std::vector<Eigen::Vector3d> &points){
		min_ = max_ = points[0];
    for(int i=1; i<(int)points.size(); i++){
    	for(int j=0; j<3; j++){
    		if(points[i][j] < min_[j]){
    			min_[j] = points[i][j];
    		}
    		if(points[i][j] > max_[j]){
    			max_[j] = points[i][j];
    		}
    	}
    }
    center_ = (max_+min_)/2.0;
	}

template class SignedDistanceField<float>;

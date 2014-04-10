#include "VolUtil.h"

namespace utility{

template void divideByPlanes(Volume<int> *vol, const std::vector<Eigen::Vector4d> &planes);
template void divideByPlanes(Volume<float> *vol, const std::vector<Eigen::Vector4d> &planes);
template void divideByPlanes(Volume<double> *vol, const std::vector<Eigen::Vector4d> &planes);	

template <class T> void divideByPlanes(Volume<T> *vol, const std::vector<Eigen::Vector4d> &planes){
	Eigen::Vector3d p0, p1, p;
	double d0, d1;
	Eigen::Vector3f pitch = vol->getPitch();
	Eigen::Vector3f minC = vol->getMinCorner();
	Eigen::Vector3i grid_size = vol->getGridSize();	

	int num_of_planes = planes.size(); 

	// augment planes //
	std::vector<Eigen::Vector4d> augment_planes = planes;
	for(int i=0; i<num_of_planes; i++){
		Eigen::Vector4d plane = planes[i];
		augment_planes.push_back(Eigen::Vector4d(-plane[0], -plane[1], -plane[2], plane[3]));
	}

	for(int l=0; l<(int)augment_planes.size(); l++){
		int l2 = (l+1)%augment_planes.size();
    p0 = Eigen::Vector3d(augment_planes[l][0], augment_planes[l][1], augment_planes[l][2]);
    p1 = Eigen::Vector3d(augment_planes[l2][0], augment_planes[l2][1], augment_planes[l2][2]);
		d0 = augment_planes[l][3];
		d1 = augment_planes[l2][3];
		for(int k=0; k<grid_size[2]; k++){
			for(int j=0; j<grid_size[1]; j++){
				for(int i=0; i<grid_size[0]; i++){
					p[0] = (i+0.5)*pitch[0]+minC[0];
					p[1] = (j+0.5)*pitch[1]+minC[1];
					p[2] = (k+0.5)*pitch[2]+minC[2];
					double v0 = p0.dot(p)+d0;
					double v1 = p1.dot(p)+d1;
					if(v0>0 && v1<0){
						vol->setValue(i, j, k, (T)l);
					}
				}
			}
		}
	}

/*
	for(int l=0; l<num_of_planes; l++){
		int l2 = (l+1)%num_of_planes;
		p0 = Eigen::Vector3d(planes[l][0], planes[l][1], planes[l][2]);
		p1 = Eigen::Vector3d(planes[l2][0], planes[l2][1], planes[l2][2]);
		d0 = planes[l][3];
		d1 = planes[l2][3];
		for(int k=0; k<grid_size[2]; k++){
			for(int j=0; j<grid_size[1]; j++){
				for(int i=0; i<grid_size[0]; i++){
					p[0] = (i+0.5)*pitch[0]+minC[0];
					p[1] = (j+0.5)*pitch[1]+minC[1];
					p[2] = (k+0.5)*pitch[2]+minC[2];
					double v0 = p0.dot(p)+d0;
					double v1 = p1.dot(p)+d1;
					if(v0>0 && v1<0){
						vol->setValue(i, j, k, (T)l);
					}
					else if(v0<0 && v1>0){
						vol->setValue(i, j, k, (T)l+num_of_planes);
					}
				}
			}
		}
	}
	*/
}

}

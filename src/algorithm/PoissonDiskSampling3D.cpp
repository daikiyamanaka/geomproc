#include "PoissonDiskSampling3D.h"

#include <iostream>
#include <algorithm>
#include <ctime>
#include <deque>

namespace sampling{

PoissonDiskSampling3D::PoissonDiskSampling3D(int size_x, int size_y, int size_z, double pitch):size_x_(size_x), size_y_(size_y), size_z_(size_z), pitch_(pitch)
{
	nodes_.resize(size_x_*size_y_*size_z_);
	for(int k=0; k<size_z_; k++){
		for(int j=0; j<size_y_; j++){
			for(int i=0; i<size_x_; i++){		
				nodes_[i+j*size_x_+k*size_x_*size_y_] = Node(i, j, k, 1.0);
			}
		}
	}
	srand(time(0));
	func_ = NULL;
	f_ = Converter();
	origin_ = Eigen::Vector3d::Zero();
}

PoissonDiskSampling3D::~PoissonDiskSampling3D(){

}

void PoissonDiskSampling3D::sample(std::vector<Eigen::Vector3d> &points){
	assert(func_ != NULL);
	points.resize(0);
	std::vector<Node>::iterator n_it;	
	std::vector<int> neighbors;	

	std::deque<Eigen::Vector3d> active_list;
	Eigen::Vector3d init_point = genRandomPoint();

	const double initial_density_value = 0.05;
	
	// create initial_point whose value is positive
	while(true){
		/*
		double R = f_((*func_)(init_point[0]+origin_[0], init_point[1]+origin_[1], init_point[2]+origin_[2]));
		std::cout << "R: " << R << std::endl;
		if(R < 0){
			init_point = genRandomPoint();
		}		
		else{
			break;
		}
		*/
		double df_val = (*func_)(init_point[0]+origin_[0], init_point[1]+origin_[1], init_point[2]+origin_[2]);
		if(df_val < initial_density_value){
			init_point = genRandomPoint();
		}		
		else{
			break;
		}		
	}
	active_list.push_back(init_point);
	insertPoint(nodes_, points, init_point);

	// sampling until active_list become empty //
	while(active_list.size() != 0){
		Eigen::Vector3d point = active_list[0];
		bool is_valid = false;
		int pid_x = point[0]/pitch_;
		int pid_y = point[1]/pitch_;
		int pid_z = point[2]/pitch_;

		int n_id = pid_x+pid_y*size_x_+pid_z*size_x_*size_y_;
		n_it = nodes_.begin()+n_id;
		double r = f_(n_it->val);
		if(r < 0){
			active_list.pop_front();
			continue;
		}

		// generate K points in r~2r region	//
		for(int i=0; i<K; i++){
			Eigen::Vector3d p = genRandomPoint(point, r, r*2);
			double R = f_((*func_)(p[0]+origin_[0], p[1]+origin_[1], p[2]+origin_[2]));
			if(R < 0){
				continue;
			}
			if(!existNeighbors(p, R, points)){
				active_list.push_back(p);
				insertPoint(nodes_, points, p);
				is_valid = true;
			}
		}		
		if(!is_valid){		
			active_list.pop_front();
		}
	}

	for(int i=0; i<(int)points.size(); i++){
		points[i] = points[i] + origin_;
	}
}

void PoissonDiskSampling3D::insertPoint(std::vector<Node> &nodes, std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &point){
	int pid_x = point[0]/pitch_;
	int pid_y = point[1]/pitch_;
	int pid_z = point[2]/pitch_;
	int n_id = pid_x+pid_y*size_x_+pid_z*size_x_*size_y_;
	nodes.at(n_id).visited = true;
	points.push_back(point);
	nodes.at(n_id).p_id.push_back(points.size()-1);
}

void PoissonDiskSampling3D::setDensityFunc(DensityFunction *func){
	func_ = func;
	for(std::vector<Node>::iterator it = nodes_.begin(); it != nodes_.end(); ++it){
		Eigen::Vector3d p((it->x_id+0.5)*pitch_, (it->y_id+0.5)*pitch_, (it->z_id+0.5)*pitch_);
		it->val = (*func_)(p[0]+origin_[0], p[1]+origin_[1], p[2]+origin_[2]);
	}
}

void PoissonDiskSampling3D::setConverter(boost::function<double(double)> f){	
	f_ = f;
}

void PoissonDiskSampling3D::test(){
	std::cout << "test" << std::endl;
	Eigen::Vector3d p;
	p[0] = size_x_/2; p[1] = size_y_/2, p[2] = size_z_/2;
	double r = 5;
	double r2 = 10;
	for(int i=0; i<100; i++){
		Eigen::Vector3d point = genRandomPoint(p, r, r2);
		std::cout << (point-p).eval().norm() << std::endl;
	}
}

void PoissonDiskSampling3D::setOrigin(Eigen::Vector3d origin){
	origin_ = origin;
}

std::vector<int> PoissonDiskSampling3D::calcNeighborIndex(Node &node, double r){
	std::vector<int> neighbors;
	double dx = static_cast<double>(node.x_id)*pitch_;
	double dy = static_cast<double>(node.y_id)*pitch_;
	double dz = static_cast<double>(node.z_id)*pitch_;


	int min_x = (dx-r)/pitch_;
	int max_x = (dx+r)/pitch_;
	int min_y = (dy-r)/pitch_;
	int max_y = (dy+r)/pitch_;
	int min_z = (dz-r)/pitch_;
	int max_z = (dz+r)/pitch_;	

	for(int k=min_z; k<=max_z; k++){
		for(int j=min_y; j<=max_y; j++){
			for(int i=min_x; i<=max_x; i++){
				if(i<0 || i>=size_x_ || j<0 || j>=size_y_ || k<0 || k>=size_z_){
					continue;
				}
				neighbors.push_back(i+j*size_x_+k*size_x_*size_y_);
			}
		}
	}
	return neighbors;
}

Eigen::Vector3d PoissonDiskSampling3D::genRandomPoint(){
	double x = ((double)rand()+1.0)/((double)RAND_MAX+2.0)*(double)(size_x_-1)*pitch_;
	double y = ((double)rand()+1.0)/((double)RAND_MAX+2.0)*(double)(size_y_-1)*pitch_;	
	double z = ((double)rand()+1.0)/((double)RAND_MAX+2.0)*(double)(size_z_-1)*pitch_;		
	return Eigen::Vector3d(x, y, z);
}

Eigen::Vector3d PoissonDiskSampling3D::genRandomPoint(const Eigen::Vector3d &point, double lower_r, double upper_r){

	Eigen::Vector3d p;
	
	double sq_lr = lower_r*lower_r;
	double sq_ur = upper_r*upper_r;	

	while(1){
		double x = ((double)rand()+1.0)/((double)RAND_MAX+2.0)*2*upper_r - upper_r;
		double y = ((double)rand()+1.0)/((double)RAND_MAX+2.0)*2*upper_r - upper_r;
		double z = ((double)rand()+1.0)/((double)RAND_MAX+2.0)*2*upper_r - upper_r;		
		double sq_r = x*x+y*y+z*z;
		if(sq_r < sq_lr || sq_r > sq_ur){
			continue;
		}
		x += point[0]; y+= point[1]; z+= point[2];
		if(x < 0 || x > size_x_*pitch_){
			continue;
		}
		if(y < 0 || y > size_y_*pitch_){
			continue;
		}		
		if(z < 0 || z > size_z_*pitch_){
			continue;
		}
		p = Eigen::Vector3d(x, y, z);
		break;
	}

	return p;
}

bool PoissonDiskSampling3D::existNeighbors(Eigen::Vector3d &point, double r, std::vector<Eigen::Vector3d> &points){
	bool exist = false;

	double r2 = r*r;

	std::vector<int> neighbors;
	double dx = point[0];
	double dy = point[1];
	double dz = point[2];

	int min_x = (dx-r)/pitch_;
	int max_x = (dx+r)/pitch_;
	int min_y = (dy-r)/pitch_;
	int max_y = (dy+r)/pitch_;
	int min_z = (dz-r)/pitch_;
	int max_z = (dz+r)/pitch_;	

	for(int k=min_z; k<=max_z; k++){
		for(int j=min_y; j<=max_y; j++){
			for(int i=min_x; i<=max_x; i++){
				if(i<0 || i>=size_x_ || j<0 || j>=size_y_ || k<0 || k>=size_z_){
					continue;
				}
				neighbors.push_back(i+j*size_x_+k*size_x_*size_y_);
			}
		}
	}
	std::vector<int>::iterator id_it = neighbors.begin();
	for(; id_it!= neighbors.end(); ++id_it){
		Node n = *(nodes_.begin()+ *id_it);
		if(n.visited == true){
		for(int i=0; i<(int)n.p_id.size(); i++){
				double x = points[n.p_id[i]][0]-point[0];
				double y = points[n.p_id[i]][1]-point[1];
				double z = points[n.p_id[i]][2]-point[2];
				double dist2 = x*x+y*y+z*z;
				if(dist2 < r2){
					exist = true;
					break;				
				}
			}
		}		
	}
	return exist;
}

}

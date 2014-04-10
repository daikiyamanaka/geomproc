#include "DensityFunction.h"

DiscreteDensityFunction::DiscreteDensityFunction(Eigen::Vector3d pitch, Eigen::Vector3i grid_size, Eigen::Vector3d minC){
	pitch_ = pitch;
	grid_size_ = grid_size;
	minC_ = minC;

	val_ = new Volume<float>(pitch_.cast<float>(), grid_size_, minC_.cast<float>());
	label_ = new Volume<int>(pitch_.cast<float>(), grid_size_, minC_.cast<float>());	
	val_->fill(0);
	label_->fill(-1);
}

DiscreteDensityFunction::~DiscreteDensityFunction(){
	delete(val_);
	delete(label_);
};

void DiscreteDensityFunction::save(std::string filename){
	val_->save(filename);
}

void　DiscreteDensityFunction::saveInside(std::string filename){
	std::ofstream ofs(filename.c_str(), std::ios::out| std::ios::binary);
	float *buff = new float[grid_size_[0]];	
	for(int k=0; k<grid_size_[2]; k++){
		for(int j=0; j<grid_size_[1]; j++){
			for(int i=0; i<grid_size_[0]; i++){
				//if(label_->getValue(i, j, k) < 0){
        double x = (i+0.5)*pitch_[0]+minC_[0];
        double y = (j+0.5)*pitch_[1]+minC_[1];
        double z = (k+0.5)*pitch_[2]+minC_[2];				
				if(!isValid(x, y, z)){
					buff[i] = -1;
				}
				else{
					buff[i] = val_->getValue(x, y, z);
				}
			}
			ofs.write((char *)buff, grid_size_[0]*sizeof(float));
		}
	}
	ofs.close();
	delete(buff);	
}

double DiscreteDensityFunction::operator()(const int i, const int j, const int k){
	return val_->getValue(i, j, k);
}

double DiscreteDensityFunction::operator()(const double x, const double y, const double z){
	return val_->getValue(x, y, z);
}

double DiscreteDensityFunction::operator()(const Eigen::Vector3d &p){
	return val_->getValue(p[0], p[1], p[2]);
}

void DiscreteDensityFunction::setFuncVal(const int ix, const int iy, const int iz, const double val){
	label_->setValue(ix, iy, iz, 1);
	val_->setValue(ix, iy, iz, val);
}

void DiscreteDensityFunction::setInterpolant(Volume<float>::INTERPOLANT interpolant){
	val_->setInterpolant(interpolant);
	//label_->setInterpolant(interpolant);	
}

void DiscreteDensityFunction::fill(float val){
	val_->fill(val);
}

/*
Eigen::Vector3d DiscreteDensityFunction::calcPosition(const int ix, const int iy, const int iz){
	return Eigen::Vector3d(ix*pitch_[0]+minC_[0], iy*pitch_[1]+minC_[1], ix*pitch_[2]+minC_[2]);
}
*/

Eigen::Vector3d DiscreteDensityFunction::getPitch(){
	return pitch_;
}

Eigen::Vector3i DiscreteDensityFunction::getGridSize(){
	return grid_size_;
}

Eigen::Vector3d DiscreteDensityFunction::getMinCorner(){
	return minC_;
}

bool DiscreteDensityFunction::include(const Eigen::Vector3d &p){
	return include(p[0], p[1], p[2]);
}

bool DiscreteDensityFunction::include(const double x, const double y, const double z){

}

bool DiscreteDensityFunction::isValid(const int ix, const int iy, const int iz){
	if(label_->getValue(ix, iy, iz) < 0){
		return false;
	}
	return true;
}

bool DiscreteDensityFunction::isValid(const double x, const double y, const double z){
	if(label_->getValue(x, y, z) < 0){
		return false;
	}
	return true;
}

bool DiscreteDensityFunction::isValid(const Eigen::Vector3d &p){
	return isValid(p[0], p[1], p[2]);
}

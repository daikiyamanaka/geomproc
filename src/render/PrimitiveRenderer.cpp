#include <GLUT/glut.h>
#include <random>

#include "PrimitiveRenderer.h"

std::vector<Eigen::Vector3d> PrimitiveRenderer::unit_circle_;

PrimitiveRenderer::PrimitiveRenderer(){
	// generate unit circle points //
	if(unit_circle_.empty()){
		init();
	}

	cut_plane_normal_ = Eigen::Vector3d(1, 0, 0);
	d_ = 0;

	// generate random colors //
	genRandomColors(num_of_random_colors_);

}

PrimitiveRenderer::~PrimitiveRenderer(){

}

void PrimitiveRenderer::init(){
	double d_theta = 2*M_PI/(double)circle_division_num;
	for(int i=0; i<circle_division_num; i++){
		double theta = d_theta*((double)i+0.5);
		double x = cos(theta);
		double y = sin(theta);
		unit_circle_.push_back(Eigen::Vector3d(x, y, 0));
	}		
}

void PrimitiveRenderer::drawPoint(const Eigen::Vector3d &p){
	glBegin(GL_POINTS);
	glVertex3d(p[0], p[1], p[2]);
	glEnd();	
}

void PrimitiveRenderer::drawSegment(const Eigen::Vector3d &s, const Eigen::Vector3d &t){
	glBegin(GL_LINES);
	glVertex3d(s[0], s[1], s[2]);
	glVertex3d(t[0], t[1], t[2]);
	glEnd();
}

void PrimitiveRenderer::drawPlane(const Eigen::Vector4d &plane_params, const Eigen::Vector3d &up_vector, const Eigen::Vector3d &center, const double &size){
	Eigen::Vector3d p0, p1, p2, p3;

	Eigen::Vector3d u1, u2, n;
	u2 = up_vector;
	n = Eigen::Vector3d(plane_params[0], plane_params[1], plane_params[2]);
	u1 = up_vector.cross(n);
	u1.normalize();
	u2.normalize();		
	double size2 = size/2;

	p0 = center + size2*u1 + size2*u2;
	p1 = center - size2*u1 + size2*u2;
	p2 = center - size2*u1 - size2*u2;
	p3 = center + size2*u1 - size2*u2;

	glBegin(GL_QUADS);
	glVertex3f(p0[0], p0[1], p0[2]);
	glVertex3f(p1[0], p1[1], p1[2]);
	glVertex3f(p2[0], p2[1], p2[2]);
	glVertex3f(p3[0], p3[1], p3[2]);
	glEnd();	
}

void PrimitiveRenderer::drawTriangle(const Eigen::Vector3d &a, const Eigen::Vector3d &b, const Eigen::Vector3d &c){
	glBegin(GL_TRIANGLES);
	glVertex3d(a[0], a[1], a[2]);
	glVertex3d(b[0], b[1], b[2]);
	glVertex3d(c[0], c[1], c[2]);		
	glEnd();
}

void PrimitiveRenderer::drawTriangle(const Eigen::Vector3d &a, const Eigen::Vector3d &b, const Eigen::Vector3d &c, const Eigen::Vector3d &n){
	glBegin(GL_TRIANGLES);
	::glNormal3d(n[0], n[1], n[2]);
	glVertex3d(a[0], a[1], a[2]);
	glVertex3d(b[0], b[1], b[2]);
	glVertex3d(c[0], c[1], c[2]);		
	glEnd();
}

void PrimitiveRenderer::drawCircle(const double scale, const Eigen::Vector3d &center, const double red, const double green, const double blue)
{
	glColor3d(red, green, blue);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glTranslated(center[0], center[1], 0.0);
	glBegin(GL_POLYGON);
	for(int i=0; i<(int)unit_circle_.size(); i++){
		glVertex3d(unit_circle_[i][0]*scale, unit_circle_[i][1]*scale, 0);
	}
	glEnd();
  glTranslated(-center[0], -center[1], 0);
}

void PrimitiveRenderer::setCuttingPlane(const Eigen::Vector3d normal, const double d){
	cut_plane_normal_ = normal;
	d_ = d;
}

void PrimitiveRenderer::getCuttingPlane(Eigen::Vector3d &normal, double &d){
	normal = cut_plane_normal_;
	d = d_;
}

bool PrimitiveRenderer::intersected(const std::vector<Eigen::Vector3d> &polygon){
	int n_poly = polygon.size();
	for(int i=0; i<n_poly; i++){
		if(intersected(polygon[i], polygon[(i+1)%n_poly])){
			return true;
		}
	}
	return false;
}

bool PrimitiveRenderer::intersected(const Eigen::Vector3d &a, const Eigen::Vector3d &b){
	double val0 = a.dot(cut_plane_normal_)+d_;
	double val1 = b.dot(cut_plane_normal_)+d_;	
	if(val0*val1 < 0){
		return true;
	}
	return false;
}

int PrimitiveRenderer::calcSide(const std::vector<Eigen::Vector3d> &polygon){
	int n_poly = polygon.size();

	int out_count = 0;
	int in_count = 0;
	for(int i=0; i<n_poly; i++){
		int side = calcSide(polygon[i], polygon[(i+1)%n_poly]);
		if(side == 0){
			return 0;
		}
		else if(side == 1){
			out_count++;
		}
		else{
			in_count++;
		}
	}
	//return false;
	if(out_count == 0){
		return -1;
	}
	return 1;
}

int PrimitiveRenderer::calcSide(const Eigen::Vector3d &a, const Eigen::Vector3d &b){
	double val0 = a.dot(cut_plane_normal_)+d_;
	double val1 = b.dot(cut_plane_normal_)+d_;	
	if(val0*val1 < 0){
		return 0;
	}
	else if(val0 > 0){
		return 1;
	}
	return -1;
}

void PrimitiveRenderer::genRandomColors(int num_of_colors){
	std::mt19937 engine;
	std::uniform_real_distribution<double> distribution(0.0, 1.0);
	random_colors_.resize(num_of_colors);
	for(int i=0; i<num_of_colors; i++){
		double r = distribution(engine);
		double g = distribution(engine);
		double b = distribution(engine);
		random_colors_[i] = Eigen::Vector3d(r, g, b);		
	}	
}


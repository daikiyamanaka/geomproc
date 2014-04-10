#include "Utility.h"

namespace utility{

void getDivisionPlanes(const Eigen::Vector3d &v, const Eigen::Vector3d &p, const int num_of_segments, std::vector<Eigen::Vector4d> &planes){
	planes.resize(num_of_segments);

  Eigen::Vector3d w, u;
  w = v;
  w.normalize();

  //calculate basis vector on the plane //
  Eigen::Vector3d x = Eigen::Vector3d(1, 0, 0);
  Eigen::Vector3d z = Eigen::Vector3d(0, 0, 1);

  Eigen::Matrix3d A, B;
  Eigen::Vector3d axis = w.cross(z);
  rodriguesRotation(A, axis, acos(w.dot(z)));
  u = A*x;

  double offset_theta = M_PI/6;

  double d_theta = M_PI/(double)num_of_segments;
  Eigen::Vector3d u2;
  for(int i=0; i<num_of_segments; i++){
    double theta = d_theta*(double)i + offset_theta;
    rodriguesRotation(B, w, theta);
    u2 = B*u;    
    planes[i] = Eigen::Vector4d(u2[0], u2[1], u2[2], -u2.dot(p));    
  }	
}

void divideByPlanes(const std::vector<Eigen::Vector3d> &points, std::vector<int> &point_segment_ids, const std::vector<Eigen::Vector4d> &planes)
{
  Eigen::Vector3d p0, p1, p;
  double d0, d1;

  int num_of_points = points.size();
  int num_of_planes = planes.size();

  point_segment_ids.resize(num_of_points);

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
    for(int i=0; i<num_of_points; i++){
      p[0] = points[i][0];
      p[1] = points[i][1];
      p[2] = points[i][2];
      double v0 = p0.dot(p)+d0;
      double v1 = p1.dot(p)+d1;
      if(v0 > 0 && v1 < 0){
        point_segment_ids[i] = l;
      }  
    }
  }

/*
  for(int l=0; l<num_of_planes-1; l++){
    p0 = Eigen::Vector3d(planes[l][0], planes[l][1], planes[l][2]);
    p1 = Eigen::Vector3d(planes[l+1][0], planes[l+1][1], planes[l+1][2]);
    d0 = planes[l][3];
    d1 = planes[l+1][3];
    for(int i=0; i<num_of_points; i++){
      p[0] = points[i][0];
      p[1] = points[i][1];
      p[2] = points[i][2];
      double v0 = p0.dot(p)+d0;
      double v1 = p1.dot(p)+d1;
      if(v0 > 0 && v1 < 0){
        point_segment_ids[i] = l;
      }
      else if(v0<0 && v1 > 0){
        point_segment_ids[i] = l+num_of_planes;
      }     
    }
  }
  */
}

void rodriguesRotation(Eigen::Matrix3d &mat, const Eigen::Vector3d &axis, const float &angle){
	//mat.resize(3, 3);
	mat(0, 0) = cos(angle) + axis[0]*axis[0]*(1-cos(angle));
	mat(0, 1) = axis[0]*axis[1]*(1-cos(angle)) - axis[2]*sin(angle);
	mat(0, 2) = axis[1]*sin(angle) + axis[0]*axis[2]*(1-cos(angle));

	mat(1, 0) = axis[2]*sin(angle) + axis[0]*axis[1]*(1-cos(angle));
	mat(1, 1) = cos(angle) + axis[1]*axis[1]*(1-cos(angle));
	mat(1, 2) = -1*axis[0]*sin(angle) + axis[1]*axis[2]*(1-cos(angle));

	mat(2, 0) = -1*axis[1]*sin(angle) + axis[0]*axis[2]*(1-cos(angle));
	mat(2, 1) = axis[0]*sin(angle) + axis[1]*axis[2]*(1-cos(angle));
	mat(2, 2) = cos(angle) + axis[2]*axis[2]*(1-cos(angle));
}

void pts2BBox(std::vector<Eigen::Vector3d> &points, Eigen::Vector3d &min, Eigen::Vector3d &max, double ratio){
	min = max = points[0];
	for(int i=0; i<(int)points.size(); i++){
		for(int j=0; j<3; j++){
			double val = points[i][j];
			if(val < min[j]){
				min[j] = val;
			}
			if(val > max[j]){
				max[j] = val;
			}
		}
	}
	if(fabs(ratio-1.0)<0.01){
		return;
	}

	Eigen::Vector3d center = (min+max)/2.0;
	Eigen::Vector3d size = max-min;
	size *= ratio;
	Eigen::Vector3d minC(center[0]-size[0]/2.0, center[1]-size[1]/2.0, center[2]-size[2]/2.0);
	Eigen::Vector3d maxC(center[0]+size[0]/2.0, center[1]+size[1]/2.0, center[2]+size[2]/2.0);

	min = minC;
	max = maxC;
}

double distPts2Seg(const Eigen::Vector3d &p, const Eigen::Vector3d &e0, const Eigen::Vector3d &e1){
	double t = (e1-e0).dot(p-e0);
	if(t<0){
    return (e0-p).norm();
  } 
  double d = (e1-e0).squaredNorm();
  t = t/d;
  if(t > 1.0){
    return (e1-p).norm();
  }
  return (p-e1*t+e0*(t-1)).norm();
}

double distPts2Line(const Eigen::Vector3d &p, const Eigen::Vector3d &x, const Eigen::Vector3d &v)
{
  double t = (p-x).dot(v)/v.squaredNorm();
  return (p-x-t*v).norm();
}

bool intersect(const Eigen::Vector3d& center, 
               const double &r,
               const Eigen::Vector3d& _p0, 
               const Eigen::Vector3d& _p1,
               std::vector<Eigen::Vector3d> &intersections,
               double intersection_threshold
               )
{

	intersections.clear();

	Eigen::Vector3d p0 = _p0 - center;
	Eigen::Vector3d p1 = _p1 - center;	

	double dx = p1[0]-p0[0];
	double dy = p1[1]-p0[1];
	double squared_dr = dx*dx+dy*dy;
	double D = p0[0]*p1[1]-p1[0]*p0[1];
	double delta = r*r*squared_dr-D*D;

	if(delta < 0){
		return false;
	}

	double sign = dy < 0 ? -1 : 1;
	double squared_r = r*r;
	double squared_D = D*D;
	double x = (D*dy+sign*dx*sqrt(squared_dr*squared_r-squared_D))/squared_dr;
	double y = (-D*dx+fabs(dy)*sqrt(squared_dr*squared_r-squared_D))/squared_dr;
	intersections.push_back(Eigen::Vector3d(x+center[0], y+center[1], 0));

	if((fabs(delta) > intersection_threshold)){
		double x1 = (D*dy-sign*dx*sqrt(squared_dr*squared_r-squared_D))/squared_dr;
		double y1 = (-D*dx-fabs(dy)*sqrt(squared_dr*squared_r-squared_D))/squared_dr;
		intersections.push_back(Eigen::Vector3d(x1+center[0], y1+center[1], 0));
	}

	return true;
}

Eigen::Vector3d circumcenter(Eigen::Vector3d &a, Eigen::Vector3d &b, Eigen::Vector3d &c){
    double xba, yba, xca, yca;
    double balength, calength;
    double denominator;
    double xcirca, ycirca;

    Eigen::Vector3d center;

    /* Use coordinates relative to point `a' of the triangle. */
    xba = b[0] - a[0];
    yba = b[1] - a[1];
    xca = c[0] - a[0];
    yca = c[1] - a[1];
    /* Squares of lengths of the edges incident to `a'. */
    balength = xba * xba + yba * yba;
    calength = xca * xca + yca * yca;

    /* Calculate the denominator of the formulae. */
  //#ifdef EXACT
    /* Use orient2d() from http://www.cs.cmu.edu/~quake/robust.html     */
    /*   to ensure a correctly signed (and reasonably accurate) result, */
    /*   avoiding any possibility of division by zero.                  */
    //denominator = 0.5 / orient2d(b, c, a);
  //#else
    /* Take your chances with floating-point roundoff. */
    denominator = 0.5 / (xba * yca - yba * xca);
  //#endif

    /* Calculate offset (from `a') of circumcenter. */
    xcirca = (yca * balength - yba * calength) * denominator;
    ycirca = (xba * calength - xca * balength) * denominator;
    center[0] = xcirca + a[0];
    center[1] = ycirca + a[1];
    center[2] = a[2];

    return center;

}

Eigen::Vector3d centroid(Eigen::Vector3d &a, Eigen::Vector3d &b, Eigen::Vector3d &c){
	return (a+b+c)/3;
}
 
}

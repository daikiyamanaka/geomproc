#include <iostream>
#include <algorithm>
#include "StrutAnalysis.h"

namespace FEM
{

int tet_face[4][3] = {{1, 2, 3}, {0, 3, 2}, {0, 1, 3}, {0, 2, 1}};

void convTet2Struts(const std::vector<Eigen::Vector3d> &tet_points,
                    const std::vector<std::vector<int> > &tets,
                    const std::vector<std::vector<int> > &n_tets,                   
                    std::vector<Eigen::Vector3d> &points,
                    std::vector<std::pair<int, int> > &edges,
                    std::vector<bool> &fixed_marker)
{
  points.clear();
  edges.clear();
  fixed_marker.clear();

  // loop for tetra //
  for(int i=0; i<(int)tets.size(); i++){
    int prev_index, next_index;
    // loop for face // 
    for(int j=0; j<4; j++){
      for(int k=0; k<3; k++){
        prev_index = tets[i][tet_face[j][k]];
        next_index = tets[i][tet_face[j][(k+1)%3]];
        bool visited_edge = false;
        for(int l=0; l<edges.size(); l++){
          if(edges[l].first == prev_index && edges[l].second == next_index){
            visited_edge = true;
            break;
          }
          if(edges[l].second == prev_index && edges[l].first == next_index){
            visited_edge = true;
            break;
          }          
        }
        if(!visited_edge){
          edges.push_back(std::pair<int, int>(prev_index, next_index));
        }
      }
    }
  }
  points.resize(tet_points.size());
  std::copy(tet_points.begin(), tet_points.end(), points.begin());

  // fix boundary points //
  fixed_marker.resize(points.size());
  std::fill(fixed_marker.begin(), fixed_marker.end(), false);
  for(int i=0; i<(int)tets.size(); i++){
    for(int j=0; j<4; j++){
      if(n_tets[i][j] < 0){ // boundary
        fixed_marker[tets[i][(j+1)%4]] = true;
        fixed_marker[tets[i][(j+2)%4]] = true;
        fixed_marker[tets[i][(j+3)%4]] = true;                
      }
    }
  }
}

void convTri2Struts(const std::vector<Eigen::Vector3d> &tri_points,
                    const std::vector<std::vector<int> > &tris,
                    const std::vector<std::vector<int> > &n_tris,                   
                    std::vector<Eigen::Vector3d> &points,
                    std::vector<std::pair<int, int> > &edges,
                    std::vector<bool> &fixed_marker)
{
	points.clear();
	edges.clear();
	fixed_marker.clear();

	// loop for tetra //
	for(int i=0; i<(int)tris.size(); i++){
    	int prev_index, next_index;
		// loop for face // 
		for(int j=0; j<3; j++){
			prev_index = tris[i][j];
	        next_index = tris[i][(j+1)%3];
	        bool visited_edge = false;
    		for(int l=0; l<edges.size(); l++){
				if(edges[l].first == prev_index && edges[l].second == next_index){
		            visited_edge = true;
        			break;
				}
				if(edges[l].second == prev_index && edges[l].first == next_index){
					visited_edge = true;
					break;
				}          
			}
			if(!visited_edge){
				edges.push_back(std::pair<int, int>(prev_index, next_index));
			}
		}
	}
	points.resize(tri_points.size());
	std::copy(tri_points.begin(), tri_points.end(), points.begin());

	// fix boundary points //
	std::cout << "creating boundary marker..." << std::endl;    
	fixed_marker.resize(points.size());
	std::fill(fixed_marker.begin(), fixed_marker.end(), false);
	for(int i=0; i<(int)tris.size(); i++){
		for(int j=0; j<3; j++){
			if(n_tris[i][j] < 0){ // boundary
				fixed_marker[tris[i][(j+1)%3]] = true;
				fixed_marker[tris[i][(j+2)%3]] = true;
				//fixed_marker[tris[i][2] = true;                
			}
		}
	}
}

void StrutAnalysis::getInternalForce(const std::vector<Eigen::Vector3d> &nodes,
  				                     const std::vector<std::pair<int, int> > &edges,	
                 				     const std::vector<double> &radiuses,                      
                  					 std::vector<double> &internal_forces,
			                         std::vector<Eigen::Vector3d> &internal_forces_direction)
{
	int num_of_nodes = nodes.size()*3;
	internal_forces.resize(num_of_nodes);
	internal_forces_direction.resize(num_of_nodes);
	std::fill(internal_forces_direction.begin(), internal_forces_direction.end(), Eigen::Vector3d::Zero());

	// loop for edges //
	for(int i=0; i<(int)edges.size(); i++){

		int p_index = edges[i].first;
		int q_index = edges[i].second;
		Eigen::Vector3d p = nodes[p_index];
		Eigen::Vector3d q = nodes[q_index];

		double area = circleArea(radiuses[i]);
		double mass = (p-q).norm()*area*0.5*rho_;

		// add force by self weight //
		for(int j=0; j<3; j++){
			internal_forces_direction[p_index][j] += mass*g_*gravity_direction_[j];
			internal_forces_direction[q_index][j] += mass*g_*gravity_direction_[j];
		}
	}

	for(int i=0; i<num_of_nodes; i++){
		internal_forces[i] = internal_forces_direction[i].norm();		
		internal_forces_direction[i].normalize();
	}

}

StrutAnalysis::StrutAnalysis():g_(9.8),young_modulus_(0.1*10e10), rho_(10e3){
	cross_section_type_ = SQUARE;
	/*
	// tree //
	young_modulus_ = 10e10; 
	rho_ = 10e3;
	*/

	// rubber //
	//young_modulus_ = 0.1*10e10; 
	//rho_ = 10e3;

	gravity_direction_ = Eigen::Vector3d(0, 0, -1);
}

StrutAnalysis::~StrutAnalysis(){

}

void StrutAnalysis::setYoungModulus(double val){
	young_modulus_ = val;
}
/*
void StrutAnalysis::setCrossSection(){

}
*/
/*
void StrutAnalysis::setRadius(double radius){

}
*/
/*
void StrutAnalysis::setDensity(){

}
*/
void StrutAnalysis::setGravityDirection(Eigen::Vector3d direction){
	gravity_direction_ = direction;
}

void StrutAnalysis::constructStiffnessMatrix(const std::vector<Eigen::Vector3d> &nodes, 
	                                         const std::vector<std::pair<int, int> > &edges,	                                           
	                                         const std::vector<double> &radiuses,
                                             Eigen::MatrixXd &stiff_mat)
{

	assert(edges.size() == radiuses.size());

	int num_of_elements = 3*nodes.size();
	stiff_mat = Eigen::MatrixXd::Zero(num_of_elements, num_of_elements);

	int indices[6][2] = {{0, 0}, {0, 1}, {0, 2}, {1, 1}, {1, 2}, {2, 2}};

  // construct stiffness matrix for each edge //
	for(int i=0; i<(int)edges.size(); i++){

		int p_index = edges[i].first;
		int q_index = edges[i].second;		
		Eigen::Vector3d p = nodes[p_index];
		Eigen::Vector3d q = nodes[q_index];

		double x = q[0]-p[0];
		double y = q[1]-p[1];
		double z = q[2]-p[2];

		double c_theta = z/sqrt(x*x+y*y+z*z);
		c_theta = std::isnan(c_theta) ? 0 : c_theta;
		double s_theta = sqrt(x*x+y*y)/sqrt(x*x+y*y+z*z);
		s_theta = std::isnan(s_theta) ? 0 : s_theta;
		double s_phi = y/sqrt(x*x+y*y);
		s_phi = std::isnan(s_phi) ? 0 : s_phi;
		double c_phi = x/sqrt(x*x+y*y);		
		c_phi = std::isnan(c_phi) ? 0 : c_phi;


		double alpha = s_theta*c_phi;
		double beta = s_theta*s_phi;
		double gamma = c_theta;

		std::vector<double> coefficients(6);
		coefficients[0] = alpha*alpha;
		coefficients[1] = alpha*beta;
		coefficients[2] = alpha*gamma;
		coefficients[3] = beta*beta;
		coefficients[4] = beta*gamma;
		coefficients[5] = gamma*gamma;

		double area = circleArea(radiuses[i]);
		double length = (q-p).norm();

		for(int k=0; k<6; k++){
			double coeff = young_modulus_*area*coefficients[k]/length;
			int row = indices[k][0];
			int col = indices[k][1];
			stiff_mat(p_index*3+row, p_index*3+col) += coeff;
			stiff_mat(p_index*3+col, p_index*3+row) += coeff;				
			stiff_mat(p_index*3+row, q_index*3+col) += -coeff;
			stiff_mat(q_index*3+col, p_index*3+row) += -coeff;
			stiff_mat(q_index*3+row, p_index*3+col) += -coeff;
			stiff_mat(p_index*3+col, q_index*3+row) += -coeff;
			stiff_mat(q_index*3+row, q_index*3+col) += coeff;
			stiff_mat(q_index*3+col, q_index*3+row) += coeff;			
		}
	}
}

void StrutAnalysis::constructSelfWeightForce(const std::vector<Eigen::Vector3d> &nodes,
	                            const std::vector<std::pair<int, int> > &edges,
	                            const std::vector<double> &radiuses,
	                            const std::vector<Eigen::Vector3d> &external_forces,
	                            Eigen::VectorXd &F)
{
	int num_of_elements = nodes.size()*3;
	F = Eigen::VectorXd::Zero(num_of_elements);

	// loop for edges //
	for(int i=0; i<(int)edges.size(); i++){

		int p_index = edges[i].first;
		int q_index = edges[i].second;
		Eigen::Vector3d p = nodes[p_index];
		Eigen::Vector3d q = nodes[q_index];

		double area = circleArea(radiuses[i]);
		double mass = (p-q).norm()*area*0.5*rho_;

		// add force by self weight //
		for(int j=0; j<3; j++){
			F(p_index*3+j) += mass*g_*gravity_direction_[j];
			F(q_index*3+j) += mass*g_*gravity_direction_[j];			
		}
	}

	// add external force //
	for(int i = 0; i<(int)external_forces.size(); i++){
		F(i*3+0) += external_forces[i][0];
		F(i*3+1) += external_forces[i][1];
		F(i*3+2) += external_forces[i][2];				
	}
}

 void StrutAnalysis::applyDirichletBoundaryCondition(const Eigen::MatrixXd &A,
                                                     const Eigen::VectorXd &b,
 	                                                   const std::vector<std::pair<int, double> > &conditions,
 	                                                   Eigen::MatrixXd &boundedA,
 	                                                   Eigen::VectorXd &boundedb)
{
	assert(A.cols() == A.rows());	
	assert(A.rows() == b.size());
	assert(conditions.size() > A.cols());

	Eigen::VectorXd _b = b;	


	boundedA = Eigen::MatrixXd::Zero(A.rows(), A.cols()-conditions.size());

	// update b vector according to conditions //
	for(int i=0; i<conditions.size(); i++){
		int col_id = conditions[i].first;
		double C = conditions[i].second;
		_b -= C*A.col(col_id);
	}

	// remove element from vector b //
	boundedb = b;

	// prepair labels //
	std::vector<bool> constrained(A.cols());
	std::fill(constrained.begin(), constrained.end(), false);
	for(int i=0; i<conditions.size(); i++){
		constrained[conditions[i].first] = true;
	}

	// remove columns from matrix A //
	int col_index = 0;
	for(int i=0; i<A.cols(); i++){
		if(constrained[i]){
			continue;
		}
		boundedA.col(col_index) = A.col(i);
		col_index++;
	}	
}

 void StrutAnalysis::fixNodePositions(const Eigen::MatrixXd &A,
                                      const Eigen::VectorXd &b,
                                      const std::vector<std::pair<int, double> > &conditions,
                                      Eigen::MatrixXd &boundedA,
                                      Eigen::VectorXd &boundedb)
{
	assert(A.cols() == A.rows());
	assert(A.rows() == b.size());
	//assert(conditions.size() > A.cols());

	Eigen::VectorXd _b = b;
	boundedA = Eigen::MatrixXd::Zero(A.rows()-conditions.size(), A.cols()-conditions.size());
	boundedb = Eigen::VectorXd::Zero(b.size()-conditions.size());

	// update b vector according to conditions //
	for(int i=0; i<conditions.size(); i++){
		int col_id = conditions[i].first;
		double C = conditions[i].second;
		_b -= C*A.col(col_id);
	}

	// prepair labels //
	std::vector<bool> constrained(A.cols());
	std::fill(constrained.begin(), constrained.end(), false);
	for(int i=0; i<conditions.size(); i++){
		constrained[conditions[i].first] = true;
	}

	// remove columns from matrix A and vector b //	
	int col_index = 0;		
	for(int i=0; i<A.cols(); i++){
		if(constrained[i]){
			continue;
		}
		int row_index = 0;
		for(int j=0; j<A.rows(); j++){		
			if(constrained[j]){
				continue;
			}
			boundedA(row_index, col_index) = A(j, i);
			row_index++;
		}
		boundedb(col_index) = b(i);		
		col_index++;
	}

	/*
	int col_index = 0;	
	for(int i=0; i<A.cols(); i++){
		if(constrained[i]){
			continue;
		}
		boundedA.col(col_index) = A.col(i);
		boundedb(col_index) = b(i);
		col_index++;
	}
	*/
}

void StrutAnalysis::constructFixedBoundaryCondition(const std::vector<Eigen::Vector3d> &nodes,
		                                 const std::vector<std::pair<int, int> > &edges,
	                                   const std::vector<bool> &fixed_marker,		                                 
		                                 Eigen::MatrixXd &A,
		                                 Eigen::VectorXd &b)
{

	assert(fixed_marker.size() == nodes.size());

	int num_of_elements = nodes.size()*3;
	int num_of_fixed_node = std::count(fixed_marker.begin(), fixed_marker.end(), true);
	int num_of_constraints = 3*num_of_fixed_node;

	A = Eigen::MatrixXd::Zero(num_of_constraints, num_of_elements);
	b = Eigen::VectorXd::Zero(num_of_constraints);

	// construct constraint //
	int index = 0;	
	for(int i=0; i<fixed_marker.size(); i++){
		if(!fixed_marker[i]){
			continue;
		}
		A(index*3  , i*3) = 1;
		A(index*3+1, i*3+1) = 1;
		A(index*3+2, i*3+2) = 1;
		index ++;
	}
}

double StrutAnalysis::circleArea(const double r){
	return r*r*M_PI;
}

double StrutAnalysis::squareArea(const double r){
	return r*r*4;
}

}

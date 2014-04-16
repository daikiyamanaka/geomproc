#include <iostream>
#include "StrutAnalysis.h"


namespace FEM
{

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

		std::cout << "s_theta: " << s_theta << std::endl;
		std::cout << "c_theta: " << c_theta << std::endl;		
		std::cout << "s_phi: " << s_phi << std::endl;
		std::cout << "c_phi: " << c_phi << std::endl;		
		std::cout << "alpha: " << alpha << std::endl;
		std::cout << "beta: " << beta << std::endl;
		std::cout << "gamma: " << gamma << std::endl;				

		std::vector<double> coefficients(6);
		coefficients[0] = alpha*alpha;
		coefficients[1] = alpha*beta;
		coefficients[2] = alpha*gamma;
		coefficients[3] = beta*beta;
		coefficients[4] = beta*gamma;
		coefficients[5] = gamma*gamma;

		double area = circleArea(radiuses[i]);
		double length = (q-p).norm();

		// loop for coordinates //
		/*		
		for(int k=0; k<3; k++){
			for(int l=k; l<3; l++){
				std::cout << k*2+l << std::endl;
				double coeff = young_modulus_*area*coefficients[k*2+l]/length;
				stiff_mat(p_index*3+k, p_index*3+l) += coeff;
				stiff_mat(p_index*3+l, p_index*3+k) += coeff;				
				stiff_mat(p_index*3+k, q_index*3+l) += -coeff;
				stiff_mat(q_index*3+l, p_index*3+k) += -coeff;
				stiff_mat(q_index*3+k, p_index*3+l) += -coeff;
				stiff_mat(p_index*3+l, q_index*3+k) += -coeff;
				stiff_mat(q_index*3+k, q_index*3+l) += coeff;
				stiff_mat(q_index*3+l, q_index*3+k) += coeff;
			}
		}		
		*/
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
	assert(conditions.size() > A.cols());

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

/*
void StrutAnalysis::constructBoundedProblem(const std::vector<Eigen::Vector3d> &nodes,
	                           const std::vector<std::pair<int, int> > &edges,
	                           const std::vector<double> &radiuses, 
	                           const std::vector<bool> &fixed_marker,
	                           const std::vector<Eigen::Vector3d> &external_forces,
                             Eigen::MatrixXd &K,
	                           Eigen::VectorXd &F)
{
	assert(edges.size() == radiuses.size());

	int num_of_fixed_nodes = std::count( fixed_marker.begin(), fixed_marker.end(), true);

	int num_of_elements = 3*nodes.size();
	int num_of_constraints = 3*num_of_fixed_nodes;
	K = Eigen::MatrixXd::Zero(num_of_elements+num_of_constraints, num_of_elements);

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

		std::cout << "s_theta: " << s_theta << std::endl;
		std::cout << "c_theta: " << c_theta << std::endl;		
		std::cout << "s_phi: " << s_phi << std::endl;
		std::cout << "c_phi: " << c_phi << std::endl;		
		std::cout << "alpha: " << alpha << std::endl;
		std::cout << "beta: " << beta << std::endl;
		std::cout << "gamma: " << gamma << std::endl;				

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
			K(p_index*3+row, p_index*3+col) += coeff;
			K(p_index*3+col, p_index*3+row) += coeff;				
			K(p_index*3+row, q_index*3+col) += -coeff;
			K(q_index*3+col, p_index*3+row) += -coeff;
			K(q_index*3+row, p_index*3+col) += -coeff;
			K(p_index*3+col, q_index*3+row) += -coeff;
			K(q_index*3+row, q_index*3+col) += coeff;
			K(q_index*3+col, q_index*3+row) += coeff;			
		}
	}

	// construct constraint //
	int index = 0;	
	for(int i=0; i<fixed_marker.size(); i++){
		if(!fixed_marker[i]){
			continue;
		}
		K(num_of_elements+index*3  , i*3) = 10e20;
		K(num_of_elements+index*3+1, i*3+1) = 10e20;
		K(num_of_elements+index*3+2, i*3+2) = 10e20;
		index ++;
	}


	// construct force vector //
	F = Eigen::VectorXd::Zero(num_of_elements+num_of_constraints);

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
*/
double StrutAnalysis::circleArea(const double r){
	return r*r*M_PI;
}

double StrutAnalysis::squareArea(const double r){
	return r*r*4;
}

}

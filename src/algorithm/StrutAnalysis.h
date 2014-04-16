#ifndef STRUTANALYSIS_H__
#define STRUTANALYSIS_H__

#include <vector>
#include <eigen3/Eigen/Core>

namespace FEM
{

/**
 * @brief StrutAnalysis
 * 
 */	
class StrutAnalysis
{

typedef enum CROSS_SECTION_TYPE{
	CIRCLE,
	SQUARE
}CROSS_SECTION_TYPE;


/**
 * @brief gravitational acceleration
 */
	const double g;

public:
	StrutAnalysis();
	~StrutAnalysis();

	//----------
	// Modifier
	//----------
	void setYoungModulus();
	void setCrossSection();
	void setRadius();
	void setDensity();
	void setGravityDirection();

	/**
	 * @brief construct stiffness matrix
	 * construct stiffness matrix from struts
	 */
	void constructStiffnessMatrix(const std::vector<Eigen::Vector3d> &nodes, 
		                            const std::vector<std::pair<int, int> > &edges,
		                            const std::vector<double> &radiuses,
                                Eigen::MatrixXd &stiff_mat);

	/**
	 * @brief construct self weight force vector
	 */
	void constructSelfWeightForce(const std::vector<Eigen::Vector3d> &nodes,
		                            const std::vector<std::pair<int, int> > &edges,
		                            const std::vector<double> &radiuses,
		                            const std::vector<Eigen::Vector3d> &external_force,		                            
		                            Eigen::VectorXd &F);
 
	/**
	 * @brief apply Dirichlet boundary condition
	 * not remove rows
	 */
	 void applyDirichletBoundaryCondition(const Eigen::MatrixXd &A,
	 	                                    const Eigen::VectorXd &b,	 	                                    
	 	                                    const std::vector<std::pair<int, double> > &condition,
	 	                                    Eigen::MatrixXd &boundedA,
	 	                                    Eigen::VectorXd &boundedb);

	/**
	 * @brief fix node positions
	 * remove columns and rows according to fixed node
	 */
  void fixNodePositions(const Eigen::MatrixXd &A,
                        const Eigen::VectorXd &b,	 	                                    
                        const std::vector<std::pair<int, double> > &condition,
                        Eigen::MatrixXd &boundedA,
                        Eigen::VectorXd &boundedb);

	/**
	 * @brif construct fixed node boundary condition
	 */
  void constructFixedBoundaryCondition(const std::vector<Eigen::Vector3d> &nodes,
		                                   const std::vector<std::pair<int, int> > &edges,
		                                   const std::vector<bool> &fixed_marker,
		                                   Eigen::MatrixXd &A,
		                                   Eigen::VectorXd &b);

	/**
	 * @brief construct problem with fixed nodes
	 */
/*
	void constructBoundedProblem(const std::vector<Eigen::Vector3d> &nodes,
		                           const std::vector<std::pair<int, int> > &edges,
		                           const std::vector<double> &radiuses, 
		                           const std::vector<bool> &fixed_marker,
  	                           const std::vector<Eigen::Vector3d> &external_forces,		                           
                               Eigen::MatrixXd &K,
		                           Eigen::VectorXd &F);
*/

private:
	double circleArea(const double r);
	double squareArea(const double r);

	CROSS_SECTION_TYPE cross_section_type_;
	double young_modulus_;
	Eigen::Vector3d gravity_direction_;
	double rho_;

};

}

#endif /* end of include guard: STRUTANALYSIS_H__ */

#ifndef POISSONDISKSAMPLING3D_H__
#define POISSONDISKSAMPLING3D_H__
 
#include <vector>
#include <eigen3/Eigen/Dense>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include "../geom/DensityFunction.h"

namespace sampling{
/**
 * @brief Poisson Disk Sampling in 3D
 * @details this implementation is extension of
 * "Fast Poisson Disk Sampling in Arbitrary Dimensions" by Bridson
 */

 class PoissonDiskSampling3D
 {
 public:
 	/**
 	 * @brief Node
 	 * @details background grid for fast PDS sampling
 	 */
	class Node{
	public:
		Node(){};
		Node(int x, int y, int z, double df):x_id(x), y_id(y), z_id(z), val(df){ 
			visited = false;
		};
		~Node(){};
		Node(const Node& rn){
			x_id = rn.x_id;
			y_id = rn.y_id;
			z_id = rn.z_id;
			val = rn.val;
			visited = rn.visited;
			p_id = rn.p_id;
		};
		int x_id, y_id, z_id;
		std::vector<int> p_id;
		double val;
		bool visited;
	};
 	/**
 	 * @brief Converter
 	 * @details Convert density value to lower bound distance
 	 */
	class Converter{
		public:
			Converter(){};
			~Converter(){};		
			double operator()(double val){
				return val;
			};
		};

/**
 * @brief parameter K
 * @details number of samples per points on active list
 */
	static const int K = 100;

public:
	PoissonDiskSampling3D(int size_x, int size_y, int size_z, double pitch);
	~PoissonDiskSampling3D();

 	/**
 	 * @brief run poisson disk sampling
 	 */
	void sample(std::vector<Eigen::Vector3d> &points);
	void test();

 	/**
 	 * @brief set origin
 	 * @details set minimum corner coordinates
 	 */
	void setOrigin(Eigen::Vector3d origin);

 	/**
 	 * @brief set density function
 	 */	
	void setDensityFunc(DensityFunction *func);

 	/**
 	 * @brief set converter from density to lower bound distance
 	 * @details set minimum corner coordinates
 	 */	
	void setConverter(boost::function<double(double)> f);

private:
 	/**
 	 * @brief generate random point
 	 */	
	Eigen::Vector3d genRandomPoint();		

 	/**
 	 * @brief generate random point within the domain {X | x\in X r < ||p-x||L2 < 2r}
 	 */		
	Eigen::Vector3d genRandomPoint(const Eigen::Vector3d &point, double lower_r, double upper_r);

	std::vector<int> calcNeighborIndex(Node &node, double r);	

	bool existNeighbors(Eigen::Vector3d &point, double r, std::vector<Eigen::Vector3d> &points);
	
 	void insertPoint(std::vector<Node> &nodes, std::vector<Eigen::Vector3d> &points, const Eigen::Vector3d &point);

 	DensityFunction *func_;

	boost::function<double(double)> f_;

	std::vector<Node> nodes_;

	int size_x_, size_y_, size_z_;

	double pitch_;

	Eigen::Vector3d origin_; 

 };

}

 
#endif /* end of include guard: POISSONDISKSAMPLING3D_H__ */

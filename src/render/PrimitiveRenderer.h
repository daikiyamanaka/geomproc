#ifndef PRIMITIVERENDERER_H__
#define PRIMITIVERENDERER_H__

#include <vector>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
 
/**
 * @brief Primitive renderer
 * @details rendering functions with GLUT
 */
class PrimitiveRenderer
{
public:
    static const int circle_division_num = 20;

    static std::vector<Eigen::Vector3d> unit_circle_;

    static const int num_of_random_colors_ = 1000;
public:
	PrimitiveRenderer();
	~PrimitiveRenderer();
    static void init();

    void drawPoint(const Eigen::Vector3d &p);

    void drawSegment(const Eigen::Vector3d &s, 
                   const Eigen::Vector3d &t);

    void drawPlane(const Eigen::Vector4d &plane_params, 
                   const Eigen::Vector3d &up_vector,
                   const Eigen::Vector3d &center, 
                   const double &size = 100);

    void drawPolygon(const std::vector<Eigen::Vector3d> &polygon);

    void drawTriangle(const Eigen::Vector3d &a, const Eigen::Vector3d &b, const Eigen::Vector3d &c);

    void drawTriangle(const Eigen::Vector3d &a, const Eigen::Vector3d &b, const Eigen::Vector3d &c, const Eigen::Vector3d &n);  


    void drawTetrahedron(const Eigen::Vector3d &a,
                         const Eigen::Vector3d &b,
                         const Eigen::Vector3d &c,
                         const Eigen::Vector3d &d);

    void drawCircle(const double scale, 
                    const Eigen::Vector3d &center, 
                    const double r, 
                    const double g,
                    const double b);

    void setCuttingPlane(const Eigen::Vector3d normal, const double d);

    void getCuttingPlane(Eigen::Vector3d &normal, double &d);

    bool intersected(const std::vector<Eigen::Vector3d> &polygon);

    bool intersected(const Eigen::Vector3d &a, const Eigen::Vector3d &b);

    int calcSide(const std::vector<Eigen::Vector3d> &polygon);
  
    int calcSide(const Eigen::Vector3d &a, const Eigen::Vector3d &b);
  
protected:    
    // ax+by+cz+d=0 //    
    Eigen::Vector3d cut_plane_normal_;
    double d_;

    void genRandomColors(int num_of_colors);

    std::vector<Eigen::Vector3d> random_colors_;    
};
 
#endif /* end of include guard: PRIMITIVERENDERER_H__ */

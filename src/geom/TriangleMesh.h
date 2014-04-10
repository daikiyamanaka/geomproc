/*
 * TriangleMesh.h
 *
 *  Created on: Jun 26, 2012
 *      Author: daikiyamanaka
 */

#ifndef TRIANGLEMESH_H_
#define TRIANGLEMESH_H_

#include <vector>
#include <eigen3/Eigen/Dense>


/*
static Eigen::Vector3f centroid(Eigen::Vector3f &a, Eigen::Vector3f &b, Eigen::Vector3f &c){
    return (a+b+c)/3;
}
*/
class TriangleMesh {

public:
	TriangleMesh();
	virtual ~TriangleMesh();

    void clear();
	void read(std::string file_name);
	void write(std::string file_name);
    void readCTM(std::string filename);
    void createFromFaceVertex(std::vector<Eigen::Vector3d> &vertices, std::vector<std::vector<int> > &faces);
    void updateGeometry();

    // --- for Editing
	void add(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c);
    void ODT(std::vector<float> &p_weights);
    void CODT(std::vector<float> &p_weights);    

    // --- for IO
    void getCentroids(std::vector<Eigen::Vector3d> &_centroids) const;
    void getVertex(int index, Eigen::Vector3d &v);
    void setVertex(int index, Eigen::Vector3d &v);
    void getFace(int index, std::vector<int> &f);
    void setEdges(std::vector<std::pair<int, int> > &edges);
    void getEdges(std::vector<std::pair<int, int> > &edges);
    float getAverageLength();
    std::vector<int> getNeighbors(int index);

    void setSharpFeature(std::vector<int> feature_ids);
    void setSharpFeature(std::vector<Eigen::Vector3d> features);

    int get_number_of_faces();
    int get_number_of_vertices();
    void getBBox(Eigen::Vector3d &min, Eigen::Vector3d &max);
    Eigen::Vector3d getCenter();

    inline static double AREA2D(Eigen::Vector3d a, Eigen::Vector3d b, Eigen::Vector3d c){
        return fabs(0.5*((b[0]-a[0])*(c[1]-a[1]) - (b[1]-a[1])*(c[0]-a[0])));
    }

    static Eigen::Vector3d centroid(Eigen::Vector3d &a, Eigen::Vector3d &b, Eigen::Vector3d &c);
    static Eigen::Vector3d circumcenter(Eigen::Vector3d &a, Eigen::Vector3d &b, Eigen::Vector3d &c);

    // point
	std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector3d> v_normals;
    std::vector<std::vector<int> > v_f_map;// 1-ring neibhbor faces
    std::vector<std::vector<int> > v_neighbors; // 1-ring neighbor vertices
    std::vector<float> point_scalars;
    std::vector<bool> boundary_marker;
    std::vector<bool> sharp_feature_marker;
    // edge
    std::vector<std::pair<int, int> > edges;

    // face
    std::vector<Eigen::Vector3d> centroids;
    std::vector<Eigen::Vector3d> circumcenters;
	std::vector<std::vector<int> > facets;
    std::vector<std::vector<int> > neighbors;// neigbor_face
    std::vector<double> areas;
    std::vector<float> scalars;
    std::vector<Eigen::Vector3d> f_normals;

private:
    void normalizeScalar();
    void normalize();
    void initialize();

    void computeAverageLength();

    int num_of_vertices;
    int num_of_faces;

    float ave_length;
};

#endif /* TRIANGLEMESH_H_ */

/* --- ***********************************************************************
 *
 *  polyline.h
 *
 *                                          Created by daikiyamanaka on 6/14/2013
 * ******************************************************************** --- */

#ifndef POLYLINE_H
#define POLYLINE_H

#include <vector>
#include <eigen3/Eigen/Core>

class PolyLine
{
public:
    PolyLine();
    PolyLine(std::vector<Eigen::Vector3d> points, std::vector<Eigen::Vector2i> edges);
    virtual ~PolyLine();

    // --- IO ---
    void save(const std::string filename);
    void load(const std::string filename);

    // --- Creation ---
    void create(std::vector<Eigen::Vector3d> points, std::vector<Eigen::Vector2i> edges);
    void readPly(const std::string filename);

    // --- Access Functions ---
    Eigen::Vector3d getPoint(int index);
    Eigen::Vector2i getEdge(int index);
    Eigen::Vector3d getNormal(int index);

    void getPoints(std::vector<Eigen::Vector3d> &points);
    void getEdges(std::vector<Eigen::Vector2i> &edges);
    void getNormals(std::vector<Eigen::Vector3d> &normals);
    void getBBox(Eigen::Vector2d &min, Eigen::Vector2d &max);

    const Eigen::Vector2i getPointNeighbors(const int point_index);
    //const Eigen::Vector3d getEdgeCentroid(const int edge_index);

    int getNumEdges();
    int getNumPoints();

    // --- Modifier ---
    void resize(double bbox_size);
    bool contain(Eigen::Vector3d point);
    Eigen::Vector3d project(const Eigen::Vector3d point);
    void setSharpFeatures(const std::vector<int> &sharp_feature_ids);
    void removePoint(int index);

    // --- Predicates ---
    bool is_sharp_feature(int i);

private:
    void init();
    double distPts2Seg(const Eigen::Vector3d &p, const Eigen::Vector3d &e0, const Eigen::Vector3d &e1);
    
    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector2i> edges;
    std::vector<Eigen::Vector3d> normals;
    std::vector<Eigen::Vector3d> edge_centroids;    
    std::vector<Eigen::Vector2i> neighbor_points;
    std::vector<double> measures;
    std::vector<bool> sharp_feature_marker;
    Eigen::Vector2d min;
    Eigen::Vector2d max;

};

#endif // POLYLINE_H

/* -8- ***********************************************************************
 *
 *  polyline.cpp
 *
 *                                          Created by daikiyamanaka on 6/14/2013
 *                 Copyright (c) 2013 ABEJA Inc. All rights reserved.
 * ******************************************************************** -8- */

#include "polyline.h"
#include "Utility.h"
#include <iostream>
#include <fstream>
#include <sstream>
PolyLine::PolyLine()
{

}

PolyLine::PolyLine(std::vector<Eigen::Vector3d> _points, std::vector<Eigen::Vector2i> _edges){
    points = _points;
    edges = _edges;
    init();
}

PolyLine::~PolyLine(){

}

void PolyLine::save(const std::string filename){
  std::ofstream ofs;
  ofs.open(filename.c_str());

  // --- header ---
  ofs << "ply" << std::endl;
  ofs << "format ascii 1.0" << std::endl;
  ofs << "comment VCGLIB generated" << std::endl;
  ofs << "element vertex " <<  points.size() << std::endl;
  ofs << "property float x" << std::endl;
  ofs << "property float y" << std::endl;
  ofs << "property float z" << std::endl;
  ofs << "element edge " << edges.size() << std::endl;
  ofs << "property int vertex1" << std::endl;
  ofs << "property int vertex2" << std::endl;
  ofs << "element property " << points.size() << std::endl;
  ofs << "property int sharp_feature_marker" << std::endl;
  ofs << "end_header" << std::endl;

  for(int i=0; i<(int)points.size(); i++){
    ofs << points[i][0] << " " << points[i][1] << " " << points[i][2] << std::endl;
  }
  for(int i=0; i<(int)edges.size(); i++){
    ofs << edges[i][0] << " " << edges[i][1] << std::endl;
  }
  for(int i=0; i<(int)points.size(); i++){
    int marker = 0;
    if(sharp_feature_marker[i]){
      marker = 1;
    }
    ofs << marker << std::endl;
  }
  ofs.close();
}

void PolyLine::load(const std::string filename){

  std::ifstream ifs;
  ifs.open(filename.c_str());  
  int vertex_number, edge_number;
  while(1){
    std::string s_tmp;
    std::getline(ifs, s_tmp);
    std::stringstream ss(s_tmp);
    std::string str;
    ss >> str;
    if(str == "element"){
      ss >> str;
      if(str == "vertex"){
        ss >> vertex_number;
      }
      else if(str == "edge"){
        ss >> edge_number;
      }
    }
    if(str == "end_header"){
      break;
    }
  }
  
  points.resize(vertex_number);
  edges.resize(edge_number);
  sharp_feature_marker.resize(vertex_number);
  //std::fill(sharp_feature_marker.begin(), sharp_feature_marker.end(), false);
  std::string s_tmp;
  // --- read vertex ---
  std::cout << "reading vertices..." << std::endl;
  for(int i=0; i<vertex_number; i++){
    std::getline(ifs, s_tmp);
    std::stringstream ss(s_tmp);
    float x, y;
    ss >> x;
    ss >> y;
    //std::cout << "x : " << x << " y : " << y << std::endl;
    Eigen::Vector3d point = Eigen::Vector3d(x, y, 0);
    points[i] = point;
  }
  // --- read edge ---
  std::cout << "reading edges..." << std::endl;  
  for(int i=0; i<edge_number; i++){
    std::getline(ifs, s_tmp);
    std::stringstream ss(s_tmp);
    int v_index;
    Eigen::Vector2i edge;
    for(int j=0; j<2; j++){
      ss >> v_index;
      edge[j] = v_index;
    }
    edges[i] = edge;
  }

  min = Eigen::Vector2d(points[0][0], points[0][1]);
  max = Eigen::Vector2d(points[0][0], points[0][1]);
  double px, py;
  for(int i=0; i<vertex_number; i++){
    px = points[i][0];
    py = points[i][1];
    if(px > max[0]){
      max[0] = px;
    }
    else if(px < min[0]){
      min[0] = px;
    }
    if(py > max[1]){
      max[1] = py;
    }
    else if(py < min[1]){
      min[1] = py;
    }
  }
  init();

  // --- read sharp_feature_marker ---
  std::cout << "reading sharp_feature_marker..." << std::endl;  
  for(int i=0; i<vertex_number; i++){
    std::getline(ifs, s_tmp);
    std::stringstream ss(s_tmp);
    int marker;
    ss >> marker;
    if(marker == 1){
      sharp_feature_marker[i] = true;
    }
    std::cout << marker << std::endl;
  }

}

void PolyLine::create(std::vector<Eigen::Vector3d> _points, std::vector<Eigen::Vector2i> _edges){
  points = _points;
  edges = _edges;
  init();
}

void PolyLine::readPly(const std::string filename){

  std::ifstream ifs;
  ifs.open(filename.c_str());
  
  int vertex_number, edge_number;
  while(1){
    std::string s_tmp;
    std::getline(ifs, s_tmp);
    std::stringstream ss(s_tmp);
    std::string str;
    ss >> str;
    if(str == "element"){
      ss >> str;
      if(str == "vertex"){
        ss >> vertex_number;
      }
      else if(str == "edge"){
        ss >> edge_number;
      }
    }
    if(str == "end_header"){
      break;
    }
  }
  
  points.resize(vertex_number);
  edges.resize(edge_number);
  
  std::string s_tmp;
    // --- read vertex ---
  for(int i=0; i<vertex_number; i++){
    std::getline(ifs, s_tmp);
    std::stringstream ss(s_tmp);
    float x, y;
    ss >> x;
    ss >> y;
    //std::cout << "x : " << x << " y : " << y << std::endl;
    Eigen::Vector3d point = Eigen::Vector3d(x, y, 0);
    points[i] = point;
  }
    // --- read edge ---
  for(int i=0; i<edge_number; i++){
    std::getline(ifs, s_tmp);
    std::stringstream ss(s_tmp);
    int v_index;
    Eigen::Vector2i edge;
    for(int j=0; j<2; j++){
      ss >> v_index;
      edge[j] = v_index;
    }
    edges[i] = edge;
  }
  min = Eigen::Vector2d(points[0][0], points[0][1]);
  max = Eigen::Vector2d(points[0][0], points[0][1]);
  double px, py;
  for(int i=0; i<vertex_number; i++){
    px = points[i][0];
    py = points[i][1];
    if(px > max[0]){
      max[0] = px;
    }
    else if(px < min[0]){
      min[0] = px;
    }
    if(py > max[1]){
      max[1] = py;
    }
    else if(py < min[1]){
      min[1] = py;
    }
  }
  init();
}
  
Eigen::Vector3d PolyLine::getPoint(int index){
  return points[index];
}

Eigen::Vector2i PolyLine::getEdge(int index){
    return edges[index];
}

Eigen::Vector3d PolyLine::getNormal(int index){
    return normals[index];
}

void PolyLine::getPoints(std::vector<Eigen::Vector3d> &_points){
    _points = points;
}

void PolyLine::getEdges(std::vector<Eigen::Vector2i> &_edges){
    _edges = edges;
}

void PolyLine::getNormals(std::vector<Eigen::Vector3d> &_normals){
    _normals = normals;
}

void PolyLine::getBBox(Eigen::Vector2d &_min, Eigen::Vector2d &_max){
    _min = min;
    _max = max;
}

const Eigen::Vector2i PolyLine::getPointNeighbors(const int point_index){
  return neighbor_points[point_index];
}

int PolyLine::getNumEdges(){
    return (int)edges.size();
}

int PolyLine::getNumPoints(){
    return (int)points.size();
}

void PolyLine::resize(double bbox_size){
    std::cout << "PolyLine::resize" << std::endl;

    std::vector<double> box(2);
    box[0] = fabs(max[0]-min[0]);
    box[1] = fabs(max[1]-min[1]);

    double scale = bbox_size/(*max_element(box.begin(), box.end()));

    for(int i=0; i<points.size(); i++){
        points[i]*=scale;
    }
    init();
}

Eigen::Vector3d PolyLine::project(const Eigen::Vector3d point){  
  int min_edge_index = 0;
  int min_dist = utility::distPts2Seg(point, points[edges[0][0]], points[edges[0][1]]);
  for(int i=1; i<(int)edges.size(); i++){
    double dist = utility::distPts2Seg(point, points[edges[i][0]], points[edges[i][1]]);
    if(dist < min_dist){
      min_dist = dist;
      min_edge_index = i;
    }
  }
  Eigen::Vector3d e1 = points[edges[min_edge_index][1]];
  Eigen::Vector3d e0 = points[edges[min_edge_index][0]];  
  double t = (e1-e0).dot(point-e0)/(e1-e0).squaredNorm();
  return (e1-e0)*t+e0;
}

void PolyLine::setSharpFeatures(const std::vector<int> &sharp_feature_ids){
  for(int i=0; i<(int)sharp_feature_ids.size(); i++){
    sharp_feature_marker[sharp_feature_ids[i]] = true;
  }
}

bool PolyLine::is_sharp_feature(int i){
  return sharp_feature_marker[i];
}

void PolyLine::init(){
  normals.resize(edges.size());
  edge_centroids.resize(edges.size());
  measures.resize(edges.size());
  for(int i=0; i<normals.size(); i++){
    Eigen::Vector3d normal = points[edges[i][1]] - points[edges[i][0]];
    measures[i] = normal.norm();
    normal.normalize();
    normals[i] = Eigen::Vector3d(normal[1], -normal[0], 0);
    edge_centroids[i] = (points[edges[i][1]] + points[edges[i][0]])/2.0;
  }
  sharp_feature_marker.resize(points.size());
  std::fill(sharp_feature_marker.begin(), sharp_feature_marker.end(), false);

  // calculate neighbors
  std::vector<std::vector<int> > neighbor_index;
  neighbor_index.resize(points.size());
  for(int i=0; i<edges.size(); i++){
    Eigen::Vector2i edge = edges[i];
    neighbor_index[edge[0]].push_back(edge[1]);
    neighbor_index[edge[1]].push_back(edge[0]);
  }
  neighbor_points.resize(points.size());
  for(int i=0; i<(int)neighbor_index.size(); i++){
    std::vector<int> neighbor = neighbor_index[i];
    if(neighbor.size() == 2){
      neighbor_points[i] = Eigen::Vector2i(neighbor[0], neighbor[1]);
    }
  }
}

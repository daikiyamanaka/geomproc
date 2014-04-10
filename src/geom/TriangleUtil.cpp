#include "triangleUtil.h"

namespace triangle_util{

/*
void reDelaunay(Shape2D &mesh, std::vector<Eigen::Vector2i> &boundary_edges, std::string switches){

    triangulateio tri, out, *vorout;
    vorout = NULL;
    out.pointlist = NULL;
    out.pointmarkerlist = NULL;    
    out.trianglelist = NULL;
    out.triangleattributelist = NULL;
    out.trianglearealist = NULL;    
    out.pointattributelist = NULL;
    out.neighborlist = NULL;
    out.segmentlist = NULL;
    out.segmentmarkerlist = NULL;
    out.edgelist = NULL;
    out.edgemarkerlist = NULL;

    tri.numberofpoints = mesh.get_number_of_vertices();
    tri.numberofpointattributes = 0;
    tri.pointlist = new REAL[tri.numberofpoints*2];
    tri.pointattributelist = NULL;
    tri.pointmarkerlist = new int[tri.numberofpoints];
    
    tri.numberofsegments = boundary_edges.size();
    tri.segmentlist = new int[tri.numberofsegments*2];
    
    tri.segmentmarkerlist = NULL;
    tri.numberofholes = 0;
    tri.numberofregions = 0;
    tri.holelist = NULL;
    tri.regionlist = NULL;

    // insert input mesh points
    for(int i=0; i<mesh.get_number_of_vertices(); i++)
    {
      tri.pointlist[i*2] = mesh.points[i][0];
      tri.pointlist[i*2+1] = mesh.points[i][1];
      tri.pointmarkerlist[i] = 0;
      if(mesh.sharp_feature_marker[i]){
        tri.pointmarkerlist[i] = 2;
      }
      else if(mesh.boundary_marker[i]){        
        tri.pointmarkerlist[i] = 1;
      }
    }

    // insert boundary edges
    for (int i = 0; i < (int)boundary_edges.size(); ++i)
    {
      tri.segmentlist[i*2] = boundary_edges[i][0];
      tri.segmentlist[i*2+1] = boundary_edges[i][1];      
    }

    triangulate((char*)switches.c_str(), &tri, &out, vorout);    

    //std::cout << "done" << std::endl;
    std::vector<Eigen::Vector3d> tri_points;
    std::vector<std::vector<int> > faces;

    tri_points.resize(out.numberofpoints);
    faces.resize(out.numberoftriangles);    

    for(int i=0; i<out.numberofpoints; i++){
      tri_points[i] = Eigen::Vector3d(out.pointlist[i*2], out.pointlist[i*2+1], 0);            
    }

    for(int i=0; i<out.numberoftriangles; i++){
      std::vector<int> face(3);
      face[0] = out.trianglelist[i*3];
      face[1] = out.trianglelist[i*3+1];
      face[2] = out.trianglelist[i*3+2];
      faces[i] = face;
    }

    mesh.createFromFaceVertex(tri_points, faces);
    for(int i=0; i<out.numberoftriangles; i++){
      if(out.pointmarkerlist[i] == 2){
        mesh.sharp_feature_marker[i] = true;
      }
      else if(out.pointmarkerlist[i] == 1){
        mesh.boundary_marker[i] = true;
      }
    }

    boundary_edges.resize(0);
    for(int i=0; i<out.numberofedges; i++){
      if(out.edgemarkerlist[i] >= 1){
        Eigen::Vector2i edge;
        edge[0] = out.edgelist[i*2];
        edge[1] = out.edgelist[i*2+1];
        boundary_edges.push_back(edge);
      }
    }

    delete(tri.pointlist);
    delete(tri.segmentlist);    
    delete(tri.pointmarkerlist);    
}
*/
void boundary2mesh(PolyLine &boundary, 
                   std::vector<Eigen::Vector3d> &constrained_points, 
                   TriangleMesh &mesh,
                   std::vector<int> &index_mapper,
                   std::string switches
                   )
{

    //double dist_threshold = 0.001;

    std::vector<Eigen::Vector3d> points;
    std::vector<Eigen::Vector2i> edges;
    boundary.getPoints(points);
    boundary.getEdges(edges);

    triangulateio tri, out, *vorout;
    vorout = NULL;
    out.pointlist = NULL;
    out.pointmarkerlist = NULL;    
    out.trianglelist = NULL;
    out.triangleattributelist = NULL;
    out.trianglearealist = NULL;    
    out.pointattributelist = NULL;
    out.neighborlist = NULL;
    out.segmentlist = NULL;
    out.segmentmarkerlist = NULL;
    out.edgelist = NULL;
    out.edgemarkerlist = NULL;

    tri.numberofpoints = boundary.getNumPoints() + constrained_points.size();
    tri.numberofpointattributes = 0;
    tri.pointlist = new REAL[tri.numberofpoints*2];
    tri.pointattributelist = NULL;
    tri.pointmarkerlist = new int[tri.numberofpoints];
    
    tri.numberofsegments = boundary.getNumEdges();
    tri.segmentlist = new int[tri.numberofsegments*2];
    
    //tri.segmentmarkerlist = NULL;
    tri.segmentmarkerlist = new int[tri.numberofsegments];
    tri.numberofholes = 0;
    tri.numberofregions = 0;
    tri.holelist = NULL;
    tri.regionlist = NULL;

    //std::cout << "insert boundary points: " << points.size()<< std::endl;
    for(int i=0; i<(int)points.size(); i++){
      tri.pointlist[i*2] = points[i][0];
      tri.pointlist[i*2+1] = points[i][1];
      tri.pointmarkerlist[i] = 1;      
      if(boundary.is_sharp_feature(i)){
        tri.pointmarkerlist[i] = 2;
      }      
    }
    //std::cout << "insert boundary edge: " << edges.size()<< std::endl;    
    for(int i=0; i<(int)edges.size(); i++){
      tri.segmentlist[i*2] = edges[i][0];
      tri.segmentlist[i*2+1] = edges[i][1];
      tri.segmentmarkerlist[i] = 1;
    }    
    //std::cout << "insert constrained points" << std::endl;
    int index_offset = points.size();
    for(int i=0; i<(int)constrained_points.size(); i++){
        int j = i+index_offset;
        tri.pointlist[j*2] = constrained_points[i][0];
        tri.pointlist[j*2+1] = constrained_points[i][1];
        tri.pointmarkerlist[j] = 0;
    }

    //std::cout << "triangulate" << std::endl;
    triangulate((char*)switches.c_str(), &tri, &out, vorout);    

    //std::cout << "done" << std::endl;
    std::vector<Eigen::Vector3d> tri_points;
    std::vector<std::vector<int> > faces;

    tri_points.resize(out.numberofpoints);
    faces.resize(out.numberoftriangles);

    for(int i=0; i<out.numberofpoints; i++){
      tri_points[i] = Eigen::Vector3d(out.pointlist[i*2], out.pointlist[i*2+1], 0);
    }

    for(int i=0; i<out.numberoftriangles; i++){
      std::vector<int> face(3);
      face[0] = out.trianglelist[i*3];
      face[1] = out.trianglelist[i*3+1];
      face[2] = out.trianglelist[i*3+2];
      faces[i] = face;
    }

    mesh.createFromFaceVertex(tri_points, faces);
    for(int i=0; i<(int)mesh.points.size(); i++){
      if(out.pointmarkerlist[i] == 1){
        mesh.boundary_marker[i] = true;
      }
      else if(out.pointmarkerlist[i] == 2){
        mesh.sharp_feature_marker[i] = true;
      }
    }

    //std::cout << "creating boundary polyline" << std::endl;
    std::vector<Eigen::Vector3d> boundary_points;
    std::vector<Eigen::Vector2i> boundary_edges;    
    index_mapper.resize(out.numberofpoints);
    std::fill(index_mapper.begin(), index_mapper.end(), -1);

    std::vector<int> sharp_feature_ids;
    for(int i=0; i<out.numberofpoints; i++){
      if(out.pointmarkerlist[i] == 1 || out.pointmarkerlist[i] == 2){
        boundary_points.push_back(Eigen::Vector3d(out.pointlist[i*2], out.pointlist[i*2+1], 0));
        index_mapper[i] = boundary_points.size()-1;
        if(out.pointmarkerlist[i] == 2){
          sharp_feature_ids.push_back(index_mapper[i]);
        }
      }
    }
    /*
    for(int i=0; i<out.numberofsegments; i++){
      std::cout << out.segmentmarkerlist[i] << std::endl;
      if(out.segmentmarkerlist[i] != 0){
        Eigen::Vector2i edge;
        edge[0] = index_mapper[out.segmentlist[i*2]];
        edge[1] = index_mapper[out.segmentlist[i*2+1]];
        boundary_edges.push_back(edge);
      }
    }
    */  
    for(int i=0; i<out.numberofedges; i++){
      if(out.edgemarkerlist[i] == 1){
        Eigen::Vector2i edge;
        edge[0] = index_mapper[out.edgelist[i*2]];
        edge[1] = index_mapper[out.edgelist[i*2+1]];
        boundary_edges.push_back(edge);
      }
    }    
    //IO::savePolyLine("boundary_test.ply", boundary_points, boundary_edges);

    boundary.create(boundary_points, boundary_edges);    
    boundary.setSharpFeatures(sharp_feature_ids);

    delete(tri.pointlist);
    delete(tri.segmentlist);    
    delete(tri.pointmarkerlist);
}

/*
void genMeshWithConstraints(std::vector<Eigen::Vector3d> &points, 
                            std::vector<Eigen::Vector2i> &edges,
                            TriangleMesh &mesh,
                            std::string switches
                            )
{
    triangulateio tri, out, *vorout;
    vorout = NULL;
    out.pointlist = NULL;
    out.pointattributelist = NULL;
    out.pointmarkerlist = NULL;    
    out.trianglelist = NULL;
    out.triangleattributelist = NULL;
    out.trianglearealist = NULL;    
    out.neighborlist = NULL;
    out.segmentlist = NULL;
    out.segmentmarkerlist = NULL;

    tri.numberofpoints = (int)points.size();
    tri.numberofpointattributes = 0;
    tri.pointlist = new REAL[tri.numberofpoints*2];
    tri.pointattributelist = NULL;
    tri.pointmarkerlist = NULL;
    tri.numberofsegments = (int)edges.size();
    tri.segmentlist = new int[tri.numberofsegments*2];
    tri.segmentmarkerlist = NULL;
    tri.numberofholes = 0;
    tri.numberofregions = 0;
    tri.holelist = NULL;
    tri.regionlist = NULL;

    std::cout << "copy input points" << std::endl;
    for(int i=0; i<(int)points.size(); i++){
      tri.pointlist[i*2] = points[i][0];
      tri.pointlist[i*2+1] = points[i][1];
    }
    for(int i=0; i<(int)edges.size(); i++){
      tri.segmentlist[i*2] = edges[i][0];
      tri.segmentlist[i*2+1] = edges[i][1];      
    }

    std::cout << "triangulate" << std::endl;
    triangulate((char*)switches.c_str(), &tri, &out, vorout);    

    std::cout << "done" << std::endl;
    std::vector<Eigen::Vector3d> tri_points;
    std::vector<std::vector<int> > faces;

    tri_points.resize(out.numberofpoints);
    faces.resize(out.numberoftriangles);
    
    for(int i=0; i<out.numberofpoints; i++){
      tri_points[i] = Eigen::Vector3d(out.pointlist[i*2], out.pointlist[i*2+1], 0);
    }

    for(int i=0; i<out.numberoftriangles; i++){
        std::vector<int> face(3);
        face[0] = out.trianglelist[i*3];
        face[1] = out.trianglelist[i*3+1];
        face[2] = out.trianglelist[i*3+2];
        faces[i] = face;
    }

    mesh.createFromFaceVertex(tri_points, faces);
    delete(tri.pointlist);
    delete(tri.segmentlist);  
}

void genDualMeshWithConstraints(
                            std::vector<Eigen::Vector3d> &points, 
                            std::vector<Eigen::Vector2i> &edges,
                            std::vector<Eigen::Vector3d> &dual_points,
                            std::vector<Eigen::Vector2i> &dual_edges)
{
    triangulateio tri, out, vorout;
    vorout.pointlist = NULL;
    vorout.pointattributelist = NULL;
    vorout.edgelist = NULL;
    vorout.normlist = NULL;
    out.pointlist = NULL;
    out.pointattributelist = NULL;
    out.pointmarkerlist = NULL;    
    out.trianglelist = NULL;
    out.triangleattributelist = NULL;
    out.trianglearealist = NULL;    
    out.neighborlist = NULL;
    out.segmentlist = NULL;
    out.segmentmarkerlist = NULL;

    tri.numberofpoints = (int)points.size();
    tri.numberofpointattributes = 0;
    tri.pointlist = new REAL[tri.numberofpoints*2];
    tri.pointattributelist = NULL;
    tri.pointmarkerlist = NULL;
    tri.numberofsegments = (int)edges.size();
    tri.segmentlist = new int[tri.numberofsegments*2];
    tri.segmentmarkerlist = NULL;
    tri.numberofholes = 0;
    tri.numberofregions = 0;
    tri.holelist = NULL;
    tri.regionlist = NULL;

    std::cout << "copy input points" << std::endl;
    for(int i=0; i<(int)points.size(); i++){
      tri.pointlist[i*2] = points[i][0];
      tri.pointlist[i*2+1] = points[i][1];
    }
    for(int i=0; i<(int)edges.size(); i++){
      tri.segmentlist[i*2] = edges[i][0];
      tri.segmentlist[i*2+1] = edges[i][1];      
    }

    std::cout << "triangulate" << std::endl;
    //triangulate("pq0zna30", &tri, &out, vorout);
    triangulate("vpq0zna30", &tri, &out, &vorout);    

    std::cout << "done" << std::endl;

    dual_points.resize(vorout.numberofpoints);
    for(int i=0; i<vorout.numberofpoints; i++){
        Eigen::Vector3d point;
        point[0] = vorout.pointlist[i*2];
        point[1] = vorout.pointlist[i*2+1];
        point[2] = 0;
        dual_points[i] = point;
    }
    std::cout << "dual_points" << std::endl;
    double norm_thresh = 0.0001;
    dual_edges.resize(vorout.numberofedges);
    for(int i=0; i<vorout.numberofedges; i++){
        double norm = vorout.normlist[i*2]*vorout.normlist[i*2] + vorout.normlist[i*2+1]*vorout.normlist[i*2+1];
        Eigen::Vector2i edge;
        edge[0] = vorout.edgelist[i*2];
        edge[1] = vorout.edgelist[i*2+1];
        if(norm > norm_thresh){
            edge[0] = -1;
            edge[1] = -1;
        }
        dual_edges[i] = edge;
    }
    std::cout << "dual_edges" << std::endl;

    delete(tri.pointlist);
    delete(tri.segmentlist);  
}
*/
}

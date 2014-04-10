#include "TetUtil.h"

void tetrahedralize(const std::vector<Eigen::Vector3d> &src_points,
	             const std::string switches,
	             tetgenio &out
	             )
{
	tetgenio in;
	in.initialize();
	in.firstnumber = 0;
	in.numberofpoints = src_points.size();
	in.pointlist = new REAL[in.numberofpoints*3];
	std::cout << "set points" << std::endl;
	for(int i=0; i<(int)src_points.size(); i++){
		in.pointlist[i*3] = src_points[i][0];
		in.pointlist[i*3+1] = src_points[i][1];
		in.pointlist[i*3+2] = src_points[i][2];		
	}	

	tetgenbehavior b;
	b.parse_commandline(const_cast<char*>(switches.c_str()));
	std::cout << "tetrahedralization" << std::endl;
	out.deinitialize();
	out.initialize();
	tetrahedralize(&b, &in, &out);

}


void mesh2tet(const std::vector<Eigen::Vector3d> &src_points,
              const std::vector<std::vector<int> > &src_faces,
              std::vector<Eigen::Vector3d > &dst_points,
              std::vector<std::vector<int> > &dst_tets,
              std::vector<std::vector<int> > &dst_ntets,
              const std::string switches
	            )
{
	tetgenio::facet *f;
	tetgenio::polygon *p;
	tetgenio in, out;

	in.initialize();
	in.firstnumber = 0;	
	in.numberofpoints = src_points.size();
	in.pointlist = new REAL[in.numberofpoints*3];
	std::cout << "set points" << std::endl;
	for(int i=0; i<(int)src_points.size(); i++){
		in.pointlist[i*3] = src_points[i][0];
		in.pointlist[i*3+1] = src_points[i][1];
		in.pointlist[i*3+2] = src_points[i][2];		
	}

/*
	in.numberoffacets = 1;
	in.facetlist = new tetgenio::facet[in.numberoffacets];
	in.facetmarkerlist = new int[in.numberoffacets];
	in.facetmarkerlist[0] = 1;
	f = &in.facetlist[0];	
	f->numberofpolygons = src_faces.size();
	f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
	f->numberofholes = 0;
	f->holelist = NULL;	
	for(int i=0; i<f->numberofpolygons; i++){
		p = &f->polygonlist[i];
		p->numberofvertices = 3;
		p->vertexlist = new int[p->numberofvertices];
		for(int j=0; j<3; j++){
			p->vertexlist[j] = src_faces[i][j];
		}
	}
*/

	in.numberoffacets = src_faces.size();
	in.facetlist = new tetgenio::facet[in.numberoffacets];
	in.facetmarkerlist = new int[in.numberoffacets];
	std::cout << "set faces" << std::endl;
	for(int i=0; i<in.numberoffacets; i++){
		in.facetmarkerlist[i] = 1;
		f = &in.facetlist[i];
		f->numberofpolygons = 1;
		f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
		f->numberofholes = 0;
		f->holelist = NULL;
		p = &f->polygonlist[0];
		p->numberofvertices = 3;
		p->vertexlist = new int[p->numberofvertices];
		for(int j=0; j<3; j++){
			p->vertexlist[j] = src_faces[i][j];
		}
	}

	//std::stringstream ss("");
	//ss << "npzqa0.01";
	//ss << "znpq";	
	tetgenbehavior b;
	b.parse_commandline(const_cast<char*>(switches.c_str()));
	//b.parse_commandline(switches.c_str());
	//b.parse_commandline("znp");
	std::cout << "tetrahedralization" << std::endl;
	out.deinitialize();
	out.initialize();
	tetrahedralize(&b, &in, &out);
	//tetrahedralize("pqz", &in, &out);	

  dst_points.resize(out.numberofpoints);
  dst_tets.resize(out.numberoftetrahedra);

  std::cout << "out.numberofpoints: " << out.numberofpoints << std::endl;
  std::cout << "out.numberoftrifaces: " << out.numberoftrifaces << std::endl;
  std::cout << "out.numoftetrahedra: " << out.numberoftetrahedra << std::endl;    

  for(int i=0; i<out.numberofpoints; i++){
  	dst_points[i] = Eigen::Vector3d(out.pointlist[i*3], out.pointlist[i*3+1], out.pointlist[i*3+2]);
  }

  for(int i=0; i<out.numberoftetrahedra; i++){
  	std::vector<int> tet(4);
  	tet[0] = out.tetrahedronlist[i*4];
  	tet[1] = out.tetrahedronlist[i*4+1];
  	tet[2] = out.tetrahedronlist[i*4+2];  	  	
  	tet[3] = out.tetrahedronlist[i*4+3]; 
  	dst_tets[i] = tet;
  }

  dst_ntets.resize(out.numberoftetrahedra);
  for(int i=0; i<out.numberoftetrahedra; i++){  	
  	std::vector<int> neighbor;
  	for(int j=0; j<4; j++){
  		if(out.neighborlist[i*4+j] < 0){
  			continue;
  		}
  		neighbor.push_back(out.neighborlist[i*4+j]);
  	}
  	dst_ntets[i] = neighbor;
  }
}

void mesh2tet(const std::vector<Eigen::Vector3d> &src_points,
              const std::vector<std::vector<int> > &src_faces,
	            const std::vector<Eigen::Vector3d> &src_inner_points,              
              std::vector<Eigen::Vector3d > &dst_points,
              std::vector<std::vector<int> > &dst_tets,
              std::vector<std::vector<int> > &dst_ntets,
              std::vector<std::pair<int, int> > &dst_edges,
              std::vector<int> &dst_edge_marker, 
              const std::string switches
	            )
{	
	tetgenio::facet *f;
	tetgenio::polygon *p;
	tetgenio in, out;

	in.initialize();
	in.firstnumber = 0;	
	in.numberofpoints = src_points.size()+src_inner_points.size();
	in.pointlist = new REAL[in.numberofpoints*3];
	for(int i=0; i<(int)src_points.size(); i++){
		in.pointlist[i*3] = src_points[i][0];
		in.pointlist[i*3+1] = src_points[i][1];
		in.pointlist[i*3+2] = src_points[i][2];
	}

	int index_offset = src_points.size();
	for(int i=0; i<(int)src_inner_points.size(); i++){
		int j = index_offset+i;
		in.pointlist[j*3] = src_inner_points[i][0];
		in.pointlist[j*3+1] = src_inner_points[i][1];
		in.pointlist[j*3+2] = src_inner_points[i][2];				
	}

	in.numberoffacets = src_faces.size();
	in.facetlist = new tetgenio::facet[in.numberoffacets];
	in.facetmarkerlist = new int[in.numberoffacets];
	for(int i=0; i<in.numberoffacets; i++){
		in.facetmarkerlist[i] = 1;
		f = &in.facetlist[i];
		f->numberofpolygons = 1;
		f->polygonlist = new tetgenio::polygon[f->numberofpolygons];
		f->numberofholes = 0;
		f->holelist = NULL;
		p = &f->polygonlist[0];
		p->numberofvertices = 3;
		p->vertexlist = new int[p->numberofvertices];
		for(int j=0; j<3; j++){
			p->vertexlist[j] = src_faces[i][j];
		}
	}

	tetgenbehavior b;
	b.parse_commandline(const_cast<char*>(switches.c_str()));
	std::cout << "tetrahedralization" << std::endl;
	out.deinitialize();
	out.initialize();
	tetrahedralize(&b, &in, &out);

  dst_points.resize(out.numberofpoints);
  dst_tets.resize(out.numberoftetrahedra);

  std::cout << "out.numberofpoints: " << out.numberofpoints << std::endl;
  std::cout << "out.numberoftrifaces: " << out.numberoftrifaces << std::endl;
  std::cout << "out.numoftetrahedra: " << out.numberoftetrahedra << std::endl;    

  // points //
  for(int i=0; i<out.numberofpoints; i++){
  	dst_points[i] = Eigen::Vector3d(out.pointlist[i*3], out.pointlist[i*3+1], out.pointlist[i*3+2]);
  }

  // tetrahedra //
  for(int i=0; i<out.numberoftetrahedra; i++){
  	std::vector<int> tet(4);
  	tet[0] = out.tetrahedronlist[i*4];
  	tet[1] = out.tetrahedronlist[i*4+1];
  	tet[2] = out.tetrahedronlist[i*4+2];  	  	
  	tet[3] = out.tetrahedronlist[i*4+3]; 
  	dst_tets[i] = tet;
  }

  // neighbors //
  dst_ntets.resize(out.numberoftetrahedra);
  for(int i=0; i<out.numberoftetrahedra; i++){  	
  	std::vector<int> neighbor;
  	for(int j=0; j<4; j++){
  		if(out.neighborlist[i*4+j] < 0){
  			continue;
  		}
  		neighbor.push_back(out.neighborlist[i*4+j]);
  	}
  	dst_ntets[i] = neighbor;
  }

  // edges //
  dst_edges.resize(out.numberofedges);
  dst_edge_marker.resize(out.numberofedges);
  for(int i=0; i<out.numberofedges; i++){
  	dst_edges[i] = std::pair<int, int>(out.edgelist[i*2], out.edgelist[i*2+1]);
  	dst_edge_marker[i] = out.edgemarkerlist[i];
  	//std::cout << "edge_marker " << i << " " << out.edgemarkerlist[i] << std::endl;
  }


}


void refinement(std::vector<Eigen::Vector3d> &points,
	              std::vector<std::vector<int> > &tets,
	              std::vector<std::vector<int> > &ntets
	              )
{
	tetgenio in, out;

	in.initialize();
	//in.firstnumber = 0;
	in.numberofpoints = points.size();
	in.pointlist = new REAL[in.numberofpoints*3];
	std::cout << "set points" << std::endl;
	for(int i=0; i<(int)points.size(); i++){
		in.pointlist[i*3] = points[i][0];
		in.pointlist[i*3+1] = points[i][1];
		in.pointlist[i*3+2] = points[i][2];		
	}

	in.numberoftetrahedra = tets.size();
	in.numberofcorners = 4;
	in.tetrahedronlist = new int[in.numberoftetrahedra*4];
	for(int i=0; i<in.numberoftetrahedra; i++){
		for(int j=0; j<4; j++){
			in.tetrahedronlist[i*4+j] = tets[i][j];
		}
	}

	tetgenbehavior b;
	b.parse_commandline("rznp");
	std::cout << "tetrahedralization" << std::endl;
	out.deinitialize();
	out.initialize();
	tetrahedralize(&b, &in, &out);

  points.resize(out.numberofpoints);
  tets.resize(out.numberoftetrahedra);
  ntets.resize(out.numberoftetrahedra);  

  std::cout << "out.numberofpoints: " << out.numberofpoints << std::endl;
  std::cout << "out.numberoftrifaces: " << out.numberoftrifaces << std::endl;
  std::cout << "out.numoftetrahedra: " << out.numberoftetrahedra << std::endl;    

  for(int i=0; i<out.numberofpoints; i++){
  	points[i] = Eigen::Vector3d(out.pointlist[i*3], out.pointlist[i*3+1], out.pointlist[i*3+2]);
  }

  for(int i=0; i<out.numberoftetrahedra; i++){
  	std::vector<int> tet(4);
  	tet[0] = out.tetrahedronlist[i*4];
  	tet[1] = out.tetrahedronlist[i*4+1];
  	tet[2] = out.tetrahedronlist[i*4+2];  	  	
  	tet[3] = out.tetrahedronlist[i*4+3]; 
  	tets[i] = tet;
  }

  for(int i=0; i<out.numberoftetrahedra; i++){
  	std::vector<int> neighbor(4);
  	for(int j=0; j<4; j++){
  		neighbor[i] = out.neighborlist[i*4+j];
  	}
  	ntets[i] = neighbor;
  }

  /*
  for(int i=0; i<out.numberoftetrahedra; i++){  	
  	std::vector<int> neighbor;
  	for(int j=0; j<4; j++){
  		if(out.neighborlist[i*4+j] < 0){
  			continue;
  		}
  		neighbor.push_back(out.neighborlist[i*4+j]);
  	}
  	ntets[i] = neighbor;
  }
  */
}

#ifndef LABELEDTRIANGLEMESH_H__
#define LABELEDTRIANGLEMESH_H__
 
#include  <map> 
#include "TriangleMesh.h"

class LabeledTriangleMesh : public TriangleMesh
{
public:
	LabeledTriangleMesh();

	~LabeledTriangleMesh();

	// Access Functions //
	int getLabel(const int &f_index) const;

	int getNumberOfLabels() const;

	void getFaceGroup(const int &group_id, std::vector<int> &faces);

	// Modifier //	
	void setLabel(const int &f_index, const int &label);

  void createFromFaceVertex(std::vector<Eigen::Vector3d> &vertices, std::vector<std::vector<int> > &faces, std::vector<int> &face_labels);

private:
	void initialize();

	std::vector<int> face_labels_;

	std::map<int, std::vector<int> > label_face_map_;
};
 
#endif /* end of include guard: LABELEDTRIANGLEMESH_H__ */

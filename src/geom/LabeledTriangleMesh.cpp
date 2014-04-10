#include <cassert>
#include "LabeledTriangleMesh.h"


LabeledTriangleMesh::LabeledTriangleMesh()
{
	TriangleMesh::TriangleMesh();
}

LabeledTriangleMesh::~LabeledTriangleMesh()
{

}

int LabeledTriangleMesh::getLabel(const int &f_index) const
{
	return face_labels_[f_index];
}

int LabeledTriangleMesh::getNumberOfLabels() const
{
	return face_labels_.size();
}

void LabeledTriangleMesh::getFaceGroup(const int &group_id, std::vector<int> &faces)
{
	faces = label_face_map_[group_id];
}

void LabeledTriangleMesh::setLabel(const int &f_index, const int &label)
{
	face_labels_[f_index] = label;
}

void LabeledTriangleMesh::createFromFaceVertex(std::vector<Eigen::Vector3d> &vertices, std::vector<std::vector<int> > &faces, std::vector<int> &face_labels)
{	
	assert( faces.size() != face_labels.size());
	TriangleMesh::createFromFaceVertex(vertices, faces);
	face_labels_ = face_labels;
	initialize();
}

void LabeledTriangleMesh::initialize()
{
	// compute map label to faces //
	for(int i=0; i<(int)face_labels_.size(); i++){
		label_face_map_[face_labels_[i]].push_back(i);
	}
}

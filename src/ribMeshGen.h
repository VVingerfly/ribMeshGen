#pragma once
#include "mesh.h"


class ribMeshGen
{
public:
	ribMeshGen(MyMesh *ptr_mesh);
	~ribMeshGen();
	void ribShellGenerator(string file);
	void ribShellGenerator();
	void ribMeshGenerator();
	void setParameters(double w, double h, double d);
	void setParametersFromVolume(double totVol, double t);
	MyMesh getRibMesh() { return *ribMesh_; }
private:
	void closeGaps();
	void addFaceFromMesh();   // add face from the face of input mesh
	void addFaceFromFace(bool addUp = true, bool addDn = true);
	void addFaceFromEdge(bool addUp = true, bool addDn = true, bool addLf = true, bool addRt = true);
	void addFaceFromVert(bool addUp = true, bool addDn = true);
	void addVert2RibMesh();
	void calcRibMeshPoint(MyMesh::FaceHandle fh);

	double calcFaceArea(MyMesh::FaceHandle f);
	void addFaceFromEdgeSelected(const vector<int> &eSelected);
	void readEdgeSelection(string file, vector<int> &eSelected);
private:
	MyMesh *mesh_;
	MyMesh *ribMesh_;
	int ribPtNum_;

	double ribW_;  // width
	double ribH_;  // height
	double disp_;  // distance to move along normal direction

	double ribVol_;

	OpenMesh::HPropHandleT<MyMesh::Point> HPropPtUp;
	OpenMesh::HPropHandleT<MyMesh::Point> HPropPtDn;
	OpenMesh::HPropHandleT<MyMesh::VertexHandle> HPropVhUp;
	OpenMesh::HPropHandleT<MyMesh::VertexHandle> HPropVhDn;
};


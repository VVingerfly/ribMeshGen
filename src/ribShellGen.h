#pragma once
#include "mesh.h"

class ribShellGen
{
public:
	ribShellGen(MyMesh *mesh, string file);
	~ribShellGen();
	void generator();
	MyMesh getRibMesh() { return *ribShell_; }
	void setParameters(double w, double h, double d);
	void setParametersFromVolume(double totVol, double t);
	void readEdgeSelection(string file, vector<int> &eSelected);
	void initEdgeSelection(const vector<int> &selection);
	void addVertFromVert();
	void addFaceFromEdge(bool up = true, bool dn = true, bool lf = true, bool rt = true);
	void calcEdgeVert();

	void initProp();
	double calcFaceArea(MyMesh::FaceHandle f);

private:
	MyMesh *mesh_;
	MyMesh *ribShell_;

	double ribW_;  // width
	double ribH_;  // height
	double disp_;  // distance to move along normal direction

	double ribVol_;

	OpenMesh::HPropHandleT<MyMesh::Point> HPropPtUp;
	OpenMesh::HPropHandleT<MyMesh::Point> HPropPtDn;
	OpenMesh::HPropHandleT<MyMesh::VertexHandle> HPropVhUp;
	OpenMesh::HPropHandleT<MyMesh::VertexHandle> HPropVhDn;

	OpenMesh::HPropHandleT<MyMesh::VertexHandle> HPropVhUp0;
	OpenMesh::HPropHandleT<MyMesh::VertexHandle> HPropVhDn0;
	OpenMesh::HPropHandleT<MyMesh::VertexHandle> HPropVhUp1;
	OpenMesh::HPropHandleT<MyMesh::VertexHandle> HPropVhDn1;

	OpenMesh::VPropHandleT<int> VPropRibCnt;

	
};


#include "ribMeshGen.h"

ribMeshGen::ribMeshGen(MyMesh *ptr_mesh)
{
	mesh_ = new MyMesh;
	mesh_ = ptr_mesh;
	ribMesh_ = new MyMesh;
	mesh_->add_property(HPropPtUp);
	mesh_->add_property(HPropPtDn);
	mesh_->add_property(HPropVhUp);
	mesh_->add_property(HPropVhDn);
	mesh_->request_face_normals();
	mesh_->request_vertex_normals();
	mesh_->update_face_normals();
	mesh_->update_vertex_normals();
	mesh_->update_normals();
	ribW_ = 0.8;
	ribH_ = 0.8;
	disp_ = -0.0;
}

ribMeshGen::~ribMeshGen()
{

}

void ribMeshGen::ribShellGenerator()
{
	addVert2RibMesh();
	ribPtNum_ = ribMesh_->n_vertices();
	addFaceFromVert(false, true);
	addFaceFromEdge(false, true, true, true);
	addFaceFromFace(true, false);
	addFaceFromMesh();
	closeGaps();
}

void ribMeshGen::ribShellGenerator(string file)
{
	vector<int> edgeSelection;
	readEdgeSelection(file, edgeSelection);

	addVert2RibMesh();
	ribPtNum_ = ribMesh_->n_vertices();
	addFaceFromVert(false, true);
	//addFaceFromEdge(false, true, true, true);
	addFaceFromEdgeSelected(edgeSelection);
	//addFaceFromFace(true, false);
	//addFaceFromMesh();
	//closeGaps();
}

void ribMeshGen::ribMeshGenerator()
{
	addVert2RibMesh();
	addFaceFromVert();
	addFaceFromEdge();
}


void ribMeshGen::setParameters(double w, double h, double d)
{
	ribW_ = w;
	ribH_ = h;
	disp_ = d;
	cout << "parameters : w" << ribW_ << " h " << ribH_ << " d " << disp_ << endl;
}

void ribMeshGen::setParametersFromVolume(double totVol, double t)
{
	disp_ = t;
	double edgeLen = 0.0;
	for (auto eit = mesh_->edges_begin(); eit != mesh_->edges_end(); eit++)
		edgeLen += mesh_->calc_edge_length(eit);
	double faceArea = 0.0;
	for (auto fit = mesh_->faces_begin(); fit != mesh_->faces_end(); fit++)
		faceArea += calcFaceArea(fit);

	double sheVol = faceArea*abs(disp_);
	ribVol_ = totVol - sheVol;
	ribH_ = sqrt(ribVol_ / edgeLen);
	ribW_ = ribH_;
	cout << "parameters : w=" << ribW_ << " h=" << ribH_ << " d=" << disp_ << endl;
}


void ribMeshGen::closeGaps()
{
	// close gaps along boundary halfedge
	MyMesh::HalfedgeHandle he_orig;
	for (auto hit = mesh_->halfedges_begin(); hit != mesh_->halfedges_end(); hit++)
		if (mesh_->is_boundary(hit))
		{
			he_orig = hit;
			break;
		}
	MyMesh::HalfedgeHandle he_curr, he_next;
	he_curr = he_orig;
	do
	{
		he_next = mesh_->next_halfedge_handle(he_curr);
		int frVi = mesh_->from_vertex_handle(he_curr).idx();
		int toVi = mesh_->to_vertex_handle(he_curr).idx();
		MyMesh::VertexHandle frVh = ribMesh_->vertex_handle(frVi + ribPtNum_);
		MyMesh::VertexHandle toVh = ribMesh_->vertex_handle(toVi + ribPtNum_);
		MyMesh::VertexHandle frVhUp = mesh_->property(HPropVhUp, he_curr);
		MyMesh::VertexHandle toVhUp = mesh_->property(HPropVhUp, he_next);

		ribMesh_->add_face(frVh, toVh, toVhUp, frVhUp);
		he_curr = mesh_->next_halfedge_handle(he_curr);

		MyMesh::HalfedgeHandle he_oppo_next;
		he_oppo_next = mesh_->next_halfedge_handle(mesh_->opposite_halfedge_handle(he_curr));
		OpenMesh::Vec3d dir = mesh_->calc_edge_vector(he_oppo_next).normalized();
		MyMesh::Point oriPos = ribMesh_->point(frVh);
		MyMesh::Point newPos = oriPos + ribW_ / 2.0 * dir;
		ribMesh_->set_point(frVh, newPos);

	} while (he_curr != he_orig);
}

void ribMeshGen::addFaceFromMesh()
{
	int nV = ribMesh_->n_vertices();
	for (auto vit = mesh_->vertices_begin(); vit != mesh_->vertices_end(); vit++)
	{
		ribMesh_->add_vertex(mesh_->point(vit));
	}
	for (auto fit = mesh_->faces_begin(); fit != mesh_->faces_end(); fit++)
	{
		vector<MyMesh::VertexHandle> face;
		for (auto fvit = mesh_->fv_begin(fit); fvit != mesh_->fv_end(fit); fvit++)
		{
			face.push_back(ribMesh_->vertex_handle(nV + fvit->idx()));
		}
		ribMesh_->add_face(face);
	}
}

void ribMeshGen::addFaceFromFace(bool addUp /*= true*/, bool addDn /*= true*/)
{
	for (auto fit = mesh_->faces_begin(); fit != mesh_->faces_end(); fit++)
	{
		vector<MyMesh::VertexHandle> upF;
		vector<MyMesh::VertexHandle> dnF;
		for (auto fhit = mesh_->fh_begin(fit); fhit != mesh_->fh_end(fit); fhit++)
		{
			upF.push_back(mesh_->property(HPropVhUp, fhit));
			dnF.push_back(mesh_->property(HPropVhDn, fhit));
		}
		if (addUp)
		{
			reverse(upF.begin(), upF.end());
			ribMesh_->add_face(upF);
		}
		if (addDn) ribMesh_->add_face(dnF);
	}
}

void ribMeshGen::addFaceFromEdge(bool addUp /*= true*/, bool addDn /*= true*/, bool addLf /*= true*/, bool addRt /*= true*/)
{
	for (auto eit = mesh_->edges_begin(); eit != mesh_->edges_end(); eit++)
	{
		MyMesh::HalfedgeHandle he = mesh_->halfedge_handle(eit, 0);
		MyMesh::VertexHandle frV = mesh_->from_vertex_handle(he);
		MyMesh::VertexHandle toV = mesh_->to_vertex_handle(he);

		MyMesh::HalfedgeHandle he_next = mesh_->next_halfedge_handle(he);
		MyMesh::HalfedgeHandle he_oppo = mesh_->opposite_halfedge_handle(he);
		MyMesh::HalfedgeHandle he_oppo_next = mesh_->next_halfedge_handle(he_oppo);
		MyMesh::VertexHandle upV[4];
		MyMesh::VertexHandle dnV[4];
		upV[0] = mesh_->property(HPropVhUp, he);
		dnV[0] = mesh_->property(HPropVhDn, he);
		upV[1] = mesh_->property(HPropVhUp, he_next);
		dnV[1] = mesh_->property(HPropVhDn, he_next);
		upV[2] = mesh_->property(HPropVhUp, he_oppo);
		dnV[2] = mesh_->property(HPropVhDn, he_oppo);
		upV[3] = mesh_->property(HPropVhUp, he_oppo_next);
		dnV[3] = mesh_->property(HPropVhDn, he_oppo_next);

		vector<MyMesh::VertexHandle> upF;
		vector<MyMesh::VertexHandle> dnF;
		vector<MyMesh::VertexHandle> heF;
		vector<MyMesh::VertexHandle> opF;
		upF.push_back(upV[3]); upF.push_back(upV[2]); upF.push_back(upV[1]); upF.push_back(upV[0]);
		dnF.push_back(dnV[0]); dnF.push_back(dnV[1]); dnF.push_back(dnV[2]); dnF.push_back(dnV[3]);
		heF.push_back(upV[0]); heF.push_back(upV[1]); heF.push_back(dnV[1]); heF.push_back(dnV[0]);
		opF.push_back(upV[2]); opF.push_back(upV[3]); opF.push_back(dnV[3]); opF.push_back(dnV[2]);
		if (addUp) ribMesh_->add_face(upF);
		if (addDn) ribMesh_->add_face(dnF);
		if (addLf) ribMesh_->add_face(heF);
		if (addRt) ribMesh_->add_face(opF);
	}
}

void ribMeshGen::addFaceFromVert(bool addUp /*= true*/, bool addDn /*= true*/)
{
	for (auto vit = mesh_->vertices_begin(); vit != mesh_->vertices_end(); vit++)
	{
		vector<MyMesh::VertexHandle> upF;
		vector<MyMesh::VertexHandle> dnF;
		for (auto vhit = mesh_->voh_begin(vit); vhit != mesh_->voh_end(vit); vhit++)
		{
			upF.push_back(mesh_->property(HPropVhUp, vhit));
			dnF.push_back(mesh_->property(HPropVhDn, vhit));
		}
		if (addUp)
		{
			reverse(upF.begin(), upF.end());
			ribMesh_->add_face(upF);
		}
		if (addDn)
			ribMesh_->add_face(dnF);
	}
}

void ribMeshGen::addVert2RibMesh()
{
	for (auto fit = mesh_->faces_begin(); fit!= mesh_->faces_end(); fit++)
		calcRibMeshPoint(fit);

	// process boundary halfedge
	MyMesh::HalfedgeHandle he_orig;
	for (auto hit=mesh_->halfedges_begin(); hit != mesh_->halfedges_end(); hit++)
		if (mesh_->is_boundary(hit))
		{
			he_orig = hit;
			break;
		}
	MyMesh::HalfedgeHandle he_curr, he_prev;
	he_curr = he_orig;
	do 
	{
		he_prev = mesh_->prev_halfedge_handle(he_curr);
		OpenMesh::Vec3d dir_curr = mesh_->calc_edge_vector(he_curr).normalized();
		OpenMesh::Vec3d dir_prev = -mesh_->calc_edge_vector(he_prev).normalized();
		//OpenMesh::Vec3d dir_disp = (dir_curr + dir_prev).normalized();
		OpenMesh::Vec3d dir_disp = mesh_->calc_edge_vector(mesh_->prev_halfedge_handle(mesh_->opposite_halfedge_handle(he_prev))).normalized();
		double sinAng = (dir_curr % dir_disp).norm();
		//double disp = ribW_ / 2.0 / sinAng;
		double disp = ribW_ / 2.0;
		auto fromVh = mesh_->from_vertex_handle(he_curr);
		MyMesh::Point   pt_ori = mesh_->point(fromVh);
		MyMesh::Point   pt_off = pt_ori + disp*dir_disp;
		OpenMesh::Vec3d pt_nor = mesh_->calc_vertex_normal(fromVh).normalized();
		MyMesh::Point   ptUp = pt_off + disp_ * pt_nor;
		MyMesh::Point   ptDn = pt_off - ribH_ * pt_nor + disp_ * pt_nor;

		mesh_->property(HPropVhUp, he_curr) = ribMesh_->add_vertex(ptUp);
		mesh_->property(HPropPtUp, he_curr) = ptUp;
		mesh_->property(HPropVhDn, he_curr) = ribMesh_->add_vertex(ptDn);
		mesh_->property(HPropPtDn, he_curr) = ptDn;
		he_curr = mesh_->next_halfedge_handle(he_curr);
	} 
	while (he_curr != he_orig);
}

void ribMeshGen::calcRibMeshPoint(MyMesh::FaceHandle fh)
{
	for (auto fhit = mesh_->fh_begin(fh); fhit != mesh_->fh_end(fh); fhit++)
	{
		MyMesh::HalfedgeHandle he_curr = fhit.handle();
		MyMesh::HalfedgeHandle he_prev = mesh_->prev_halfedge_handle(he_curr);
		OpenMesh::Vec3d dir_curr =  mesh_->calc_edge_vector(he_curr);
		OpenMesh::Vec3d dir_prev = -mesh_->calc_edge_vector(he_prev);
		dir_curr.normalize();
		dir_prev.normalize();
		OpenMesh::Vec3d dir_disp = (dir_curr + dir_prev).normalized();
		double sinAng = (dir_disp % dir_curr).norm();
		double disp = ribW_ / 2.0 / sinAng;
		auto fromVh = mesh_->from_vertex_handle(he_curr);
		MyMesh::Point   pt_ori = mesh_->point(fromVh);
		MyMesh::Point   pt_off = pt_ori + disp*dir_disp;
		OpenMesh::Vec3d pt_nor = mesh_->calc_vertex_normal(fromVh).normalized();
		MyMesh::Point   ptUp = pt_off + disp_ * pt_nor;
		MyMesh::Point   ptDn = pt_off - ribH_ * pt_nor + disp_ * pt_nor;

		mesh_->property(HPropVhUp, he_curr) = ribMesh_->add_vertex(ptUp);
		mesh_->property(HPropPtUp, he_curr) = ptUp;
		mesh_->property(HPropVhDn, he_curr) = ribMesh_->add_vertex(ptDn);
		mesh_->property(HPropPtDn, he_curr) = ptDn;
	}
}

double ribMeshGen::calcFaceArea(MyMesh::FaceHandle f)
{
	double area = 0.0;
	MyMesh::Point cen = mesh_->calc_face_centroid(f);
	for (auto fhit = mesh_->fh_begin(f); fhit != mesh_->fh_end(f); fhit++)
	{
		MyMesh::VertexHandle frVh = mesh_->from_vertex_handle(fhit);
		MyMesh::VertexHandle toVh = mesh_->to_vertex_handle(fhit);
		MyMesh::Point frPt = mesh_->point(frVh);
		MyMesh::Point toPt = mesh_->point(toVh);
		area += 0.5*((frPt - cen) % (toPt - cen)).norm();
	}
	return area;
}

void ribMeshGen::addFaceFromEdgeSelected(const vector<int> &eSelected)
{
	bool addUp = false;
	bool addDn = true;
	bool addLf = true;
	bool addRt = true;
	for (int i = 0; i < eSelected.size(); i++)
	{
		MyMesh::EdgeHandle e = mesh_->edge_handle(eSelected.at(i));
		MyMesh::HalfedgeHandle he = mesh_->halfedge_handle(e, 0);
		MyMesh::VertexHandle frV = mesh_->from_vertex_handle(he);
		MyMesh::VertexHandle toV = mesh_->to_vertex_handle(he);

		MyMesh::HalfedgeHandle he_next = mesh_->next_halfedge_handle(he);
		MyMesh::HalfedgeHandle he_oppo = mesh_->opposite_halfedge_handle(he);
		MyMesh::HalfedgeHandle he_oppo_next = mesh_->next_halfedge_handle(he_oppo);
		MyMesh::VertexHandle upV[4];
		MyMesh::VertexHandle dnV[4];
		upV[0] = mesh_->property(HPropVhUp, he);
		dnV[0] = mesh_->property(HPropVhDn, he);
		upV[1] = mesh_->property(HPropVhUp, he_next);
		dnV[1] = mesh_->property(HPropVhDn, he_next);
		upV[2] = mesh_->property(HPropVhUp, he_oppo);
		dnV[2] = mesh_->property(HPropVhDn, he_oppo);
		upV[3] = mesh_->property(HPropVhUp, he_oppo_next);
		dnV[3] = mesh_->property(HPropVhDn, he_oppo_next);

		vector<MyMesh::VertexHandle> upF;
		vector<MyMesh::VertexHandle> dnF;
		vector<MyMesh::VertexHandle> heF;
		vector<MyMesh::VertexHandle> opF;
		upF.push_back(upV[3]); upF.push_back(upV[2]); upF.push_back(upV[1]); upF.push_back(upV[0]);
		dnF.push_back(dnV[0]); dnF.push_back(dnV[1]); dnF.push_back(dnV[2]); dnF.push_back(dnV[3]);
		heF.push_back(upV[0]); heF.push_back(upV[1]); heF.push_back(dnV[1]); heF.push_back(dnV[0]);
		opF.push_back(upV[2]); opF.push_back(upV[3]); opF.push_back(dnV[3]); opF.push_back(dnV[2]);
		if (addUp) ribMesh_->add_face(upF);
		if (addDn) ribMesh_->add_face(dnF);
		if (addLf) ribMesh_->add_face(heF);
		if (addRt) ribMesh_->add_face(opF);
	}
}

void ribMeshGen::readEdgeSelection(string file, vector<int> &eSelected)
{
	eSelected.clear();
	vector<pair<int, int>> vvSelected; // vertex pair
	std::ifstream inFile;
	inFile.open(file.data());
	if (!inFile.is_open())
	{
		std::cout << "warning [can't open file]" << std::endl;
		return;
	}
	//assert(inRibs.is_open());

	std::string line;
	std::stringstream lineInput;
	int v1, v2;
	while (getline(inFile, line))
	{
		lineInput.str("");
		lineInput.clear();
		lineInput << line;
		lineInput >> v1;
		getline(inFile, line);
		lineInput.str("");
		lineInput.clear();
		lineInput << line;
		lineInput >> v2;
		vvSelected.push_back(make_pair(v1, v2));
	}
	inFile.close();


	for (auto eit = mesh_->edges_begin(); eit != mesh_->edges_end(); eit++)
	{
		MyMesh::HalfedgeHandle he = mesh_->halfedge_handle(eit, 0);
		MyMesh::VertexHandle frV = mesh_->from_vertex_handle(he);
		MyMesh::VertexHandle toV = mesh_->to_vertex_handle(he);
		pair<int, int> vv = make_pair(frV.idx(), toV.idx());
		vector<pair<int, int>>::iterator result = find(vvSelected.begin(), vvSelected.end(), vv);
		if (result != vvSelected.end())
			eSelected.push_back(eit->idx());
	}
}

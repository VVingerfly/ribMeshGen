#include "ribShellGen.h"

ribShellGen::ribShellGen(MyMesh *mesh, string file)
{
	mesh_ = new MyMesh;
	mesh_ = mesh;
	ribShell_ = new MyMesh;
	mesh_->add_property(HPropPtUp);
	mesh_->add_property(HPropPtDn);
	mesh_->add_property(HPropVhUp);
	mesh_->add_property(HPropVhDn);
	mesh_->add_property(VPropRibCnt);
	mesh_->add_property(HPropVhUp0);
	mesh_->add_property(HPropVhDn0);
	mesh_->add_property(HPropVhUp1);
	mesh_->add_property(HPropVhDn1);

	mesh_->request_face_normals();
	mesh_->request_vertex_normals();
	mesh_->update_face_normals();
	mesh_->update_vertex_normals();
	mesh_->update_normals();
	mesh_->request_edge_status();
	ribW_ = 0.8;
	ribH_ = 0.8;
	disp_ = 0.0;
	initProp();

	vector<int> eSelected;
	readEdgeSelection(file, eSelected);
	initEdgeSelection(eSelected);
}

ribShellGen::~ribShellGen()
{
}



void ribShellGen::generator()
{
	addVertFromVert();

	calcEdgeVert();
	addFaceFromEdge(true, true, true, true);

}

void ribShellGen::setParameters(double w, double h, double d)
{
	ribW_ = w;
	ribH_ = h;
	disp_ = d;
	cout << "parameters : w=" << ribW_ << " h=" << ribH_ << " d=" << disp_ << endl;
}

void ribShellGen::setParametersFromVolume(double totVol, double t)
{
	disp_ = t;

	double faceArea = 0.0;
	for (auto fit = mesh_->faces_begin(); fit != mesh_->faces_end(); fit++)
		faceArea += calcFaceArea(fit);

	double edgeLen = 0.0;
	for (auto eit = mesh_->edges_begin(); eit != mesh_->edges_end(); eit++)
	{
		if (!mesh_->status(eit).selected()) continue;
		edgeLen += mesh_->calc_edge_length(eit);
	}

	double sheVol = faceArea*abs(disp_);
	ribVol_ = totVol - sheVol;
	ribH_ = sqrt(ribVol_ / edgeLen);
	ribW_ = ribH_;


	ribH_ = ribH_ + abs(t);
	disp_ = 0;


	cout << "parameters : w=" << ribW_ << " h=" << ribH_ << " d=" << disp_ << endl;
}

void ribShellGen::readEdgeSelection(string file, vector<int> &eSelected)
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

void ribShellGen::initEdgeSelection(const vector<int> &selection)
{
	for (auto eit = mesh_->edges_begin(); eit != mesh_->edges_end(); eit++)
		mesh_->status(eit).set_selected(false);
	for (int i = 0; i < selection.size(); i++)
	{
		MyMesh::EdgeHandle e = mesh_->edge_handle(selection.at(i));
		mesh_->status(e).set_selected(true);
	}

}

void ribShellGen::addVertFromVert()
{
	for (auto vit = mesh_->vertices_begin(); vit != mesh_->vertices_end(); vit++)
	{
		mesh_->property(VPropRibCnt, vit) = 0;
		vector<MyMesh::HalfedgeHandle> vhSelected;
		for (auto vhit = mesh_->voh_begin(vit); vhit != mesh_->voh_end(vit); vhit++)
			if (mesh_->status(mesh_->edge_handle(vhit)).selected())
				vhSelected.push_back(vhit);

		mesh_->property(VPropRibCnt, vit) = vhSelected.size();
		if (vhSelected.size() > 2)
		{
			for (int i = 0; i < vhSelected.size(); i++)
			{
				MyMesh::HalfedgeHandle he_curr = vhSelected.at(i);
				MyMesh::HalfedgeHandle he_next = (i == 0 ? vhSelected.back() : vhSelected.at(i-1));
				OpenMesh::Vec3d dir_curr = mesh_->calc_edge_vector(he_curr).normalized();
				OpenMesh::Vec3d dir_next = mesh_->calc_edge_vector(he_next).normalized();
				OpenMesh::Vec3d dir_disp = (dir_curr + dir_next).normalized();
				double sinAng = (dir_disp % dir_curr).norm();
				double len_disp = ribW_ / 2.0 / sinAng;
				auto fromVh = mesh_->from_vertex_handle(he_curr);
				MyMesh::Point   pt_ori = mesh_->point(fromVh);
				MyMesh::Point   pt_off = pt_ori + len_disp*dir_disp;
				OpenMesh::Vec3d pt_nor = mesh_->calc_vertex_normal(fromVh).normalized();
				MyMesh::Point   ptUp = pt_off + disp_ * pt_nor;
				MyMesh::Point   ptDn = pt_off - ribH_ * pt_nor + disp_ * pt_nor;

				mesh_->property(HPropVhUp, he_curr) = ribShell_->add_vertex(ptUp);
				mesh_->property(HPropPtUp, he_curr) = ptUp;
				mesh_->property(HPropVhDn, he_curr) = ribShell_->add_vertex(ptDn);
				mesh_->property(HPropPtDn, he_curr) = ptDn;
			}
			
		}
		if (vhSelected.size() == 2)
		{
			MyMesh::HalfedgeHandle he_0 = vhSelected.at(0);
			MyMesh::HalfedgeHandle he_1 = vhSelected.at(1);
			OpenMesh::Vec3d dir_0 = mesh_->calc_edge_vector(he_0).normalized();
			OpenMesh::Vec3d dir_1 = mesh_->calc_edge_vector(he_1).normalized();
			auto fromVh = mesh_->from_vertex_handle(he_0);
			MyMesh::Point   pt_ori = mesh_->point(fromVh);
			OpenMesh::Vec3d pt_nor = mesh_->calc_vertex_normal(fromVh).normalized();
			OpenMesh::Vec3d dir_disp_0 = ((dir_1 - dir_0) % pt_nor).normalized();
			OpenMesh::Vec3d dir_disp_1 = ((dir_0 - dir_1) % pt_nor).normalized();
			double len_disp = ribW_ / 2.0;
			MyMesh::Point   pt_off_0 = pt_ori + len_disp*dir_disp_0;
			MyMesh::Point   pt_off_1 = pt_ori + len_disp*dir_disp_1;

			MyMesh::Point   ptUp_0 = pt_off_0 + disp_ * pt_nor;
			MyMesh::Point   ptDn_0 = pt_off_0 - ribH_ * pt_nor + disp_ * pt_nor;
			MyMesh::Point   ptUp_1 = pt_off_1 + disp_ * pt_nor;
			MyMesh::Point   ptDn_1 = pt_off_1 - ribH_ * pt_nor + disp_ * pt_nor;

			mesh_->property(HPropVhUp, he_0) = ribShell_->add_vertex(ptUp_0);
			mesh_->property(HPropPtUp, he_0) = ptUp_0;
			mesh_->property(HPropVhDn, he_0) = ribShell_->add_vertex(ptDn_0);
			mesh_->property(HPropPtDn, he_0) = ptDn_0;

			mesh_->property(HPropVhUp, he_1) = ribShell_->add_vertex(ptUp_1);
			mesh_->property(HPropPtUp, he_1) = ptUp_1;
			mesh_->property(HPropVhDn, he_1) = ribShell_->add_vertex(ptDn_1);
			mesh_->property(HPropPtDn, he_1) = ptDn_1;

		}
		if (vhSelected.size() == 1)
		{
			MyMesh::HalfedgeHandle he = vhSelected.at(0);
			auto fromVh = mesh_->from_vertex_handle(he);
			MyMesh::Point   pt_ori = mesh_->point(fromVh);
			OpenMesh::Vec3d pt_nor = mesh_->calc_vertex_normal(fromVh).normalized();
			OpenMesh::Vec3d dir_disp = (-mesh_->calc_edge_vector(mesh_->prev_halfedge_handle(he))).normalized();
			double len_disp = ribW_ / 2.0;
			MyMesh::Point   pt_off0 = pt_ori + len_disp*dir_disp;
			MyMesh::Point   ptUp0 = pt_off0 + disp_ * pt_nor;
			MyMesh::Point   ptDn0 = pt_off0 - ribH_ * pt_nor + disp_ * pt_nor;
			MyMesh::Point   pt_off1 = pt_ori - len_disp*dir_disp;
			MyMesh::Point   ptUp1 = pt_off1 + disp_ * pt_nor;
			MyMesh::Point   ptDn1 = pt_off1 - ribH_ * pt_nor + disp_ * pt_nor;

			//mesh_->property(HPropVhUp, he) = ribShell_->add_vertex(ptUp);
			//mesh_->property(HPropPtUp, he) = ptUp;
			//mesh_->property(HPropVhDn, he) = ribShell_->add_vertex(ptDn);
			//mesh_->property(HPropPtDn, he) = ptDn;

			mesh_->property(HPropVhUp0, he) = ribShell_->add_vertex(ptUp0);
			mesh_->property(HPropVhDn0, he) = ribShell_->add_vertex(ptDn0);
			mesh_->property(HPropVhUp1, he) = ribShell_->add_vertex(ptUp1);
			mesh_->property(HPropVhDn1, he) = ribShell_->add_vertex(ptDn1);
		}

	}
}

void ribShellGen::addFaceFromEdge(bool up /*= true*/, bool dn /*= true*/, bool lf /*= true*/, bool rt /*= true*/)
{
	for (auto eit = mesh_->edges_begin(); eit != mesh_->edges_end(); eit++)
	{
		if (!mesh_->status(eit).selected()) continue;

		MyMesh::HalfedgeHandle he_curr = mesh_->halfedge_handle(eit, 0);
		MyMesh::HalfedgeHandle he_oppo = mesh_->opposite_halfedge_handle(he_curr);
		MyMesh::VertexHandle   frVh = mesh_->from_vertex_handle(he_curr);
		
		MyMesh::Point          frPt = mesh_->point(frVh);
		OpenMesh::Vec3d        vNor = mesh_->calc_vertex_normal(frVh);
		OpenMesh::Vec3d        vVec = mesh_->calc_edge_vector(he_curr);
		OpenMesh::Vec3d        vDir = vNor % vVec;

		MyMesh::VertexHandle v_curr[4];
		v_curr[0] = mesh_->property(HPropVhUp0, he_curr);
		v_curr[1] = mesh_->property(HPropVhUp1, he_curr);
		v_curr[2] = mesh_->property(HPropVhDn0, he_curr);
		v_curr[3] = mesh_->property(HPropVhDn1, he_curr);
		MyMesh::VertexHandle v_oppo[4];
		v_oppo[0] = mesh_->property(HPropVhUp0, he_oppo);
		v_oppo[1] = mesh_->property(HPropVhUp1, he_oppo);
		v_oppo[2] = mesh_->property(HPropVhDn0, he_oppo);
		v_oppo[3] = mesh_->property(HPropVhDn1, he_oppo);

		vector<MyMesh::VertexHandle> upF, dnF;
		upF.push_back(v_curr[0]); upF.push_back(v_curr[1]); upF.push_back(v_oppo[0]); upF.push_back(v_oppo[1]);
		dnF.push_back(v_curr[2]); dnF.push_back(v_curr[3]); dnF.push_back(v_oppo[2]); dnF.push_back(v_oppo[3]);
		reverse(dnF.begin(), dnF.end());
		if (up) ribShell_->add_face(upF);
		if (dn) ribShell_->add_face(dnF);
		
		if (lf)
		{
			MyMesh::Point p0 = ribShell_->point(v_curr[0]);
			MyMesh::Point p1 = ribShell_->point(v_oppo[1]);
			MyMesh::Point p2 = ribShell_->point(v_oppo[3]);
			OpenMesh::Vec3d fNor = (p0 - p1) % (p0 - p2);
			if ((fNor | vDir) > 0)
				ribShell_->add_face(v_curr[0], v_oppo[1], v_oppo[3], v_curr[2]);
			else
				ribShell_->add_face(v_curr[2], v_oppo[3], v_oppo[1], v_curr[0]);
		}
		if (rt)
		{
			MyMesh::Point p0 = ribShell_->point(v_oppo[0]);
			MyMesh::Point p1 = ribShell_->point(v_curr[1]);
			MyMesh::Point p2 = ribShell_->point(v_curr[3]);
			OpenMesh::Vec3d fNor = (p0 - p1) % (p0 - p2);
			if ((fNor | vDir) < 0)
				ribShell_->add_face(v_oppo[0], v_curr[1], v_curr[3], v_oppo[2]);
			else
				ribShell_->add_face(v_oppo[2], v_curr[3], v_curr[1], v_oppo[0]);
		}
	}
}

void ribShellGen::calcEdgeVert()
{
	MyMesh mesh_test;
	for (auto vit = mesh_->vertices_begin(); vit != mesh_->vertices_end(); vit++)
	{
		int ribCnt = mesh_->property(VPropRibCnt, vit);
		if (ribCnt == 0) continue;

		MyMesh::HalfedgeHandle he_curr, he_orig, he_next;
		for (auto vhit = mesh_->voh_begin(vit); vhit != mesh_->voh_end(vit); vhit++)
			if (mesh_->status(mesh_->edge_handle(vhit)).selected())
			{
				he_orig = vhit;
				break;
			}
		vector<MyMesh::VertexHandle> upF, dnF;
		if (ribCnt > 1)
		{
			he_curr = he_orig;
			do 
			{
				he_next = he_curr;
				while (true)
				{
					he_next = mesh_->next_halfedge_handle(mesh_->opposite_halfedge_handle(he_next));
					if (mesh_->status(mesh_->edge_handle(he_next)).selected())
						break;
				}
				if (!ribShell_->is_valid_handle(mesh_->property(HPropVhUp, he_curr)) || !ribShell_->is_valid_handle(mesh_->property(HPropVhDn, he_curr)))
				{
					cout << vit << " invalid vertex (" << mesh_->property(HPropVhUp, he_curr) << "," << mesh_->property(HPropVhDn, he_curr) <<") from curr" << endl;
				}
				if (!ribShell_->is_valid_handle(mesh_->property(HPropVhUp, he_next)) || !ribShell_->is_valid_handle(mesh_->property(HPropVhDn, he_next)))
				{
					cout << vit << " invalid vertex (" << mesh_->property(HPropVhUp, he_curr) << "," << mesh_->property(HPropVhDn, he_curr) << ") from next" << endl;
				}
				mesh_->property(HPropVhUp0, he_curr) = mesh_->property(HPropVhUp, he_curr);
				mesh_->property(HPropVhDn0, he_curr) = mesh_->property(HPropVhDn, he_curr);
				mesh_->property(HPropVhUp1, he_curr) = mesh_->property(HPropVhUp, he_next);
				mesh_->property(HPropVhDn1, he_curr) = mesh_->property(HPropVhDn, he_next);
				
				upF.push_back(mesh_->property(HPropVhUp, he_curr));
				dnF.push_back(mesh_->property(HPropVhDn, he_curr));
				he_curr = he_next;
			} 
			while (he_curr != he_orig);
			if (ribCnt > 2)
			{
				reverse(upF.begin(), upF.end());
				ribShell_->add_face(upF);
				ribShell_->add_face(dnF);
			}
		}
	}

	return;

	for (auto eit = mesh_->edges_begin(); eit != mesh_->edges_end(); eit++)
	{
		if (mesh_->status(eit).selected())
		{
			MyMesh::HalfedgeHandle he0 = mesh_->halfedge_handle(eit, 1);
			MyMesh::HalfedgeHandle he1 = mesh_->opposite_halfedge_handle(he0);
			MyMesh::VertexHandle v[8];
			v[0] = mesh_->property(HPropVhUp0, he0);
			v[1] = mesh_->property(HPropVhUp1, he0);
			v[2] = mesh_->property(HPropVhDn0, he0);
			v[3] = mesh_->property(HPropVhDn1, he0);
			v[4] = mesh_->property(HPropVhUp0, he1);
			v[5] = mesh_->property(HPropVhUp1, he1);
			v[6] = mesh_->property(HPropVhDn0, he1);
			v[7] = mesh_->property(HPropVhDn1, he1);
			for (int i = 0; i < 8; i++)
			{
				if (!ribShell_->is_valid_handle(v[i]))
				{
					cout << "invalid vertex " << i << " " << v[i].idx() << " from e " << eit << endl;
					
				}
			}
			

		}
	}
		
}

void ribShellGen::initProp()
{
	for (auto eit = mesh_->edges_begin(); eit != mesh_->edges_end(); eit++)
		mesh_->status(eit).set_tagged(false);
}

double ribShellGen::calcFaceArea(MyMesh::FaceHandle f)
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

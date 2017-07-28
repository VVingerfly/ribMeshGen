#pragma once

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <direct.h>  
#include "conio.h"
using std::ifstream;
using std::ofstream;
using std::stringstream;
using std::string;
using std::cout;
using std::endl;

// -------------------- OpenMesh
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/Handles.hh>
#include <OpenMesh/Core/Mesh/Traits.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>

struct PolyTraits : public OpenMesh::DefaultTraits
{
	typedef OpenMesh::Vec3d Point;
	typedef OpenMesh::Vec3d Normal;
};
typedef OpenMesh::PolyMesh_ArrayKernelT<PolyTraits>   MyMesh;
using namespace std;
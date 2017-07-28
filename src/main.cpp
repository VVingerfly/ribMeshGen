#include "ribMeshGen.h"
#include "ribShellGen.h"


void test1()
{
	// surfQuadGrid50  surfSelfSupp50 surfVoroGrid50
	string file = "lilium";
	MyMesh *mesh = new MyMesh;
	OpenMesh::IO::read_mesh(*mesh, file + ".obj");

	cout << "begins..." << endl;
	ribShellGen shellGenerator(mesh, "liliumSelection.txt");
	//shellGenerator.setParameters(0.8, 0.8, -0.4);
	shellGenerator.setParametersFromVolume(6000, -0.4);
	shellGenerator.generator();
	cout << "begins. done." << endl;
}

void test2()
{
	string file = "mVoro";
	MyMesh *mesh = new MyMesh;
	OpenMesh::IO::read_mesh(*mesh, file + ".obj");

	ribMeshGen generator(mesh);
	//generator.setParameters(0.8, 0.8, -0.4);
	generator.setParametersFromVolume(6000, -0.4);
	generator.ribMeshGenerator();
	


	OpenMesh::IO::write_mesh(generator.getRibMesh(), file + "-out.obj");
}

int main(int argc, const char *argv[])
{
	test2();
	
	return 0;

}

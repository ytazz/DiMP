#include "RobotArm.h"
#include "mymodule.h"

int main(int argc, char* argv[]){
	MyModule mod;
	if(!mod.Init(argc, argv))
		return -1;

	mod.MainLoop();
	return 0;
}


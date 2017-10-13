#include <window/viewer.h>
#include <module/rendering.h>
#include <module/simulation.h>
#include <module/request.h>
#include <module/module.h>

///////////////////////////////////////////////////////////////////////////////////////////////////

Viewer::Viewer(GLWin::Window* p):GLWin::Viewer(p){
	
}

bool Viewer::OnEvent(SDL_Event* ev){
	Module* mod = Module::Get();
	RenderingManager* renManager = mod->renManager;

	return renManager->curCamera->OnEvent(ev);
}
	
void Viewer::DrawView(){
	Module::Get()->renManager->DrawScene();
}

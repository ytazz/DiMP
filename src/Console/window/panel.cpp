#include <window/panel.h>
#include <module/request.h>
#include <module/module.h>

#include <SDL.h>

///////////////////////////////////////////////////////////////////////////////////////////////////

Panel::Panel(GLWin::Window* p):GLWin::Window(p){
}

bool Panel::Init(){
	Module* mod = Module::Get();

	if( !(btnPlan = (GLWin::Button*)FindChild("btn_plan")) ) return false;

	btnPlan->AddCallback(this);
	
	return true;
}

bool Panel::OnUpdate(){
	Module* mod = Module::Get();
	CriticalSection _cs(&mod->cs);

	return true;
}

bool Panel::OnEvent(GLWin::Window* win, int code){
	Module* mod = Module::Get();
	
	if(code == GLWin::Button::Clicked){
		stringstream ss;
	
		if(win == btnPlan){
			ss << (btnPlan->onoff ? "start plan" : "stop plan");
		}
	
		mod->reqManager->Query(ss.str());
		return true;
	}
	return false;

}

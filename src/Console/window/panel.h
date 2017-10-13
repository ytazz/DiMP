#pragma once

#include <base/typedefs.h>

#include <glwin/glwin.h>

class Panel : public GLWin::Window, public GLWin::WindowCallback{
public:
	GLWin::Button*            btnPlan;
	
public:
	virtual bool Init    ();
	virtual bool OnUpdate();
	virtual bool OnEvent (Window* win, int code);
	
	Panel(Window* p);
};

#pragma once

#include <base/typedefs.h>

#include <glwin/glwin.h>

class Viewer : public GLWin::Viewer, public GLWin::WindowCallback{
public:
	
public:
	virtual void DrawView();
	virtual bool OnEvent (SDL_Event* ev);
	
	Viewer(Window* p);
};

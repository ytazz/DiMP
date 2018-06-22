#pragma once

#include <base/typedefs.h>
#include <glwin/glwin.h>

#include <DiMP/Render/Canvas.h>

#include <sbmessage.h>

class Viewer;
class Panel ;

class RenderingManager : public GLWin::WindowManager, public Scenebuilder::MessageCallback{
public:
	int	 timerPeriod;		///< �`��^�C�}����[ms]
	int  consolePosX;
	int  consolePosY;
	int  consoleWidth;
	int  consoleHeight;
	int  consoleFontSize;
	int  consoleMessageRow;
	int  consoleMessageCol;
	int  consoleMessageDepth;
	int  consoleInputRow;

	deque<string>  msgLines;
    deque<string>  infoLines;
	int            timeGL;
	int            timeText;

	UTRef<DiMP::Render::CanvasGL >   canvasGL;
	UTRef<DiMP::Render::CanvasSVG>   canvasSVG;

	Event       evResize;		///< �E�B���h�E�T�C�Y�ύX   
	Event		evDraw;			///< �`��
	Event		evUpdate;		///< GUI�X�V

	SDL_Window*         sdlWindow;
	SDL_GLContext       glContext;

	GLWin::Viewer*  viewer;
	GLWin::Button*  btnPlan;

public:	
	void Read      (XML& xml);
	void OnEvent   (SDL_Event* ev);
	void Handle    ();
	void DrawScene ();
	void DrawText  (); 
	
	virtual bool Init     ();
	virtual bool OnEvent  (GLWin::Window* win, int code);
	virtual void OnMessage(int lv, const char* str);
	
	 RenderingManager();
	~RenderingManager();
};

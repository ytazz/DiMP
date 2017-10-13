#pragma once

#include <base/typedefs.h>
#include <glwin/glwin.h>

#include <DiMP/Render/Canvas.h>

#include <sbmessage.h>

class Viewer;
class Panel ;

class RenderingManager : public GLWin::WindowManager, public Scenebuilder::MessageCallback{
public:
	struct View{
		enum{
			Mode2D,
			Mode3D,
		};
	};

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

	UTRef<GLWin::Camera> cam2D;
	UTRef<GLWin::Camera> cam3D;
	GLWin::Camera*       curCamera;

	UTRef<DiMP::Render::CanvasGL >   canvasGL;
	UTRef<DiMP::Render::CanvasSVG>   canvasSVG;

	Viewer*		viewer;
	Panel *     panel ;
	
	Event       evResize;		///< �E�B���h�E�T�C�Y�ύX   
	Event		evDraw;			///< �`��
	Event		evUpdate;		///< GUI�X�V

	SDL_Window*         sdlWindow;
	SDL_GLContext       glContext;

public:	
	void Read      (XML& xml);
	void OnEvent   (SDL_Event* ev);
	void Handle    ();
	void DrawScene ();
	void DrawText  (); 
	
	virtual GLWin::Window* CreateWindow(string type, GLWin::Window* par);
	virtual bool	       Init        ();
	virtual void           OnMessage   (int lv, const char* str);
	
	 RenderingManager();
	~RenderingManager();
};

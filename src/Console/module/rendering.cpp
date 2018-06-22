#include <window/panel.h>
#include <module/rendering.h>
#include <module/module.h>

#include <sbconsole.h>

#include <Foundation/UTPreciseTimer.h>
static UTPreciseTimer ptimer;

RenderingManager::RenderingManager(){
	timerPeriod	        = 100;
	consolePosX         = -1;
	consolePosY         = -1;
	consoleWidth        = 80;
	consoleHeight       = 60;
	consoleFontSize     = 10;
	consoleMessageRow   = 0;
	consoleMessageCol   = 60;
	consoleMessageDepth = 5;
	consoleInputRow     = 50;
}

RenderingManager::~RenderingManager(){

}

void RenderingManager::Read(XML& xml){
	XMLNode* renNode = xml.GetRootNode()->GetNode("rendering", 0, false);
	if(!renNode){
		Message::Error("rendering manager: xml node not found");
		return;
	}
	renNode->Get(timerPeriod        , ".timer_period");
	renNode->Get(consolePosX        , ".console_pos_x"        );
	renNode->Get(consolePosY        , ".console_pos_y"        );
	renNode->Get(consoleWidth       , ".console_width"        );
	renNode->Get(consoleHeight      , ".console_height"       );
	renNode->Get(consoleFontSize    , ".console_font_size"    );
	renNode->Get(consoleMessageRow  , ".console_message_row"  );
	renNode->Get(consoleMessageCol  , ".console_message_col"  );
	renNode->Get(consoleMessageDepth, ".console_message_depth");
	renNode->Get(consoleInputRow    , ".console_input_row"    );

	WindowManager::Read(xml.GetRootNode()->GetNode("window_manager"));
}

bool RenderingManager::Init(){
	Module* mod = Module::Get();

	// コンソール設定
	Scenebuilder::Console::SetFontSize      (consoleFontSize);
	Scenebuilder::Console::SetBufferSize    (consoleHeight, consoleWidth);
	Scenebuilder::Console::SetDisplaySize   (consoleHeight, consoleWidth);
	Scenebuilder::Console::SetWindowPosition(consolePosX, consolePosY);
	
	// glew初期化
	glewInit();

	// glut初期化
	int   argc   = 1;
	char* argv[] = {"", NULL};
	glutInit(&argc, argv);
	
	// レンダラ作成
	render = mod->grSdk->CreateRender();
	device = mod->grSdk->CreateDeviceGL();
	device->Init();
	render->SetDevice(device);
	//Message::Out("OpenGL initialized as version %d.%d", device->GetGLMajorVersion(), device->GetGLMinorVersion());
	
	render->Reshape(Vec2f(), Vec2f(windowWidth, windowHeight));

	canvasGL  = new DiMP::Render::CanvasGL ();
	canvasSVG = new DiMP::Render::CanvasSVG();

	evUpdate.Set();

	return WindowManager::Init();
}

void RenderingManager::Handle(){
	Module* mod = Module::Get();

	if(evResize.IsSet()){
		root->SetSize((float)windowWidth, (float)windowHeight);
	}
	if(evDraw.IsSet()){
		static bool first = true;
		if(first){
			evResize.Set();
			first = false;
		}
		Step(0.001f * timerPeriod);
		WindowManager::Draw();
		SDL_GL_SwapWindow(sdlWindow);

		// コンソールに情報出力
		ptimer.CountUS();
		infoLines.clear();
		mod->OnPrint(infoLines);
		
		Scenebuilder::Console::Fill(0, 0, consoleWidth, consoleInputRow, ' ');
		for(uint i = 0; i < infoLines.size(); i++)
			Scenebuilder::Console::Write(0, i, infoLines[i]);
		
		for(uint i = 0; i < msgLines.size(); i++)
			Scenebuilder::Console::Write(consoleMessageCol, consoleMessageRow + i, msgLines[i]); 

		Scenebuilder::Console::Refresh(0, consoleInputRow);
		timeText = ptimer.CountUS();
	}
	if(evUpdate.IsSet()){
		OnUpdate();
	}
}

void RenderingManager::OnEvent(SDL_Event* ev){
	if(ev->type == SDL_USEREVENT){
		if(ev->user.code == (int)this){
			evDraw.Set();
		}
	}
	if(ev->type == SDL_WINDOWEVENT && ev->window.event == SDL_WINDOWEVENT_RESIZED){
		windowWidth  = ev->window.data1;
		windowHeight = ev->window.data2;
		evResize.Set();
	}
	WindowManager::OnEvent(ev);
}

GLWin::Window* RenderingManager::CreateWindow(string type, GLWin::Window* par){
	if(type == "panel"){
		panel = new Panel(par);
		return panel;
	}

	return GLWin::WindowManager::CreateWindow(type, par);
}

void RenderingManager::DrawScene(){
	Module* mod = Module::Get();
	CriticalSection _cs(&mod->cs);

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_BLEND);

	mod->OnDraw(canvasGL);
}

void RenderingManager::OnMessage(int lv, const char* str){
	string line(str);
	if(line.back() == '\n')
		line.pop_back();

	msgLines.push_back(line);
	if((int)msgLines.size() > consoleMessageDepth)
		msgLines.pop_front();
}

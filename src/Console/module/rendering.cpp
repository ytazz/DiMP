#include <module/rendering.h>
#include <module/request.h>
#include <module/module.h>

#include <sbconsole.h>

#include <sbtimer.h>
static Timer timer;

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

    joystick = 0;
    fill(joystickValues, joystickValues + JoystickAxis::End, 0);
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

    //
    joystick = SDL_JoystickOpen(0);
    if(joystick){
		Message::Out("joystick(gamepad) detected: %s", SDL_JoystickName(joystick));
		Message::Out("num of axes: %d", SDL_JoystickNumAxes   (joystick));
		Message::Out("num of btns: %d", SDL_JoystickNumButtons(joystick));
	}
    SDL_JoystickEventState(SDL_ENABLE);

	canvasGL  = new DiMP::Render::CanvasGL ();
	canvasSVG = new DiMP::Render::CanvasSVG();

	evUpdate.Set();

	viewer = (GLWin::Viewer*)root->FindChild("viewer");
	if(!viewer){
		Message::Error("viewer not found");
		return false;
	}
	viewer->AddCallback(this);	
	
	btnPlan = (GLWin::Button*)root->FindChild("btn_plan");
	if(!btnPlan){
		Message::Error("btn_plan not found");
		return false;
	}
	btnPlan->AddCallback(this);	

	return WindowManager::Init();
}

void RenderingManager::Handle(){
	Module* mod = Module::Get();

	if(evResize.IsSet()){
		//cout << "ren resize" << endl;
		root->SetSize((float)windowWidth, (float)windowHeight);
	}
	if(evDraw.IsSet()){
		//cout << "ren draw" << endl;
		static bool first = true;
		if(first){
			evResize.Set();
			first = false;
		}
		Step(0.001f * timerPeriod);
		WindowManager::Draw();
		SDL_GL_SwapWindow(sdlWindow);

		// コンソールに情報出力
		timer.CountUS();
		infoLines.clear();
		mod->OnPrint(infoLines);
		
		Scenebuilder::Console::Fill(0, 0, consoleWidth, consoleInputRow, ' ');
		for(uint i = 0; i < infoLines.size(); i++)
			Scenebuilder::Console::Write(0, i, infoLines[i]);
		
		for(uint i = 0; i < msgLines.size(); i++)
			Scenebuilder::Console::Write(consoleMessageCol, consoleMessageRow + i, msgLines[i]); 

		Scenebuilder::Console::Refresh(0, consoleInputRow);
		timeText = timer.CountUS();
	}
	if(evUpdate.IsSet()){
		//cout << "ren update" << endl;
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

    switch(ev->type){
	case SDL_JOYAXISMOTION:{
		int idx = ev->jaxis.axis;
		int val = ev->jaxis.value;
		
		int axis = -1;
		if(idx == 0) axis = JoystickAxis::H1;
		if(idx == 1) axis = JoystickAxis::V1;
		if(idx == 2) axis = JoystickAxis::H2;
		if(idx == 3) axis = JoystickAxis::V2;
		if(idx == 4) axis = JoystickAxis::T1;
		if(idx == 5) axis = JoystickAxis::T2;
		if(axis == -1)
			break;

		// 符号付
		joystickValues[axis] = val;

		}
		break;
	case SDL_JOYBUTTONDOWN:{
		int idx = ev->jbutton.button;
		if(idx < 0 || idx >= JoystickButton::End)
			break;
		joystickValues[idx] = 1;
		}
		break;
	case SDL_JOYBUTTONUP:{	
		int idx = ev->jbutton.button;
		if(idx < 0 || idx >= JoystickButton::End)
			break;
		joystickValues[idx] = 0;
		}
		break;
	}

    WindowManager::OnEvent(ev);
}

bool RenderingManager::OnEvent(GLWin::Window* win, int code){
	Module* mod = Module::Get();

	Affinef aff, affInv;

	if(viewer == win && code == GLWin::Viewer::Event::Draw){
		Module* mod = Module::Get();
		CriticalSection _cs(&mod->cs);

		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glEnable(GL_BLEND);

		mod->OnDraw(canvasGL);

		return true;
	}
	if(btnPlan == win && code == GLWin::Button::Clicked){
		stringstream ss;
		ss << (btnPlan->onoff ? "start plan" : "stop plan");
		mod->reqManager->Query(ss.str());
	}
	return false;
}

void RenderingManager::OnMessage(int lv, const char* str){
	string line(str);
	if(line.back() == '\n')
		line.pop_back();

	msgLines.push_back(line);
	if((int)msgLines.size() > consoleMessageDepth)
		msgLines.pop_front();
}

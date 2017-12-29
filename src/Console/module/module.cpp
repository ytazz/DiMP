#include <window/viewer.h>
#include <module/rendering.h>
#include <module/request.h>
#include <module/simulation.h>
#include <module/module.h>

#include <Foundation/UTPreciseTimer.h>
static UTPreciseTimer ptimer;
static UTPreciseTimer ptimer1;

///////////////////////////////////////////////////////////////////////////////////////////////////

Uint32 TimerCallback(Uint32 interval, void *param){
	SDL_Event ev;
	ev.type = SDL_USEREVENT;
	ev.user.code = (int)param;
	SDL_PushEvent(&ev);
	return interval;
}

Module* Module::instance = 0;

Module* Module::Get(){
	return Module::instance;
}

Module::Module(){
	instance = this;

	Message::SetVerboseLevel(Message::Level::Normal);

	graph         = new DiMP::Graph      ();
	iterCount     = 0;
	compTime      = 0;
	compTimeTotal = 0;
	deltaNorm     = 0.0;
	isPlaying     = true;
	playTime      = 0.0;
	
	renManager = new RenderingManager ();
	reqManager = new RequestManager   ();
	simManager = new SimulationManager();

	// タスク開始・停止
	reqManager->Add("start")->AddArg("task", ArgType::String);
	reqManager->Add("stop" )->AddArg("task", ArgType::String);

	// 描画設定
	reqManager->Add("show")->AddArg("item", ArgType::String);
	reqManager->Add("show")->AddArg("item", ArgType::String)->AddArg("index", ArgType::Int);
	reqManager->Add("hide")->AddArg("item", ArgType::String);
	reqManager->Add("hide")->AddArg("item", ArgType::String)->AddArg("index", ArgType::Int);

	// 視点設定
	reqManager->Add("view")->AddArg("mode", ArgType::String);

	evExit.Create(true);	//< 終了イベントはマニュアルリセット
}

Module::~Module(){

}

bool Module::Init(int argc, char* argv[]){
	// メッセージ出力バイパス
	Message::SetStream  (0);
	Message::SetCallback(renManager);
	
	// コンフィグロード
	XML xml;
	xml.Load("conf/console.xml");
	renManager->Read(xml);
	simManager->Read(xml);
	
	// SDL初期化
	if(SDL_Init(SDL_INIT_TIMER | SDL_INIT_VIDEO | SDL_INIT_AUDIO | SDL_INIT_JOYSTICK) == -1){
		Message::Error("failed to initialize SDL");
		return false;
	}
	// グラフィクス初期化
	SDL_GL_SetAttribute(SDL_GL_RED_SIZE    ,  8);
    SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE  ,  8);
    SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE   ,  8);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE,  8);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE  , 24);	//< 16bitだと不足．24bitまでなら普通サポートされてると期待
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER,  1);
    renManager->sdlWindow = SDL_CreateWindow(
		 renManager->windowTitle.c_str(),
		(renManager->windowPosX == -1 ? SDL_WINDOWPOS_UNDEFINED : renManager->windowPosX),
		(renManager->windowPosY == -1 ? SDL_WINDOWPOS_UNDEFINED : renManager->windowPosY),
		 renManager->windowWidth,
		 renManager->windowHeight,
		 SDL_WINDOW_RESIZABLE | SDL_WINDOW_OPENGL);

	renManager->glContext = SDL_GL_CreateContext(renManager->sdlWindow);
	SDL_StartTextInput();
	
	// Springhead初期化
	fwSdk   = FWSdkIf::CreateSdk();
	grSdk   = fwSdk->GetGRSdk();

	// レンダラ初期化
	if(renManager->Init())
		 Message::Out("rendering manager initialized");
	else Message::Error("rendering manager initialization failed");
	renManager->OnUpdate();

	// シミュレータ初期化
	if(simManager->Init())
		 Message::Out("simulation manager initialized"); 
	else Message::Error("failed to initialize simulation manager");

	// シーン構築
	Build();
	
	// スレッドを作成
	reqManager->Run();
	simManager->Run();
	
	// タイマ初期化
	SDL_AddTimer(renManager->timerPeriod, TimerCallback, (void*)renManager);
	SDL_AddTimer(simManager->timerPeriod, TimerCallback, (void*)simManager);
	
	return true;
}

void Module::MainLoop(){
	// ループ処理
	while(true){
		// 標準入力からのリクエスト処理
		if(reqManager->Handle()){
			if(OnRequest())
				renManager->OnUpdate();
		}

		// レンダラのイベント処理
		renManager->Handle();
		
		// イベント処理
		if(evExit.IsSet())
			break;
		
		// SDLのイベント処理
		SDL_Event ev;
		while(SDL_PollEvent(&ev)){
			if(ev.type == SDL_QUIT){
				evExit.Set();
				break;
			}
			renManager->OnEvent(&ev);
			simManager->OnEvent(&ev);
		}
	}

	// request managerが終了するまで待つ
	Sleep(100);

	// スレッド終了待ち
	simManager->Join();
	
	// 終了処理
	Cleanup();
}

bool Module::OnRequest(){
	string name           = reqManager->name;
	vector<ArgData>& args = reqManager->args;

	bool ret = false;

	if(name == "q" || name == "quit"){
		Message::Out("exiting...");
		evExit.Set();
		ret = true;
	}
	if(name == "start"){
		if(args[0].str == "plan"){
			Message::Out("start planning");
			simManager->evStartPlan.Set();
		}
		ret = true;
	}
	if(name == "stop"){
		if(args[0].str == "plan"){
			Message::Out("stop planning");
			simManager->evStopPlan.Set();
		}
		ret = true;
	}
	return ret;
}

void Module::OnStep(){
	ptimer.CountUS();
	graph->Step();
	compTime       = ptimer.CountUS();
	compTimeTotal += compTime;
		
	iterCount++;
		
	// 変数変化量のノルム
	deltaNorm = 0.0;
	int nvar = (int)graph->solver->vars.size();
	for(int i = 0; i < nvar; i++){
		Variable* var = graph->solver->vars[i];
		deltaNorm += graph->solver->vars[i]->dx.square();
	}
	deltaNorm = sqrt(deltaNorm);
}

void Module::OnDraw(DiMP::Render::Canvas* canvas){
	renManager->render->SetLighting (false);
	graph->Draw(canvas, this);

	if(isPlaying){
		playTime += 0.001 * renManager->timerPeriod;
		if(playTime > graph->ticks.back()->time)
			playTime = 0.0;
	}
}

void Module::OnPrint(deque<string>& lines){
	char line[256];

	sprintf(line, "iter. count: %6d"  , iterCount       ); lines.push_back(line);
	sprintf(line, "comp. time : %6d"  , compTime        ); lines.push_back(line);
	sprintf(line, "comp. total: %6d"  , compTimeTotal   ); lines.push_back(line);
	sprintf(line, "delta. norm: %6.3f", deltaNorm       ); lines.push_back(line);
	lines.push_back("");

	if(!graph->solver->varInfoType.empty()){
		sprintf(line, "               name : num  lock");
		lines.push_back(line);

		for(int i = 0; i < (int)graph->solver->varInfoType.size(); i++){
			sprintf(line, "%20s: %4d %4d",
				DiMP::VarNames[i],
				graph->solver->varInfoType[i].num      ,
				graph->solver->varInfoType[i].numLocked);
			lines.push_back(line);
		}
	}
	lines.push_back("");

	if(!graph->solver->conInfoType.empty()){
		sprintf(line, "               name : num  en   act  error");
		lines.push_back(line);

		for(int i = 0; i < (int)graph->solver->conInfoType.size(); i++){
			sprintf(line, "%20s: %4d %4d %4d %6.3f",
				DiMP::ConNames[i],
				graph->solver->conInfoType[i].num       ,
				graph->solver->conInfoType[i].numEnabled,
				graph->solver->conInfoType[i].numActive ,
				graph->solver->conInfoType[i].error     );
			lines.push_back(line);
		}
	}
	lines.push_back("");

	// 拘束種別の誤差
	if(!graph->solver->conInfoLevel.empty()){
		sprintf(line, "              level : num    en     act    error");
		lines.push_back(line);

		for(int i = 0; i < (int)graph->solver->conInfoLevel.size(); i++){
			sprintf(line, "%20d: %4d %4d %4d %6.3f",
				i,
				graph->solver->conInfoLevel[i].num       ,
				graph->solver->conInfoLevel[i].numEnabled,
				graph->solver->conInfoLevel[i].numActive ,
				graph->solver->conInfoLevel[i].error     );
			lines.push_back(line);
		}
	}
}

void Module::Cleanup(){
	renManager->Close();
	SDL_Quit();

	delete renManager;
	delete reqManager;
	delete simManager;

	graph->Clear();
	delete graph;
}

#include <DiMP/App/App.h>
#include <DiMP/Graph/Graph.h>
#include <DiMP/Solver/Solver.h>
#include <DiMP/Render/Config.h>
#include <DiMP/Render/Canvas.h>

namespace DiMP{;

///////////////////////////////////////////////////////////////////////////////////////////////////

App::Action* App::Action::AddHotKey(int key, string alt){
	keys.push_back(make_pair(key, alt));
	return this;
}

App::Action::Action(){
	id       =  0;
	type     =  NoValue;
	boolean  =  true;
	integer  =  0;
	intStep  =  1;
	intMin   = -INT_MAX;
	intMax   =  INT_MAX;
	real     =  0.0;
	realStep =  1.0;
	realMin  = -FLT_MAX;
	realMax  =  FLT_MAX;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

int App::Menu::Query(int key){
	for(int i = 0; i < (int)actions.size(); i++){
		App::Action& a = actions[i];
		for(int j = 0; j < (int)a.keys.size(); j++){
			if(a.keys[j].first == key)
				return a.id;
		}
	}
	return -1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

App::App(){
	graph     = new Graph();
	conf      = 0;
	canvasGL  = new Render::CanvasGL ();
	canvasSVG = new Render::CanvasSVG();
	showHelp  = true;
	appName	  = "untitled";
		
	/// ���ł��L���n
	AddMenu(MENU_ALWAYS, "");
	AddAction(MENU_ALWAYS, ID_EXIT, "exit"            )->AddHotKey(DVKeyCode::ESC, "ESC")->AddHotKey('q')->AddHotKey('Q');
	AddAction(MENU_ALWAYS, ID_RUN , "simulation timer")->AddHotKey(' ', "space")->SetType(Action::Boolean)->SetBool(false);
	AddAction(MENU_ALWAYS, ID_STEP, "step"            )->AddHotKey(';');
	AddAction(MENU_ALWAYS, ID_PLAY, "play trajectory" )->AddHotKey('p')->SetType(Action::Boolean)->SetBool(false);
	AddAction(MENU_ALWAYS, ID_LOG , "logging"         )->AddHotKey('l')->SetType(Action::Boolean)->SetBool(false);
	AddAction(MENU_ALWAYS, ID_SVG , "save svg"        )->AddHotKey('v');

	/// ���L�R�}���h�̓V�[���R�}���h�Ƃ̏Փˉ���̂��߂ɑ啶�������蓖�Ă�
	/// �V�~�����[�V�����ݒ�
	AddMenu(MENU_CONFIG, "< simulation settings >");
	AddAction(MENU_CONFIG, ID_TIMER_PERIOD , "timer interval"     )->SetType(Action::Integer)->SetIntStep(10)->SetIntMin(10)->SetIntMax(100);
	AddAction(MENU_CONFIG, ID_NUM_ITERATION, "number of iteration")->SetType(Action::Integer)->SetIntStep(1)->SetIntMin(1)->SetIntMax(20);

	/// ����v��̓�����ԕ\��
	///  ���j���[�o�^���邾���ŕ`��͓��ʏ�������
	AddMenu(MENU_STATE, "< internal states >");

	/// �`��ݒ�n
	AddMenu(MENU_DRAW, "< drawing setting >");
	AddAction(MENU_DRAW, ID_SOLID , "solid rendering"         )->SetType(Action::Boolean);
	AddAction(MENU_DRAW, ID_WIRE  , "wireframe rendering"     )->SetType(Action::Boolean);
	AddAction(MENU_DRAW, ID_CAMERA, "camera perspective|ortho")->SetType(Action::Boolean);

	// ������Ԑݒ�
	appName		= "DiMP sample application";
	clearColor	= Vec4f(1.0f, 1.0f, 1.0f, 1.0f);
	textColor	= Vec4f(0.0f, 0.0f, 0.0f, 1.0f);
	dispMenu	= MENU_COMMON;
	focusMenu	= MENU_ALWAYS;
	focusAction	= 0;

	playTime  = 0.0;
	iterCount = 0;
	compTime  = 0.0;
}

App::~App(){
}

void App::AddMenu(int mid, string brief){
	if((int)menus.size() <= mid)
		menus.resize(mid+1);
	menus[mid].brief = brief;
}

App::Action* App::AddAction(int mid, int aid, string desc){
	Menu& menu = menus[mid];
	if((int)menu.actions.size() <= aid)
		menu.actions.resize(aid+1);
	Action& act = menu.actions[aid];
	act.id   = aid;
	act.desc = desc;
	return &act;
}

App::Action* App::GetAction(int mid, int aid){
	return &menus[mid].actions[aid];
}

void App::HitAction(int mid, int aid, bool on){
	Action* act = GetAction(mid, aid);

	// ��ނɉ����Ēl��ύX
	if(act->type == Action::NoValue){
		message = act->desc + " is executed.";
	}
	if(act->type == Action::Boolean){
		//act.boolean = on;
		act->boolean = !act->boolean;
		message = act->desc + " is " + (on ? "enabled." : "disabled.");
	}
	if(act->type == Action::Integer){
		if(on){
			act->integer = std::min(act->integer + act->intStep, act->intMax);
		}
		else{
			act->integer = std::max(act->integer - act->intStep, act->intMin);
		}
		ss.str("");
		ss << act->desc << " is now " << act->integer;
		message = ss.str();
	}
	if(act->type == Action::Real){
		if(on)
			 act->real = std::min(act->real + act->realStep, act->realMax);
		else act->real = std::max(act->real - act->realStep, act->realMin);

		ss.str("");
		ss << act->desc << " is now " << act->real;
		message = ss.str();
	}
	// �n���h�����Ă�
	OnAction(mid, aid);
}

void App::DrawText(GRRenderIf* render, Vec2f pos, string str, bool bold){
	render->DrawFont(pos, str);
	if(bold)
		render->DrawFont(pos + Vec2f(1,0), str);
}

template<class T>
void DrawValue(GRRenderIf* render, Vec2f& offset, string name, T val){
	render->DrawFont(offset, name);
	stringstream ss;
	ss.str("");
	ss << val;
	render->DrawFont(offset + Vec2f((float)App::Metric::ValueX, 0.0f), ss.str());
	offset.y += (float)App::Metric::LineY;
}
	
void App::DrawState(GRRenderIf* render, Vec2f& offset){
	DrawValue(render, offset, "iter. count:", iterCount      );
	DrawValue(render, offset, "comp. time:" , compTime        );
	DrawValue(render, offset, "delta. norm:", deltaNorm       );
	DrawValue(render, offset, "total error:", graph->solver->e);
		
	// �S����ʂ̌덷
	if(!graph->solver->e_type.empty()){
		for(int i = 0; i < DiMP::ConTag::NumTypes; i++)
			DrawValue(render, offset, DiMP::ConNames[i], graph->solver->e_type[i]);
	}

	offset.y += (float)Metric::LineY;

	// �S����ʂ̌덷
	if(!graph->solver->e_level.empty()){
		for(int i = 0; i <= (int)graph->solver->maxLevel; i++){
			ss.str("");
			ss << "level" << i;
			DrawValue(render, offset, ss.str(), graph->solver->e_level[i]);
		}
	}
}

void App::DrawMenu(GRRenderIf* render, int mid, Vec2f& offset){
	Vec2f pos;
	Menu& menu = menus[mid];

	render->DrawFont(pos + offset, menu.brief);
	pos.y += (float)Metric::LineY;

	// ������ԕ\���̏ꍇ
	if(mid == MENU_STATE){
		offset += pos;
		DrawState(render, offset);
		return;
	}
	// ����ȊO�̓A�N�V�����ꗗ��\��
	for(int aid = 0; aid < (int)menu.actions.size(); aid++){
		bool focus = ((mid == focusMenu || (focusMenu == MENU_COMMON && mid == dispMenu)) && aid == focusAction);

		Action* act = GetAction(mid, aid);

		// �z�b�g�L�[
		pos.x = (float)Metric::KeyX;
		ss.str("");
			
		for(int i = 0; i < (int)act->keys.size(); i++){
			if(act->keys[i].second.empty())
				 ss << (char)act->keys[i].first;
			else ss << act->keys[i].second;
			ss << ' ';
		}
		DrawText(render, pos + offset, ss.str(), focus);
			
		// ����
		pos.x = (float)Metric::DescX;
		DrawText(render, pos + offset, act->desc, focus);
			
		// ���
		pos.x = (float)Metric::ValueX;
		if(act->type == Action::NoValue){
		}
		else if(act->type == Action::Boolean){
			DrawText(render, pos + offset, act->boolean ? "enabled" : "disabled", focus);
		}
		else if(act->type == Action::Integer){
			ss.str("");
			ss << act->integer;
			DrawText(render, pos + offset, ss.str(), focus);
		}
		else if(act->type == Action::Real){
			ss.str("");
			ss << act->real;
			DrawText(render, pos + offset, ss.str(), focus);
		}
		pos.y += (float)Metric::LineY;
		pos.x = 0;
	}
	offset += pos;
}

void App::DrawHelp(GRRenderIf* render){
	render->SetLighting (false);
	render->SetDepthTest(false);
	render->EnterScreenCoordinate();

	Vec2f pos((float)Metric::MarginX, (float)Metric::MarginY);

	// �w���v�ɂ���
	if(showHelp){
		render->DrawFont(pos, "hit \'h\' to hide help"            ); pos.y += (float)Metric::LineY;
		render->DrawFont(pos, "hit TAB to switch menu"            ); pos.y += (float)Metric::LineY;
		render->DrawFont(pos, "hit Up/Down key to select item"    ); pos.y += (float)Metric::LineY;
		render->DrawFont(pos, "hit Left/Right key to change value");
	}
	else render->DrawFont(pos, "hit \'h\' to show help");
	pos.y += (float)Metric::LineY;

	if(showHelp){
		DrawMenu(render, MENU_ALWAYS, pos); pos.y += (float)Metric::LineY;
		DrawMenu(render, MENU_USER  , pos); pos.y += (float)Metric::LineY;
		DrawMenu(render, dispMenu   , pos); pos.y += (float)Metric::LineY;
		pos.y += (float)Metric::LineY;
	}

	// ���b�Z�[�W
	render->DrawFont(pos, message);

	render->LeaveScreenCoordinate();
	render->SetLighting (true);
	render->SetDepthTest(true);
}

void App::SaveSVG(){
	FWWinIf* win = GetCurrentWin();
	GRRenderIf *render = win->GetRender();

	Vec2f sz     = render->GetViewportSize();
	float aspect = sz.y / sz.x;
	canvasSVG->SetViewportSize(sz.x, sz.y);

	Affinef affProj, affView;
	render->GetProjectionMatrix(affProj);
	affView = win->GetTrackball()->GetAffine();

	canvasSVG->SetProjMatrix(affProj);
	canvasSVG->SetViewMatrix(affView);
	canvasSVG->svg.SetScale (1000.0f, -1000.0f);
	canvasSVG->svg.SetOffset(0.0f, 1000.0f);
	canvasSVG->svg.SetAspect(aspect);
	canvasSVG->svg.SaveStart("hoge.svg", 1000, 1000);
	if(zAxisUp){
		canvasSVG->Push  ();
		canvasSVG->Rotate((float)Rad(-90.0), Vec3f(1.0f, 0.0f, 0.0f));
	}
	
	graph->Draw(canvasSVG, conf);
	
	if(zAxisUp){
		canvasSVG->Pop();
	}	
	canvasSVG->svg.SaveEnd();
}

void App::OnStep(){
	ptimer.CountUS();
	graph->Step();
	compTime += ptimer.CountUS();
		
	iterCount++;
		
	// �ϐ��ω��ʂ̃m����
	deltaNorm = 0.0;
	int nvar = (int)graph->solver->vars.size();
	for(int i = 0; i < nvar; i++)
		deltaNorm += graph->solver->vars[i]->d.square();
	deltaNorm = sqrt(deltaNorm);
}

void App::OnDraw(GRRenderIf* render){
	// �f�t�H���g����
	// - ���C�e�B���O�𖳌������ċO����`�悵�C�O���Đ������̃X�i�b�v�V���b�g��`��
	render->SetLighting(false);

	// �f�t�H���g��y�����ゾ��z������ɕ`�悵�����ꍇ������
	if(zAxisUp){
		render->PushModelMatrix();
		render->MultModelMatrix(Affinef::Rot((float)Rad(-90.0), 'x'));
	}
		
	graph->Draw(canvasGL, conf);

	if(GetAction(MENU_ALWAYS, ID_PLAY)->GetBool()){
		graph->DrawSnapshot(playTime, canvasGL);
	}
	if(zAxisUp)
		render->PopModelMatrix();

	render->SetLighting(true);
}

void App::OnAction(int menu, int id){
	/// ���ł��L���A�N�V����
	Action* act = GetAction(menu, id);
	if(menu == MENU_ALWAYS){
		if(id == ID_EXIT)
			exit(0);
		if(id == ID_STEP)
			OnStep();
		if(id == ID_PLAY){
			if(act->GetBool())
				playTime = 0.0;
		}
		if(id == ID_LOG){
			graph->solver->EnableLogging(DiMP::Logging::MajorLoop, act->GetBool());
		}
		if(id == ID_SVG){
			SaveSVG();
		}
	}
	if(menu == MENU_CONFIG){
		if(id == ID_TIMER_PERIOD)
			timerPlan->SetInterval(act->GetInt());
	}
	if(menu == MENU_DRAW){
		
	}
}

void App::Init(int argc, char* argv[]){
	CreateSdk();
	GetSdk()->CreateScene();
	GRInit(argc, argv);
		
	FWWinDesc windowDesc;
	windowDesc.width = 1024;
	windowDesc.height = 768;
	windowDesc.title = appName;
	CreateWin(windowDesc);

	// �g���b�N�{�[���ݒ�
	HITrackballIf* tb = GetCurrentWin()->GetTrackball();
	tb->SetDistanceRange(0.1f, 100.0f);
	tb->SetDistance(3.0f);
	tb->SetLatitudeRange((float)Rad(-90.0), (float)Rad(90.0));
		
	// �����F
	GRRenderIf* render = GetCurrentWin()->GetRender();
	GRFont font;
	font.color = (int)(0xff * textColor[0]) << 16 | (int)(0xff * textColor[1]) << 8 | (int)(0xff * textColor[2]);
	render->SetFont(font);

	// �V�[���\�z
	BuildScene();
		
	// �^�C�}
	timerDraw = CreateTimer(UTTimerIf::FRAMEWORK);
	timerDraw->SetInterval(50);

	timerPlan = CreateTimer(UTTimerIf::FRAMEWORK);
	timerPlan->SetInterval(20);

	EnableIdleFunc(false);
}

void App::TimerFunc(int id) {
	if(timerPlan && id == timerPlan->GetID() && GetAction(MENU_ALWAYS, ID_RUN)->GetBool()){
		OnStep();			
	}
	if(timerDraw && id == timerDraw->GetID()){
		// �Đ�������i�߂�
		if(GetAction(MENU_ALWAYS, ID_PLAY)->GetBool() && !graph->ticks.empty()){
			// �Đ����x x0.5
			playTime += 0.5 * ((double)timerDraw->GetInterval() * 0.001);
			if(playTime > graph->ticks.back()->time)
				playTime = 0.0;
		}
		// �ĕ`��v��
		PostRedisplay();
	}
}

void App::Display() {
	FWWinIf* win = GetCurrentWin();
	GRRenderIf *render = win->GetRender();

	// �w�i�N���A
	render->SetClearColor(clearColor);
	render->ClearBuffer();
	render->BeginScene();

	// ���_�ݒ�
	const float fov = 0.3f;
	Vec2f vpSize = render->GetViewportSize();
	float aspect = vpSize.y / vpSize.x;
	GRCameraDesc camera = render->GetCamera();
	camera.front =   0.01f;
	camera.back  = 100.0f;
	if(GetAction(MENU_DRAW, ID_CAMERA)->GetBool()){
		camera.type = GRCameraDesc::PERSPECTIVE;
		camera.size.x = 2.0f * fov * camera.front;
	}
	else{
		camera.type = GRCameraDesc::ORTHO;
		camera.size.x = 2.0f * fov * win->GetTrackball()->GetDistance();
	}
	camera.size.y = camera.size.x * aspect;
	render->SetCamera(camera);
	render->SetViewMatrix(win->GetTrackball()->GetAffine().inv());

	// �����ݒ�
	GRLightDesc ld;
	ld.diffuse  = Vec4f(0.6f, 0.6f, 0.6f, 1.0f);
	ld.specular = Vec4f(0.0f, 0.0f, 0.0f, 1.0f);
	ld.ambient  = Vec4f(0.1f, 0.1f, 0.1f, 1.0f);
	ld.position = Vec4f( 20.0f, 50.0f,  20.0f, 1.0f);
	render->PushLight(ld);

	// �`��
	OnDraw(render);

	// �w���v�`��
	DrawHelp(render);

	render->PopLight();

	render->EndScene();
	render->SwapBuffers();
}

void App::Keyboard(int key, int x, int y) {
	// 'h' : �w���v�̕\���؂�ւ�
	if(key == 'h' || key == 'H'){
		showHelp = !showHelp;
		return;
	}

	// TAB : ���j���[�؂�ւ�
	if(showHelp){
		if(key == '\t'){
			if(++dispMenu == MENU_COMMON_LAST)
				dispMenu = MENU_COMMON;
			// ���L���j���[���t�H�[�J�X����Ă���ꍇ�C�V�������j���[��1�ڂ̃A�N�V�����Ƀt�H�[�J�X
			if(focusMenu == MENU_COMMON)
				focusAction = 0;
		}
		/* �㉺�L�[ : �I���A�N�V�����؂�ւ�
			�펞���j���[�C���[�U�ŗL���j���[�C���L���j���[�iTAB�؂�ւ��j�̏��ɏォ�����
			*/
		if(key == DVKeyCode::UP || key == DVKeyCode::DOWN){
			bool up = (key == DVKeyCode::UP);

			// �t�H�[�J�X�����j���[
			Menu* m = (focusMenu == MENU_COMMON ? &menus[dispMenu] : &menus[focusMenu]);
					
			// �t�H�[�J�X�A�N�V�����̏㉺
			// ���j���[�̒[�܂ł������玟�̃��j���[��
			if(up){
				if(focusAction == 0){
					if(focusMenu == MENU_ALWAYS){
						/// ��ԏ�Ȃ̂œ������Ȃ�
					}
					else{
						focusMenu--;
						focusAction = (int)menus[focusMenu].actions.size() - 1;
					}
				}
				else focusAction--;
			}
			else{
				if(focusAction == m->actions.size()-1){
					if(focusMenu == MENU_COMMON){
						///< ��ԉ��Ȃ̂œ������Ȃ�
					}
					else{
						focusMenu++;
						focusAction = 0;
					}
				}
				else focusAction++;
			}
		}
		// ���E�L�[ : �I���A�N�V�������s
		if(key == DVKeyCode::LEFT || key == DVKeyCode::RIGHT){
			int mid = focusMenu == MENU_COMMON ? dispMenu : focusMenu;
			int aid = focusAction;
			Action* act = GetAction(mid, aid);
			if(act->type == Action::Boolean || act->type == Action::Integer || act->type == Action::Real)
				HitAction(mid, aid, (key == DVKeyCode::RIGHT));
		}
	}

	// �L�[�ɑΉ�����A�N�V���������s
	int id;
	message = "";
	// �펞�\�����j���[
	id = menus[MENU_ALWAYS].Query(key);
	if(id != -1)
		HitAction(MENU_ALWAYS, id, true);
	// �V�[�����j���[
	id = menus[MENU_USER].Query(key);
	if(id != -1)
		HitAction(MENU_USER, id, true);
	// ���L���j���[
	id = menus[dispMenu].Query(key);
	if(id != -1)
		HitAction(dispMenu, id, true);
}

}

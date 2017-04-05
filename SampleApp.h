#ifndef SAMPLE_APP_H
#define SAMPLE_APP_H

/**
	DiMP�T���v���v���O�������p�A�v���P�[�V�����N���X
	- Springhead2 SampleApp�𗬗p
 **/

#include <Springhead.h>
#include <Framework/SprFWApp.h>
#include <map>

#include <DiMP2/Graph.h>
#include <DiMP2/Solver.h>
#include <DiMP2/DrawCanvas.h>

#include <boost/lexical_cast.hpp>

/*
#include <windows.h>	//< timeGetTime
#undef min
#undef max
*/

using namespace std;
using namespace boost;
using namespace Spr;

class SampleApp : public FWApp{
public:
	/** ���j���[ID
		MENU_ALWAYS�͂��ł��\�������
		�V�[���ɑΉ����郁�j���[��1�`99��ID���g�p(�V�[����99�܂�)
		100�ȍ~�����L���j���[���g�p
	 */
	enum MenuID{
		MENU_ALWAYS		= 0,			///< ���ł��L���ȃ��j���[
		MENU_USER		= 1,			///< �A�v���ŗL���j���[
		MENU_COMMON		= 2,
		MENU_CONFIG = MENU_COMMON,		///< �p�����[�^�ݒ�n
		MENU_STATE,						///< ������ԕ\��
		MENU_DRAW,						///< �`��ݒ�n
		MENU_COMMON_LAST,
	};

	/// �A�N�V����ID
	/// ��ɂ�����A�N�V����
	enum ActionAlways{
		ID_EXIT,					///< �I��
		ID_RUN,						///< �V�~�����[�V�����̊J�n�ƒ�~
		ID_STEP,					///< �X�e�b�v���s
		ID_PLAY,					///< �O���Đ��̊J�n�ƒ�~
		ID_LOG,						///< ���O���̊J�n�ƒ�~
		ID_SVG,						///< SVG�����o��
	};
	
	/// �����V�~�����[�V�����̐ݒ�
	enum ActionConfig{
		ID_TIMER_PERIOD,			///< �^�C�}����
		ID_NUM_ITERATION,			///< �����񐔂𑝂₷
	};
	/// �`��̐ݒ�
	enum ActionDraw{
		ID_SOLID,
		ID_WIRE,
	};

	/// �A�N�V�������
	struct Action{
		/// �A�N�V�������
		enum{
			NoValue,	///< ���s����̂�
			Boolean,	///< On, Off�̐؂�ւ�
			Integer,	///< �����l�̑���
			Real,		///< �����l�̑���
		};
		int			id;							///< �A�N�V����ID
		int			type;						///< �A�N�V�������
		bool		boolean;					///< ��l
		int			integer;					///< �����l
		int			intStep;					///< �����l�̕ύX��
		int			intMin;						///< �����l�̍ŏ��l
		int			intMax;						///< �����l�̍ő�l
		double		real;						///< �����l
		double		realStep;					///< �����l�̕ύX��
		double		realMin;					///< �����l�̍ŏ��l
		double		realMax;					///< �����l�̍ő�l
		
		vector< pair<int, UTString> > keys;		///< �L�[�Ƒ�փe�L�X�g
		UTString	desc;						///< ����

		/// �A�N�V�����ƃL�[�̑Ή�
		Action* AddHotKey(int key, UTString alt = ""){
			keys.push_back(make_pair(key, alt));
			return this;
		}
		Action* SetType(int t){ type = t; return this; }
		bool GetBool(){ return boolean; }
		Action* SetBool(bool b){ boolean = b; return this; }
		int GetInt(){ return integer; }
		Action* SetInt(int i){ integer = i; return this; }
		Action* SetIntStep(int i){ intStep = i; return this; }
		Action* SetIntMin(int i){ intMin = i; return this; }
		Action* SetIntMax(int i){ intMax = i; return this; }
		double GetReal(){ return real; }
		Action* SetReal(double r){ real = r; return this; }
		Action* SetRealStep(double r){ realStep = r; return this; }
		Action* SetRealMin(double r){ realMin = r; return this; }
		Action* SetRealMax(double r){ realMax = r; return this; }

		Action(){
			id = 0;
			type = NoValue;
			boolean = true;
			integer = 0;
			intStep = 1;
			intMin = -INT_MAX;
			intMax =  INT_MAX;
			real = 0.0;
			realStep = 1.0;
			realMin = -FLT_MAX;
			realMax =  FLT_MAX;
		}
	};

	struct Menu : map<int, Action>{
		UTString	brief;						///< ���j���[�̐���
		/// �L�[�ɑΉ�����A�N�V����ID��Ԃ�
		int Query(int key){
			for(iterator it = begin(); it != end(); it++){
				Action& a = it->second;
				for(int i = 0; i < (int)a.keys.size(); i++){
					if(a.keys[i].first == key)
						return a.id;
				}
			}
			return -1;
		}
	};

	/// ����: �h���N���X���R���X�g���N�^�Őݒ肷��
	UTString				appName;		///< �T���v����
	Vec4f					clearColor;		///< �w�i�F
	Vec4f					textColor;		///< �����F

	/// ���j���[�֌W
	typedef map<int, Menu>	Menus;
	Menus					menus;				///< ���j���[
	int						dispMenu;			///< �\�����̋��L���j���[
	int						focusMenu;			///< �t�H�[�J�X���̃��j���[
	Menu::iterator			focusAction;		///< �t�H�[�J�X���̃A�N�V����
	stringstream			ss;
	UTString				message;			///< ��s���b�Z�[�W
	
	/// �w���v�̕`�摮��
	struct Metric{
		enum{
			MarginY = 15,
			MarginX = 20,
			LineY	= 20,
			KeyX	= 0,
			DescX	= 100,
			ValueX	= 300,
		};
	};
	
	/// �^�C�}
	UTTimerIf*				timerDraw;		///< timer for rendering
	UTTimerIf*				timerPlan;		///< timer for planning
	
	/// DiMP�I�u�W�F�N�g
	DiMP2::Graph*			graph;			///< reference to DiMP graph
	DiMP2::DrawConfig*		drawConf;
	double					playTime;		///< play time
	
	/// ���
	bool			showHelp;		///< �w���v�\��
	bool			running;		///< running the planner
	bool			logging;		///< taking error log
	bool			playing;		///< playing the trajectory
	bool			renderMode;		///< render in solid or wireframe

public:
	/// ���j���[�̓o�^
	void AddMenu(int menu, UTString brief){
		menus[menu].brief = brief;
	}

	/// �A�N�V�����̓o�^
	Action* AddAction(int menu, int id, UTString desc){
		Action& act = menus[menu][id];
		act.id = id;
		act.desc = desc;
		return &act;
	}
	/// �A�N�V�����擾
	Action* GetAction(int menu, int id){
		return &menus[menu][id];
	}

	/// �A�N�V�������s
	void HitAction(int menu, int id, bool on){
		Action& act = menus[menu][id];
		// ��ނɉ����Ēl��ύX
		if(act.type == Action::NoValue){
			message = act.desc + " is executed.";
		}
		if(act.type == Action::Boolean){
			//act.boolean = on;
			act.boolean = !act.boolean;
			message = act.desc + " is " + (on ? "enabled." : "disabled.");
		}
		if(act.type == Action::Integer){
			if(on){
				act.integer = std::min(act.integer + act.intStep, act.intMax);
			}
			else{
				act.integer = std::max(act.integer - act.intStep, act.intMin);
			}
			ss.str("");
			ss << act.desc << " is now " << act.integer;
			message = ss.str();
		}
		if(act.type == Action::Real){
			if(on){
				act.real = std::min(act.real + act.realStep, act.realMax);
			}
			else{
				act.real = std::max(act.real - act.realStep, act.realMin);
			}
			ss.str("");
			ss << act.desc << " is now " << act.real;
			message = ss.str();
		}
		// �n���h�����Ă�
		OnAction(menu, id);
	}

	/// �e�L�X�g�`��
	void DrawText(GRRenderIf* render, Vec2f pos, string str, bool bold){
		render->DrawFont(pos, str);
		if(bold)
			render->DrawFont(pos + Vec2f(1,0), str);
	}

	/// ���x���ƒl����ׂĕ`�悵�ĉ��s����
	template<class T>
	void DrawValue(GRRenderIf* render, Vec2f& offset, string name, T val){
		render->DrawFont(offset, name);
		ss.str("");
		ss << val;
		render->DrawFont(offset + Vec2f((float)Metric::ValueX, 0.0f), ss.str());
		offset.y += (float)Metric::LineY;
	}
	
	/// ����v��̓�����Ԃ̕\��
	void DrawState(GRRenderIf* render, Vec2f& offset){
		// �S�S���덷
		DrawValue(render, offset, "total error:", graph->solver->e);
		
		offset.y += (float)Metric::LineY;

		// �S����ʂ̌덷
		if(!graph->solver->e_type.empty()){
			for(int i = 0; i < DiMP2::ConTag::NumTypes; i++)
				DrawValue(render, offset, DiMP2::ConNames[i], graph->solver->e_type[i]);
		}

		offset.y += (float)Metric::LineY;

		// �S����ʂ̌덷
		if(!graph->solver->e_level.empty()){
			for(int i = 0; i <= (int)graph->solver->maxLevel; i++)
				DrawValue(render, offset, "level" + lexical_cast<string>(i), graph->solver->e_level[i]);
		}
	}

	/// ���j���[�̕\��
	void DrawMenu(GRRenderIf* render, int id, Vec2f& offset){
		Vec2f pos;

		Menu& menu = menus[id];

		render->DrawFont(pos + offset, menu.brief);
		pos.y += (float)Metric::LineY;

		// ������ԕ\���̏ꍇ
		if(id == MENU_STATE){
			offset += pos;
			DrawState(render, offset);
			return;
		}
		// ����ȊO�̓A�N�V�����ꗗ��\��
		for(Menu::iterator it = menu.begin(); it != menu.end(); it++){
			bool focus = ((id == focusMenu || (focusMenu == MENU_COMMON && id == dispMenu)) && it == focusAction);

			Action& a = it->second;
			// �z�b�g�L�[
			pos.x = (float)Metric::KeyX;
			ss.str("");
			
			for(int i = 0; i < (int)a.keys.size(); i++){
				if(a.keys[i].second.empty())
					 ss << (char)a.keys[i].first;
				else ss << a.keys[i].second;
				ss << ' ';
			}
			DrawText(render, pos + offset, ss.str(), focus);
			
			// ����
			pos.x = (float)Metric::DescX;
			DrawText(render, pos + offset, a.desc, focus);
			
			// ���
			pos.x = (float)Metric::ValueX;
			if(a.type == Action::NoValue){
			}
			else if(a.type == Action::Boolean){
				DrawText(render, pos + offset, a.boolean ? "enabled" : "disabled", focus);
			}
			else if(a.type == Action::Integer){
				ss.str("");
				ss << a.integer;
				DrawText(render, pos + offset, ss.str(), focus);
			}
			else if(a.type == Action::Real){
				ss.str("");
				ss << a.real;
				DrawText(render, pos + offset, ss.str(), focus);
			}
			pos.y += (float)Metric::LineY;
			pos.x = 0;
		}
		offset += pos;
	}

	/// �t�����̕\��
	void DrawHelp(GRRenderIf* render){
		render->SetLighting(false);
		render->SetDepthTest(false);
		render->EnterScreenCoordinate();

		Vec2f pos((float)Metric::MarginX, (float)Metric::MarginY);

		// �w���v�ɂ���
		if(showHelp){
			render->DrawFont(pos, "hit \'h\' to hide help");
			pos.y += (float)Metric::LineY;
			render->DrawFont(pos, "hit TAB to switch menu");
			pos.y += (float)Metric::LineY;
			render->DrawFont(pos, "hit Up/Down key to select item");
			pos.y += (float)Metric::LineY;
			render->DrawFont(pos, "hit Left/Right key to change value");
		}
		else render->DrawFont(pos, "hit \'h\' to show help");
		pos.y += (float)Metric::LineY;

		if(showHelp){
			// ���ł��\���n���j���[
			DrawMenu(render, MENU_ALWAYS, pos);
			pos.y += (float)Metric::LineY;

			// �V�[�����j���[
			DrawMenu(render, MENU_USER, pos);
			pos.y += (float)Metric::LineY;

			// ���L���j���[
			DrawMenu(render, dispMenu, pos);
			pos.y += (float)Metric::LineY;

			pos.y += (float)Metric::LineY;
		}

		// ���b�Z�[�W
		render->DrawFont(pos, message);

		render->LeaveScreenCoordinate();
		render->SetLighting(true);
		render->SetDepthTest(true);
	}

	SampleApp(){
		graph    = 0;
		drawConf = 0;

		showHelp	= true;
		appName		= "untitled";
		
		/// ���ł��L���n
		AddMenu(MENU_ALWAYS, "");
		AddAction(MENU_ALWAYS, ID_EXIT, "exit")
			->AddHotKey(DVKeyCode::ESC, "ESC")
			->AddHotKey('q')
			->AddHotKey('Q');
		AddAction(MENU_ALWAYS, ID_RUN, "simulation timer")
			->AddHotKey(' ', "space")
			->SetType(Action::Boolean)
			->SetBool(false);
		AddAction(MENU_ALWAYS, ID_STEP, "step")
			->AddHotKey(';');
		AddAction(MENU_ALWAYS, ID_PLAY, "play trajectory")
			->AddHotKey('p')
			->SetType(Action::Boolean)
			->SetBool(false);
		AddAction(MENU_ALWAYS, ID_LOG, "logging")
			->AddHotKey('l')
			->SetType(Action::Boolean)
			->SetBool(false);
		AddAction(MENU_ALWAYS, ID_SVG, "save svg")
			->AddHotKey('v');

		/// ���L�R�}���h�̓V�[���R�}���h�Ƃ̏Փˉ���̂��߂ɑ啶�������蓖�Ă�
		/// �V�~�����[�V�����ݒ�
		AddMenu(MENU_CONFIG, "< simulation settings >");
		AddAction(MENU_CONFIG, ID_TIMER_PERIOD, "timer interval")
			->SetType(Action::Integer)
			->SetIntStep(10)
			->SetIntMin(10)
			->SetIntMax(100);
		AddAction(MENU_CONFIG, ID_NUM_ITERATION, "number of iteration")
			->SetType(Action::Integer)
			->SetIntStep(1)
			->SetIntMin(1)
			->SetIntMax(20);

		/// ����v��̓�����ԕ\��
		///  ���j���[�o�^���邾���ŕ`��͓��ʏ�������
		AddMenu(MENU_STATE, "< internal states >");

		/// �`��ݒ�n
		AddMenu(MENU_DRAW, "< drawing setting >");
		AddAction(MENU_DRAW, ID_SOLID, "solid rendering")
			->SetType(Action::Boolean);
		AddAction(MENU_DRAW, ID_WIRE, "wireframe rendering")
			->SetType(Action::Boolean);

		// ������Ԑݒ�
		appName		= "DiMP2 sample application";
		clearColor	= Vec4f(1.0f, 1.0f, 1.0f, 1.0f);
		textColor	= Vec4f(0.0f, 0.0f, 0.0f, 1.0f);
		dispMenu	= MENU_COMMON;
		focusMenu	= MENU_ALWAYS;
		focusAction	= menus[MENU_ALWAYS].begin();

		playTime = 0.0;
	}
	~SampleApp(){}

public: /** �h���N���X����������֐� **/

	/// �V�[���\�z���s���D
	virtual void BuildScene(){}

	/// 1�X�e�b�v�̃V�~�����[�V����
	virtual void OnStep(){
		graph->Step();
	}

	/// �`��
	virtual void OnDraw(GRRenderIf* render){
		// �f�t�H���g����
		// - ���C�e�B���O�𖳌������ċO����`�悵�C�O���Đ������̃X�i�b�v�V���b�g��`��
		render->SetLighting(false);

		graph->Draw(render, drawConf);

		if(GetAction(MENU_ALWAYS, ID_PLAY)->GetBool()){
			graph->DrawSnapshot(playTime, render);
		}
		
		render->SetLighting(true);
	}

	/// �A�N�V��������
	virtual void OnAction(int menu, int id){
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
				if(act->GetBool()){
					graph->solver->EnableLogging(DiMP2::Logging::MajorLoop, true);
				}
				else{
					graph->solver->EnableLogging(DiMP2::Logging::MajorLoop, false);
				}
			}
		}
		if(menu == MENU_CONFIG){
			if(id == ID_TIMER_PERIOD)
				timerPlan->SetInterval(act->GetInt());
		}
		if(menu == MENU_DRAW){
		
		}
	}

public: /** FWApp�̎��� **/

	virtual void Init(int argc, char* argv[]){
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

	// �^�C�}�R�[���o�b�N�֐��D�^�C�}�����ŌĂ΂��
	virtual void TimerFunc(int id) {
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

	// �`��֐��D�`��v���������Ƃ��ɌĂ΂��
	virtual void Display() {
		FWWinIf* win = GetCurrentWin();
		GRRenderIf *render = win->GetRender();

		// �w�i�N���A
		render->SetClearColor(clearColor);
		render->ClearBuffer();
		render->BeginScene();

		// ���_�ݒ�
		GRCameraDesc camera = render->GetCamera();
		camera.front = 0.3f;
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

	virtual void Keyboard(int key, int x, int y) {
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
					focusAction = menus[dispMenu].begin();
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
					if(focusAction == m->begin()){
						if(focusMenu == MENU_ALWAYS){
							/// ��ԏ�Ȃ̂œ������Ȃ�
						}
						else{
							focusMenu--;
							m = (focusMenu == MENU_COMMON ? &menus[dispMenu] : &menus[focusMenu]);
							focusAction = m->end();
							focusAction--;
						}
					}
					else focusAction--;
				}
				else{
					focusAction++;
					if(focusAction == m->end()){
						if(focusMenu == MENU_COMMON){
							///< ��ԉ��Ȃ̂œ������Ȃ�
							focusAction--;
						}
						else{
							focusMenu++;
							m = (focusMenu == MENU_COMMON ? &menus[dispMenu] : &menus[focusMenu]);
							focusAction = m->begin();
						}
					}
				}
			}
			// ���E�L�[ : �I���A�N�V�������s
			if(key == DVKeyCode::LEFT || key == DVKeyCode::RIGHT){
				int menu = focusMenu;
				int id = focusAction->first;
				Action& act = menus[menu][id];
				if(act.type == Action::Boolean || act.type == Action::Integer || act.type == Action::Real)
					HitAction(menu, id, (key == DVKeyCode::RIGHT));
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

};

#endif

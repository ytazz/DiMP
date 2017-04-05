#ifndef SAMPLE_APP_H
#define SAMPLE_APP_H

/**
	DiMPサンプルプログラム共用アプリケーションクラス
	- Springhead2 SampleAppを流用
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
	/** メニューID
		MENU_ALWAYSはいつでも表示される
		シーンに対応するメニューは1～99のIDを使用(シーンは99個まで)
		100以降を共有メニューが使用
	 */
	enum MenuID{
		MENU_ALWAYS		= 0,			///< いつでも有効なメニュー
		MENU_USER		= 1,			///< アプリ固有メニュー
		MENU_COMMON		= 2,
		MENU_CONFIG = MENU_COMMON,		///< パラメータ設定系
		MENU_STATE,						///< 内部状態表示
		MENU_DRAW,						///< 描画設定系
		MENU_COMMON_LAST,
	};

	/// アクションID
	/// 常につかえるアクション
	enum ActionAlways{
		ID_EXIT,					///< 終了
		ID_RUN,						///< シミュレーションの開始と停止
		ID_STEP,					///< ステップ実行
		ID_PLAY,					///< 軌道再生の開始と停止
		ID_LOG,						///< ログ取りの開始と停止
		ID_SVG,						///< SVG書き出し
	};
	
	/// 物理シミュレーションの設定
	enum ActionConfig{
		ID_TIMER_PERIOD,			///< タイマ周期
		ID_NUM_ITERATION,			///< 反復回数を増やす
	};
	/// 描画の設定
	enum ActionDraw{
		ID_SOLID,
		ID_WIRE,
	};

	/// アクション情報
	struct Action{
		/// アクション種類
		enum{
			NoValue,	///< 実行するのみ
			Boolean,	///< On, Offの切り替え
			Integer,	///< 整数値の増減
			Real,		///< 実数値の増減
		};
		int			id;							///< アクションID
		int			type;						///< アクション種類
		bool		boolean;					///< 二値
		int			integer;					///< 整数値
		int			intStep;					///< 整数値の変更幅
		int			intMin;						///< 整数値の最小値
		int			intMax;						///< 整数値の最大値
		double		real;						///< 実数値
		double		realStep;					///< 実数値の変更幅
		double		realMin;					///< 実数値の最小値
		double		realMax;					///< 実数値の最大値
		
		vector< pair<int, UTString> > keys;		///< キーと代替テキスト
		UTString	desc;						///< 説明

		/// アクションとキーの対応
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
		UTString	brief;						///< メニューの説明
		/// キーに対応するアクションIDを返す
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

	/// 属性: 派生クラスがコンストラクタで設定する
	UTString				appName;		///< サンプル名
	Vec4f					clearColor;		///< 背景色
	Vec4f					textColor;		///< 文字色

	/// メニュー関係
	typedef map<int, Menu>	Menus;
	Menus					menus;				///< メニュー
	int						dispMenu;			///< 表示中の共有メニュー
	int						focusMenu;			///< フォーカス中のメニュー
	Menu::iterator			focusAction;		///< フォーカス中のアクション
	stringstream			ss;
	UTString				message;			///< 一行メッセージ
	
	/// ヘルプの描画属性
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
	
	/// タイマ
	UTTimerIf*				timerDraw;		///< timer for rendering
	UTTimerIf*				timerPlan;		///< timer for planning
	
	/// DiMPオブジェクト
	DiMP2::Graph*			graph;			///< reference to DiMP graph
	DiMP2::DrawConfig*		drawConf;
	double					playTime;		///< play time
	
	/// 状態
	bool			showHelp;		///< ヘルプ表示
	bool			running;		///< running the planner
	bool			logging;		///< taking error log
	bool			playing;		///< playing the trajectory
	bool			renderMode;		///< render in solid or wireframe

public:
	/// メニューの登録
	void AddMenu(int menu, UTString brief){
		menus[menu].brief = brief;
	}

	/// アクションの登録
	Action* AddAction(int menu, int id, UTString desc){
		Action& act = menus[menu][id];
		act.id = id;
		act.desc = desc;
		return &act;
	}
	/// アクション取得
	Action* GetAction(int menu, int id){
		return &menus[menu][id];
	}

	/// アクション実行
	void HitAction(int menu, int id, bool on){
		Action& act = menus[menu][id];
		// 種類に応じて値を変更
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
		// ハンドラを呼ぶ
		OnAction(menu, id);
	}

	/// テキスト描画
	void DrawText(GRRenderIf* render, Vec2f pos, string str, bool bold){
		render->DrawFont(pos, str);
		if(bold)
			render->DrawFont(pos + Vec2f(1,0), str);
	}

	/// ラベルと値を並べて描画して改行する
	template<class T>
	void DrawValue(GRRenderIf* render, Vec2f& offset, string name, T val){
		render->DrawFont(offset, name);
		ss.str("");
		ss << val;
		render->DrawFont(offset + Vec2f((float)Metric::ValueX, 0.0f), ss.str());
		offset.y += (float)Metric::LineY;
	}
	
	/// 動作計画の内部状態の表示
	void DrawState(GRRenderIf* render, Vec2f& offset){
		// 全拘束誤差
		DrawValue(render, offset, "total error:", graph->solver->e);
		
		offset.y += (float)Metric::LineY;

		// 拘束種別の誤差
		if(!graph->solver->e_type.empty()){
			for(int i = 0; i < DiMP2::ConTag::NumTypes; i++)
				DrawValue(render, offset, DiMP2::ConNames[i], graph->solver->e_type[i]);
		}

		offset.y += (float)Metric::LineY;

		// 拘束種別の誤差
		if(!graph->solver->e_level.empty()){
			for(int i = 0; i <= (int)graph->solver->maxLevel; i++)
				DrawValue(render, offset, "level" + lexical_cast<string>(i), graph->solver->e_level[i]);
		}
	}

	/// メニューの表示
	void DrawMenu(GRRenderIf* render, int id, Vec2f& offset){
		Vec2f pos;

		Menu& menu = menus[id];

		render->DrawFont(pos + offset, menu.brief);
		pos.y += (float)Metric::LineY;

		// 内部状態表示の場合
		if(id == MENU_STATE){
			offset += pos;
			DrawState(render, offset);
			return;
		}
		// それ以外はアクション一覧を表示
		for(Menu::iterator it = menu.begin(); it != menu.end(); it++){
			bool focus = ((id == focusMenu || (focusMenu == MENU_COMMON && id == dispMenu)) && it == focusAction);

			Action& a = it->second;
			// ホットキー
			pos.x = (float)Metric::KeyX;
			ss.str("");
			
			for(int i = 0; i < (int)a.keys.size(); i++){
				if(a.keys[i].second.empty())
					 ss << (char)a.keys[i].first;
				else ss << a.keys[i].second;
				ss << ' ';
			}
			DrawText(render, pos + offset, ss.str(), focus);
			
			// 説明
			pos.x = (float)Metric::DescX;
			DrawText(render, pos + offset, a.desc, focus);
			
			// 状態
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

	/// 付加情報の表示
	void DrawHelp(GRRenderIf* render){
		render->SetLighting(false);
		render->SetDepthTest(false);
		render->EnterScreenCoordinate();

		Vec2f pos((float)Metric::MarginX, (float)Metric::MarginY);

		// ヘルプについて
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
			// いつでも表示系メニュー
			DrawMenu(render, MENU_ALWAYS, pos);
			pos.y += (float)Metric::LineY;

			// シーンメニュー
			DrawMenu(render, MENU_USER, pos);
			pos.y += (float)Metric::LineY;

			// 共有メニュー
			DrawMenu(render, dispMenu, pos);
			pos.y += (float)Metric::LineY;

			pos.y += (float)Metric::LineY;
		}

		// メッセージ
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
		
		/// いつでも有効系
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

		/// 共有コマンドはシーンコマンドとの衝突回避のために大文字を割り当てる
		/// シミュレーション設定
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

		/// 動作計画の内部状態表示
		///  メニュー登録するだけで描画は特別処理する
		AddMenu(MENU_STATE, "< internal states >");

		/// 描画設定系
		AddMenu(MENU_DRAW, "< drawing setting >");
		AddAction(MENU_DRAW, ID_SOLID, "solid rendering")
			->SetType(Action::Boolean);
		AddAction(MENU_DRAW, ID_WIRE, "wireframe rendering")
			->SetType(Action::Boolean);

		// 初期状態設定
		appName		= "DiMP2 sample application";
		clearColor	= Vec4f(1.0f, 1.0f, 1.0f, 1.0f);
		textColor	= Vec4f(0.0f, 0.0f, 0.0f, 1.0f);
		dispMenu	= MENU_COMMON;
		focusMenu	= MENU_ALWAYS;
		focusAction	= menus[MENU_ALWAYS].begin();

		playTime = 0.0;
	}
	~SampleApp(){}

public: /** 派生クラスが実装する関数 **/

	/// シーン構築を行う．
	virtual void BuildScene(){}

	/// 1ステップのシミュレーション
	virtual void OnStep(){
		graph->Step();
	}

	/// 描画
	virtual void OnDraw(GRRenderIf* render){
		// デフォルト処理
		// - ライティングを無効化して軌道を描画し，軌道再生時刻のスナップショットを描画
		render->SetLighting(false);

		graph->Draw(render, drawConf);

		if(GetAction(MENU_ALWAYS, ID_PLAY)->GetBool()){
			graph->DrawSnapshot(playTime, render);
		}
		
		render->SetLighting(true);
	}

	/// アクション処理
	virtual void OnAction(int menu, int id){
		/// いつでも有効アクション
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

public: /** FWAppの実装 **/

	virtual void Init(int argc, char* argv[]){
		CreateSdk();
		GetSdk()->CreateScene();
		GRInit(argc, argv);
		
		FWWinDesc windowDesc;
		windowDesc.width = 1024;
		windowDesc.height = 768;
		windowDesc.title = appName;
		CreateWin(windowDesc);

		// トラックボール設定
		HITrackballIf* tb = GetCurrentWin()->GetTrackball();
		tb->SetDistanceRange(0.1f, 100.0f);
		tb->SetDistance(3.0f);
		
		// 文字色
		GRRenderIf* render = GetCurrentWin()->GetRender();
		GRFont font;
		font.color = (int)(0xff * textColor[0]) << 16 | (int)(0xff * textColor[1]) << 8 | (int)(0xff * textColor[2]);
		render->SetFont(font);

		// シーン構築
		BuildScene();
		
		// タイマ
		timerDraw = CreateTimer(UTTimerIf::FRAMEWORK);
		timerDraw->SetInterval(50);

		timerPlan = CreateTimer(UTTimerIf::FRAMEWORK);
		timerPlan->SetInterval(20);

		EnableIdleFunc(false);
	}

	// タイマコールバック関数．タイマ周期で呼ばれる
	virtual void TimerFunc(int id) {
		if(timerPlan && id == timerPlan->GetID() && GetAction(MENU_ALWAYS, ID_RUN)->GetBool()){
			OnStep();			
		}
		if(timerDraw && id == timerDraw->GetID()){
			// 再生時刻を進める
			if(GetAction(MENU_ALWAYS, ID_PLAY)->GetBool() && !graph->ticks.empty()){
				// 再生速度 x0.5
				playTime += 0.5 * ((double)timerDraw->GetInterval() * 0.001);
				if(playTime > graph->ticks.back()->time)
					playTime = 0.0;
			}
			// 再描画要求
			PostRedisplay();
		}
	}

	// 描画関数．描画要求が来たときに呼ばれる
	virtual void Display() {
		FWWinIf* win = GetCurrentWin();
		GRRenderIf *render = win->GetRender();

		// 背景クリア
		render->SetClearColor(clearColor);
		render->ClearBuffer();
		render->BeginScene();

		// 視点設定
		GRCameraDesc camera = render->GetCamera();
		camera.front = 0.3f;
		render->SetCamera(camera);
		render->SetViewMatrix(win->GetTrackball()->GetAffine().inv());

		// 光源設定
		GRLightDesc ld;
		ld.diffuse  = Vec4f(0.6f, 0.6f, 0.6f, 1.0f);
		ld.specular = Vec4f(0.0f, 0.0f, 0.0f, 1.0f);
		ld.ambient  = Vec4f(0.1f, 0.1f, 0.1f, 1.0f);
		ld.position = Vec4f( 20.0f, 50.0f,  20.0f, 1.0f);
		render->PushLight(ld);

		// 描画
		OnDraw(render);

		// ヘルプ描画
		DrawHelp(render);

		render->PopLight();

		render->EndScene();
		render->SwapBuffers();
	}

	virtual void Keyboard(int key, int x, int y) {
		// 'h' : ヘルプの表示切り替え
		if(key == 'h' || key == 'H'){
			showHelp = !showHelp;
			return;
		}
		
		// TAB : メニュー切り替え
		if(showHelp){
			if(key == '\t'){
				if(++dispMenu == MENU_COMMON_LAST)
					dispMenu = MENU_COMMON;
				// 共有メニューがフォーカスされている場合，新しいメニューの1つ目のアクションにフォーカス
				if(focusMenu == MENU_COMMON)
					focusAction = menus[dispMenu].begin();
			}
			/* 上下キー : 選択アクション切り替え
				常時メニュー，ユーザ固有メニュー，共有メニュー（TAB切り替え）の順に上から並ぶ
			 */
			if(key == DVKeyCode::UP || key == DVKeyCode::DOWN){
				bool up = (key == DVKeyCode::UP);

				// フォーカス中メニュー
				Menu* m = (focusMenu == MENU_COMMON ? &menus[dispMenu] : &menus[focusMenu]);
					
				// フォーカスアクションの上下
				// メニューの端までいったら次のメニューへ
				if(up){
					if(focusAction == m->begin()){
						if(focusMenu == MENU_ALWAYS){
							/// 一番上なので動かさない
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
							///< 一番下なので動かさない
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
			// 左右キー : 選択アクション実行
			if(key == DVKeyCode::LEFT || key == DVKeyCode::RIGHT){
				int menu = focusMenu;
				int id = focusAction->first;
				Action& act = menus[menu][id];
				if(act.type == Action::Boolean || act.type == Action::Integer || act.type == Action::Real)
					HitAction(menu, id, (key == DVKeyCode::RIGHT));
			}
		}

		// キーに対応するアクションを実行
		int id;
		message = "";
		// 常時表示メニュー
		id = menus[MENU_ALWAYS].Query(key);
		if(id != -1)
			HitAction(MENU_ALWAYS, id, true);
		// シーンメニュー
		id = menus[MENU_USER].Query(key);
		if(id != -1)
			HitAction(MENU_USER, id, true);
		// 共有メニュー
		id = menus[dispMenu].Query(key);
		if(id != -1)
			HitAction(dispMenu, id, true);
	}

};

#endif

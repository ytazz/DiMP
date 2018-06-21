#pragma once

/**
	DiMPサンプルプログラム共用アプリケーションクラス
	- Springhead2 SampleAppを流用
 **/

#include <DiMP/Types.h>
#include <DiMP/Render/Canvas.h>
#include <DiMP/Render/Config.h>

#include <Springhead.h>
#include <Framework/SprFWApp.h>
#include <Foundation/UTQPTimer.h>

namespace DiMP{;

class Graph;

class App : public Spr::FWApp{
public:
	/** メニューID
	 */
	enum MenuID{
		MENU_ALWAYS		= 0,			///< いつでも有効なメニュー
		MENU_USER		= 1,			///< アプリ固有メニュー
		MENU_COMMON		= 2,            ///< 共有メニュー　タブで切り替え
		MENU_CONFIG = MENU_COMMON,		///<  パラメータ設定系
		MENU_STATE,						///<  内部状態表示
		MENU_DRAW,						///<  描画設定系
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
		ID_CAMERA,
	};

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
		
		vector< pair<int, string> > keys;		///< キーと代替テキスト
		string	desc;						    ///< 説明

		/// アクションとキーの対応
		Action* AddHotKey(int key, string alt = "");

		Action* SetType    (int t)   { type = t; return this;     }
		bool    GetBool    ()        { return boolean;            }
		Action* SetBool    (bool b)  { boolean = b; return this;  }
		int     GetInt     ()        { return integer;            }
		Action* SetInt     (int i)   { integer = i; return this;  }
		Action* SetIntStep (int i)   { intStep = i; return this;  }
		Action* SetIntMin  (int i)   { intMin = i; return this;   }
		Action* SetIntMax  (int i)   { intMax = i; return this;   }
		double  GetReal    ()        { return real;               }
		Action* SetReal    (double r){ real = r; return this;     }
		Action* SetRealStep(double r){ realStep = r; return this; }
		Action* SetRealMin (double r){ realMin = r; return this;  }
		Action* SetRealMax (double r){ realMax = r; return this;  }

		Action();
	};

	struct Menu{
		string	        brief;       ///< メニューの説明
		vector<Action>  actions;
		
		int Query(int key);  ///< キーに対応するアクションIDを返す
	};

	/// 属性: 派生クラスがコンストラクタで設定する
	string					appName;		///< サンプル名
	Vec4f					clearColor;		///< 背景色
	Vec4f					textColor;		///< 文字色
	bool					zAxisUp;		///< Z軸を上にして描画

	/// メニュー関係
	vector<Menu>			menus;				///< メニュー
	int						dispMenu;			///< 表示中の共有メニュー
	int						focusMenu;			///< フォーカス中のメニュー
	int                     focusAction;		///< フォーカス中のアクション
	stringstream			ss;
	string					message;			///< 一行メッセージ
	
	/// タイマ
	UTTimerIf*				timerDraw;		///< timer for rendering
	UTTimerIf*				timerPlan;		///< timer for planning
	UTQPTimer               ptimer;
	
	/// DiMPオブジェクト
	UTRef<Graph>	          graph;			///< reference to DiMP graph
	Render::Config*           conf;
	UTRef<Render::CanvasGL >  canvasGL;
	UTRef<Render::CanvasSVG>  canvasSVG;

	double  playTime;		///< play time
	int     iterCount;
	double  deltaNorm;
	double  compTime;
	
	/// 状態
	bool    showHelp;		///< ヘルプ表示
	bool    running;		///< running the planner
	bool    logging;		///< taking error log
	bool    playing;		///< playing the trajectory
	bool    renderMode;		///< render in solid or wireframe

public:
	void    AddMenu  (int mid, string brief);        ///< メニューの登録
	Action* AddAction(int mid, int aid, string desc); ///< アクションの登録
	Action* GetAction(int mid, int aid);              ///< アクション取得
	void    HitAction(int mid, int aid, bool on);     ///< アクション実行
	
	void    DrawText (GRRenderIf* render, Vec2f pos, string str, bool bold); ///< テキスト描画
	void    DrawState(GRRenderIf* render, Vec2f& offset);                    ///< 動作計画の内部状態の表示
	void    DrawMenu (GRRenderIf* render, int mid, Vec2f& offset);            ///< メニューの表示
	void    DrawHelp (GRRenderIf* render);                                   ///< 付加情報の表示

	void    SaveSVG  ();
	
public: /** 派生クラスが実装する関数 **/

	virtual void BuildScene(){}                  ///< シーン構築を行う．
	virtual void OnStep    ();                   ///< 1ステップのシミュレーション
	virtual void OnDraw    (GRRenderIf* render); ///< 描画
	virtual void OnAction  (int menu, int id);   ///< アクション処理
	
public: /** FWAppの実装 **/
	virtual void Init     (int argc, char* argv[]);
	virtual void TimerFunc(int id);                 ///< タイマコールバック関数．タイマ周期で呼ばれる
	virtual void Display  ();                       ///< 描画関数．描画要求が来たときに呼ばれる
	virtual void Keyboard (int key, int x, int y);

    App();
	virtual ~App();
};

}

#pragma once

#include <module/module.h>

class RobotArm;
class Workspace;

class MyModule : public Module {
public:
	// 構築するロボットアームの選択
	enum {
		Reaching2D,		    //< 平面上のロボットアームのリーチング
		Reaching3D,		    //< 3D上のロボットアームのリーチング＋障害物回避
        Reaching3DTwoPhase, //< 3D上のロボットアームのリーチング＋障害物回避　＋局所軌道計画
		Toss3D,			    //< 3D上の複数のロボットアームの物体搬送
		Welding,
	};

	struct Config {
		struct Welding {
			struct Segment{
				string           timeslotName;
				DiMP::TimeSlot*  timeslot;
				vvec_t           startPosture;
				vvec_t           endPosture;
				int              startIndex;
				int              endIndex;
				
				Segment();
			};
			
			string           pointsFilename;
			vec3_t           mockupPos;
			quat_t           mockupOri;
			bool             useTree;
			vector<Segment>  segments;

			Welding();
		};

		Welding  welding;
	};

    struct PlanPhase{
        enum{
            Global = 0,
            Local  = 1,
        };
    };

    string          sceneName;
	int             sceneSelect;
	string          sceneFilename;
	string          confFilename;

	Config          conf;

    typedef vector< UTRef<Workspace> >  Workspaces;
    Workspaces      workspace;

	vector<vec3_t>  weldingPoints;   ///< weldingにおける溶接点列

    int             planPhase;       ///< two phaseにおける計画工程
    real_t          localPlanTime;   ///< local planningの対象時刻

public:
	void Read(XML& xml);

	void DrawSnapshot(real_t time);

	void EnableConstraints (string mode, bool enable);
    void SwitchPhase(string phase);
	void Report();

public:
	virtual bool Build();
	virtual bool OnRequest();	  ///< リクエスト処理
	virtual void OnStep();
	virtual void OnDraw(DiMP::Render::Canvas* canvas);

	virtual bool Set(DiMP::Render::Canvas* canvas, int attr, DiMP::Node* node);

public:
	MyModule();
	~MyModule();

};

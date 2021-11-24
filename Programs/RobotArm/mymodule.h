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
        Reaching3DTwoStage, //< 3D上のロボットアームのリーチング＋障害物回避　＋局所軌道計画
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

    string          sceneName;
	int             sceneSelect;
	string          sceneFilename;
	string          confFilename;

	Config          conf;

    typedef vector< UTRef<Workspace> >  Workspaces;
    Workspaces      workspace;

	vector<vec3_t>  weldingPoints;   ///< weldingにおける溶接点列

public:
	void Read(XML& xml);

	void DrawSnapshot(real_t time);

	void EnableConstraints (string mode, bool enable);
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

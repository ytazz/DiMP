#pragma once

#include <module/module.h>

#include <sbbuilder.h>
#include <sbscenelocal.h>
#include <DiMP/sbdimp.h>
#include <SprGraphics/sbsprgraphics.h>

class RobotArm;

class MyModule : public Module {
public:
	// 構築するロボットアームの選択
	enum {
		Reaching2D,		//< 平面上のロボットアームのリーチング
		Reaching3D,		//< 3D上のロボットアームのリーチング＋障害物回避
		Toss3D,			//< 3D上の複数のロボットアームの物体搬送
		Welding,
	};

	struct Config {
		struct Welding {
			string pointsFilename;
			real_t startTime;
			real_t endTime;
			int    numTicks;
			vec3_t mockupOffset;
			bool   useTree;



		
			Welding();
		};

		Welding  welding;
	};

	int             sceneSelect;
	string          sceneFilename;
	string          confFilename;

	Config          conf;

	SceneLocal      scene;
	TypeDB          typedb;
	ModelContainer  models;
	Builder         builder;

	AdaptorDiMP		adaptorDiMP;
	AdaptorSprGR	adaptorSprGR;

	typedef vector< UTRef<RobotArm> >	Robots;
	Robots                      robot;			///< robotic arms;
	vector<DiMP::Object*>       target;		    ///< target objects
	vector<DiMP::Object*>       obstacle;		///< obstacles
	vector<DiMP::TimeSlot*>     timeSlot;		///< time slots
	vector<DiMP::MatchTask*>    matchTask;		///< match tasks
	vector<DiMP::AvoidTask*>    avoidTask;      ///< avoid tasks

	vector<vec3_t>  weldingPoints;   ///< weldingにおける溶接点列

public:
	void Read(XML& xml);

	void DrawSnapshot(real_t time);

	void EnableConstraints (string mode, bool enable);

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

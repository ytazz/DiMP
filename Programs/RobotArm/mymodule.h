#pragma once

#include <module/module.h>

#include <sbbuilder.h>
#include <sbscenelocal.h>
#include <DiMP/sbdimp.h>
#include <SprGraphics/sbsprgraphics.h>

class RobotArm;

class MyModule : public Module {
public:
	// �\�z���郍�{�b�g�A�[���̑I��
	enum {
		Reaching2D,		//< ���ʏ�̃��{�b�g�A�[���̃��[�`���O
		Reaching3D,		//< 3D��̃��{�b�g�A�[���̃��[�`���O�{��Q�����
		Toss3D,			//< 3D��̕����̃��{�b�g�A�[���̕��̔���
		Welding,
	};

	struct Config {
		struct Welding {
			string pointsFilename;
			real_t weldingStartTime;
			real_t weldingEndTime;
			real_t lowerStartTime;
			real_t lowerEndTime;
			int    lowerStartIndex;
			int    lowerEndIndex;
			int    lowerDiv;
			real_t upperStartTime;
			real_t upperEndTime;
			int    upperStartIndex;
			int    upperEndIndex;
			int    upperDiv;
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

	vector<vec3_t>  weldingPoints;   ///< welding�ɂ�����n�ړ_��

public:
	void Read(XML& xml);

	void DrawSnapshot(real_t time);

	void EnableConstraints (string mode, bool enable);

public:
	virtual bool Build();
	virtual bool OnRequest();	  ///< ���N�G�X�g����
	virtual void OnStep();
	virtual void OnDraw(DiMP::Render::Canvas* canvas);

	virtual bool Set(DiMP::Render::Canvas* canvas, int attr, DiMP::Node* node);

public:
	MyModule();
	~MyModule();

};

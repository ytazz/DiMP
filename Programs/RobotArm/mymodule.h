#pragma once

#include <module/module.h>

#include <sbbuilder.h>
#include <sbscenelocal.h>
#include <DiMP/sbdimp.h>
#include <SprGraphics/sbsprgraphics.h>

class RobotArm;

class MyModule : public Module{
public:
	// �\�z���郍�{�b�g�A�[���̑I��
	enum{
		Reaching2D,		//< ���ʏ�̃��{�b�g�A�[���̃��[�`���O
		Reaching3D,		//< 3D��̃��{�b�g�A�[���̃��[�`���O�{��Q�����
		Toss3D,			//< 3D��̕����̃��{�b�g�A�[���̕��̔���
		Welding,
	};

	struct Config{
		struct Welding{
			string pointsFilename;
			real_t startTime;
			real_t endTime;
			int    numTicks;
			vec3_t mockupOffset;

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
	vector<DiMP::TimeSlot*>     timeslot;		///< time slots
	vector<DiMP::MatchTask*>    task;			///< tasks

	vector<vec3_t>  weldingPoints;   ///< welding�ɂ�����n�ړ_��

public:
	void Read(XML& xml);

	void DrawSnapshot(real_t time);

public:
	virtual bool Build    ();
	virtual bool OnRequest();	  ///< ���N�G�X�g����
	virtual void OnStep   ();
	virtual void OnDraw   (DiMP::Render::Canvas* canvas);
	
public:
	 MyModule();
	~MyModule();

};
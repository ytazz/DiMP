#pragma once

#include <module/module.h>

class RobotArm;
class Workspace;

class MyModule : public Module {
public:
	// �\�z���郍�{�b�g�A�[���̑I��
	enum {
		Reaching2D,		    //< ���ʏ�̃��{�b�g�A�[���̃��[�`���O
		Reaching3D,		    //< 3D��̃��{�b�g�A�[���̃��[�`���O�{��Q�����
        Reaching3DTwoStage, //< 3D��̃��{�b�g�A�[���̃��[�`���O�{��Q������@�{�Ǐ��O���v��
		Toss3D,			    //< 3D��̕����̃��{�b�g�A�[���̕��̔���
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

	vector<vec3_t>  weldingPoints;   ///< welding�ɂ�����n�ړ_��

public:
	void Read(XML& xml);

	void DrawSnapshot(real_t time);

	void EnableConstraints (string mode, bool enable);
	void Report();

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

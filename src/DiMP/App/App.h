#pragma once

/**
	DiMP�T���v���v���O�������p�A�v���P�[�V�����N���X
	- Springhead2 SampleApp�𗬗p
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
	/** ���j���[ID
	 */
	enum MenuID{
		MENU_ALWAYS		= 0,			///< ���ł��L���ȃ��j���[
		MENU_USER		= 1,			///< �A�v���ŗL���j���[
		MENU_COMMON		= 2,            ///< ���L���j���[�@�^�u�Ő؂�ւ�
		MENU_CONFIG = MENU_COMMON,		///<  �p�����[�^�ݒ�n
		MENU_STATE,						///<  ������ԕ\��
		MENU_DRAW,						///<  �`��ݒ�n
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
		ID_CAMERA,
	};

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
		
		vector< pair<int, string> > keys;		///< �L�[�Ƒ�փe�L�X�g
		string	desc;						    ///< ����

		/// �A�N�V�����ƃL�[�̑Ή�
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
		string	        brief;       ///< ���j���[�̐���
		vector<Action>  actions;
		
		int Query(int key);  ///< �L�[�ɑΉ�����A�N�V����ID��Ԃ�
	};

	/// ����: �h���N���X���R���X�g���N�^�Őݒ肷��
	string					appName;		///< �T���v����
	Vec4f					clearColor;		///< �w�i�F
	Vec4f					textColor;		///< �����F
	bool					zAxisUp;		///< Z������ɂ��ĕ`��

	/// ���j���[�֌W
	vector<Menu>			menus;				///< ���j���[
	int						dispMenu;			///< �\�����̋��L���j���[
	int						focusMenu;			///< �t�H�[�J�X���̃��j���[
	int                     focusAction;		///< �t�H�[�J�X���̃A�N�V����
	stringstream			ss;
	string					message;			///< ��s���b�Z�[�W
	
	/// �^�C�}
	UTTimerIf*				timerDraw;		///< timer for rendering
	UTTimerIf*				timerPlan;		///< timer for planning
	UTQPTimer               ptimer;
	
	/// DiMP�I�u�W�F�N�g
	UTRef<Graph>	          graph;			///< reference to DiMP graph
	Render::Config*           conf;
	UTRef<Render::CanvasGL >  canvasGL;
	UTRef<Render::CanvasSVG>  canvasSVG;

	double  playTime;		///< play time
	int     iterCount;
	double  deltaNorm;
	double  compTime;
	
	/// ���
	bool    showHelp;		///< �w���v�\��
	bool    running;		///< running the planner
	bool    logging;		///< taking error log
	bool    playing;		///< playing the trajectory
	bool    renderMode;		///< render in solid or wireframe

public:
	void    AddMenu  (int mid, string brief);        ///< ���j���[�̓o�^
	Action* AddAction(int mid, int aid, string desc); ///< �A�N�V�����̓o�^
	Action* GetAction(int mid, int aid);              ///< �A�N�V�����擾
	void    HitAction(int mid, int aid, bool on);     ///< �A�N�V�������s
	
	void    DrawText (GRRenderIf* render, Vec2f pos, string str, bool bold); ///< �e�L�X�g�`��
	void    DrawState(GRRenderIf* render, Vec2f& offset);                    ///< ����v��̓�����Ԃ̕\��
	void    DrawMenu (GRRenderIf* render, int mid, Vec2f& offset);            ///< ���j���[�̕\��
	void    DrawHelp (GRRenderIf* render);                                   ///< �t�����̕\��

	void    SaveSVG  ();
	
public: /** �h���N���X����������֐� **/

	virtual void BuildScene(){}                  ///< �V�[���\�z���s���D
	virtual void OnStep    ();                   ///< 1�X�e�b�v�̃V�~�����[�V����
	virtual void OnDraw    (GRRenderIf* render); ///< �`��
	virtual void OnAction  (int menu, int id);   ///< �A�N�V��������
	
public: /** FWApp�̎��� **/
	virtual void Init     (int argc, char* argv[]);
	virtual void TimerFunc(int id);                 ///< �^�C�}�R�[���o�b�N�֐��D�^�C�}�����ŌĂ΂��
	virtual void Display  ();                       ///< �`��֐��D�`��v���������Ƃ��ɌĂ΂��
	virtual void Keyboard (int key, int x, int y);

    App();
	virtual ~App();
};

}

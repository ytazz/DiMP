#pragma once

#include <base/typedefs.h>

class RenderingManager;
class RequestManager;
class SimulationManager;

class Module : public DiMP::Render::Config{
public:
	static Module* instance;

	/// Springhead
	UTRef<FWSdkIf>		 fwSdk;
	GRSdkIf*			 grSdk;

	/// DiMP
	DiMP::Graph*		 graph;
	
	int     iterCount;
	int     compTime;
	bool    isPlaying;
	double  playTime;		///< play time
	double  deltaNorm;
	
	/// Managers
	RenderingManager*	 renManager;		///< �����_�����O
	RequestManager*		 reqManager;		///< ���N�G�X�g
	SimulationManager*   simManager;		///< �v�Z
		
	Event				 evExit;			///< �I���C�x���g
	CriticalSection		 cs;

public:
	static Module* Get();

	bool Init     (int argc, char* argv[]);		///< ������
	void MainLoop ();							///< ���C�����[�v
	void Cleanup  ();							///< �I������
	
	virtual bool Build    () = 0;
	virtual bool OnRequest();	                ///< ���N�G�X�g����
	virtual void OnStep   ();
	virtual void OnDraw   (DiMP::Render::Canvas* canvas);
	virtual void OnPrint  (deque<string>& lines);
	
public:
	 Module();
	~Module();

};

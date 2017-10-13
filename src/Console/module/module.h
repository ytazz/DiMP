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
	RenderingManager*	 renManager;		///< レンダリング
	RequestManager*		 reqManager;		///< リクエスト
	SimulationManager*   simManager;		///< 計算
		
	Event				 evExit;			///< 終了イベント
	CriticalSection		 cs;

public:
	static Module* Get();

	bool Init     (int argc, char* argv[]);		///< 初期化
	void MainLoop ();							///< メインループ
	void Cleanup  ();							///< 終了処理
	
	virtual bool Build    () = 0;
	virtual bool OnRequest();	                ///< リクエスト処理
	virtual void OnStep   ();
	virtual void OnDraw   (DiMP::Render::Canvas* canvas);
	virtual void OnPrint  (deque<string>& lines);
	
public:
	 Module();
	~Module();

};

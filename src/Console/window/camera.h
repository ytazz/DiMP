#pragma once

#include <base/typedefs.h>

#include <glwin/glwin.h>

class Camera : public GLWin::Camera{
public:
	
public:
	void Read(XMLNode* node);

};

/// カメラ切り替え器
class CameraSwitcher : public UTRefCount{
public:
	float		moveRate;		///< 目標までの移動所要時間
	
	Camera*		camera;			///< 切り替え先カメラのID

	Affinef		affProj;
	Affinef		affViewTarget;
	Affinef		affView;
	Affinef		affViewInv;

public:
	void Switch (Camera* c);			///< 視点切り替え
	void Step   (float dt);				///< 更新
	void Set    ();						///< 視点変換と射影変換を設定
	void OnEvent(SDL_Event* ev);

	CameraSwitcher();
};

#pragma once

#include <base/typedefs.h>

#include <glwin/glwin.h>

class Camera : public GLWin::Camera{
public:
	
public:
	void Read(XMLNode* node);

};

/// �J�����؂�ւ���
class CameraSwitcher : public UTRefCount{
public:
	float		moveRate;		///< �ڕW�܂ł̈ړ����v����
	
	Camera*		camera;			///< �؂�ւ���J������ID

	Affinef		affProj;
	Affinef		affViewTarget;
	Affinef		affView;
	Affinef		affViewInv;

public:
	void Switch (Camera* c);			///< ���_�؂�ւ�
	void Step   (float dt);				///< �X�V
	void Set    ();						///< ���_�ϊ��Ǝˉe�ϊ���ݒ�
	void OnEvent(SDL_Event* ev);

	CameraSwitcher();
};

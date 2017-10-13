#pragma once

#include <base/typedefs.h>

#include <sdl.h>

class Manager : public Thread{
public:
	CriticalSection		cs;	

public:
	virtual void Read   (XML& xml){}
	virtual bool Init   (){ return true; }
	virtual void Close  (){}
	virtual void OnEvent(SDL_Event* ev){}
	virtual void Func   (){}

public:
	 Manager();
	~Manager();
};

#pragma once

#include <DiMP/Graph/Node.h>

namespace DiMP{;

class Graph;
class Object;
class ObjectKey;
class TimeSlot;
class Task;

/** atomic task
	- ��{�^�X�N
	- 2�̍��̂̏�ԗʂ�����̎��ԋ�Ԃɂ����Ĉ�v������Ƃ����`��
 **/
class TaskKey : public ScheduledKey{
public:
	// �S���ΏۃI�u�W�F�N�g�̃L�[�|�C���g
	ObjectKey*		obj0;
	ObjectKey*		obj1;

public:
	virtual void AddVar(Solver* s);
	virtual void Draw  (Render::Canvas* canvas);
	
	TaskKey();
};

class Task : public ScheduledNode{
public:
	// �S���ΏۃI�u�W�F�N�g
	Object*		obj0;
	Object*		obj1;
	
public:
	Task(Object* _obj0, Object* _obj1, TimeSlot* _time, const string& n);
	virtual ~Task();
};

}

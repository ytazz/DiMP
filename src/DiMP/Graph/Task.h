#pragma once

#include <DiMP/Graph/Node.h>

namespace DiMP{;

class Graph;
class Object;
class ObjectKey;
class TimeSlot;
class Task;

/** atomic task
	- 基本タスク
	- 2つの剛体の状態量を所定の時間区間において一致させるという形式
 **/
class TaskKey : public ScheduledKey{
public:
	// 拘束対象オブジェクトのキーポイント
	ObjectKey*		obj0;
	ObjectKey*		obj1;

public:
	virtual void AddVar(Solver* s);
	virtual void Draw  (Render::Canvas* canvas);
	
	TaskKey();
};

class Task : public ScheduledNode{
public:
	// 拘束対象オブジェクト
	Object*		obj0;
	Object*		obj1;
	
public:
	Task(Object* _obj0, Object* _obj1, TimeSlot* _time, const string& n);
	virtual ~Task();
};

}

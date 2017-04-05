#include <DiMP2/DiMP.h>

/*
 * simple demo of planejoint
 * 
 */

class MyApp : public DiMP2::App, public DiMP2::DrawConfig{
public:
	DiMP2::Object*		object[2];
	DiMP2::Object*		target[2];
	DiMP2::Joint*		planejoint;
	DiMP2::MatchTask*	task[2];

public:
	MyApp(){
		appName = "Planejoint";
	}
	virtual ~MyApp(){}

	virtual void BuildScene(){
		graph->param.gravity = vec3_t();

		DiMP2::Box* box = new DiMP2::Box(graph, vec3_t(0.1, 0.1, 0.1));

		object[0] = new DiMP2::Object(graph, "floor");
		object[0]->param.dynamical = false;
		(new DiMP2::Connector(object[0]))->Attach(box);
		
		object[1] = new DiMP2::Object(graph, "object");
		(new DiMP2::Connector(object[1]))->Attach(box);

		planejoint = new DiMP2::Planejoint(
			new DiMP2::Connector(object[0]),
			new DiMP2::Connector(object[1]),
			0, "planejoint");

		target[0] = new DiMP2::Object(graph, "target0");
		(new DiMP2::Connector(target[0]))->Attach(box);
		target[0]->param.dynamical = false;
		target[0]->param.iniPos    = vec3_t(0.5, 0.5, 0.0);
		target[0]->param.iniOri    = quat_t::Rot(Rad(30.0), 'z');

		target[1] = new DiMP2::Object(graph, "target1");
		(new DiMP2::Connector(target[1]))->Attach(box);
		target[1]->param.dynamical = false;
		target[1]->param.iniPos    = vec3_t(-0.5, 1.5, 0.0);
		target[1]->param.iniOri    = quat_t::Rot(Rad(-45.0), 'z');

		task[0] = new DiMP2::MatchTask(object[1], target[0], new DiMP2::TimeSlot(graph, 1.0, 2.0, true, "slot0"), "task0");
		task[0]->param.match_rp = true;
		task[0]->param.match_rv = true;
		task[1] = new DiMP2::MatchTask(object[1], target[1], new DiMP2::TimeSlot(graph, 3.0, 4.0, true, "slot1"), "task1");
		task[1]->param.match_rp = true;
		task[1]->param.match_rv = true;
		
		for(real_t t = 0.0; t <= 5.0; t += 0.5)
			new DiMP2::Tick(graph, t);
		
		graph->Init();

		graph->SetPriority(DiMP2::ID(DiMP2::ConTag::MatchTP), 1);
		graph->SetPriority(DiMP2::ID(DiMP2::ConTag::MatchTV), 1);

		object[0]->ForwardKinematics();

	}

	virtual bool Set(GRRenderIf* render, int attr, DiMP2::Node* node){
		return true;
	}
	virtual float Scale(int attr, DiMP2::Node* node){
		return 0.1f;
	}


} app;

DiMP2::Graph graph;

int main(int argc, char* argv[]){
	app.graph = &graph;
	app.Init(argc, argv);
	app.StartMainLoop();
}

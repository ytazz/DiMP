#include <DiMP/DiMP.h>

/*
 * simple demo of planejoint
 * 
 */

class MyApp : public DiMP::App, public DiMP::Render::Config{
public:
	DiMP::Object*		object[2];
	DiMP::Object*		target[2];
	DiMP::Joint*		planejoint;
	DiMP::MatchTask*	task[2];

public:
	MyApp(){
		appName = "Planejoint";
	}
	virtual ~MyApp(){}

	virtual void BuildScene(){
		graph->param.gravity = vec3_t();

		DiMP::Box* box = new DiMP::Box(graph, vec3_t(0.1, 0.1, 0.1));

		object[0] = new DiMP::Object(graph, "floor");
		object[0]->param.dynamical = false;
		(new DiMP::Connector(object[0]))->Attach(box);
		
		object[1] = new DiMP::Object(graph, "object");
		(new DiMP::Connector(object[1]))->Attach(box);

		planejoint = new DiMP::Planejoint(
			new DiMP::Connector(object[0]),
			new DiMP::Connector(object[1]),
			0, "planejoint");

		target[0] = new DiMP::Object(graph, "target0");
		(new DiMP::Connector(target[0]))->Attach(box);
		target[0]->param.dynamical = false;
		target[0]->param.iniPos    = vec3_t(0.5, 0.5, 0.0);
		target[0]->param.iniOri    = quat_t::Rot(Rad(30.0), 'z');

		target[1] = new DiMP::Object(graph, "target1");
		(new DiMP::Connector(target[1]))->Attach(box);
		target[1]->param.dynamical = false;
		target[1]->param.iniPos    = vec3_t(-0.5, 1.5, 0.0);
		target[1]->param.iniOri    = quat_t::Rot(Rad(-45.0), 'z');

		task[0] = new DiMP::MatchTask(object[1], target[0], new DiMP::TimeSlot(graph, 1.0, 2.0, true, "slot0"), "task0");
		task[0]->param.match_rp = true;
		task[0]->param.match_rv = true;
		task[1] = new DiMP::MatchTask(object[1], target[1], new DiMP::TimeSlot(graph, 3.0, 4.0, true, "slot1"), "task1");
		task[1]->param.match_rp = true;
		task[1]->param.match_rv = true;
		
		for(real_t t = 0.0; t <= 5.0; t += 0.5)
			new DiMP::Tick(graph, t);
		
		graph->Init();

		graph->solver->SetPriority(ID(DiMP::ConTag::MatchTP), 1);
		graph->solver->SetPriority(ID(DiMP::ConTag::MatchTV), 1);

		object[0]->ForwardKinematics();

	}

	virtual bool Set(GRRenderIf* render, int attr, DiMP::Node* node){
		return true;
	}
	virtual float Scale(int attr, DiMP::Node* node){
		return 0.1f;
	}


} app;

DiMP::Graph graph;

int main(int argc, char* argv[]){
	app.graph = &graph;
	app.Init(argc, argv);
	app.StartMainLoop();
}

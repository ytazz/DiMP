#include <DiMP/DiMP.h>

/*
 * simple demo of balljoint
 * 
 */

class MyApp : public DiMP::App, public DiMP::Render::Config{
public:
	DiMP::Object*	object[2];
	DiMP::Object*	target[3];
	DiMP::Joint *	balljoint;

public:
	MyApp(){
		appName = "Balljoint";
	}
	virtual ~MyApp(){}

	virtual void BuildScene(){
		graph->param.gravity = vec3_t();

		object[0] = new DiMP::Object(graph, "floor");
		object[0]->param.dynamical = false;
		(new DiMP::Connector(object[0]))->Attach(new DiMP::Box(graph, vec3_t(1.0, 0.1, 1.0)));
			
		object[1] = new DiMP::Object(graph, "object");
		(new DiMP::Connector(object[1]))->Attach(new DiMP::Box(graph, vec3_t(0.1, 1.0, 0.1)));

		balljoint = new DiMP::Balljoint(
			new DiMP::Connector(object[0], vec3_t(0.0,  0.05, 0.0)),
			new DiMP::Connector(object[1], vec3_t(0.0, -0.5 , 0.0)),
			0, "balljoint");

		target[0] = new DiMP::Object(graph, "target0");
		(new DiMP::Connector(target[0]))->Attach(new DiMP::Sphere(graph, 0.1));
		target[0]->param.dynamical = false;
		target[0]->param.iniPos = vec3_t(0.5, 0.5, 0.0);

		target[1] = new DiMP::Object(graph, "target1");
		(new DiMP::Connector(target[1]))->Attach(new DiMP::Sphere(graph, 0.1));
		target[1]->param.dynamical = false;
		target[1]->param.iniPos = vec3_t(-0.5, 1.5, 1.0);

		target[2] = new DiMP::Object(graph, "target2");
		(new DiMP::Connector(target[2]))->Attach(new DiMP::Sphere(graph, 0.1));
		target[2]->param.dynamical = false;
		target[2]->param.iniPos = vec3_t(-0.5, 1.5, -1.0);

		new DiMP::MatchTask(object[1], target[0], new DiMP::TimeSlot(graph, 1.0, 2.0, true, "slot0"), "task0");
		new DiMP::MatchTask(object[1], target[1], new DiMP::TimeSlot(graph, 2.0, 3.5, true, "slot1"), "task1");
		new DiMP::MatchTask(object[1], target[2], new DiMP::TimeSlot(graph, 4.0, 5.0, true, "slot2"), "task2");

		for(real_t t = 0.0; t != 5.0; t += 0.5)
			new DiMP::Tick(graph, t);
		
		graph->Init();

		graph->SetPriority(DiMP::ID(DiMP::ConTag::MatchTP), 1);
		graph->SetPriority(DiMP::ID(DiMP::ConTag::MatchTV), 1);

		graph->Enable(DiMP::ID(DiMP::ConTag::ObjectC1T), false);
		graph->Enable(DiMP::ID(DiMP::ConTag::ObjectC1R), false);
		graph->Enable(DiMP::ID(DiMP::ConTag::MatchRP  ), false);
		graph->Enable(DiMP::ID(DiMP::ConTag::MatchRV  ), false);
		
		object[0]->ForwardKinematics();

	}

} app;

DiMP::Graph graph;

int main(int argc, char* argv[]){
	app.graph = &graph;
	app.Init(argc, argv);
	app.StartMainLoop();
}

#include <DiMP2/DiMP.h>

/*
 * simple demo of balljoint
 * 
 */

class MyApp : public DiMP2::App, public DiMP2::DrawConfig{
public:
	DiMP2::Object*	object[2];
	DiMP2::Object*	target[3];
	DiMP2::Joint *	balljoint;

public:
	MyApp(){
		appName = "Balljoint";
	}
	virtual ~MyApp(){}

	virtual void BuildScene(){
		graph->param.gravity = vec3_t();

		object[0] = new DiMP2::Object(graph, "floor");
		object[0]->param.dynamical = false;
		(new DiMP2::Connector(object[0]))->Attach(new DiMP2::Box(graph, vec3_t(1.0, 0.1, 1.0)));
			
		object[1] = new DiMP2::Object(graph, "object");
		(new DiMP2::Connector(object[1]))->Attach(new DiMP2::Box(graph, vec3_t(0.1, 1.0, 0.1)));

		balljoint = new DiMP2::Balljoint(
			new DiMP2::Connector(object[0], vec3_t(0.0,  0.05, 0.0)),
			new DiMP2::Connector(object[1], vec3_t(0.0, -0.5 , 0.0)),
			0, "balljoint");

		target[0] = new DiMP2::Object(graph, "target0");
		(new DiMP2::Connector(target[0]))->Attach(new DiMP2::Sphere(graph, 0.1));
		target[0]->param.dynamical = false;
		target[0]->param.iniPos = vec3_t(0.5, 0.5, 0.0);

		target[1] = new DiMP2::Object(graph, "target1");
		(new DiMP2::Connector(target[1]))->Attach(new DiMP2::Sphere(graph, 0.1));
		target[1]->param.dynamical = false;
		target[1]->param.iniPos = vec3_t(-0.5, 1.5, 1.0);

		target[2] = new DiMP2::Object(graph, "target2");
		(new DiMP2::Connector(target[2]))->Attach(new DiMP2::Sphere(graph, 0.1));
		target[2]->param.dynamical = false;
		target[2]->param.iniPos = vec3_t(-0.5, 1.5, -1.0);

		new DiMP2::MatchTask(object[1], target[0], new DiMP2::TimeSlot(graph, 1.0, 2.0, true, "slot0"), "task0");
		new DiMP2::MatchTask(object[1], target[1], new DiMP2::TimeSlot(graph, 2.0, 3.5, true, "slot1"), "task1");
		new DiMP2::MatchTask(object[1], target[2], new DiMP2::TimeSlot(graph, 4.0, 5.0, true, "slot2"), "task2");

		for(real_t t = 0.0; t != 5.0; t += 0.5)
			new DiMP2::Tick(graph, t);
		
		graph->Init();

		graph->SetPriority(DiMP2::ID(DiMP2::ConTag::MatchTP), 1);
		graph->SetPriority(DiMP2::ID(DiMP2::ConTag::MatchTV), 1);

		graph->Enable(DiMP2::ID(DiMP2::ConTag::ObjectC1T), false);
		graph->Enable(DiMP2::ID(DiMP2::ConTag::ObjectC1R), false);
		graph->Enable(DiMP2::ID(DiMP2::ConTag::MatchRP  ), false);
		graph->Enable(DiMP2::ID(DiMP2::ConTag::MatchRV  ), false);
		
		object[0]->ForwardKinematics();

	}

} app;

DiMP2::Graph graph;

int main(int argc, char* argv[]){
	app.graph = &graph;
	app.Init(argc, argv);
	app.StartMainLoop();
}

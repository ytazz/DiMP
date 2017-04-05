#include <DiMP2/DiMP.h>

/*
 * simple demo of slider
 * 
 */

class MyApp : public DiMP2::App, public DiMP2::DrawConfig{
public:
	DiMP2::Object*	object[2];
	DiMP2::Object*	target[2];
	DiMP2::Joint *	slider;

public:
	MyApp(){
		appName = "Slider";
	}
	virtual ~MyApp(){}

	virtual void BuildScene(){
		graph->param.gravity = vec3_t();

		object[0] = new DiMP2::Object(graph, "floor");
		object[0]->param.dynamical = false;
		(new DiMP2::Connector(object[0]))->Attach(new DiMP2::Box(graph, vec3_t(1.0, 0.1, 1.0)));
			
		object[1] = new DiMP2::Object(graph, "object");
		(new DiMP2::Connector(object[1]))->Attach(new DiMP2::Box(graph, vec3_t(0.1, 1.0, 0.1)));

		slider = new DiMP2::Slider(
			new DiMP2::Connector(object[0], vec3_t(0.0,  0.05, 0.0), quat_t::Rot(Rad(90.0), 'x')),
			new DiMP2::Connector(object[1], vec3_t(0.0, -0.5 , 0.0), quat_t::Rot(Rad(90.0), 'x')),
			0, "slider");

		target[0] = new DiMP2::Object(graph, "target0");
		target[0]->param.dynamical = false;
		target[0]->param.iniPos    = vec3_t(0.5, 0.5, 0.0);
		(new DiMP2::Connector(target[0]))->Attach(new DiMP2::Sphere(graph, 0.1));
		
		target[1] = new DiMP2::Object(graph, "target1");
		target[1]->param.dynamical = false;
		target[1]->param.iniPos    = vec3_t(-0.5, 1.5, 0.0);
		(new DiMP2::Connector(target[1]))->Attach(new DiMP2::Sphere(graph, 0.1));
		
		new DiMP2::MatchTask(object[1], target[0], new DiMP2::TimeSlot(graph, 1.0, 2.0));
		new DiMP2::MatchTask(object[1], target[1], new DiMP2::TimeSlot(graph, 3.0, 4.0));

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

int main(int argc, char* argv[]){
	app.Init(argc, argv);
	app.StartMainLoop();
}

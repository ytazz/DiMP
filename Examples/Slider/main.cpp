#include <DiMP/DiMP.h>

/*
 * simple demo of slider
 * 
 */

class MyApp : public DiMP::App, public DiMP::Render::Config{
public:
	DiMP::Object*	object[2];
	DiMP::Object*	target[2];
	DiMP::Joint *	slider;

public:
	MyApp(){
		appName = "Slider";
	}
	virtual ~MyApp(){}

	virtual void BuildScene(){
		graph->param.gravity = vec3_t();

		object[0] = new DiMP::Object(graph, "floor");
		object[0]->param.dynamical = false;
		(new DiMP::Connector(object[0]))->Attach(new DiMP::Box(graph, vec3_t(1.0, 0.1, 1.0)));
			
		object[1] = new DiMP::Object(graph, "object");
		(new DiMP::Connector(object[1]))->Attach(new DiMP::Box(graph, vec3_t(0.1, 1.0, 0.1)));

		slider = new DiMP::Slider(
			new DiMP::Connector(object[0], vec3_t(0.0,  0.05, 0.0), quat_t::Rot(Rad(90.0), 'x')),
			new DiMP::Connector(object[1], vec3_t(0.0, -0.5 , 0.0), quat_t::Rot(Rad(90.0), 'x')),
			0, "slider");

		target[0] = new DiMP::Object(graph, "target0");
		target[0]->param.dynamical = false;
		target[0]->param.iniPos    = vec3_t(0.5, 0.5, 0.0);
		(new DiMP::Connector(target[0]))->Attach(new DiMP::Sphere(graph, 0.1));
		
		target[1] = new DiMP::Object(graph, "target1");
		target[1]->param.dynamical = false;
		target[1]->param.iniPos    = vec3_t(-0.5, 1.5, 0.0);
		(new DiMP::Connector(target[1]))->Attach(new DiMP::Sphere(graph, 0.1));
		
		new DiMP::MatchTask(object[1], target[0], new DiMP::TimeSlot(graph, 1.0, 2.0));
		new DiMP::MatchTask(object[1], target[1], new DiMP::TimeSlot(graph, 3.0, 4.0));

		for(real_t t = 0.0; t <= 5.0; t += 0.5)
			new DiMP::Tick(graph, t);
		
		graph->Init();

		graph->SetPriority(DiMP::ID(DiMP::ConTag::MatchTP), 1);
		graph->SetPriority(DiMP::ID(DiMP::ConTag::MatchTV), 1);

		object[0]->ForwardKinematics();

	}

	virtual bool Set(GRRenderIf* render, int attr, DiMP::Node* node){
		return true;
	}
	virtual float Scale(int attr, DiMP::Node* node){
		return 0.1f;
	}


} app;

int main(int argc, char* argv[]){
	app.Init(argc, argv);
	app.StartMainLoop();
}

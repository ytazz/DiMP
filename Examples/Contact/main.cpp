#include <DiMP/DiMP.h>

/*
 * simple demo of contact
 * 
 */

class MyApp : public DiMP::App, public DiMP::Render::Config{
public:
	DiMP::Object *	 object[2];
	DiMP::Contact*  contact;

public:
	MyApp(){
		appName		= "Contact";
	}
	virtual ~MyApp(){}

	virtual void BuildScene(){
		graph->param.gravity = vec3_t(0.0, -0.1, 0.0);

		object[0] = new DiMP::Object(graph, "floor");
		object[0]->param.dynamical = false;
		(new DiMP::Connector(object[0]))->Attach(new DiMP::Box(graph, vec3_t(1.0, 0.1, 1.0)));
			
		object[1] = new DiMP::Object(graph, "object");
		object[1]->param.iniVel = vec3_t(0.1, 0.0, 0.0 );
		(new DiMP::Connector(object[1]))->Attach(new DiMP::Box(graph, vec3_t(0.2, 0.2, 0.2)));
			
		contact = new DiMP::Contact(
			new DiMP::Connector(object[0], vec3_t(0.0, 0.05, 0.0), quat_t::Rot(Rad(90.0), 'z')),
			new DiMP::Connector(object[1], vec3_t(0.0, -0.1, 0.0), quat_t::Rot(Rad(90.0), 'z')),
			0, "contact");
		contact->param.ini_p[0] =  0.5;
		contact->param.ini_v[1] = -0.1;
		contact->con_param.mu   =  0.0;
		
		for(real_t t = 0.0; t <= 5.0; t += 0.5)
			new DiMP::Tick(graph, t);

		graph->SetCorrectionRate(DiMP::ID(), 0.1);
		graph->Init();

		graph->Enable(DiMP::ID(DiMP::ConTag::ObjectC1T), false);
		graph->Enable(DiMP::ID(DiMP::ConTag::ObjectC1R), false);

		object[0]->ForwardKinematics();
	}

} app;

DiMP::Graph graph;

int main(int argc, char* argv[]){
	app.graph = &graph;
	app.conf  = &app;
	app.Init(argc, argv);
	app.StartMainLoop();
}

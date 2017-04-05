#include <DiMP2/DiMP.h>

/*
 * simple demo of contact
 * 
 */

class MyApp : public DiMP2::App, public DiMP2::DrawConfig{
public:
	DiMP2::Object *	 object[2];
	DiMP2::Contact*  contact;

public:
	MyApp(){
		appName		= "Contact";
	}
	virtual ~MyApp(){}

	virtual void BuildScene(){
		graph->param.gravity = vec3_t(0.0, -0.1, 0.0);

		object[0] = new DiMP2::Object(graph, "floor");
		object[0]->param.dynamical = false;
		(new DiMP2::Connector(object[0]))->Attach(new DiMP2::Box(graph, vec3_t(1.0, 0.1, 1.0)));
			
		object[1] = new DiMP2::Object(graph, "object");
		object[1]->param.iniVel = vec3_t(0.1, 0.0, 0.0 );
		(new DiMP2::Connector(object[1]))->Attach(new DiMP2::Box(graph, vec3_t(0.2, 0.2, 0.2)));
			
		contact = new DiMP2::Contact(
			new DiMP2::Connector(object[0], vec3_t(0.0, 0.05, 0.0), quat_t::Rot(Rad(90.0), 'z')),
			new DiMP2::Connector(object[1], vec3_t(0.0, -0.1, 0.0), quat_t::Rot(Rad(90.0), 'z')),
			0, "contact");
		contact->param.ini_p[0] =  0.5;
		contact->param.ini_v[1] = -0.1;
		contact->con_param.mu   =  0.0;
		
		for(real_t t = 0.0; t <= 5.0; t += 0.5)
			new DiMP2::Tick(graph, t);

		graph->SetCorrectionRate(DiMP2::ID(), 0.1);
		graph->Init();

		graph->Enable(DiMP2::ID(DiMP2::ConTag::ObjectC1T), false);
		graph->Enable(DiMP2::ID(DiMP2::ConTag::ObjectC1R), false);

		object[0]->ForwardKinematics();
	}

} app;

DiMP2::Graph graph;

int main(int argc, char* argv[]){
	app.graph    = &graph;
	app.drawConf = &app;
	app.Init(argc, argv);
	app.StartMainLoop();
}

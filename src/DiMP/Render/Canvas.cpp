#include <DiMP/Render/Canvas.h>

#include <GL/glut.h>

namespace DiMP{;
namespace Render{;

//-------------------------------------------------------------------------------------------------
// Canvas

Canvas::Canvas(){
	SetPointColor("black");
	SetLineColor ("black", 0);
	SetLineColor ("black", 1);
	SetFillColor ("white");
	SetPointSize (3.0f);
	SetLineWidth (1.0f);
	nCircleDiv	= 18;
	drawFill	= false;
	drawLine	= true;
	gradation	= false;
}
void Canvas::SetPointColor(const string& c, float alpha){
	if(pointColor.name != c){
		pointColor.name = c;
		pointColor.Init();
	}
	pointColor.rgba[3] = alpha;
}
void Canvas::SetLineColor(const string& c, uint idx, float alpha){
	if(lineColor[idx].name != c){
		lineColor[idx].name = c;
		lineColor[idx].Init();
	}
	lineColor[idx].rgba[3] = alpha;
}
void Canvas::SetFillColor(const string& c, float alpha){
	if(fillColor.name != c){
		fillColor.name = c;
		fillColor.Init();
	}
	fillColor.rgba[3] = alpha;
}
void Canvas::SetPointSize(float ps){
	pointSize = ps;
}
void Canvas::SetLineWidth(float lw){
	lineWidth = lw;
}
void Canvas::Translate(float tx, float ty){
	Translate(tx, ty, 0.0f);
}
void Canvas::Rotate(float r){
	Rotate(r, Vec3f(0.0f, 0.0f, 1.0f));
}
void Canvas::Scale(float sx, float sy){
	Scale(sx, sy, 1.0f);
}

//-------------------------------------------------------------------------------------------------
// CanvasGL

CanvasGL::CanvasGL(){

}
void CanvasGL::Push(){
	glPushMatrix();
}
void CanvasGL::Pop(){
	glPopMatrix();
}
void CanvasGL::Translate(float tx, float ty, float tz){
	glTranslatef(tx, ty, tz);
}
void CanvasGL::Rotate(float angle, const Vec3f& axis){
	glRotatef((float)Deg(angle), axis.x, axis.y, axis.z);
}
void CanvasGL::Scale(float sx, float sy, float sz){
	glScalef(sx, sy, sz);
}
void CanvasGL::Transform(const Affinef& aff){
	glMultMatrixf(aff);
}
void CanvasGL::SetProjMatrix(const Affinef& aff){
	glMatrixMode(GL_PROJECTION);
	glLoadMatrixf(aff);
}
void CanvasGL::SetViewMatrix(const Affinef& aff){
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(aff);
}

void CanvasGL::Line(const Vec2f& p0, const Vec2f& p1){
	glLineWidth(lineWidth);
	glBegin(GL_LINES);
	glColor4fv(lineColor[0].rgba);                                 glVertex3f(p0.x, p0.y, 0.0);
	glColor4fv(gradation ? lineColor[1].rgba : lineColor[0].rgba); glVertex3f(p1.x, p1.y, 0.0);
	glEnd();
}

void CanvasGL::Line(const Vec3f& p0, const Vec3f& p1){
	glLineWidth(lineWidth);
	glBegin(GL_LINES);
	glColor4fv(lineColor[0].rgba);                                 glVertex3f(p0.x, p0.y, p0.z);
	glColor4fv(gradation ? lineColor[1].rgba : lineColor[0].rgba); glVertex3f(p1.x, p1.y, p1.z);
	glEnd();
}

void CanvasGL::BeginPath(){
	glLineWidth(lineWidth);
	glBegin(GL_LINE_STRIP);
	glColor4fv(lineColor[0].rgba);
}

void CanvasGL::EndPath(){
	glEnd();
}

void CanvasGL::MoveTo(const Vec3f& p){
	glVertex3f(p.x, p.y, p.z);
}

void CanvasGL::LineTo(const Vec3f& p){
	glVertex3f(p.x, p.y, p.z);
}

void CanvasGL::Point(const Vec2f& p){
	glPointSize(pointSize);
	glBegin(GL_POINTS);
	glColor4fv(pointColor.rgba);
	glVertex3f(p.x, p.y, 0.0);
	glEnd();
}

void CanvasGL::Point(const Vec3f& p){
	glPointSize(pointSize);
	glBegin(GL_POINTS);
	glColor4fv(pointColor.rgba);
	glVertex3f(p.x, p.y, p.z);
	glEnd();
}

inline void Rect(const Vec2f& rmin, const Vec2f& rmax){
	glVertex3f(rmin.x, rmin.y, 0.0);
	glVertex3f(rmax.x, rmin.y, 0.0);
	glVertex3f(rmax.x, rmax.y, 0.0);
	glVertex3f(rmin.x, rmax.y, 0.0);
}

void CanvasGL::Rectangle(const Vec2f& rmin, const Vec2f& rmax){
	if(drawFill){
		glBegin(GL_TRIANGLE_FAN);
		glColor4fv(fillColor.rgba);
		Rect(rmin, rmax);
		glEnd();
	}
	if(drawLine){
		glLineWidth(lineWidth);
		glBegin(GL_LINE_LOOP);
		glColor4fv(lineColor[0].rgba);
		Rect(rmin, rmax);
		glEnd();
	}
}

inline void Circ(const Vec2f& c, float r, const Matrix2d& R, size_t ndiv){
	Vec2f p(r, 0.0);
	for(size_t i = 0; i <= ndiv; i++){
		glVertex3f(c.x + p.x, c.y + p.y, 0.0);
		p = R * p;
	}
}

void CanvasGL::Circle(const Vec2f& center, float r){
	Matrix2d R = Matrix2d::Rot(2.0 * M_PI / (float)nCircleDiv);
	if(drawFill){
		glBegin(GL_TRIANGLE_FAN);
		glColor4fv(fillColor.rgba);
		glVertex3f(center.x, center.y, 0.0);
		Circ(center, r, R, nCircleDiv);
		glEnd();
	}
	if(drawLine){
		glLineWidth(lineWidth);
		glBegin(GL_LINE_STRIP);
		glColor4fv(lineColor[0].rgba);
		Circ(center, r, R, nCircleDiv);
		glEnd();
	}
}

void CanvasGL::Box(const Vec3f& rmin, const Vec3f& rmax){
	if(drawFill){
		glColor4fv(fillColor.rgba);
		glBegin(GL_TRIANGLE_FAN);
		glVertex3f(rmin[0], rmin[1], rmin[2]);
		glVertex3f(rmin[0], rmax[1], rmin[2]);
		glVertex3f(rmin[0], rmax[1], rmax[2]);
		glVertex3f(rmin[0], rmin[1], rmax[2]);
		glEnd();

		glBegin(GL_TRIANGLE_FAN);
		glVertex3f(rmax[0], rmin[1], rmin[2]);
		glVertex3f(rmax[0], rmin[1], rmax[2]);
		glVertex3f(rmax[0], rmax[1], rmax[2]);
		glVertex3f(rmax[0], rmax[1], rmin[2]);
		glEnd();

		glBegin(GL_TRIANGLE_FAN);
		glVertex3f(rmin[0], rmin[1], rmin[2]);
		glVertex3f(rmin[0], rmin[1], rmax[2]);
		glVertex3f(rmax[0], rmin[1], rmax[2]);
		glVertex3f(rmax[0], rmin[1], rmin[2]);
		glEnd();

		glBegin(GL_TRIANGLE_FAN);
		glVertex3f(rmin[0], rmax[1], rmin[2]);
		glVertex3f(rmax[0], rmax[1], rmin[2]);
		glVertex3f(rmax[0], rmax[1], rmax[2]);
		glVertex3f(rmin[0], rmax[1], rmax[2]);
		glEnd();

		glBegin(GL_TRIANGLE_FAN);
		glVertex3f(rmin[0], rmin[1], rmin[2]);
		glVertex3f(rmax[0], rmin[1], rmin[2]);
		glVertex3f(rmax[0], rmax[1], rmin[2]);
		glVertex3f(rmin[0], rmax[1], rmin[2]);
		glEnd();

		glBegin(GL_TRIANGLE_FAN);
		glVertex3f(rmin[0], rmin[1], rmax[2]);
		glVertex3f(rmin[0], rmax[1], rmax[2]);
		glVertex3f(rmax[0], rmax[1], rmax[2]);
		glVertex3f(rmax[0], rmin[1], rmax[2]);
		glEnd();
	}
	if(drawLine){
		glLineWidth(lineWidth);
		glColor4fv(lineColor[0].rgba);

		glBegin(GL_LINE_LOOP);
		glVertex3f(rmin[0], rmin[1], rmin[2]);
		glVertex3f(rmin[0], rmax[1], rmin[2]);
		glVertex3f(rmin[0], rmax[1], rmax[2]);
		glVertex3f(rmin[0], rmin[1], rmax[2]);
		glEnd();

		glBegin(GL_LINE_LOOP);
		glVertex3f(rmax[0], rmin[1], rmin[2]);
		glVertex3f(rmax[0], rmax[1], rmin[2]);
		glVertex3f(rmax[0], rmax[1], rmax[2]);
		glVertex3f(rmax[0], rmin[1], rmax[2]);
		glEnd();

		glBegin(GL_LINES);
		glVertex3f(rmin[0], rmin[1], rmin[2]);
		glVertex3f(rmax[0], rmin[1], rmin[2]);
		glVertex3f(rmin[0], rmax[1], rmin[2]);
		glVertex3f(rmax[0], rmax[1], rmin[2]);
		glVertex3f(rmin[0], rmax[1], rmax[2]);
		glVertex3f(rmax[0], rmax[1], rmax[2]);
		glVertex3f(rmin[0], rmin[1], rmax[2]);
		glVertex3f(rmax[0], rmin[1], rmax[2]);
		glEnd();
	}
}

// precomputed array of cos and sin
const int angle_res = 12;
const double cos_array[] = {1.0, 0.86, 0.5,  0.0, -0.5 , -0.86, -1.0, -0.86, -0.5 ,  0.0,  0.5 ,  0.86, 1.0};
const double sin_array[] = {0.0, 0.5,  0.86, 1.0,  0.86,  0.5 ,  0.0, -0.5 , -0.86, -1.0, -0.86, -0.5 , 0.0};

void CanvasGL::Sphere(const Vec3f& center, float r){
	glLineWidth(lineWidth);
	glColor4fv(lineColor[0].rgba);

	/// three orthogonal circles
	glBegin(GL_LINE_LOOP);
	for(int i = 0; i < angle_res; i++){
		glVertex3f(r * cos_array[i], r * sin_array[i], 0.0f);
	}
	glEnd();
	glBegin(GL_LINE_LOOP);
	for(int i = 0; i < angle_res; i++){
		glVertex3f(0.0f, r * cos_array[i], r * sin_array[i]);
	}
	glEnd();
	glBegin(GL_LINE_LOOP);
	for(int i = 0; i < angle_res; i++){
		glVertex3f(r * sin_array[i], 0.0f, r * cos_array[i]);
	}
	glEnd();
	
}

void CanvasGL::Cylinder(float r, float l){
	glLineWidth(lineWidth);
	glColor4fv(lineColor[0].rgba);

	float l_half = 0.5 * l;
	// caps
	glBegin(GL_LINE_LOOP);
	for(int i = 0; i < angle_res; i++){
		glVertex3f(r * cos_array[i], r * sin_array[i], l_half);
	}
	glEnd();
	
	glBegin(GL_LINE_LOOP);
	for(int i = 0; i < angle_res; i++){
		glVertex3f(r * cos_array[i], r * sin_array[i], -l_half);
	}
	glEnd();
	
	// sides
	glBegin(GL_LINES);
	glVertex3f( r, 0.0f, l_half); glVertex3f( r, 0.0f, -l_half);
	glVertex3f(-r, 0.0f, l_half); glVertex3f(-r, 0.0f, -l_half);
	glVertex3f(0.0f,  r, l_half); glVertex3f(0.0f,  r, -l_half);
	glVertex3f(0.0f, -r, l_half); glVertex3f(0.0f, -r, -l_half);
	glEnd();
	
}

//-------------------------------------------------------------------------------------------------
// CanvasSVG

CanvasSVG::CanvasSVG(){
}
void CanvasSVG::Push(){
	svg.PushModelMatrix();
}
void CanvasSVG::Pop(){
	svg.PopModelMatrix();
}
void CanvasSVG::Translate(float tx, float ty, float tz){
	svg.MultModelMatrix(Affinef::Trn(tx, ty, tz));
}
void CanvasSVG::Rotate(float angle, const Vec3f& axis){
	svg.MultModelMatrix(Affinef::Rot(angle, axis));
}
void CanvasSVG::Scale(float sx, float sy, float sz){
	svg.MultModelMatrix(Affinef::Scale(sx, sy, sz));
}
void CanvasSVG::Transform(const Affinef& aff){
	svg.MultModelMatrix(aff);
}
void CanvasSVG::SetProjMatrix(const Affinef& aff){
	svg.SetProjMatrix(aff);
}
void CanvasSVG::SetViewMatrix(const Affinef& aff){
	svg.SetViewMatrix(aff);
}
void CanvasSVG::Point(const Vec2f& p){
	svg.DrawLine(p, p);
}
void CanvasSVG::Point(const Vec3f& p){
	svg.DrawLine(p, p);
}
void CanvasSVG::Line(const Vec2f& p0, const Vec2f& p1){
	svg.SetStrokeWidth(lineWidth);
	svg.DrawLine(p0, p1);
}
void CanvasSVG::Line(const Vec3f& p0, const Vec3f& p1){
	svg.SetStrokeWidth(lineWidth);
	svg.DrawLine(p0, p1);
}
void CanvasSVG::BeginPath(){
	svg.SetStrokeWidth(lineWidth);
	svg.BeginPath();
}
void CanvasSVG::EndPath(){
	svg.EndPath(false);
}
void CanvasSVG::MoveTo(const Vec3f& p){
	svg.MoveTo(p);
}
void CanvasSVG::LineTo(const Vec3f& p){
	svg.LineTo(p);
}
void CanvasSVG::Rectangle(const Vec2f& p0, const Vec2f& p1){
	svg.DrawRect(p0, p1);
}
void CanvasSVG::Circle(const Vec2f& p, float r){
	svg.DrawCircle(p, r);
}
void CanvasSVG::Box(const Vec3f& rmin, const Vec3f& rmax){
	svg.DrawBox(rmin, rmax);
}
void CanvasSVG::Sphere(const Vec3f& p, float r){
	svg.DrawSphere(p, r);
}
void CanvasSVG::BeginLayer(string name, bool shown){
	svg.BeginLayer(name, shown);
}
void CanvasSVG::EndLayer(){
	svg.EndLayer();
}

}
}

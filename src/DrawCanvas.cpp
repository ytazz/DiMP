#include <DiMP2/DrawCanvas.h>
#include <GL/glut.h>

namespace DiMP2{;

//-------------------------------------------------------------------------------------------------
// DrawCanvas

DrawCanvas::DrawCanvas(){
	SetLineColor("black", 0);
	SetLineColor("black", 1);
	SetFillColor("white");
	SetPointSize(3.0f);
	SetLineWidth(1.0f);
	nCircleDiv	= 18;
	drawFill	= false;
	drawLine	= true;
	gradation	= false;
}
void DrawCanvas::SetLineColor(const string& c, uint idx){
	lineColorName[idx] = c;
	Converter::ColorFromName(lineColorName[idx], lineColor[idx]);
}
void DrawCanvas::SetFillColor(const string& c){
	fillColorName = c;
	Converter::ColorFromName(fillColorName, fillColor);
}
void DrawCanvas::Translate(float tx, float ty){
	Translate(tx, ty, 0.0f);
}
void DrawCanvas::Rotate(float r){
	Rotate(r, Vec3f(0.0f, 0.0f, 1.0f));
}
void DrawCanvas::Scale(float sx, float sy){
	Scale(sx, sy, 1.0f);
}

//-------------------------------------------------------------------------------------------------
// DrawCanvasGL

DrawCanvasGL::DrawCanvasGL(){

}
void DrawCanvasGL::SetPointSize(float ps){
	glPointSize(ps);
}
void DrawCanvasGL::SetLineWidth(float lw){
	glLineWidth(lw);
}
void DrawCanvasGL::Push(){
	glPushMatrix();
}
void DrawCanvasGL::Pop(){
	glPopMatrix();
}
void DrawCanvasGL::Translate(float tx, float ty, float tz){
	glTranslatef(tx, ty, tz);
}
void DrawCanvasGL::Rotate(float angle, const Vec3f& axis){
	glRotatef(angle, axis.x, axis.y, axis.z);
}
void DrawCanvasGL::Scale(float sx, float sy, float sz){
	glScalef(sx, sy, sz);
}
void DrawCanvasGL::Transform(const Affinef& aff){
	glMultMatrixf(aff);
}
void DrawCanvasGL::SetProjMatrix(const Affinef& aff){
	glMatrixMode(GL_PROJECTION);
	glLoadMatrixf(aff);
}
void DrawCanvasGL::SetViewMatrix(const Affinef& aff){
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(aff);
}

void DrawCanvasGL::Line(const Vec2f& p0, const Vec2f& p1){
	glLineWidth(lineWidth);
	glBegin(GL_LINES);
	glColor4fv(lineColor[0]);                            glVertex3f(p0.x, p0.y, 0.0);
	glColor4fv(gradation ? lineColor[1] : lineColor[0]); glVertex3f(p1.x, p1.y, 0.0);
	glEnd();
}

void DrawCanvasGL::Line(const Vec3f& p0, const Vec3f& p1){
	glLineWidth(lineWidth);
	glBegin(GL_LINES);
	glColor4fv(lineColor[0]);                            glVertex3f(p0.x, p0.y, p0.z);
	glColor4fv(gradation ? lineColor[1] : lineColor[0]); glVertex3f(p1.x, p1.y, p1.z);
	glEnd();
}

void DrawCanvasGL::BeginPath(){
	glLineWidth(lineWidth);
	glBegin(GL_LINE_STRIP);
	glColor4fv(lineColor[0]);
}

void DrawCanvasGL::EndPath(){
	glEnd();
}

void DrawCanvasGL::MoveTo(const Vec3f& p){
	glVertex3f(p.x, p.y, p.z);
}

void DrawCanvasGL::LineTo(const Vec3f& p){
	glVertex3f(p.x, p.y, p.z);
}

void DrawCanvasGL::Point(const Vec2f& p){
	glPointSize(pointSize);
	glBegin(GL_POINTS);
	glColor4fv(lineColor[0]);
	glVertex3f(p.x, p.y, 0.0);
	glEnd();
}

void DrawCanvasGL::Point(const Vec3f& p){
	glPointSize(pointSize);
	glBegin(GL_POINTS);
	glColor4fv(lineColor[0]);
	glVertex3f(p.x, p.y, p.z);
	glEnd();
}

inline void Rect(const Vec2f& rmin, const Vec2f& rmax){
	glVertex3f(rmin.x, rmin.y, 0.0);
	glVertex3f(rmax.x, rmin.y, 0.0);
	glVertex3f(rmax.x, rmax.y, 0.0);
	glVertex3f(rmin.x, rmax.y, 0.0);
}

void DrawCanvasGL::Rectangle(const Vec2f& rmin, const Vec2f& rmax){
	if(drawFill){
		glBegin(GL_TRIANGLE_FAN);
		glColor4fv(fillColor);
		Rect(rmin, rmax);
		glEnd();
	}
	if(drawLine){
		glLineWidth(lineWidth);
		glBegin(GL_LINE_LOOP);
		glColor4fv(lineColor[0]);
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

void DrawCanvasGL::Circle(const Vec2f& center, float r){
	Matrix2d R = Matrix2d::Rot(2.0 * M_PI / (float)nCircleDiv);
	if(drawFill){
		glBegin(GL_TRIANGLE_FAN);
		glColor4fv(fillColor);
		glVertex3f(center.x, center.y, 0.0);
		Circ(center, r, R, nCircleDiv);
		glEnd();
	}
	if(drawLine){
		glLineWidth(lineWidth);
		glBegin(GL_LINE_STRIP);
		glColor4fv(lineColor[0]);
		Circ(center, r, R, nCircleDiv);
		glEnd();
	}
}

void DrawCanvasGL::Box(const Vec3f& rmin, const Vec3f& rmax){
	if(drawFill){
		glColor4fv(fillColor);
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
		glColor4fv(lineColor[0]);

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

void DrawCanvasGL::Sphere(const Vec3f& center, float r){
	glLineWidth(lineWidth);
	glColor4fv(lineColor[0]);

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

void DrawCanvasGL::Cylinder(float r, float l){
	glLineWidth(lineWidth);
	glColor4fv(lineColor[0]);

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
// DrawCanvasSVG

DrawCanvasSVG::DrawCanvasSVG(){
}
void DrawCanvasSVG::SetPointSize(float ps){
	
}
void DrawCanvasSVG::SetLineWidth(float lw){
	svg.SetStrokeWidth(lw);
}
void DrawCanvasSVG::Push(){
	svg.PushModelMatrix();
}
void DrawCanvasSVG::Pop(){
	svg.PopModelMatrix();
}
void DrawCanvasSVG::Translate(float tx, float ty, float tz){
	svg.MultModelMatrix(Affinef::Trn(tx, ty, tz));
}
void DrawCanvasSVG::Rotate(float angle, const Vec3f& axis){
	svg.MultModelMatrix(Affinef::Rot(angle, axis));
}
void DrawCanvasSVG::Scale(float sx, float sy, float sz){
	svg.MultModelMatrix(Affinef::Scale(sx, sy, sz));
}
void DrawCanvasSVG::Transform(const Affinef& aff){
	svg.MultModelMatrix(aff);
}
void DrawCanvasSVG::SetProjMatrix(const Affinef& aff){
	svg.SetProjMatrix(aff);
}
void DrawCanvasSVG::SetViewMatrix(const Affinef& aff){
	svg.SetViewMatrix(aff);
}
void DrawCanvasSVG::Point(const Vec2f& p){
	svg.DrawLine(p, p);
}
void DrawCanvasSVG::Point(const Vec3f& p){
	svg.DrawLine(p, p);
}
void DrawCanvasSVG::Line(const Vec2f& p0, const Vec2f& p1){
	svg.DrawLine(p0, p1);
}
void DrawCanvasSVG::Line(const Vec3f& p0, const Vec3f& p1){
	svg.DrawLine(p0, p1);
}
void DrawCanvasSVG::BeginPath(){
	svg.BeginPath();
}
void DrawCanvasSVG::EndPath(){
	svg.EndPath(false);
}
void DrawCanvasSVG::MoveTo(const Vec3f& p){
	svg.MoveTo(p);
}
void DrawCanvasSVG::LineTo(const Vec3f& p){
	svg.LineTo(p);
}
void DrawCanvasSVG::Rectangle(const Vec2f& p0, const Vec2f& p1){
	svg.DrawRect(p0, p1);
}
void DrawCanvasSVG::Circle(const Vec2f& p, float r){
	svg.DrawCircle(p, r);
}
void DrawCanvasSVG::Box(const Vec3f& rmin, const Vec3f& rmax){
	svg.DrawBox(rmin, rmax);
}
void DrawCanvasSVG::Sphere(const Vec3f& p, float r){
	svg.DrawSphere(p, r);
}
void DrawCanvasSVG::BeginLayer(string name, bool shown){
	svg.BeginLayer(name, shown);
}
void DrawCanvasSVG::EndLayer(){
	svg.EndLayer();
}

}

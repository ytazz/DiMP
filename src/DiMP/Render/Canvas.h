#pragma once

#include <DiMP/Types.h>

#include <stack>
#include <map>

#include <sbimagesvg.h>

namespace DiMP{;
namespace Render{;

class Canvas : public UTRefCount{
public:
	Color   pointColor;
	Color   lineColor[2];		///< line color
	Color   fillColor;		    ///< fill color
	
	float   pointSize;			///< point size
	float   lineWidth;			///< line width
	size_t	nCircleDiv;			///< number of corners for drawing circle

	bool	drawLine;			///< draw outline?
	bool	drawFill;			///< fill inside shape?
	bool	transparent;		///< enable transparent?
	bool	gradation;			///< line gradation?

	void SetPointColor(const string& c, float alpha = 1.0f);
	void SetLineColor (const string& c, uint idx = 0, float alpha = 1.0f);
	void SetFillColor (const string& c, float alpha = 1.0f);
	void SetPointSize (float ps);
	void SetLineWidth (float lw);
	void Translate    (float tx, float ty);
	void Rotate       (float r);
	void Scale        (float sx, float sy);
	
	virtual void Push           (){}
	virtual void Pop            (){}
	virtual void Translate      (float tx, float ty, float tz){}
	virtual void Rotate         (float angle, const Vec3f& axis){}
	virtual void Scale          (float sx, float sy, float sz){}
	virtual void Transform      (const Affinef& aff){}
	virtual void SetProjMatrix  (const Affinef& aff){}
	virtual void SetViewMatrix  (const Affinef& aff){}
	virtual void SetViewportSize(float sx, float sy){}
	virtual void Point          (const Vec2f& p){}
	virtual void Point          (const Vec3f& p){}
	virtual void Line           (const Vec2f& p0, const Vec2f& p1){}
	virtual void Line           (const Vec3f& p0, const Vec3f& p1){}
	virtual void BeginPath      (){}
	virtual void EndPath        (){}
	virtual void MoveTo         (const Vec3f& p){}
	virtual void LineTo         (const Vec3f& p){}
	virtual void Rectangle      (const Vec2f& rmin, const Vec2f& rmax){}
	virtual void Circle         (const Vec2f& center, float r){}
	virtual void Box            (const Vec3f& rmin, const Vec3f& rmax){}
	virtual void Sphere         (const Vec3f& center, float r){}
	virtual void Cylinder       (float r, float l){}
	virtual void BeginLayer     (string name, bool shown){}
	virtual void EndLayer       (){}

	Canvas();
};

class CanvasGL : public Canvas{
public:
	virtual void Push         ();
	virtual void Pop          ();
	virtual void Translate    (float tx, float ty, float tz);
	virtual void Rotate       (float angle, const Vec3f& axis);
	virtual void Scale        (float sx, float sy, float sz);
	virtual void Transform    (const Affinef& aff);
	virtual void SetProjMatrix(const Affinef& aff);
	virtual void SetViewMatrix(const Affinef& aff);
	virtual void Point        (const Vec2f& p);
	virtual void Point        (const Vec3f& p);
	virtual void Line         (const Vec2f& p0,   const Vec2f& p1);
	virtual void Line         (const Vec3f& p0,   const Vec3f& p1);
	virtual void BeginPath    ();
	virtual void EndPath      ();
	virtual void MoveTo       (const Vec3f& p);
	virtual void LineTo       (const Vec3f& p);
	virtual void Rectangle    (const Vec2f& rmin, const Vec2f& rmax);
	virtual void Circle       (const Vec2f& center, float r);
	virtual void Box          (const Vec3f& rmin, const Vec3f& rmax);
	virtual void Sphere       (const Vec3f& center, float r);
	virtual void Cylinder     (float r, float l);

	CanvasGL();
};

class CanvasSVG : public Canvas{
public:
	SVG  svg;

public:
	virtual void Push         ();
	virtual void Pop          ();
	virtual void Translate    (float tx, float ty, float tz);
	virtual void Rotate       (float angle, const Vec3f& axis);
	virtual void Scale        (float sx, float sy, float sz);
	virtual void Transform    (const Affinef& aff);
	virtual void SetProjMatrix(const Affinef& aff);
	virtual void SetViewMatrix(const Affinef& aff);
	virtual void Point        (const Vec2f& p);
	virtual void Point        (const Vec3f& p);
	virtual void Line         (const Vec2f& p0, const Vec2f& p1);
	virtual void Line         (const Vec3f& p0, const Vec3f& p1);
	virtual void BeginPath    ();
	virtual void EndPath      ();
	virtual void MoveTo       (const Vec3f& p);
	virtual void LineTo       (const Vec3f& p);
	virtual void Rectangle    (const Vec2f& p0, const Vec2f& p1);
	virtual void Circle       (const Vec2f& p, float r);
	virtual void Box          (const Vec3f& p0, const Vec3f& p1);
	virtual void Sphere       (const Vec3f& p, float r);
	virtual void BeginLayer   (string name, bool shown);
	virtual void EndLayer     ();

	CanvasSVG();
};

}
}

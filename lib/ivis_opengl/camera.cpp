/*
 T his file is par*t of Warzone 2100.
 Copyright (C) 2007-2011  Warzone 2100 Project
 
 Warzone 2100 is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.
 
 Warzone 2100 is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with Warzone 2100; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
 */

#ifndef CAMERA_CPP
#define CAMERA_CPP

#include "camera.h"

#ifdef info
// An Eigen class has an info() member function which this conflicts with this
#undef info
#endif

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "lib/framework/frame.h"
#include "lib/framework/fixedpoint.h"
#include "lib/framework/opengl.h"

// Degrees to radians
#define RAD(x) ((x)*0.01745329251994329577)

Eye::Eye() {}
Eye::~Eye() {}

void Eye::updateCamRefFrame()
{
	using Eigen::AngleAxisd;
	using Eigen::Vector3d;
	/*
	 * forward = Trans(pos)*RotY(rot.y)*RotX(rot.x)*RotZ(rot.z)*Scale(scale)*Trans(0,0,distance)
	 * eyeL = normalize(forward.col(0))
	 * eyeU = normalize(forward.col(1))
	 * eyeF = normalize(forward.col(2))
	 * eye = forward.col(3)
	 */
	Eigen::Matrix3d rot;
	rot = AngleAxisd(FP2RAD(this->rot.y), Vector3d::UnitY())
		* AngleAxisd(FP2RAD(this->rot.x), Vector3d::UnitX())
		* AngleAxisd(FP2RAD(this->rot.z), Vector3d::UnitZ());
	/*
	 * Convert to a right handed camera referential frame
	 * remember: the camera faces the -Z axis with no transformations.
	 * so we negate the first and third columns.
	 */
	eyeL.x = -rot(0,0); eyeU.x = rot(0,1); eyeF.x = -rot(0,2);
	eyeL.y = -rot(1,0); eyeU.y = rot(1,1); eyeF.y = -rot(1,2);
	eyeL.z = -rot(2,0); eyeU.z = rot(2,1); eyeF.z = -rot(2,2);

	eye.x = pos.x + rot(0,2)*scale*distance;
	eye.y = pos.y + rot(1,2)*scale*distance;
	eye.z = pos.z + rot(2,2)*scale*distance;
}

void Eye::setEyeAsWzCam (Vector3i const &p, Vector3i const &r, float dist, float s)
{
	pos = p;
	rot = r;
	distance = dist;
	scale = s;
	updateCamRefFrame();
}

void Eye::setEyeAsgluLookAt(Vector3f const &eye, Vector3f const &observed, Vector3f const &up)
{
	this->eye = eye;
	Vector3f EO = observed - eye;
	pos = observed;
	eyeF = normalize(EO);
	eyeL = normalize(crossProduct(up, eyeF));
	eyeU = crossProduct(eyeF, eyeL);
	distance = sqrt(EO*EO);
	scale = 1.f;
	rot.x = RAD2FP(asin(eyeF.y));
	rot.y = RAD2FP(acos(-eyeF.z/cos(rot.x)));
	rot.z = RAD2FP(acos(eyeU.y/cos(rot.y)));
}

void Eye::setObservedPos(Vector3i const &p)
{
	// No need to update everything
	eye -= pos;
	eye += p;
	pos = p;
}

void Eye::setEyeRot(Vector3i const &r)
{
	rot = r;
	updateCamRefFrame();
}

void Eye::setScaling(float s)
{
	scale = s;
	updateCamRefFrame();
}

void Eye::setEyeDistance(float dist)
{
	distance = dist;
	updateCamRefFrame();
}

Vector3f Eye::getEyePos() const { return eye; }
Vector3f Eye::getObservedPos() const { return pos; }
Vector3f Eye::getEyeL() const { return eyeL;}
Vector3f Eye::getEyeU() const { return eyeU;}
Vector3f Eye::getEyeF() const { return eyeF;}
Vector3i Eye::getEyeRot() const { return rot; }
float Eye::getEyeScaling() const { return scale; }
float Eye::getEyeDistance() const { return distance; }

/* forward = I*Trans(pos)*RotY(rot.y)*RotX(rot.x)*RotZ(rot.z)*Scale(scale)*Trans(0,0,distance)
 * View = inv(forward)
 * View = Trans(0,0,-distance)*Scale(1/scale)*RotZ(-rot.z)*RotX(-rot.x)*RotY(-rot.y)*Trans(-pos)
 */
void Eye::getViewMatrix(float mat[4][4], ViewMatrixType type) const
{
	using Eigen::AngleAxisf;
	using Eigen::Translation3f;
	using Eigen::Scaling;
	using Eigen::Vector3f;
	Eigen::Affine3f view;
	view = Translation3f(0, 0, -distance)
			* Scaling(1.f/scale)
			* AngleAxisf(FP2RAD(-this->rot.z), Vector3f::UnitZ())
			* AngleAxisf(FP2RAD(-this->rot.x), Vector3f::UnitX())
			* AngleAxisf(FP2RAD(-this->rot.y), Vector3f::UnitY());
	if (type == NORMAL)
	{
		view *= Translation3f(-eye.x, -eye.y, -eye.z);
	}
	else // if (type == XZ_RELATIVE)
	{
		view *= Translation3f(0, -eye.y, 0);
	}
	memcpy(mat, view.matrix().data(), sizeof(float)*16);
}

void Eye::getViewMatrix(double mat[4][4], ViewMatrixType type) const
{
	using Eigen::AngleAxisd;
	using Eigen::Translation3d;
	using Eigen::Scaling;
	using Eigen::Vector3d;
	Eigen::Affine3d view;
	view = Translation3d(0, 0, -distance)
		* Scaling(1./scale)
		* AngleAxisd(-FP2RAD(this->rot.z), Vector3d::UnitZ())
		* AngleAxisd(-FP2RAD(this->rot.x), Vector3d::UnitX())
		* AngleAxisd(-FP2RAD(this->rot.y), Vector3d::UnitY());
	if (type == NORMAL)
	{
		view *= Translation3d(-eye.x, -eye.y, -eye.z);
	}
	else // if (type == XZ_RELATIVE)
	{
		view *= Translation3d(0, -eye.y, 0);
	}
	memcpy(mat, view.matrix().data(), sizeof(double)*16);
}

void Eye::getViewMatrix3x4(float mat[4][3], ViewMatrixType type) const
{
	using Eigen::AngleAxisf;
	using Eigen::Translation3f;
	using Eigen::Scaling;
	using Eigen::Vector3f;
	Eigen::AffineCompact3f view;
	view = Translation3f(0, 0, -distance)
			* Scaling(1.f/scale)
			* AngleAxisf(-FP2RAD(this->rot.z), Vector3f::UnitZ())
			* AngleAxisf(-FP2RAD(this->rot.x), Vector3f::UnitX())
			* AngleAxisf(-FP2RAD(this->rot.y), Vector3f::UnitY());
	if (type == NORMAL)
	{
		view *= Translation3f(-eye.x, -eye.y, -eye.z);
	}
	else // if (type == XZ_RELATIVE)
	{
		view *= Translation3f(0, -eye.y, 0);
	}
	memcpy(mat, view.matrix().data(), sizeof(float)*12);
}

void Eye::getViewMatrix3x4(double mat[4][3], ViewMatrixType type) const
{
	using Eigen::AngleAxisd;
	using Eigen::Translation3d;
	using Eigen::Scaling;
	using Eigen::Vector3d;
	Eigen::AffineCompact3d view;
	view = Translation3d(0, 0, -distance)
			* Scaling(1./scale)
			* AngleAxisd(-FP2RAD(this->rot.z), Vector3d::UnitZ())
			* AngleAxisd(-FP2RAD(this->rot.x), Vector3d::UnitX())
			* AngleAxisd(-FP2RAD(this->rot.y), Vector3d::UnitY());
	if (type == NORMAL)
	{
		view *= Translation3d(-eye.x, -eye.y, -eye.z);
	}
	else // if (type == XZ_RELATIVE)
	{
		view *= Translation3d(0, -eye.y, 0);
	}
	memcpy(mat, view.matrix().data(), sizeof(double)*12);
}

/*////////////////////////////////
 * Helpers for common functions between
 * the two camera classes
 */

static inline void camSetVertsFromPlanes(Plane const p[6], Vector3f v[2][2][2])
{
	#define printIntersectError() debug(LOG_ERROR, "Could not find plane intersection!")
	if (!intersect(p[NEARp], p[LEFTp], p[BOTTOMp],	&v[0][0][0])) printIntersectError();
	if (!intersect(p[NEARp], p[LEFTp], p[TOPp],	&v[0][0][1])) printIntersectError();
	if (!intersect(p[NEARp], p[RIGHTp],p[BOTTOMp],	&v[0][1][0])) printIntersectError();
	if (!intersect(p[NEARp], p[RIGHTp],p[TOPp],	&v[0][1][1])) printIntersectError();
	if (!intersect(p[FARp],  p[LEFTp], p[BOTTOMp],	&v[1][0][0])) printIntersectError();
	if (!intersect(p[FARp],  p[LEFTp], p[TOPp],	&v[1][0][1])) printIntersectError();
	if (!intersect(p[FARp],  p[RIGHTp],p[BOTTOMp],	&v[1][1][0])) printIntersectError();
	if (!intersect(p[FARp],  p[RIGHTp],p[TOPp],	&v[1][1][1])) printIntersectError();
	#undef printIntersectError
}

static inline void camSetPlanesFromVerts(Plane p[6], Vector3f const v[2][2][2])
{
	p[BOTTOMp].from3Points(v[1][0][0], v[0][0][0], v[0][1][0]);
	p[TOPp].from3Points(v[0][1][1], v[0][0][1], v[1][0][1]);
	p[LEFTp].from3Points(v[1][0][0], v[0][0][0], v[0][0][1]);
	p[RIGHTp].from3Points(v[1][1][0], v[0][1][0], v[1][1][1]);
	p[NEARp].from3Points(v[0][1][0], v[0][0][0], v[0][0][1]);
	p[FARp].from3Points(v[1][0][1], v[1][0][0], v[1][1][0]);
}

static inline OBB2D camComputeBAs(Vector3f const v[2][2][2], AABB2D *aabb)
{
	/* We make the assumption that there's nothing to see
	 * below the xz plane to further minimize the bounding area
	 */
	// 10 = maximum vertices after intersection with xz plane
	const Vector3f* const _v = (const Vector3f*)v;
	Vector2f verts[10];
	int numverts = 0;
	int i, j;
	bool vertAboveXZ[8];
	
	for (i = 0; i < 8; ++i)
	{
		//vertAboveXZ[i] = v[(i>>2)&1][(i>>1)&1][i&1].y >= 0;
		vertAboveXZ[i] = _v[i].y >= 0;
		if (vertAboveXZ[i])
			//verts[numverts++] = v[(i>>2)&1][(i>>1)&1][i&1].r_xz();
			verts[numverts++] = _v[i].r_xz();
	}
	// find where the edges intersect the plane
	for (i = 0; i < 8; ++i)
	{
		if (vertAboveXZ[i])
		{
			//const Vector3f * const a = &v[(i>>2)&1][(i>>1)&1][i&1];
			const Vector3f * const a = &_v[i];
			
			// We are only considering vertices which are connected by an edge
			// i.e. Whose indices for the v variable differ by 1 bit.
			for (j = 1; j < 8; j<<=1)
			{
				const int jXi = j^i;
				if (!vertAboveXZ[jXi])
				{
					//const Vector3f * const b = &v[(jXi>>2)&1][(jXi>>1)&1][jXi&1];
					const Vector3f * const b = &_v[jXi];
					const float factor = a->y / (a->y - b->y);
					verts[numverts++] = a->r_xz() + (b->r_xz() - a->r_xz())*factor;
				}
			}
		}
	}
	
	return OBB2D::createOptimal(numverts, verts, aabb);
}

static inline void camRenderEye(Vector3f const v[2][2][2],
								Vector3f const &eye,
								Vector3f const &eyeF,
								float farZ)
{
	glPushAttrib(GL_COLOR_BUFFER_BIT | GL_ENABLE_BIT);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_ALPHA_TEST);
	glDisable(GL_CULL_FACE);

	glColor4f(0,0,1,.5); // transparent blue
	glBegin(GL_TRIANGLES);

	// Right side
	glVertex3f(v[0][1][0].x, v[0][1][0].y, v[0][1][0].z);
	glVertex3f(v[0][1][1].x, v[0][1][1].y, v[0][1][1].z);
	glVertex3f(eye.x, eye.y, eye.z);

	// top side
	glVertex3f(v[0][0][1].x, v[0][0][1].y, v[0][0][1].z);
	glVertex3f(v[0][1][1].x, v[0][1][1].y, v[0][1][1].z);
	glVertex3f(eye.x, eye.y, eye.z);
	glEnd();

	glColor4f(0,1,0,.5); // transparent green
	glBegin(GL_TRIANGLES);
	// left side
	glVertex3f(v[0][0][0].x, v[0][0][0].y, v[0][0][0].z);
	glVertex3f(v[0][0][1].x, v[0][0][1].y, v[0][0][1].z);
	glVertex3f(eye.x, eye.y, eye.z);
	glEnd();

	glColor4f(1,0,0,.5); // transparent red
	glBegin(GL_TRIANGLES);
	// bottom
	glVertex3f(v[0][0][0].x, v[0][0][0].y, v[0][0][0].z);
	glVertex3f(v[0][1][0].x, v[0][1][0].y, v[0][1][0].z);
	glVertex3f(eye.x, eye.y, eye.z);
	glEnd();

	// ray for forward axis
	glBegin(GL_LINES);
	glVertex3f(eye.x, eye.y, eye.z);
	glVertex3f(eye.x+eyeF.x*farZ, eye.y+eyeF.y*farZ, eye.z+eyeF.z*farZ);
	glEnd();

	glPopAttrib();
}

void Camera::setVerticesFromPlanes()
{
	camSetVertsFromPlanes(p,v);
}

void Camera::setPlanesFromVertices()
{
	camSetPlanesFromVerts(p,v);
}

void Camera::computeBAs()
{
	obb = camComputeBAs(v, &aabb);
}

void Camera::extractFrustum(double aspect)
{
	const double demiNearH = nearZ * vFactor;
	const double demiNearW = demiNearH * aspect; 
	const double demiFarH = farZ * vFactor;
	const double demiFarW = demiFarH * aspect;
	const Vector3f eye = getEyePos();
	const Vector3f eyeL = getEyeL();
	const Vector3f eyeU = getEyeU();
	const Vector3f eyeF = getEyeF();

	// far and near centers
	Vector3f nearC = eye + eyeF * nearZ;
	Vector3f farC = eye + eyeF * farZ;
	
	// compute the vertices of the frustum
	v[0][0][0] = nearC + eyeL * demiNearW - eyeU * demiNearH;
	v[0][0][1] = nearC + eyeL * demiNearW + eyeU * demiNearH;
	v[0][1][0] = nearC - eyeL * demiNearW - eyeU * demiNearH;
	v[0][1][1] = nearC - eyeL * demiNearW + eyeU * demiNearH;
	v[1][0][0] = farC  + eyeL * demiFarW - eyeU * demiFarH;
	v[1][0][1] = farC  + eyeL * demiFarW + eyeU * demiFarH;
	v[1][1][0] = farC  - eyeL * demiFarW - eyeU * demiFarH;
	v[1][1][1] = farC  - eyeL * demiFarW + eyeU * demiFarH;

	Vector3f point, normal;

	point = nearC + eyeL*demiNearW;
	normal = crossProduct(point-eye, eyeU);
	this->p[LEFTp].fromNormalAndPoint(normal, point);
	
	point = nearC - eyeL*demiNearW;
	normal = crossProduct(eyeU, point-eye);
	this->p[RIGHTp].fromNormalAndPoint(normal, point);
	
	point = nearC - eyeU*demiNearH;
	normal = crossProduct(eyeL, point-eye);
	this->p[BOTTOMp].fromNormalAndPoint(normal, point);
	
	point = nearC + eyeU*demiNearH;
	normal = crossProduct(point-eye, eyeL);
	this->p[TOPp].fromNormalAndPoint(normal, point);
	
	this->p[FARp].fromNormalAndPoint(eyeF, farC);
	this->p[NEARp].fromNormalAndPoint(-eyeF, nearC);

	computeBAs();
}

Camera::Camera() {}
Camera::~Camera() {}

void Camera::setAsGluPerspective(double fovY, double aspect, double zNear, double zFar)
{
	using namespace std;
	const double radDemiFovY = RAD(fovY*.5);

	nearZ = zNear;
	farZ = zFar;

	vFactor = tan(radDemiFovY);
	hFactor = vFactor * aspect;
	radiusVFactor = 1./cos(radDemiFovY);
	radiusHFactor = 1./cos(atan(hFactor));
	extractFrustum(aspect);
}

void Camera::setAsGlFrustum(double l, double r, double b, double t, double nearVal, double farVal)
{
	const double nearH = t-b;
	const double nearW = r-l; 
	setAsGlFrustum(nearH, nearW, nearVal, farVal);
}

void Camera::setAsGlFrustum(double nearH, double nearW, double nearVal, double farVal)
{
	using namespace std;
	const double aspect = nearW/nearH;
	const double tanDemiFovY = nearH/(2.0*nearVal);
	nearZ = nearVal;
	farZ = farVal;
	vFactor = tanDemiFovY;
	hFactor = vFactor * aspect;
	radiusVFactor = 1./cos(atan(tanDemiFovY));
	radiusHFactor = 1./cos(atan(hFactor));
	extractFrustum(aspect);
}

/*
 * See article in Game Programming Gems 5
 * entitled “Improved Frustum Culling”
 * for more details.
 * Or see: http://www.lighthouse3d.com/tutorials/view-frustum-culling/
 */

bool Camera::pointContained(const Vector3f &p) const
{
	Vector3f OP = p - getEyePos();

	const float f = OP * getEyeF();
	if (f < nearZ || farZ < f)
		return false;

	const float h = OP * getEyeL();
	const float hLimit = hFactor * f;
	if (std::abs(h) > hLimit)
		return false;

	const float v = OP * getEyeU();
	const float vLimit = vFactor * f;
	if (std::abs(v) > vLimit)
		return false;

	return true;
}

bool Camera::sphereIntersectb(const Vector3f &center, float radius) const
{
	const Vector3f OP = center - getEyePos();
	const float f = OP * getEyeF();
	if (f < nearZ - radius || farZ + radius < f)
		return false;

	const float h = std::abs(OP * getEyeL());
	const float hLimit = hFactor * f + radiusHFactor * radius;
	if (h > hLimit)
		return false;

	const float v = std::abs(OP * getEyeU());
	const float vLimit = vFactor * f + radiusVFactor * radius;
	if (v > vLimit)
		return false;

	return true;
}

Intersect Camera::sphereIntersectt(const Vector3f &center, float radius) const
{
	const Vector3f OP = center - getEyePos();
	const float f = OP * getEyeF();

	if (f < nearZ - radius || farZ + radius < f)
		return Disjoint;

	const float h = std::abs(OP * getEyeL());
	const float hCompRad = radiusHFactor * radius;
	const float hLimit = hFactor * f;
	if (h > hLimit + hCompRad)
		return Disjoint;

	const float v = std::abs(OP * getEyeU());
	const float vCompRad = radiusVFactor * radius;
	const float vLimit = vFactor * f;
	if (v > vLimit + vCompRad)
		return Disjoint;

	if (f > farZ - radius || f < nearZ + radius)
		return Partial;
	if (v > vLimit - vCompRad)
		return Partial;
	if (h > hLimit - hCompRad)
		return Partial;

	return Contained;
}

OBB2D Camera::get2DOBB() const { return obb; }
AABB2D Camera::get2DAABB() const { return aabb; }

void Camera::renderEye() const
{
	const Vector3f eye = getEyePos();
	const Vector3f eyeF = getEyeF();
	camRenderEye(v, eye, eyeF, farZ);
}

Plane const & Camera::getPlane(Planes plane) const
{
#ifndef NDEBUG
	ASSERT(plane <= 5 && plane >= 0, "plane out of range!");
#endif
	return p[plane];
}

Vector3f const & Camera::getVertex(bool far, bool right, bool top) const
{
	return v[far][right][top];
}

void GeneralCamera::computeBAs()
{
	obb = camComputeBAs(v, &aabb);
}

// void GeneralCamera::extractFrustum()

GeneralCamera::GeneralCamera() {}

GeneralCamera::~GeneralCamera() {}


void GeneralCamera::setAsGlFrustum(double l, double r, double b, double t, double nearVal, double farVal)
{
	using namespace std;
	nearZ = nearVal;
	farZ = farVal;
	tFactor = t/nearVal;
	bFactor = -b/nearVal;
	lFactor = -l/nearVal;
	rFactor = r/nearVal;
	radiusTFactor = 1./cos(atan(tFactor));
	radiusBFactor = 1./cos(atan(bFactor));
	radiusLFactor = 1./cos(atan(lFactor));
	radiusRFactor = 1./cos(atan(rFactor));
	const double nearT = nearZ * tFactor;
	const double nearB = nearZ * bFactor;
	const double nearL = nearZ * lFactor;
	const double nearR = nearZ * rFactor;
	const double farT = farZ * tFactor;
	const double farB = farZ * bFactor;
	const double farL = farZ * lFactor;
	const double farR = farZ * rFactor;
	const Vector3f eye = getEyePos();
	const Vector3f eyeL = getEyeL();
	const Vector3f eyeU = getEyeU();
	const Vector3f eyeF = getEyeF();

	// far and near centers
	Vector3f nearC = eye + eyeF * nearZ;
	Vector3f farC = eye + eyeF * farZ;

	// compute the vertices of the frustum
	v[0][0][0] = nearC + eyeL * nearL - eyeU * nearB;
	v[0][0][1] = nearC + eyeL * nearL + eyeU * nearT;
	v[0][1][0] = nearC - eyeL * nearR - eyeU * nearB;
	v[0][1][1] = nearC - eyeL * nearR + eyeU * nearT;
	v[1][0][0] = farC  + eyeL * farL - eyeU * farB;
	v[1][0][1] = farC  + eyeL * farL + eyeU * farT;
	v[1][1][0] = farC  - eyeL * farR - eyeU * farB;
	v[1][1][1] = farC  - eyeL * farR + eyeU * farT;

	Vector3f point, normal;

	point = nearC + eyeL*nearL;
	normal = crossProduct(point-eye, eyeU);
	this->p[LEFTp].fromNormalAndPoint(normal, point);

	point = nearC + eyeL*nearR;
	normal = crossProduct(eyeU, point-eye);
	this->p[RIGHTp].fromNormalAndPoint(normal, point);

	point = nearC + eyeU*nearB;
	normal = crossProduct(eyeL, point-eye);
	this->p[BOTTOMp].fromNormalAndPoint(normal, point);

	point = nearC + eyeU*nearT;
	normal = crossProduct(point-eye, eyeL);
	this->p[TOPp].fromNormalAndPoint(normal, point);

	this->p[FARp].fromNormalAndPoint(eyeF, farC);
	this->p[NEARp].fromNormalAndPoint(-eyeF, nearC);

	computeBAs();
}


bool GeneralCamera::pointContained(const Vector3f &p) const
{
	Vector3f OP = p - getEyePos();

	const float f = OP * getEyeF();
	if (f < nearZ || farZ < f)
		return false;

	const float h = OP * getEyeL();
	const float lLimit = lFactor * f;

	if (h > lLimit)
		return false;

	const float rLimit = -rFactor * f;

	if (h < rLimit)
		return false;


	const float v = OP * getEyeU();
	const float tLimit = tFactor * f;

	if (v > tLimit)
		return false;

	const float bLimit = -bFactor * f;

	if (v < bLimit)
		return false;

	return true;
}


bool GeneralCamera::sphereIntersectb(const Vector3f &center, float radius) const
{
	const Vector3f OP = center - getEyePos();
	const float f = OP * getEyeF();
	if (f < nearZ - radius || farZ + radius < f)
		return false;
	
	const float h = OP * getEyeL();
	const float lLimit = lFactor * f + radiusLFactor * radius;
	if (h > lLimit)
		return false;

	const float rLimit = -(rFactor * f + radiusRFactor * radius);
	if (h < rLimit)
		return false;

	const float v = OP * getEyeU();
	const float tLimit = tFactor * f + radiusTFactor * radius;
	if (v > tLimit)
		return false;

	const float bLimit = -(bFactor * f + radiusBFactor * radius);
	if (v < bLimit)
		return false;

	return true;
}

Intersect GeneralCamera::sphereIntersectt(const Vector3f &p, float radius) const
{
	const Vector3f OP = p - getEyePos();
	const float f = OP * getEyeF();
	if (f < nearZ - radius || farZ + radius < f)
		return Disjoint;
	
	const float h = OP * getEyeL();
	const float lCompRad = radiusLFactor * radius;
	const float lLimit = lFactor * f;
	if (h > lLimit + lCompRad)
		return Disjoint;

	const float rCompRad = radiusRFactor * radius;
	const float rLimit = -rFactor * f;
	if (h < rLimit-rCompRad)
		return Disjoint;

	const float v = OP * getEyeU();
	const float tCompRad = radiusTFactor * radius;
	const float tLimit = tFactor * f;
	if (v > tLimit + tCompRad)
		return Disjoint;

	const float bCompRad = radiusBFactor * radius;
	const float bLimit =  -bFactor * f;
	if (v < bLimit - bCompRad)
		return Disjoint;

	if (f > farZ - radius || f < nearZ + radius)
		return Partial;

	if (v < bLimit+bCompRad)
		return Partial;
	if (v > tLimit-tCompRad)
		return Partial;

	if (h < rLimit+rCompRad)
		return Partial;
	if (h > lLimit-lCompRad)
		return Partial;
	
	return Contained;
}

OBB2D GeneralCamera::get2DOBB() const { return obb; }
AABB2D GeneralCamera::get2DAABB() const { return aabb; }


void GeneralCamera::renderEye() const
{
	const Vector3f eye = getEyePos();
	const Vector3f eyeF = getEyeF();
	camRenderEye(v, eye, eyeF, farZ);
}


Plane const & GeneralCamera::getPlane(Planes plane) const
{
#ifndef NDEBUG
	ASSERT(plane <= 5 && plane >= 0, "plane out of range!");
#endif
	return p[plane];
}

Vector3f const & GeneralCamera::getVertex(bool far, bool right, bool top) const
{
	return v[far][right][top];
}

#endif // CAMERA_CPP
/*
	This file is part of Warzone 2100.
	Copyright (C) 1999-2004  Eidos Interactive
	Copyright (C) 2005-2011  Warzone 2100 Project

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
/** \file
 *  Matrix manipulation functions.
 */

#include "lib/framework/frame.h"
#include "lib/framework/opengl.h"

#include "lib/framework/fixedpoint.h"
#include "lib/ivis_opengl/pieclip.h"
#include "piematrix.h"
#include "lib/ivis_opengl/piemode.h"

#ifdef info
// An Eigen class has an info() member function which this conflicts with this
#undef info
#endif

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <stack>

/***************************************************************************/
/*
 *	Local Definitions
 */
/***************************************************************************/

#define MATRIX_MAX 8

/**
 * 3x4 matrix, that's used as a 4x4 matrix with the last row containing
 * [ 0 0 0 1 ].
 */
typedef double MatScalarType;
typedef Eigen::Transform<MatScalarType, 3, Eigen::AffineCompact> Affine3;
typedef Affine3::VectorType Vector3;

typedef std::vector<Affine3, Eigen::aligned_allocator<Affine3> > stdVectorAffine3;

// Model View Matrix Stack
static std::stack<Affine3, stdVectorAffine3> matrixStack;

#define curMatrix matrixStack.top()
#define curMatrixAs4x4 Eigen::Transform<MatScalarType,3, Eigen::Affine>(curMatrix).matrix()

typedef Eigen::Vector4d Vector4;

static int viewport[2][2];

static Camera * cam;

/*
 * A general perspective transformation
 * It is stored as the following
 * sparse 4x4 matrix:
 * [ a 0  c 0 ]
 * [ 0 b  d 0 ]
 * [ 0 0  e f ]
 * [ 0 0 -1 0 ]
 * Note: c and d are unnecessary for symmetric frusta
 */
class PerspectiveTransform
{
	typedef MatScalarType s;
	s a,b,c,d,e,f;
public:
	PerspectiveTransform() {}
	void setAsglFrustum(s left, s right, s bottom, s top, s nearVal, s farVal)
	{
		a = 2*nearVal/(right-left);
		b = 2*nearVal/(top-bottom);
		c = (right+left)/(right-left);
		d = (top+bottom)/(top-bottom);
		e = -(farVal+nearVal)/(farVal-nearVal);
		f = -2*farVal*nearVal/(farVal-nearVal);
	}
	/*
	 * This is meant as an optimization
	 * by not doing the whole 4x4 matrix multiplication
	 * for projecting a vector
	 */
	inline Vector4 operator *(Vector4 const &v) const
	{
		Vector4 result;
		result.x() = a * v.x() + c * v.z();
		result.y() = b * v.y() + d * v.z();
		result.z() = e * v.z() + f * v.w();
		result.w() = -v.z();
		return result;
	}
	inline Vector4 operator *(Vector3 const &v) const
	{
		Vector4 result;
		result.x() = a * v.x() + c * v.z();
		result.y() = b * v.y() + d * v.z();
		result.z() = e * v.z() + f;
		result.w() = -v.z();
		return result;
	}
};

static PerspectiveTransform Proj;
#ifndef NDEBUG
static bool current_is_perspective;
#endif

//*************************************************************************

/** Sets up transformation matrices
 */
void pie_MatInit(void)
{
	// init matrix stack
	if (!matrixStack.empty())
	{
		while (matrixStack.size()>1)
		{
			matrixStack.pop();
			glPopMatrix();
		}
		matrixStack.pop();
	}

	matrixStack.push(Affine3::Identity());
	glLoadIdentity();

	cam = NULL;

#ifndef NDEBUG
	current_is_perspective = false;
#endif
}

void pie_SetViewport(int x, int y, int width, int height)
{
	viewport[0][0] = x;
	viewport[0][1] = width;
	viewport[1][0] = y;
	viewport[1][1] = height;
}

void pie_SetCamera(Camera * camera)
{
	cam = camera;
}

//*************************************************************************
//*** create new matrix from current transformation matrix and make current
//*
//******

void pie_MatBegin(void)
{
	ASSERT(matrixStack.size() < MATRIX_MAX, "past top of the stack" );

	matrixStack.push(matrixStack.top());
	glPushMatrix();
}


//*************************************************************************
//*** make current transformation matrix previous one on stack
//*
//******

void pie_MatEnd(void)
{
	matrixStack.pop();
	ASSERT(!matrixStack.empty(), "past the bottom of the stack" );
	glPopMatrix();
}

void pie_MatIdentity(void)
{
	curMatrix = Affine3::Identity();
}

void pie_MatLoadView(Eye::ViewMatrixType viewMatType)
{
	float viewMat[4][4];
	cam->getViewMatrix(viewMat, viewMatType);
	glLoadMatrixf((float*)viewMat);
	for (int c = 0; c < 4; ++c)
		for (int r = 0; r < 3; ++r)
			curMatrix.matrix()(r,c) = viewMat[c][r];
}

void pie_TRANSLATE(int32_t x, int32_t y, int32_t z)
{
	/*
	 * curMatrix = curMatrix . translationMatrix(x, y, z)
	 *
	 *                         [ 1 0 0 x ]
	 *                         [ 0 1 0 y ]
	 * curMatrix = curMatrix . [ 0 0 1 z ]
	 *                         [ 0 0 0 1 ]
	 */
	curMatrix.translate(Vector3(x,y,z));
	glTranslatef(x, y, z);
}

//*************************************************************************
//*** matrix scale current transformation matrix
//*
//******
void pie_MatScale(float scale)
{
	/*
	 * s := scale
	 * curMatrix = curMatrix . scaleMatrix(s, s, s)
	 *
	 *                         [ s 0 0 0 ]
	 *                         [ 0 s 0 0 ]
	 * curMatrix = curMatrix . [ 0 0 s 0 ]
	 *                         [ 0 0 0 1 ]
	 *
	 * curMatrix = scale * curMatrix
	 */

	curMatrix.scale(scale);
	glScalef(scale, scale, scale);
}

//*************************************************************************
//*** matrix scale current transformation matrix
//*
//******
void pie_MatScale(float x, float y, float z)
{
	/*
	 * curMatrix = curMatrix . scaleMatrix(x, y, z)
	 *
	 *                         [ x 0 0 0 ]
	 *                         [ 0 y 0 0 ]
	 * curMatrix = curMatrix . [ 0 0 z 0 ]
	 *                         [ 0 0 0 1 ]
	 *
	 * curMatrix = scale * curMatrix
	 */

	curMatrix.scale(Vector3(x, y, z));
	glScalef(x, y, z);
}

//*************************************************************************
//*** matrix rotate y (yaw) current transformation matrix
//*
//******

void pie_MatRotY(uint16_t y)
{
	/*
	 * a := angle
	 * c := cos(a)
	 * s := sin(a)
	 *
	 * curMatrix = curMatrix . rotationMatrix(a, 0, 1, 0)
	 *
	 *                         [  c  0  s  0 ]
	 *                         [  0  1  0  0 ]
	 * curMatrix = curMatrix . [ -s  0  c  0 ]
	 *                         [  0  0  0  1 ]
	 */
	if (y != 0)
	{
		curMatrix.rotate(Eigen::AngleAxis<MatScalarType>(FP2RAD(y), Vector3::UnitY()));
		glRotatef(UNDEG(y), 0.0f, 1.0f, 0.0f);
	}
}


//*************************************************************************
//*** matrix rotate z (roll) current transformation matrix
//*
//******

void pie_MatRotZ(uint16_t z)
{
	/*
	 * a := angle
	 * c := cos(a)
	 * s := sin(a)
	 *
	 * curMatrix = curMatrix . rotationMatrix(a, 0, 0, 1)
	 *
	 *                         [ c  -s  0  0 ]
	 *                         [ s   c  0  0 ]
	 * curMatrix = curMatrix . [ 0   0  1  0 ]
	 *                         [ 0   0  0  1 ]
	 */
	if (z != 0)
	{
		curMatrix.rotate(Eigen::AngleAxis<MatScalarType>(FP2RAD(z), Vector3::UnitZ()));
		glRotatef(UNDEG(z), 0.0f, 0.0f, 1.0f);
	}
}


//*************************************************************************
//*** matrix rotate x (pitch) current transformation matrix
//*
//******

void pie_MatRotX(uint16_t x)
{
	/*
	 * a := angle
	 * c := cos(a)
	 * s := sin(a)
	 *
	 * curMatrix = curMatrix . rotationMatrix(a, 0, 0, 1)
	 *
	 *                         [ 1  0   0  0 ]
	 *                         [ 0  c  -s  0 ]
	 * curMatrix = curMatrix . [ 0  s   c  0 ]
	 *                         [ 0  0   0  1 ]
	 */
	if (x != 0.f)
	{
		curMatrix.rotate(Eigen::AngleAxis<MatScalarType>(FP2RAD(x), Vector3::UnitX()));
		glRotatef(UNDEG(x), 1.0f, 0.0f, 0.0f);
	}
}


/*!
 * 3D vector perspective projection
 * Projects 3D vector into 2D screen space
 * \param v3d       3D vector to project
 * \param[out] v3d  resulting 2D vector + depth in a 3D vector
 * \return Whether the vertice is outside the clipping planes.
 */
bool pie_Project(Vector3f const &obj, Vector3i *proj)
{
#ifndef NDEBUG
	ASSERT(current_is_perspective, "called in orthographic projection mode.");
#endif
	// v = Proj * ( ModelView * obj)
	Vector4 v(Proj*(curMatrix*Vector4(obj.x, obj.y, obj.z, 1)));
	// v in clip coords.

	if (std::abs(v.w()) <= 0.000001) // At infinity (magic number = made up epsilon)
	{
		*proj = Vector3i(-1,-1,-1);
		return true;
	}

	v *= 1/v.w(); // Perpective division
	// v now in normalized device coords.

	// Most calling code needs this anyways, that's why we do it here
	// while it is clean and cheap
	bool clipped = std::abs(v.x()) > 1 || std::abs(v.y()) > 1 || std::abs(v.z()) > 1;

	/*
	 * The following contains another vestige of Warzone's DirectX past:
	 * Mouse coords and UI positioning assume a top left origin
	 * So for the sake of consistency we use this convention everywhere,
	 * and there is no point swiching it until the UI is rewritten.
	 * The commented out line is the bottom left version.
	 */
	// Map x, y and z to range [0,1]
	v.x() = v.x() * 0.5 + 0.5;
// 	v.y() = v.y() * 0.5 + 0.5;
	v.y() = 0.5 - v.y() * 0.5;
	v.z() = v.z() * 0.5 + 0.5;

	// Map to window coords
	proj->x = v.x() * viewport[0][1] + viewport[0][0];
	proj->y = v.y() * viewport[1][1] + viewport[1][0];
	proj->z = v.z() * MAX_Z;
	/*
	 * Though the multiplication by MAX_Z is a workaround
	 * it should be consistent with what people have assumed about the depth values.
	 * This is so that the float->int conversion doesn't kill the [0-1] depth.
	 */

	return clipped;
}

bool pie_Project(Vector3i *proj)
{
	// See other pie_Project for comments.
#ifndef NDEBUG
	ASSERT(current_is_perspective, "called in orthographic projection mode.");
#endif
	Vector4 v(Proj*Vector3(curMatrix.translation())); // Vector3 construction is a workaround...
	if (std::abs(v.w()) <= 0.000001) // At infinity (magic number = made up epsilon)
	{
		*proj = Vector3i(-1,-1,-1);
		return true;
	}
	v *= 1/v.w();
	bool clipped = std::abs(v.x()) > 1 || std::abs(v.y()) > 1 || std::abs(v.z()) > 1;
	v.x() = v.x() * 0.5 + 0.5;
	v.y() = 0.5 - v.y() * 0.5;
	v.z() = v.z() * 0.5 + 0.5;
	proj->x = v.x() * viewport[0][1] + viewport[0][0];
	proj->y = v.y() * viewport[1][1] + viewport[1][0];
	proj->z = v.z() * MAX_Z;
	return clipped;
}

bool pie_ProjectSphere(Vector3f const &obj, int32_t &radius, Vector3i *proj)
{
	Vector3f ptOnSphere = obj;
	Vector3i ptOnSphereProj;
	bool clipped;

#ifndef NDEBUG
	ASSERT(current_is_perspective, "called in orthographic projection mode.");
#endif

	ptOnSphere.y += radius;
	clipped = pie_Project(ptOnSphere, proj);
	if (!clipped)
	{
		/* For now just take the point on the bottom of the sphere
		 * this can be changed later to be something more meaningful (such as the min or max radius)
		 */
		pie_Project(obj, &ptOnSphereProj);
		radius = iHypot(ptOnSphereProj.r_xy() - proj->r_xy());
	}
	else
	{
		radius = 0;
	}
	return clipped;
}
bool pie_ProjectSphere(int32_t &radius, Vector3i *proj)
{
	// See other pie_ProjectSphere for comments.
	Vector3f ptOnSphere(0,radius,0);
	Vector3i ptOnSphereProj;
	bool clipped;
#ifndef NDEBUG
	ASSERT(current_is_perspective, "called in orthographic projection mode.");
#endif

	clipped = pie_Project(proj);
	if (!clipped)
	{
		pie_Project(ptOnSphere, &ptOnSphereProj);
		radius = iHypot(ptOnSphereProj.r_xy() - proj->r_xy());
	}
	else
	{
		radius = 0;
	}
	return clipped;
}

void pie_SetPerspectiveProj()
{
	const float width = pie_GetVideoBufferWidth();
	const float height = pie_GetVideoBufferHeight();
	const float left = -width/6.0f;
	const float right = width/6.0f;
	const float bottom = -height/6.0f;
	const float top = height/6.f;
	const int nearVal = 330;
	const int farVal = 49152;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glFrustum(left, right, bottom, top, nearVal, farVal);
	glMatrixMode(GL_MODELVIEW);

	Proj.setAsglFrustum(left, right, bottom, top, nearVal, farVal);
	cam->setAsGlFrustum(left, right, bottom, top, nearVal, farVal);

#ifndef NDEBUG
	current_is_perspective = true;
#endif
}

void pie_SetOrthoProj(bool originAtTheTop, float nearDepth, float farDepth)
{
	const double left = 0.0;
	const double right = pie_GetVideoBufferWidth();
	const double bottom = originAtTheTop ? pie_GetVideoBufferHeight() : 0.0;
	const double top = originAtTheTop ? 0.0 : pie_GetVideoBufferHeight();
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(left, right, bottom, top, nearDepth, farDepth);
	glMatrixMode(GL_MODELVIEW);
#ifndef NDEBUG
	current_is_perspective = false;
#endif
}

void pie_GetModelViewMatrix(float * const mat)
{
	const Eigen::Transform<float,3, Eigen::Affine>::MatrixType
	MV (curMatrixAs4x4.cast<float>());
	memcpy(mat, MV.data(), 16*sizeof(float));
}

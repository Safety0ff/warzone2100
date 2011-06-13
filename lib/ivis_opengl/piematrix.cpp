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
#include <stack>

/***************************************************************************/
/*
 *	Local Definitions
 */
/***************************************************************************/

/**
 * 3x4 matrix, that's used as a 4x4 matrix with the last row containing
 * [ 0 0 0 1 ].
 */
typedef Eigen::Transform<Fixed, 3, Eigen::AffineCompact> Transform;
typedef Eigen::Matrix<Fixed, 3, 1> Vector3;

static std::stack<Transform> matrixStack;
#define curMatrix (ASSERT(!matrixStack.empty(), "No matrix on the stack"), matrixStack.top())

bool drawing_interface = true;

//*************************************************************************
//*** reset transformation matrix stack and make current identity
//*
//******

static void pie_MatReset(void)
{
	// Clear the matrix stack
	while (!matrixStack.empty())
		matrixStack.pop();

	// make 1st matrix identity
	matrixStack.push(Transform(Transform::Identity()));

	glLoadIdentity();
}


//*************************************************************************
//*** create new matrix from current transformation matrix and make current
//*
//******

void pie_MatBegin(void)
{
	matrixStack.push(curMatrix);
	glPushMatrix();
}


//*************************************************************************
//*** make current transformation matrix previous one on stack
//*
//******

void pie_MatEnd(void)
{
	matrixStack.pop();
	ASSERT(!matrixStack.empty(), "pie_MatEnd past the bottom of the stack" );

	glPopMatrix();
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
	curMatrix.translate(Vector3(x, y, z));

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

	curMatrix.scale(Vector3(scale, scale, scale));

	glScalef(scale, scale, scale);
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
		curMatrix.rotate(Eigen::AngleAxis<Fixed>(y / 32768.0f * M_PI, Vector3::UnitY()));

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
		curMatrix.rotate(Eigen::AngleAxis<Fixed>(z / 32768.0f * M_PI, Vector3::UnitZ()));

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
	 * curMatrix = curMatrix . rotationMatrix(a, 1, 0, 0)
	 *
	 *                         [ 1  0   0  0 ]
	 *                         [ 0  c  -s  0 ]
	 * curMatrix = curMatrix . [ 0  s   c  0 ]
	 *                         [ 0  0   0  1 ]
	 */
	if (x != 0.f)
	{
		curMatrix.rotate(Eigen::AngleAxis<Fixed>(x / 32768.0f * M_PI, Vector3::UnitX()));

		glRotatef(UNDEG(x), 1.0f, 0.0f, 0.0f);
	}
}


/*!
 * 3D vector perspective projection
 * Projects 3D vector into 2D screen space
 * \param v3d       3D vector to project
 * \param[out] v2d  resulting 2D vector
 * \return projected z component of v2d
 */
int32_t pie_RotateProject(const Vector3i *v3d, Vector2i *v2d)
{
	/*
	 * v = curMatrix . v3d
	 */
	Vector3 v(v3d->x, v3d->y, v3d->z);
	v = curMatrix * v;

	const Fixed zz = v.z() * 4;

	if (zz < MIN_STRETCHED_Z)
	{
		v2d->x = LONG_WAY; //just along way off screen
		v2d->y = LONG_WAY;
	}
	else
	{
		v2d->x = rendSurface.xcentre + (v.x() / zz).value_;
		v2d->y = rendSurface.ycentre - (v.y() / zz).value_;
	}

	return zz;
}

void pie_PerspectiveBegin(void)
{
	const float width = pie_GetVideoBufferWidth();
	const float height = pie_GetVideoBufferHeight();
	const float xangle = width/6.0f;
	const float yangle = height/6.0f;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glTranslatef(
		(2 * rendSurface.xcentre-width) / width,
		(height - 2 * rendSurface.ycentre) / height,
		0);
	glFrustum(-xangle, xangle, -yangle, yangle, 330, 100000);
	glScalef(1, 1, -1);
	glMatrixMode(GL_MODELVIEW);
}

void pie_PerspectiveEnd(void)
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0.0f, (double) pie_GetVideoBufferWidth(), (double) pie_GetVideoBufferHeight(), 0.0f, 1.0f, -1.0f);
	glMatrixMode(GL_MODELVIEW);
}

void pie_Begin3DScene(void)
{
	glDepthRange(0.1, 1);
	drawing_interface = false;
}

void pie_BeginInterface(void)
{
	glDepthRange(0, 0.1);
	drawing_interface = true;
}

void pie_SetGeometricOffset(int x, int y)
{
	rendSurface.xcentre = x;
	rendSurface.ycentre = y;
}


/** Inverse rotate 3D vector with current rotation matrix.
 *  @param v1       3D vector to rotate
 *  @param[out] v2  inverse rotated 3D vector
 */
void pie_VectorInverseRotate0(const Vector3i *v1, Vector3i *v2)
{
	/*
	 * invMatrix = transpose(linear_part(curMatrix))
	 * v2 = invMatrix . v1
	 */
	Vector3 u(v1->x, v1->y, v1->z);

	u = curMatrix.linear().transpose() * u;
	v2->x = u.x();
	v2->y = u.y();
	v2->z = u.z();
}

/** Sets up transformation matrices/quaternions and trig tables
 */
void pie_MatInit(void)
{
	// init matrix/quat stack
	pie_MatReset();
}

void pie_RotateTranslate3i(const Vector3i *v, Vector3i *s)
{
	/*
	 *               [ 1 0 0 0 ]
	 *               [ 0 0 1 0 ]
	 * swapMatrix := [ 0 1 0 0 ]
	 *               [ 0 0 0 1 ]
	 *
	 * s = swapMatrix . curMatrix . swapMatrix . v
	 */

	// u = swapMatrix . v
	Vector3 u(v->x, v->z, v->y);
	// u = curMatrix . swapMatrix . v
	u = curMatrix * u;
	// s = swapMatrix . u
	s->x = u.x();
	s->y = u.z();
	s->z = u.y();
}

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

bool drawing_interface = true;

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
 * \param[out] v2d  resulting 2D vector
 * \return projected z component of v2d
 */
int32_t pie_Project(const Vector3i *v3d, Vector2i *v2d)
{
	GLdouble screenX, screenY, depth;// arrays to hold matrix information
	GLint viewport[4];
	GLdouble projection[16];

	// TODO we don't actually need to query opengl for these...
	// we also need to put a implementation of gluProject in here.
	// Then we can cache the matrix product P*MV

	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);   // get 3D coordinates based on window coordinates

	gluProject(v3d->x, v3d->y, v3d->z,
			   Eigen::Transform<double,3,  Eigen::Affine>(curMatrix).matrix().data(),
			   projection, viewport, &screenX, &screenY, &depth);

	v2d->x = screenX;
	v2d->y = pie_GetVideoBufferHeight()-screenY; // I don't know why this is needed... yet

	/*
	 * Though the multiplication by MAX_Z is a workaround
	 * it should be consistent with what people have assumed about the depth values.
	 * This is so that the float->int conversion doesn't kill the depth values.
	 */
	return depth * MAX_Z;
}

void pie_PerspectiveBegin(void)
{
	const float width = pie_GetVideoBufferWidth();
	const float height = pie_GetVideoBufferHeight();
	const float xangle = width/6.0f;
	const float yangle = height/6.0f;

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
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


/*
	This file is part of Warzone 2100.
	Copyright (C) 1999-2004  Eidos Interactive
	Copyright (C) 2005-2010  Warzone 2100 Project

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
#include "lib/ivis_common/pieclip.h"
#include "piematrix.h"
#include "lib/ivis_common/piemode.h"

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
class Matrix
{
	public:
		enum
		{
			ROWS = 3,
			COLS = 4,
		};

		inline int32_t operator()(int row, int col) const
		{
			ASSERT(row < ROWS && col < COLS, "Row or column index out of range: (%d, %d)", row, col);

			return a[col * ROWS + row];
		}

		inline int32_t& operator()(int row, int col)
		{
			ASSERT(row < ROWS && col < COLS, "Row or column index out of range: (%d, %d)", row, col);

			return a[col * ROWS + row];
		}

		void setIdentity()
		{
			(*this)(0,0) = 1; (*this)(0,1) = 0; (*this)(0,2) = 0; (*this)(0,3) = 0;
			(*this)(1,0) = 0; (*this)(1,1) = 1; (*this)(1,2) = 0; (*this)(1,3) = 0;
			(*this)(2,0) = 0; (*this)(2,1) = 0; (*this)(2,2) = 1; (*this)(2,3) = 0;

			*this *= FP12_MULTIPLIER;
		}

		/**
		 * Scalar multiplication.
		 */
		template <typename NumericType>
		Matrix& operator*=(NumericType scalar)
		{
			for (unsigned i = 0; i < ARRAY_SIZE(a); ++i)
				a[i] *= scalar;
			return *this;
		}

	private:
		int32_t a[12];
};

static std::stack<Matrix> matrixStack;
#define curMatrix (ASSERT(!matrixStack.empty(), "No matrix on the stack"), matrixStack.top())

BOOL drawing_interface = true;

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
	Matrix identity;
	identity.setIdentity();
	matrixStack.push(identity);

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


void pie_MATTRANS(float x, float y, float z)
{
	GLfloat matrix[16];

	curMatrix(0,3) = x * FP12_MULTIPLIER;
	curMatrix(1,3) = y * FP12_MULTIPLIER;
	curMatrix(2,3) = z * FP12_MULTIPLIER;

	glGetFloatv(GL_MODELVIEW_MATRIX, matrix);
	matrix[12] = x;
	matrix[13] = y;
	matrix[14] = z;
	glLoadMatrixf(matrix);
}

void pie_TRANSLATE(float x, float y, float z)
{
	/*
	 * curMatrix = curMatrix . translationMatrix(x, y, z)
	 *
	 *                         [ 1 0 0 x ]
	 *                         [ 0 1 0 y ]
	 * curMatrix = curMatrix . [ 0 0 1 z ]
	 *                         [ 0 0 0 1 ]
	 */
	curMatrix(0,3) += x * curMatrix(0,0) + y * curMatrix(0,1) + z * curMatrix(0,2);
	curMatrix(1,3) += x * curMatrix(1,0) + y * curMatrix(1,1) + z * curMatrix(1,2);
	curMatrix(2,3) += x * curMatrix(2,0) + y * curMatrix(2,1) + z * curMatrix(2,2);

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

	curMatrix *= scale;

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
		int t;
		int64_t cra = iCos(y), sra = iSin(y);

		t = (cra*curMatrix(0,0) - sra*curMatrix(0,2))>>16;
		curMatrix(0,2) = (sra*curMatrix(0,0) + cra*curMatrix(0,2))>>16;
		curMatrix(0,0) = t;

		t = (cra*curMatrix(1,0) - sra*curMatrix(1,2))>>16;
		curMatrix(1,2) = (sra*curMatrix(1,0) + cra*curMatrix(1,2))>>16;
		curMatrix(1,0) = t;

		t = (cra*curMatrix(2,0) - sra*curMatrix(2,2))>>16;
		curMatrix(2,2) = (sra*curMatrix(2,0) + cra*curMatrix(2,2))>>16;
		curMatrix(2,0) = t;

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
		int t;
		int64_t cra = iCos(z), sra = iSin(z);

		t = (cra*curMatrix(0,0) + sra*curMatrix(0,1))>>16;
		curMatrix(0,1) = (cra*curMatrix(0,1) - sra*curMatrix(0,0))>>16;
		curMatrix(0,0) = t;

		t = (cra*curMatrix(1,0) + sra*curMatrix(1,1))>>16;
		curMatrix(1,1) = (cra*curMatrix(1,1) - sra*curMatrix(1,0))>>16;
		curMatrix(1,0) = t;

		t = (cra*curMatrix(2,0) + sra*curMatrix(2,1))>>16;
		curMatrix(2,1) = (cra*curMatrix(2,1) - sra*curMatrix(2,0))>>16;
		curMatrix(2,0) = t;

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
		int t;
		int64_t cra = iCos(x), sra = iSin(x);

		t = (cra*curMatrix(0,1) + sra*curMatrix(0,2))>>16;
		curMatrix(0,2) = (cra*curMatrix(0,2) - sra*curMatrix(0,1))>>16;
		curMatrix(0,1) = t;

		t = (cra*curMatrix(1,1) + sra*curMatrix(1,2))>>16;
		curMatrix(1,2) = (cra*curMatrix(1,2) - sra*curMatrix(1,1))>>16;
		curMatrix(1,1) = t;

		t = (cra*curMatrix(2,1) + sra*curMatrix(2,2))>>16;
		curMatrix(2,2) = (cra*curMatrix(2,2) - sra*curMatrix(2,1))>>16;
		curMatrix(2,1) = t;

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
	Vector3i v = {
		v3d->x * curMatrix(0,0) + v3d->y * curMatrix(0,1) + v3d->z * curMatrix(0,2) + curMatrix(0,3),
		v3d->x * curMatrix(1,0) + v3d->y * curMatrix(1,1) + v3d->z * curMatrix(1,2) + curMatrix(1,3),
		v3d->x * curMatrix(2,0) + v3d->y * curMatrix(2,1) + v3d->z * curMatrix(2,2) + curMatrix(2,3)
	};

	const int zz = v.z >> STRETCHED_Z_SHIFT;

	if (zz < MIN_STRETCHED_Z)
	{
		v2d->x = LONG_WAY; //just along way off screen
		v2d->y = LONG_WAY;
	}
	else
	{
		v2d->x = rendSurface.xcentre + (v.x / zz);
		v2d->y = rendSurface.ycentre - (v.y / zz);
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
	 * invMatrix = transpose(sub3x3Matrix(curMatrix))
	 * v2 = invMatrix . v1
	 */
	v2->x = (v1->x * curMatrix(0,0) + v1->y * curMatrix(1,0) + v1->z * curMatrix(2,0)) / FP12_MULTIPLIER;
	v2->y = (v1->x * curMatrix(0,1) + v1->y * curMatrix(1,1) + v1->z * curMatrix(2,1)) / FP12_MULTIPLIER;
	v2->z = (v1->x * curMatrix(0,2) + v1->y * curMatrix(1,2) + v1->z * curMatrix(2,2)) / FP12_MULTIPLIER;
}

/** Sets up transformation matrices/quaternions and trig tables
 */
void pie_MatInit(void)
{
	// init matrix/quat stack
	pie_MatReset();
}

void pie_RotateTranslate3f(const Vector3f *v, Vector3f *s)
{
	/*
	 *     [ 1 0 0 0 ]               [ 1 0 0 0 ]
	 *     [ 0 0 1 0 ]               [ 0 0 1 0 ]
	 * s = [ 0 1 0 0 ] . curMatrix . [ 0 1 0 0 ] . v
	 *     [ 0 0 0 1 ]               [ 0 0 0 1 ]
	 */
	s->x = (v->x * curMatrix(0,0) + v->y * curMatrix(0,2) + v->z * curMatrix(0,1) + curMatrix(0,3)) / FP12_MULTIPLIER;
	s->y = (v->x * curMatrix(2,0) + v->y * curMatrix(2,2) + v->z * curMatrix(2,1) + curMatrix(2,3)) / FP12_MULTIPLIER;
	s->z = (v->x * curMatrix(1,0) + v->y * curMatrix(1,2) + v->z * curMatrix(1,1) + curMatrix(1,3)) / FP12_MULTIPLIER;
}

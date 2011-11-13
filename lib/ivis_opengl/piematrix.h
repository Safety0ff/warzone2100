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
/***************************************************************************/
/*
 * pieMatrix.h
 *
 * matrix functions for pumpkin image library.
 *
 */
/***************************************************************************/
#ifndef _pieMatrix_h
#define _pieMatrix_h

#include "lib/ivis_opengl/piedef.h"
#include "lib/ivis_opengl/camera.h"

//*************************************************************************

/*!
 * Calculate surface normal
 * Eg. if a polygon (with n points in clockwise order) normal is required,
 * \c p1 = point 0, \c p2 = point 1, \c p3 = point n-1
 * \param p1,p2,p3 Points for forming 2 vector for cross product
 * \return Normal vector
 */
static inline WZ_DECL_CONST WZ_DECL_WARN_UNUSED_RESULT
		Vector3f pie_SurfaceNormal3fv(const Vector3f p1, const Vector3f p2, const Vector3f p3)
{
	return normalize(crossProduct(p3 - p1, p2 - p1));
}

//*************************************************************************

extern void pie_MatInit(void);

// Note: Can be called before pie_MatInit
extern void pie_SetViewport(int x, int y, int width, int height);

/// Set the camera for perspective view transformation (Mandatory!)
extern void pie_SetCamera(Camera * camera);

//*************************************************************************

extern void pie_GetModelViewMatrix(float * const mat);

//*************************************************************************

extern void pie_MatBegin(void);
extern void pie_MatEnd(void);
extern void pie_TRANSLATE(int32_t x, int32_t y, int32_t z);
extern void pie_MatScale(float scale);
extern void pie_MatScale(float x, float y, float z);
extern void pie_MatRotX(uint16_t x);
extern void pie_MatRotY(uint16_t y);
extern void pie_MatRotZ(uint16_t z);
extern void pie_MatIdentity(void);

// loads the view matrix from the camera
extern void pie_MatLoadView(Eye::ViewMatrixType viewMatType);

//*************************************************************************
// Functions for projecting geometry onto the screen
// Note: They are only for use with perspective projection!
// The y coordinate of the screen position is given in terms of a top left origin

extern bool pie_Project(Vector3f const &obj, Vector3i *proj);
extern bool pie_Project(Vector3i *proj); // Overload optimized for projecting the origin

/** An approximation for the perspective projection of a sphere onto the screen
 * This approximates the ellipse that would result to a circle.
 * Note: the approximation will be somewhat arbitrary but it's better than using a hack
 * Radius is a input and output parameter.
 */
extern bool pie_ProjectSphere(Vector3f const &obj, int32_t &radius, Vector3i *proj);
extern bool pie_ProjectSphere(int32_t &radius, Vector3i *proj); // Overload optimized for projecting the origin

//*************************************************************************
// Projection matrix functions
/*
 * Set the projection matrix to a
 * bottom left origin orthographic projection
 * (currently used mostly for the 3D scene)
 *
 * Argument is the type of view matrix to load from
 * the camera.
 */
extern void pie_SetPerspectiveProj(void);

/*
 * Set the projection matrix to a orthographic projection
 * The argument chooses between an upper left origin and
 * a bottom left origin.
 * The former is currently used mostly for the UI/widgets
 * The latter is currently used mostly for the droids/structures in the UI
 */
extern void pie_SetOrthoProj(bool originAtTheTop, float nearDepth, float farDepth);

#endif

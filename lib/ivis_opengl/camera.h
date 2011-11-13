/*
 T his file is part of Warzone 210*0.
 Copyright (C) 2011  Warzone 2100 Project
 
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

#ifndef CAMERA_H
#define CAMERA_H

#include "lib/ivis_opengl/plane.h"
#include "lib/ivis_opengl/ba2d.h"

#include "lib/framework/vector.h"


/*
 * Eye class (camera base class)
 */
class Eye {

	// Eye/camera referential frame
	Vector3f eyeL, eyeU, eyeF;	// Eye: left, up, forward orientation vectors (unit & orthogonal)
	Vector3f eye;				// Actual eye position

	/*
		* This is data is for recomputing the referential frame when
		* the setters are called and is the data for certain getters
		*/
	Vector3f pos;
	Vector3i rot;
	float distance, scale;

protected:
	/* For updating the forward matrix and 
	 * frustum planes & vertices respectively
	 */
	void updateCamRefFrame();

public:
	Eye();
	~Eye();

	/***********************
	 * Eye setters
	 */

	void setEyeAsWzCam (Vector3i const &pos, Vector3i const &rot, float offset, float scale);
	void setEyeAsgluLookAt (Vector3f const &eye, Vector3f const &center, Vector3f const &up);

	void setObservedPos(Vector3i const &pos);
	void setEyeRot(Vector3i const &rot);
	void setScaling(float scale);
	void setEyeDistance(float offset);

	/***********************
	 * Eye getters
	 */

	Vector3f getEyePos() const;
	Vector3f getEyeL() const;
	Vector3f getEyeU() const;
	Vector3f getEyeF() const;

	Vector3f getObservedPos() const;
	Vector3i getEyeRot() const;
	float getEyeScaling() const;
	float getEyeDistance() const;

	//***********************
	// View matrix computation
	//
	// Note: only depends on eye being set

	enum ViewMatrixType {
		// Normal = render everything in world coords
		NORMAL,

		/* XZ relative = render everything with their x and z
		 * coords relative to the eye's x and z coords.
		 * i.e. substract the eye's x and z coords from the
		 * objects world coords before rendering
		 */
		XZ_RELATIVE
	};

	// mat = pointer to float[4][4] (column major)
	void getViewMatrix(float mat[4][4], ViewMatrixType type) const;
	void getViewMatrix(double mat[4][4], ViewMatrixType type) const;

	// compact versions i.e. row 4 = [0 0 0 1]
	void getViewMatrix3x4(float mat[4][3], ViewMatrixType type) const;
	void getViewMatrix3x4(double mat[4][3], ViewMatrixType type) const;
};

enum Planes {
	LEFTp = 0,
	RIGHTp,
	BOTTOMp,
	TOPp,
	FARp,
	NEARp
};

enum Intersect {
	Disjoint,
	Partial,
	Contained
};

/*
 * Camera class with many bells and whistles
 * Note: assumes symmetric frustum! general version below
 */
class Camera : public Eye {

	// Frustum variables
	double nearZ, farZ;
	double vFactor, hFactor; // Vertical and Horizontal factors
	double radiusVFactor, radiusHFactor;

	// Bell and whistle variables
	Plane p[6];
	Vector3f v[2][2][2]; // Vertices [Near|Far] [Left|Right] [Bottom|Top] (order: [0|1])

	OBB2D obb;
	AABB2D aabb;

	void setVerticesFromPlanes();
	void setPlanesFromVertices();
protected:
	/* Internal update functions (frustum planes & vertices respectively)
	 */
	void computeBAs(); // Note: Frustum clipped against xz plane!
	void extractFrustum(double aspect);

public:
	Camera();
	~Camera();

/*************************************
 * Frustum setters
 */

	void setAsGluPerspective(double fovY, double aspect, double zNear, double zFar);
	void setAsGlFrustum(double l, double r, double b, double t, double nearVal, double farVal);

	/// like glFrustum with left = -right and top = -bottom	(i.e. symmetric frustum)
	void setAsGlFrustum(double nearH, double nearW, double nearVal, double farVal);

/*************************************
 * Frustum intersection tests
 */

	bool pointContained(const Vector3f &p) const;

	bool sphereIntersectb(const Vector3f &center, float radius) const; // b for boolean

	Intersect sphereIntersectt(const Vector3f &p, float radius) const; // t for trivalent

/*************************************
 * Frustum bounds
 */
	OBB2D get2DOBB() const;
	AABB2D get2DAABB() const;

/*************************************
 * Bells and whistles
 */

	/* 
	 * Set the view matrix such that world coords can
	 * be used. (Remember: rendering the eye which was used
	 * to set the view matrix should not change any pixels.)
	 */
	void renderEye() const;

	Plane const & getPlane(Planes plane) const;
	Vector3f const & getVertex(bool far, bool right, bool top) const;
};

// General frustum version (i.e. takes care of asymmetric case)
class GeneralCamera : public Eye {
	double nearZ, farZ;
	/*
	 * Left, right, up, down factors
	 */
	double lFactor, rFactor, tFactor, bFactor;
	double radiusLFactor, radiusRFactor;
	double radiusTFactor, radiusBFactor;

	Plane p[6];
	Vector3f v[2][2][2];
	OBB2D obb;
	AABB2D aabb;
protected:
	void computeBAs();
public:
	GeneralCamera();
	~GeneralCamera();
	void setAsGlFrustum(double l, double r, double b, double t, double nearVal, double farVal);
	bool pointContained(const Vector3f &p) const;
	bool sphereIntersectb(const Vector3f &center, float radius) const;
	Intersect sphereIntersectt(const Vector3f &p, float radius) const;
	OBB2D get2DOBB() const;
	AABB2D get2DAABB() const;
	void renderEye() const;
	Plane const & getPlane(Planes plane) const;
	Vector3f const & getVertex(bool far, bool right, bool top) const;
};

#endif // CAMERA_H
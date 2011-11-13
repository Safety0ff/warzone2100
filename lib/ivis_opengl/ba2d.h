/*
	This file is part of Warzone 2100.
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
/*! \file 2dba.h
 * \brief 2D bounding area classes
 */
#ifndef BA2D_H
#define BA2D_H

#include "lib/framework/vector.h"
#include "lib/framework/math_ext.h"

struct AABB2D
{
	Vector2f min; // minimum corner
	Vector2f max; // maximum corner
public:
	AABB2D () {}
	AABB2D (Vector2f const &_min, Vector2f const &_max) : min(_min), max(_max) {}
	Vector2f getMin() const { return min; }
	Vector2f getMax() const { return max; }
	bool intersect(Vector2f const &point) const;
	float area () const { return (min-max).x*(min-max).y; }
};

struct OBB2D
{
	Vector2f pos; // centre
	Vector2f axes[2]; // unit length
	Vector2f extents; // half-width/half-height

	OBB2D () {}
	OBB2D (Vector2f const &centre, Vector2f const &extents, Vector2f const axes[2]);

	OBB2D (AABB2D const &aabb);

	void computeVertices(Vector2f verts[4]) const;

	operator AABB2D() const;

	bool intersect(Vector2f const &point) const;

	float area () const { return extents.x*extents.y*4; }

	/* sorts verts lexicographically then uses
	 * uses monotone chains to create a convex hull and then
	 * rotating calipers to find optimal 2d obb.
	 * bounded by O(nlogn) sorting step.
	 */
	static OBB2D createOptimal(unsigned numVerts, Vector2f* verts, AABB2D * const aabb);
};

#endif // BA2D_H
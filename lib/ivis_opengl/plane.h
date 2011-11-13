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

#ifndef PLANE_H
#define PLANE_H

#include "lib/framework/vector.h"
#include <limits>
#include <math.h>

class Plane  
{
	Vector3f normal;
	float d;
public:

	Plane(Plane const &p) : normal(p.normal), d(p.d) {}
	Plane(void) : normal(Vector3f(0,0,-1)), d(0) {}
	~Plane() {}

	void from3Points(const Vector3f &p1, const Vector3f &p0, const Vector3f &p2); // p0 will be used as the origin
	void fromNormalAndPoint(const Vector3f &normal, const Vector3f &point);
	void fromCoefficients(float a, float b, float c, float d);

	Vector3f const & getNormal() const { return normal; }
	float getOffset() const { return d; }
	void setOffset(float d) { this->d = d; }
	void setOffsetFromPoint(Vector3f p0) { d = normal*p0; }
	float distance(const Vector3f &p) const;
};

static inline bool intersect(Plane const &p0, Plane const &p1, Plane const &p2, Vector3f * const intersection)
{
	float denom = scalarTripleProduct(p0.getNormal(), p1.getNormal(), p2.getNormal());
	if (std::abs(denom) <= std::numeric_limits<float>::epsilon()*12)
		return false;
	*intersection = (crossProduct(p1.getNormal(), p2.getNormal())*p0.getOffset()
					+ crossProduct(p2.getNormal(), p0.getNormal())*p1.getOffset()
					+ crossProduct(p0.getNormal(), p1.getNormal())*p2.getOffset()) / denom;
	return true;
}

inline void Plane::from3Points(Vector3f const &p1, Vector3f const &p0, Vector3f const &p2)
{
	normal = normalize(crossProduct(p2 - p0, p1 - p0));
	d = -(normal*p0);
}

inline void Plane::fromNormalAndPoint(Vector3f const &normal, Vector3f const &p0)
{
	this->normal = normalize(normal);
	d = -(normal*p0);
}

inline void Plane::fromCoefficients(float a, float b, float c, float d)
{
	normal = Vector3f(a,b,c);
	float l = std::sqrt(normal*normal);
	if (l <= std::numeric_limits<float>::epsilon()*6)
		{ *this = Plane(); return; }
	normal = normal/l;
	d /= l;
}

#endif // PLANE_H
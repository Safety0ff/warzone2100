/*
 * This file contains code derived from: http://www.geometrictools.com/LibMathematics/Containment/Wm5ContMinBox2.cpp
 * which is distributed under the Boost Software License, Version 1.0.
 * (See accompanying file COPYING.BOOST or copy at
 *  http://www.boost.org/LICENSE_1_0.txt )
 */
#include "ba2d.h"

#include <algorithm>
#include <string.h>

bool AABB2D::intersect(Vector2f const &point) const {
	return point.x >= min.x && point.x <= max.x
	&& point.y >= min.y && point.y <= max.y;
}

OBB2D::OBB2D (Vector2f const &p, Vector2f const &bounds, Vector2f const _axes[2]) : pos(p), extents(bounds)
{
	axes[0] = _axes[0];
	axes[1] = _axes[1];
}


OBB2D::OBB2D (AABB2D const &aabb) : pos(aabb.getMin()), extents((aabb.getMax()-aabb.getMin())*0.5f)
{
	axes[0] = Vector2f(1,0);
	axes[1] = Vector2f(0,1);
}
void OBB2D::computeVertices(Vector2f verts[4]) const
{
	verts[0] = pos - axes[0]*extents.x + axes[1]*extents.y;
	verts[1] = pos + axes[0]*extents.x + axes[1]*extents.y;
	verts[2] = pos + axes[0]*extents.x - axes[1]*extents.y;
	verts[3] = pos - axes[0]*extents.x - axes[1]*extents.y;
}

OBB2D::operator AABB2D() const 
{
	using namespace std;
	Vector2f mn, mx;
	Vector2f verts[4];
	computeVertices(verts);
	mn.x = min(verts[0].x,min(verts[1].x,min(verts[2].x,verts[3].x)));
	mx.x = max(verts[0].x,max(verts[1].x,max(verts[2].x,verts[3].x)));
	mn.y = min(verts[0].y,min(verts[1].y,min(verts[2].y,verts[3].y)));
	mx.y = max(verts[0].y,max(verts[1].y,max(verts[2].y,verts[3].y)));
	return AABB2D(mn, mx);
}

bool OBB2D::intersect(Vector2f const &point) const
{
	const Vector2f delta = point - pos;
	if (std::abs(delta * axes[0]) >= extents.x)
		return false;
	if (std::abs(delta * axes[1]) >= extents.y)
		return false;
	return true;
}

static bool lexComp(const Vector2f &a, const Vector2f &b)
{
	return a.x < b.x || (a.x == b.x && a.y < b.y);
}

// Clockwise or collinear
static bool cwOrCollinear(const Vector2f &a, const Vector2f &b, const Vector2f &c)
{
	const Vector2f ab = b-a;
	const Vector2f ac = c-b;
	const float result = (ab.x*ac.y - ab.y*ac.x);
	return result >= 0.0;
}

static void replaceIfSmaller(Vector2f const &calip, Vector2f const &perp,
								Vector2f const &L, Vector2f const &R,
								Vector2f const &B, Vector2f const &T,
								OBB2D * const obb, float * const area)
{
	Vector2f newExtents;
	newExtents.x = calip * (L - R);
	newExtents.y = perp * (T - B);
	float newArea = newExtents.x * newExtents.y;
	if (newArea < *area)
	{
		obb->axes[0] = calip;
		obb->axes[1] = perp;
		obb->extents = newExtents*0.5f;
		obb->pos = R + calip * obb->extents.x + perp*(obb->extents.y - perp*(R - B));
		*area = newArea;
	}
}

bool coincident(Vector2f const &a, Vector2f const &b)
{
	const Vector2f ab = b-a;
	return std::abs(ab.x) < 0.001 && std::abs(ab.y) < 0.001; // magic numbers = made epsilon
}

OBB2D OBB2D::createOptimal(unsigned numVerts, Vector2f* verts, AABB2D * const aabb)
{
	std::sort(verts, verts + numVerts, lexComp);

	// Remove coincident vertices
	int i = numVerts;
	for (int read = 1, write = 1; read < numVerts; ++read)
	{
		if (!coincident(verts[read],verts[read-1]))
		{
			verts[write++] = verts[read];
		}
		else
		{
			--i;
		}
	}
	numVerts = i;

	Vector2f *hull = (Vector2f*) alloca (sizeof(Vector2f)*numVerts*2);
	int hullVerts = 0, bottomStart;

	// top part of hull
	for (int i = 0; i < numVerts; ++i)
	{
		while (hullVerts >= 2 && cwOrCollinear(hull[hullVerts - 2], hull[hullVerts - 1], verts[i]))
			--hullVerts;
		hull[hullVerts] = verts[i];
		++hullVerts;
	};
	bottomStart = hullVerts;
	// bottom part
	for (i = numVerts - 2; i >= 0; --i)
	{
		
		while (hullVerts > bottomStart && cwOrCollinear(hull[hullVerts - 2], hull[hullVerts - 1], verts[i]))
			--hullVerts;
		hull[hullVerts] = verts[i];
		++hullVerts;
	}
	--hullVerts; // Remove duplicated first vertex

	// End monotone chains, begin rotating calipers

	Vector2f * edges = (Vector2f*) alloca (sizeof(Vector2f)*hullVerts);
	bool * traversed = (bool*) alloca (sizeof(bool)*hullVerts);

	int yminIdx = hullVerts-1, ymaxIdx = yminIdx;
	int xminIdx = hullVerts-1, xmaxIdx = xminIdx;
	float ymin = hull[yminIdx].y, ymax = ymin;
	float xmin = hull[xminIdx].x, xmax = xmin;

	// Compute edge direction vectors and compute starting 2D AABB

	edges[hullVerts - 1] = normalize(hull[0] - hull[hullVerts - 1]);
	traversed[hullVerts - 1] = false;

	for (i = 0; i < hullVerts - 1; ++i)
	{
		edges[i] = normalize(hull[i+1] - hull[i]);
		traversed[i] = false;
		if (hull[i].y >= ymax)
			ymax = hull[i].y, ymaxIdx = i;
		if (hull[i].y <= ymin)
			ymin = hull[i].y, yminIdx = i;
		if (hull[i].x >= xmax)
			xmax = hull[i].x, xmaxIdx = i;
		if (hull[i].x <= xmin)
			xmin = hull[i].x, xminIdx = i;
	}
	if (yminIdx != 0 && ymin == hull[hullVerts-1].y)
		yminIdx = hullVerts-1;
	if (ymaxIdx != 0 && ymax == hull[hullVerts-1].y)
		ymaxIdx = hullVerts-1;
	if (xminIdx != 0 && xmin == hull[hullVerts-1].x)
		xminIdx = hullVerts-1;
	if (xmaxIdx != 0 && xmax == hull[hullVerts-1].x)
		xmaxIdx = hullVerts-1;

	if (aabb != NULL)
	{
		aabb->max = Vector2f(xmax, ymax);
		aabb->min = Vector2f(xmin, ymin);
	}

	// Minize starting from the 2D AABB
	OBB2D result;
	result.axes[0] = Vector2f(1,0);
	result.axes[1] = Vector2f(0,1);
	result.pos.x = (xmax + xmin) * 0.5f;
	result.pos.y = (ymax + ymin) * 0.5f;
	result.extents.x = (xmax - xmin) * 0.5f;
	result.extents.y = (ymax - ymin) * 0.5f;

	float area = (xmax - xmin) * (ymax - ymin);

	Vector2f calip = Vector2f(1,0);
	int side = 1; // 1 to enter the loop

	int tIdx = ymaxIdx, bIdx = yminIdx, lIdx = xmaxIdx, rIdx = xminIdx;

	while (side)
	{
		/*
		 * We always rotate in the direction of the edges (CCW)
		 * therefore the dot products are always positive.
		 * All the vectors involved are unit, so the dot products
		 * are proportional, so just compare dots
		 */
		side = 0; // None or rectangle
		// made up epsilon (so that rectanglular case falls through)
		float maxCAngle = 0.001f;
		float cAngle;

		// Top
		cAngle = calip * edges[tIdx];
		if (cAngle > maxCAngle)
		{
			maxCAngle = cAngle;
			side = 1;
		}

		// Right
		cAngle = Vector2f(-calip.y, calip.x) * edges[rIdx];
		if (cAngle > maxCAngle)
		{
			maxCAngle = cAngle;
			side = 2;
		}

		// Bottom
		cAngle = -calip * edges[bIdx];
		if (cAngle > maxCAngle)
		{
			maxCAngle = cAngle;
			side = 3;
		}

		// Left
		cAngle = Vector2f(calip.y, -calip.x) * edges[lIdx];
		if (cAngle > maxCAngle)
		{
			maxCAngle = cAngle;
			side = 4;
		}
	
		if (side == 1)
		{
			if (traversed[tIdx])
			{
				side = 0; // done
			}
			else
			{
				calip = edges[tIdx];
				replaceIfSmaller(calip, Vector2f(-calip.y, calip.x),
								hull[lIdx], hull[rIdx],
								hull[bIdx], hull[tIdx],
								&result, &area);
				traversed[tIdx] = true;
				// Wrap around
				tIdx = (tIdx + 1 == hullVerts) ? 0 : tIdx + 1;
			}
		}
		else if (side == 2)
		{
			if (traversed[rIdx])
			{
				side = 0; // done
			}
			else
			{
				calip = Vector2f(edges[rIdx].y, -edges[rIdx].x);
				replaceIfSmaller(calip, edges[rIdx],
								hull[lIdx], hull[rIdx],
								hull[bIdx], hull[tIdx],
								&result, &area);
				traversed[rIdx] = true;
				// Wrap around
				rIdx = (rIdx + 1 == hullVerts) ? 0 : rIdx + 1;
			}
		}
		else if (side == 3)
		{
			if (traversed[bIdx])
			{
				side = 0; // done
			}
			else
			{
				calip = -edges[bIdx];
				replaceIfSmaller(calip, Vector2f(edges[bIdx].y, -edges[bIdx].x),
								hull[lIdx], hull[rIdx],
								hull[bIdx], hull[tIdx],
								&result, &area);
				traversed[bIdx] = true;
				// Wrap around
				bIdx = (bIdx + 1 == hullVerts) ? 0 : bIdx + 1;
			}
		}
		else if (side == 4)
		{
			if (traversed[lIdx])
			{
				side = 0; // done
			}
			else
			{
				calip = Vector2f(-edges[lIdx].y, edges[lIdx].x);
				replaceIfSmaller(calip, -edges[lIdx],
								 hull[lIdx], hull[rIdx],
					 hull[bIdx], hull[tIdx],
					 &result, &area);
				
				traversed[lIdx] = true;
				// Wrap around
				lIdx = (lIdx + 1 == hullVerts) ? 0 : lIdx + 1;
			}
		}
	}

	return result;
}
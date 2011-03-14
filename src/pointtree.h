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
#ifndef _point_tree_h
#define _point_tree_h

#include "lib/framework/types.h"

#include <algorithm>
#include <iterator>
#include <vector>

template <typename UnaryPred>
class Filter;

struct PointTreeRange
{
	uint64_t a, z;
	static const int MAX_RANGES = 4;
};
typedef PointTreeRange RangePack[PointTreeRange::MAX_RANGES];

class PointTree
{
public:
	typedef std::vector<void *> ResultVector;
	typedef std::vector<unsigned> IndexVector;

	void insert(void *pointData, int32_t x, int32_t y);                       ///< Inserts a point into the point tree.
	void clear();                                                             ///< Clears the PointTree.
	void sort();                                                              ///< Must be done between inserting and querying, to get meaningful results.
	int size() const { return points.size(); }

	/// Returns all points within the given bounds which Condition.operator() evaluates to true
	/// Note: Not thread safe, because it modifies lastQueryResults.
	template <typename Condition>
	ResultVector &query(Condition &pred, int32_t minX, int32_t maxX, int32_t minY, int32_t maxY);

	/// Overloaded version of query with memoization support for faster lookups.
	/// Note: Not thread safe, because it modifies lastQueryResults and the filter.
	template <typename Condition, typename UnaryPred>
	ResultVector &query(Condition &pred, Filter<UnaryPred> &filter, int32_t minX, int32_t maxX, int32_t minY, int32_t maxY);

	ResultVector lastQueryResults;

private:
	typedef std::pair<uint64_t, void *> Point;
	typedef std::vector<Point> Vector;

	Vector points;
};

/**
 * Filters provide a method to memoize results of a predicate
 *  which are common to a set of queries.
 * UnaryPred is either a function pointer or a class which overloads
 *  bool operator(void *pointData)
 */
template <typename UnaryPred>
class Filter  ///< Filters are invalidated when modifying the PointTree.
{
public:
	Filter() : data(1) {}  ///< Must be reset before use.
	Filter(PointTree const &pointTree, UnaryPred const &pred_) : data(pointTree.size() + 1), pred(pred_) {}
	void setPred(UnaryPred const &pred_) { pred = pred_; }
	void reset(PointTree const &pointTree) { data.assign(pointTree.size() + 1, 0); }
	void erase(unsigned index) { data[index] = std::max(data[index], 1u); }  ///< Erases the point from query results using the filter.

	unsigned next(unsigned i);

	inline bool operator()(void *pointData)
	{
		const bool ret = pred(pointData);
		if (!ret) erase(i);
		return ret;
	}

private:

	typedef std::vector<unsigned> Data;

	Data data;
	UnaryPred pred;
	unsigned i;
};

template <typename UnaryPred>
unsigned Filter<UnaryPred>::next(unsigned i)
{
	unsigned ret = i;
	while (data[ret])
	{
		ret += data[ret];
	}
	while (data[i])
	{
		unsigned next = i + data[i];
		data[i] = ret - i;
		i = next;
	}

	this->i = ret; // Cache the index for the subsequent filter.operator() call (because it tidies the interface)
	return ret;
}

// Expands bit pattern abcd efgh to 0a0b 0c0d 0e0f 0g0h
static inline uint64_t expand(uint32_t d, bool x)
{
	uint64_t r = d + 0x80000000u;
	r = (r | r<<16) & 0x0000FFFF0000FFFFULL;
	r = (r | r<<8)  & 0x00FF00FF00FF00FFULL;
	r = (r | r<<4)  & 0x0F0F0F0F0F0F0F0FULL;
	r = (r | r<<2)  & 0x3333333333333333ULL;
	r = (r | r<<1)  & 0x5555555555555555ULL;
	return r << (x?1:0);
}

// Inverse of expand
static inline uint32_t unexpand(uint64_t d, bool x)
{
	uint64_t r = d >> (x?1:0);
	r &= 0x5555555555555555ULL;
	r = (r | r>>1)  & 0x3333333333333333ULL;
	r = (r | r>>2)  & 0x0F0F0F0F0F0F0F0FULL;
	r = (r | r>>4)  & 0x00FF00FF00FF00FFULL;
	r = (r | r>>8)  & 0x0000FFFF0000FFFFULL;
	r = (r | r>>16) & 0x00000000FFFFFFFFULL;
	return r - 0x80000000u;
}

// Returns v with highest set bit and all higher bits set, and all following bits 0. Example: 0000 0110 1001 1100 -> 1111 1100 0000 0000.
static inline uint32_t findSplit(uint32_t v)
{
	v |= v>>1;
	v |= v>>2;
	v |= v>>4;
	v |= v>>8;
	v |= v>>16;
	return ~(v>>1);
}

// Interleaves x and y, but after adding 0x80000000u to both, to make their ranges unsigned.
static inline uint64_t interleave(int32_t x, int32_t y)
{
	return expand(x, true) | expand(y, false);
}

void PointTree::insert(void *pointData, int32_t x, int32_t y)
{
	points.push_back(Point(interleave(x, y), pointData));
}

void PointTree::clear()
{
	points.clear();
}

static inline bool pointTreeSortFunction(std::pair<uint64_t, void *> const &a, std::pair<uint64_t, void *> const &b)
{
	return a.first < b.first;  // Sort only by position, not by pointer address, even if two units are in the same place.
}

void PointTree::sort()
{
	std::stable_sort(points.begin(), points.end(), pointTreeSortFunction);  // Stable sort to avoid unspecified behaviour when two objects are in exactly the same place.
}

static inline int getQuadRanges(int32_t minXo, int32_t maxXo, int32_t minYo, int32_t maxYo, RangePack& ranges)
{
	uint64_t minX = expand(minXo, true);
	uint64_t maxX = expand(maxXo, true);
	uint64_t minY = expand(minYo, false);
	uint64_t maxY = expand(maxYo, false);

	uint32_t splitXo = maxXo & findSplit(minXo ^ maxXo);
	uint32_t splitYo = maxYo & findSplit(minYo ^ maxYo);

	uint64_t splitX1 = expand(splitXo - 1, true);
	uint64_t splitX2 = expand(splitXo, true);
	uint64_t splitY1 = expand(splitYo - 1, false);
	uint64_t splitY2 = expand(splitYo, false);

	ranges[0].a = minX    | minY;
	ranges[0].z = splitX1 | splitY1;
	ranges[1].a = splitX2 | minY;
	ranges[1].z = maxX    | splitY1;
	ranges[2].a = splitX1 | maxY;
	ranges[2].z = splitX1 | splitY1;
	ranges[3].a = splitX2 | splitY2;
	ranges[3].z = maxX    | maxY;

	int numRanges = 4;

	// Sort ranges ready to be merged.
	if (ranges[1].a > ranges[2].a)
	{
		std::swap(ranges[1], ranges[2]);
	}
	// Merge ranges if needed.
	if (ranges[2].z + 1 >= ranges[3].a)
	{
		ranges[2].z = ranges[3].z;
		--numRanges;
	}
	if (ranges[1].z + 1 >= ranges[2].a)
	{
		ranges[1].z = ranges[2].z;
		ranges[2] = ranges[3];
		--numRanges;
	}
	if (ranges[0].z + 1 >= ranges[1].a)
	{
		ranges[0].z = ranges[1].z;
		ranges[1] = ranges[2];
		ranges[2] = ranges[3];
		--numRanges;
	}
	return numRanges;
}

template <typename Condition>
PointTree::ResultVector &PointTree::query(Condition &pred, int32_t minXo, int32_t maxXo, int32_t minYo, int32_t maxYo)
{
	RangePack ranges;
	int numRanges = getQuadRanges(minXo, maxXo, minYo, maxYo, ranges);

	lastQueryResults.clear();

	for (int r = 0; r != numRanges; ++r)
	{
		// Find range of points which may be close enough. Range is [i1 ... i2 - 1]. The pointers are ignored when searching.
		Vector::iterator i1 = std::lower_bound(points.begin(), points.end(), Point(ranges[r].a, (void *)NULL), pointTreeSortFunction);
		Vector::iterator i2 = std::upper_bound(i1, points.end(), Point(ranges[r].z, (void *)NULL), pointTreeSortFunction);

		for (; i1 < i2; ++i1)
		{
			int32_t x = unexpand(i1->first, true);
			int32_t y = unexpand(i1->first, false);
			if (pred(x, y, i1->second))	lastQueryResults.push_back(i1->second);
		}
	}

	return lastQueryResults;
}

template <typename Condition, typename UnaryPred>
PointTree::ResultVector &PointTree::query(Condition &pred, Filter<UnaryPred> &filter, int32_t minXo, int32_t maxXo, int32_t minYo, int32_t maxYo)
{
	RangePack ranges;
	int numRanges = getQuadRanges(minXo, maxXo, minYo, maxYo, ranges);

	lastQueryResults.clear();
	for (int r = 0; r != numRanges; ++r)
	{
		// Find range of points which may be close enough. Range is [i1 ... i2 - 1]. The pointers are ignored when searching.
		unsigned i1 = std::distance(points.begin(), std::lower_bound(points.begin(),
																			points.end(),
																			Point(ranges[r].a,
																			(void *)NULL),
																			pointTreeSortFunction));
		unsigned i2 = std::distance(points.begin(), std::upper_bound(points.begin() + i1,
																			points.end(),
																			Point(ranges[r].z,
																			(void *)NULL),
																			pointTreeSortFunction));

		for (unsigned i = filter.next(i1); i < i2; i = filter.next(i + 1))
		{
			if (filter(points[i].second))
			{
				int32_t x = unexpand(points[i].first, true);
				int32_t y = unexpand(points[i].first, false);
				if (pred(x, y, points[i].second))	lastQueryResults.push_back(points[i].second);
			}
		}
	}

	return lastQueryResults;
}

#endif //_point_tree_h

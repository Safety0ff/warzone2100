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
/*
 * mapgrid.cpp
 *
 * Functions for storing objects in a quad-tree like object over the map.
 * The objects are stored in the quad-tree.
 *
 */
#include "lib/framework/types.h"
#include "objects.h"
#include "map.h"

#include "mapgrid.h"
#include "pointtree.h"

// the current state of the iterator
void **gridIterator;

struct ConditionUnseen
{
	ConditionUnseen() : player(0) {}
	ConditionUnseen(int32_t player_) : player(player_) {}
	inline bool operator()(void *obj_) const
	{
		const BASE_OBJECT * const obj = static_cast<BASE_OBJECT*>(obj_);
		return obj->seenThisTick[player] < UINT8_MAX;
	}
	inline bool operator()(int32_t, int32_t, void *obj_) const
	{
		const BASE_OBJECT * const obj = static_cast<BASE_OBJECT*>(obj_);
		return obj->seenThisTick[player] < UINT8_MAX;
	}
	int player;
};

struct ConditionDroidsByPlayer
{
	ConditionDroidsByPlayer() : player(0) {}
	ConditionDroidsByPlayer(int32_t player_) : player(player_) {}
	inline bool operator()(void *obj_) const
	{
		const BASE_OBJECT * const obj = static_cast<BASE_OBJECT*>(obj_);
		return obj->type == OBJ_DROID && obj->player == player;
	}
	inline bool operator()(int32_t, int32_t, void *obj_) const
	{
		const BASE_OBJECT * const obj = static_cast<BASE_OBJECT*>(obj_);
		return obj->type == OBJ_DROID && obj->player == player;
	}
	int player;
};

struct ConditionInCircle
{
	ConditionInCircle() : x(0), y(0), radiusSq(0) {}
	ConditionInCircle(int32_t x_, int32_t y_, uint32_t radius) : x(x_), y(y_), radiusSq(radius*radius) {}
	inline bool operator()(int32_t x_, int32_t y_, void *) const
	{
		return (uint32_t)((x_ - x)*(x_ - x) + (y_ - y)*(y_ - y)) <= radiusSq;
	}
	int32_t x, y;
	uint64_t radiusSq;
};

PointTree *gridPointTree = NULL;  // A quad-tree-like object.
Filter<ConditionUnseen> *gridFiltersUnseen;
Filter<ConditionDroidsByPlayer> *gridFiltersDroidsByPlayer;

// initialise the grid system
bool gridInitialise(void)
{
	ASSERT(gridPointTree == NULL, "gridInitialise already called, without calling gridShutDown.");
	gridPointTree = new PointTree;
	gridFiltersUnseen = new Filter<ConditionUnseen>[MAX_PLAYERS];
	gridFiltersDroidsByPlayer = new Filter<ConditionDroidsByPlayer>[MAX_PLAYERS];

	for (int i = 0; i < MAX_PLAYERS; ++i)
	{
		gridFiltersDroidsByPlayer[i].setPred(ConditionDroidsByPlayer(i));
		gridFiltersUnseen[i].setPred(ConditionUnseen(i));
	}

	return true;  // Yay, nothing failed!
}

// reset the grid system
void gridReset(void)
{
	gridPointTree->clear();

	// Put all existing objects into the point tree.
	for (unsigned player = 0; player < MAX_PLAYERS; player++)
	{
		BASE_OBJECT *start[3] = {(BASE_OBJECT *)apsDroidLists[player], (BASE_OBJECT *)apsStructLists[player], (BASE_OBJECT *)apsFeatureLists[player]};
		for (unsigned type = 0; type != sizeof(start)/sizeof(*start); ++type)
		{
			for (BASE_OBJECT *psObj = start[type]; psObj != NULL; psObj = psObj->psNext)
			{
				if (!psObj->died)
				{
					gridPointTree->insert(psObj, psObj->pos.x, psObj->pos.y);
					for (unsigned viewer = 0; viewer < MAX_PLAYERS; ++viewer)
					{
						psObj->seenThisTick[viewer] = 0;
					}
				}
			}
		}
	}

	gridPointTree->sort();

	for (unsigned player = 0; player < MAX_PLAYERS; ++player)
	{
		gridFiltersUnseen[player].reset(*gridPointTree);
		gridFiltersDroidsByPlayer[player].reset(*gridPointTree);
	}
}

// shutdown the grid system
void gridShutDown(void)
{
	delete gridPointTree;
	gridPointTree = NULL;
	delete[] gridFiltersUnseen;
	gridFiltersUnseen = NULL;
	delete[] gridFiltersDroidsByPlayer;
	gridFiltersDroidsByPlayer = NULL;
}

void gridStartIterate(int32_t x, int32_t y, uint32_t radius)
{
	ConditionInCircle cond(x, y, radius);
	gridPointTree->query(cond, x - radius,
							  x + radius,
							  y - radius,
							  y + radius);
	gridPointTree->lastQueryResults.push_back(NULL);  // NULL-terminate the result.
	gridIterator = &gridPointTree->lastQueryResults[0];
}

void gridStartIterateDroidsByPlayer(int32_t x, int32_t y, uint32_t radius, int player)
{
	ConditionInCircle cond(x, y, radius);
	gridPointTree->query(cond, gridFiltersDroidsByPlayer[player], x - radius,
																  x + radius,
																  y - radius,
																  y + radius);
	gridPointTree->lastQueryResults.push_back(NULL);  // NULL-terminate the result.
	gridIterator = &gridPointTree->lastQueryResults[0];
}

void gridStartIterateUnseen(int32_t x, int32_t y, uint32_t radius, int player)
{
	ConditionInCircle cond(x, y, radius);
	gridPointTree->query(cond, gridFiltersUnseen[player], x - radius,
														  x + radius,
														  y - radius,
														  y + radius);
	gridPointTree->lastQueryResults.push_back(NULL);  // NULL-terminate the result.
	gridIterator = &gridPointTree->lastQueryResults[0];
}

BASE_OBJECT **gridIterateDup(void)
{
	size_t bytes = gridPointTree->lastQueryResults.size()*sizeof(void *);
	BASE_OBJECT **ret = (BASE_OBJECT **)malloc(bytes);
	memcpy(ret, &gridPointTree->lastQueryResults[0], bytes);
	return ret;
}

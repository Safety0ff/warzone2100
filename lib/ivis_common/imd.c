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
#include "imd.h"
#include "ivisdef.h"
#include "tex.h"
#include "pietypes.h"

//*************************************************************************
//*** free IMD shape memory
//*
//* pre		shape successfully allocated
//*
//* params	shape = pointer to IMD shape
//*
//******
void iV_IMDRelease(iIMDShape *s)
{
	iIMDShape *d;

	if (s)
	{
		if (s->points)
		{
			free(s->points);
			s->points = NULL;
		}
		if (s->polys)
		{
			free(s->polys);
			s->polys = NULL;
		}
		if (s->textureArrays)
		{
			free(s->textureArrays);
			s->textureArrays = NULL;
		}
		if (s->connectors)
		{
			free(s->connectors);
			s->connectors = NULL;
		}
		if (s->shadowEdgeList)
		{
			free(s->shadowEdgeList);
			s->shadowEdgeList = NULL;
		}
		d = s->next;
		free(s);
		iV_IMDRelease(d);
	}
}

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
	MapDisplay - Renders the world view necessary for the intelligence map
	Alex McLean, Pumpkin Studios, EIDOS Interactive, 1997

	Makes heavy use of the functions available in display3d.c. Could have
	messed about with display3d.c to make to world render dual purpose, but
	it's neater as a separate file, as the intelligence map has special requirements
	and overlays and needs to render to a specified buffer for later use.
*/

#include "lib/framework/frame.h"
#include "lib/ivis_opengl/piematrix.h"

#include "component.h"
#include "intdisplay.h"
#include "mapdisplay.h"

#define ROTATE_TIME	(2*GAME_TICKS_PER_SEC)

/* renders the Research IMDs into the surface - used by message display in
Intelligence Map */
void renderResearchToBuffer(RESEARCH *psResearch, UDWORD OriginX, UDWORD OriginY, int width, int height)
{
	UDWORD   angle = 0;

	UDWORD          compID;
	Vector3i		Position;
	Vector2i		Rotation;
	Vector2i		bounds(width/2, height/2);

	// Rotate round
	// full rotation once every 2 seconds..
	angle = (gameTime2 % ROTATE_TIME) * 360 / ROTATE_TIME;

	Position.x = OriginX+10;
	Position.y = OriginY+10;
	Position.z = BUTTON_DEPTH;

	// Rotate round
	Rotation.x = -30;
	Rotation.y = angle;

	//draw the IMD for the research
	if (psResearch->psStat)
	{
		//we have a Stat associated with this research topic
		if  (StatIsStructure(psResearch->psStat))
		{
			displayStructureStatButton((STRUCTURE_STATS *)psResearch, Rotation, Position, bounds, true);
		}
		else
		{
			compID = StatIsComponent(psResearch->psStat);
			if (compID != COMP_UNKNOWN)
			{
				displayComponentButton((BASE_STATS *)psResearch, Rotation, Position, bounds, true);
			}
			else
			{
				ASSERT( false, "intDisplayMessageButton: invalid stat" );
				displayResearchButton((BASE_STATS *)psResearch, Rotation, Position, bounds, true);
			}
		}
	}
	else
	{
		//no Stat for this research topic so use the research topic to define what is drawn
		displayResearchButton((BASE_STATS *)psResearch, Rotation, Position, bounds, true);
	}
}

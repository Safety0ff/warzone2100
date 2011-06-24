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
/**
 * @file bucket3d.c
 *
 * Stores object render calls in a linked list renders after bucket sorting objects.
 */

#include "lib/framework/frame.h"
#include "lib/ivis_opengl/piematrix.h"

#include "atmos.h"
#include "bucket3d.h"
#include "component.h"
#include "display3d.h"
#include "effects.h"
#include "map.h"
#include "miscimd.h"

#include <algorithm>

#define CLIP_LEFT	((SDWORD)0)
#define CLIP_RIGHT	((SDWORD)pie_GetVideoBufferWidth())
#define CLIP_TOP	((SDWORD)0)
#define CLIP_BOTTOM ((SDWORD)pie_GetVideoBufferHeight())

struct BUCKET_TAG
{
	bool operator <(BUCKET_TAG const &b) const { return actualZ > b.actualZ; }  // Sort in reverse z order.

	RENDER_TYPE     objectType; //type of object held
	void *          pObject;    //pointer to the object
	int32_t         actualZ;
};

static std::vector<BUCKET_TAG> bucketArray;

static SDWORD bucketCalculateZ(RENDER_TYPE objectType, void* pObject)
{
	SDWORD				radius;
	Vector3i			pixel;
	Vector3f			position;
	DROID				*psDroid;
	BODY_STATS			*psBStats;
	SIMPLE_OBJECT		*psSimpObj;
	COMPONENT_OBJECT	*psCompObj;
	const iIMDShape		*pImd;
	Spacetime			spacetime;
	bool 				clipped = true;

	switch(objectType)
	{
		case RENDER_PARTICLE:
			position = ((ATPART*)pObject)->position;

			position.l_xz() -= player.p.r_xz();

			radius = ((ATPART*)pObject)->imd->radius;
			clipped = pie_ProjectSphere(position, radius, &pixel);
			break;
		case RENDER_PROJECTILE:
			if(((PROJECTILE*)pObject)->psWStats->weaponSubClass == WSC_FLAME ||
                ((PROJECTILE*)pObject)->psWStats->weaponSubClass == WSC_COMMAND ||
                ((PROJECTILE*)pObject)->psWStats->weaponSubClass == WSC_EMP)
			{
				/* We don't do projectiles from these guys, cos there's an effect instead */
				clipped = true;
			}
			else
			{

				//the weapon stats holds the reference to which graphic to use
				pImd = ((PROJECTILE*)pObject)->psWStats->pInFlightGraphic;
				psSimpObj = (SIMPLE_OBJECT*) pObject;

				position = swapYZ(psSimpObj->pos);
				position.l_xz() -= player.p.r_xz();

				radius = pImd->radius;
				clipped = pie_ProjectSphere(position, radius, &pixel);
			}
			break;
		case RENDER_STRUCTURE://not depth sorted
			psSimpObj = (SIMPLE_OBJECT*) pObject;

			position = swapYZ(psSimpObj->pos);
			position.l_xz() -= player.p.r_xz();

			//if((objectType == RENDER_STRUCTURE) && (((STRUCTURE*)pObject)->
			//	pStructureType->type >= REF_DEFENSE) &&
			//	(((STRUCTURE*)pObject)->pStructureType->type<=REF_TOWER4))
			if((objectType == RENDER_STRUCTURE) &&
				((((STRUCTURE*)pObject)->pStructureType->type == REF_DEFENSE) ||
				 (((STRUCTURE*)pObject)->pStructureType->type == REF_WALL) ||
				 (((STRUCTURE*)pObject)->pStructureType->type == REF_WALLCORNER)))
			{
				position.y += 64;
				radius = ((STRUCTURE*)pObject)->sDisplay.imd->radius;//walls guntowers and tank traps clip tightly
			}
			else
			{
				radius = (((STRUCTURE*)pObject)->sDisplay.imd->radius);
			}

			clipped = pie_ProjectSphere(position, radius, &pixel);
			break;
		case RENDER_FEATURE://not depth sorted
			psSimpObj = (SIMPLE_OBJECT*) pObject;

			position = swapYZ(psSimpObj->pos);
			position.l_xz() -= player.p.r_xz();

			radius = ((FEATURE*)pObject)->sDisplay.imd->radius;
			clipped = pie_ProjectSphere(position, radius, &pixel);
			break;
		case RENDER_ANIMATION://not depth sorted
			psCompObj = (COMPONENT_OBJECT *) pObject;
			spacetime = interpolateObjectSpacetime((SIMPLE_OBJECT *)psCompObj->psParent, graphicsTime);

			pie_MatBegin();

			position = swapYZ(spacetime.pos);
			position.l_xz() -= player.p.r_xz();
			pie_TRANSLATE(position.x, position.y, position.z);

			/* parent object rotations */
			pie_MatRotY(spacetime.rot.direction);
			pie_MatRotX(spacetime.rot.pitch);

			/* object (animation) translations - ivis z and y flipped */
			pie_TRANSLATE( psCompObj->position.x, psCompObj->position.z,
							-psCompObj->position.y );

			/* object (animation) rotations */
			pie_MatRotY(psCompObj->orientation.z);
			pie_MatRotZ(-psCompObj->orientation.y);
			pie_MatRotX(psCompObj->orientation.x);

			clipped = pie_Project(Vector3f(0,0,0) ,&pixel);

			pie_MatEnd();

			break;
		case RENDER_DROID:
		case RENDER_SHADOW:
			psDroid = (DROID*) pObject;

			psSimpObj = (SIMPLE_OBJECT*) pObject;

			position = swapYZ(psSimpObj->pos);
			position.l_xz() -= player.p.r_xz();

			if(objectType == RENDER_SHADOW)
			{
				position.y+=4;
			}

			psBStats = asBodyStats + psDroid->asBits[COMP_BODY].nStat;
			radius = psBStats->pIMD->radius;
			clipped = pie_ProjectSphere(position, radius, &pixel);
			break;
		case RENDER_PROXMSG:
			if (((PROXIMITY_DISPLAY *)pObject)->type == POS_PROXDATA)
			{
				position.x = ((VIEW_PROXIMITY *)((VIEWDATA *)((PROXIMITY_DISPLAY *)
							pObject)->psMessage->pViewData)->pData)->x;
				position.z = ((VIEW_PROXIMITY *)((VIEWDATA *)((PROXIMITY_DISPLAY *)
							pObject)->psMessage->pViewData)->pData)->y;
				position.y = ((VIEW_PROXIMITY *)((VIEWDATA *)((PROXIMITY_DISPLAY *)
							pObject)->psMessage->pViewData)->pData)->z;
			}
			else if (((PROXIMITY_DISPLAY *)pObject)->type == POS_PROXOBJ)
			{
				position = ((BASE_OBJECT *)((PROXIMITY_DISPLAY *)pObject)->psMessage->pViewData)->pos;
			}
			position.l_xz() -= player.p.r_xz();

			pImd = getImdFromIndex(MI_BLIP_ENEMY);//use MI_BLIP_ENEMY as all are same radius
			radius = pImd->radius;
			clipped = pie_ProjectSphere(position, radius, &pixel);
			break;
		case RENDER_EFFECT:
			position = ((EFFECT*)pObject)->position;
			position.l_xz() -= player.p.r_xz();

			pImd = ((EFFECT*)pObject)->imd;
			radius = pImd ? pImd->radius : 0;
			clipped = pie_Project(position,&pixel);
			break;

		case RENDER_DELIVPOINT:
			position = swapYZ(((FLAG_POSITION *)pObject)->coords);
			position.l_xz() -= player.p.r_xz();

			radius = pAssemblyPointIMDs[((FLAG_POSITION*)pObject)->factoryType][((FLAG_POSITION*)pObject)->factoryInc]->radius;
			clipped = pie_ProjectSphere(position, radius, &pixel);
			break;
		default:
		break;
	}

	if (clipped)
		return -1;
	return pixel.z;
}

/* add an object to the current render list */
void bucketAddTypeToList(RENDER_TYPE objectType, void* pObject)
{
	const iIMDShape* pie;
	BUCKET_TAG	newTag;
	int32_t		z = bucketCalculateZ(objectType, pObject);

	if (z < 0)
	{
		/* Object will not be render - has been clipped! */
		if(objectType == RENDER_DROID || objectType == RENDER_STRUCTURE)
		{
			/* Won't draw selection boxes */
			((BASE_OBJECT *)pObject)->sDisplay.frameNumber = 0;
		}
		
		return;
	}

	switch(objectType)
	{
		case RENDER_EFFECT:
			switch(((EFFECT*)pObject)->group)
			{
				case EFFECT_EXPLOSION:
				case EFFECT_CONSTRUCTION:
				case EFFECT_SMOKE:
				case EFFECT_FIREWORK:
					// Use calculated Z
					break;

				case EFFECT_WAYPOINT:
					pie = ((EFFECT*)pObject)->imd;
					z = INT32_MAX - pie->texpage;
					break;

				default:
					z = INT32_MAX - 42;
					break;
			}
			break;
		case RENDER_DROID:
			pie = BODY_IMD(((DROID*)pObject),0);
			z = INT32_MAX - pie->texpage;
			break;
		case RENDER_STRUCTURE:
			pie = ((STRUCTURE*)pObject)->sDisplay.imd;
			z = INT32_MAX - pie->texpage;
			break;
		case RENDER_FEATURE:
			pie = ((FEATURE*)pObject)->sDisplay.imd;
			z = INT32_MAX - pie->texpage;
			break;
		case RENDER_ANIMATION:
			pie = ((COMPONENT_OBJECT*)pObject)->psShape;
			z = INT32_MAX - pie->texpage;
			break;
		case RENDER_DELIVPOINT:
			pie = pAssemblyPointIMDs[((FLAG_POSITION*)pObject)->
			factoryType][((FLAG_POSITION*)pObject)->factoryInc];
			z = INT32_MAX - pie->texpage;
			break;
		case RENDER_PARTICLE:
			z = 0;
			break;
		default:
			// Use calculated Z
			break;
	}

	//put the object data into the tag
	newTag.objectType = objectType;
	newTag.pObject = pObject;
	newTag.actualZ = z;

	//add tag to bucketArray
	bucketArray.push_back(newTag);
}


/* render Objects in list */
void bucketRenderCurrentList(void)
{
	std::sort(bucketArray.begin(), bucketArray.end());

	for (std::vector<BUCKET_TAG>::const_iterator thisTag = bucketArray.begin(); thisTag != bucketArray.end(); ++thisTag)
	{
		switch(thisTag->objectType)
		{
			case RENDER_PARTICLE:
				renderParticle((ATPART*)thisTag->pObject);
				break;
			case RENDER_EFFECT:
				renderEffect((EFFECT*)thisTag->pObject);
				break;
			case RENDER_DROID:
				renderDroid((DROID*)thisTag->pObject);
				break;
			case RENDER_SHADOW:
				renderShadow((DROID*)thisTag->pObject, getImdFromIndex(MI_SHADOW));
				break;
			case RENDER_STRUCTURE:
				renderStructure((STRUCTURE*)thisTag->pObject);
				break;
			case RENDER_FEATURE:
				renderFeature((FEATURE*)thisTag->pObject);
				break;
			case RENDER_PROXMSG:
				renderProximityMsg((PROXIMITY_DISPLAY*)thisTag->pObject);
				break;
			case RENDER_PROJECTILE:
				renderProjectile((PROJECTILE*)thisTag->pObject);
				break;
			case RENDER_ANIMATION:
				renderAnimComponent((COMPONENT_OBJECT*)thisTag->pObject);
				break;
			case RENDER_DELIVPOINT:
				renderDeliveryPoint((FLAG_POSITION*)thisTag->pObject, false);
				break;
		}
	}

	//reset the bucket array as we go
	//reset the tag array
	bucketArray.clear();
}

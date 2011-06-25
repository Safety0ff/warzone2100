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
#include "lib/framework/geometry.h"
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
	bool 				clipped = false;

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
				/* HACK: We don't do projectiles from these guys, cos there's an effect instead */
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
		case RENDER_STRUCTURE: // Pre-clipped/only clipped if behind cam.
			psSimpObj = (SIMPLE_OBJECT*) pObject;

			position = swapYZ(psSimpObj->pos);
			position.l_xz() -= player.p.r_xz();

			if((objectType == RENDER_STRUCTURE) &&
				((((STRUCTURE*)pObject)->pStructureType->type == REF_DEFENSE) ||
				 (((STRUCTURE*)pObject)->pStructureType->type == REF_WALL) ||
				 (((STRUCTURE*)pObject)->pStructureType->type == REF_WALLCORNER)))
			{
				//walls guntowers and tank traps clip tightly
				position.y += ((STRUCTURE*)pObject)->sDisplay.imd->max.y;
			}
			radius = (((STRUCTURE*)pObject)->sDisplay.imd->radius);

			pie_ProjectSphere(position, radius, &pixel);
			break;
		case RENDER_FEATURE: // Pre-clipped/only clipped if behind cam.
			psSimpObj = (SIMPLE_OBJECT*) pObject;

			position = swapYZ(psSimpObj->pos);
			position.l_xz() -= player.p.r_xz();

			radius = ((FEATURE*)pObject)->sDisplay.imd->radius;
			pie_ProjectSphere(position, radius, &pixel);
			break;
		case RENDER_ANIMATION: // Pre-clipped/only clipped if behind cam.
		{
			psCompObj = (COMPONENT_OBJECT *) pObject;
			spacetime = interpolateObjectSpacetime((SIMPLE_OBJECT *)psCompObj->psParent, graphicsTime);

			/* Note: We're using this instead of piematrix
			 * because we don't need to change the opengl state.
			 * This can be changed when piematrix transformations
			 * don't also change the opengl state.
			 */
			Affine3F af;

			position = swapYZ(spacetime.pos);
			position.l_xz() -= player.p.r_xz();

			af.Trans(position.x, position.y, position.z);

			/* parent object rotations */
			af.RotY(spacetime.rot.direction);
			af.RotX(spacetime.rot.pitch);

			/* object (animation) translations - ivis z and y flipped */
			af.Trans( psCompObj->position.x, psCompObj->position.z,
							-psCompObj->position.y );

			/* object (animation) rotations (these are noops for our purposes)*/
//			af.RotY(psCompObj->orientation.z);
//			af.RotZ(-psCompObj->orientation.y);
//			af.RotX(psCompObj->orientation.x);

			pie_Project(af.translation() ,&pixel);

			break;
		}
		case RENDER_DROID: // Pre-clipped/only clipped if behind cam.
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
			pie_ProjectSphere(position, radius, &pixel);
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
				case EFFECT_WAYPOINT:
					pie = ((EFFECT*)pObject)->imd;
					z = INT32_MAX - pie->texpage;
					break;
				default:
					// Use calculated Z
					break;
			}
			break;
		/* Nothing but terrain will render behind
		 * these anyways.
		 */
		case RENDER_DELIVPOINT:
			pie = pAssemblyPointIMDs[((FLAG_POSITION*)pObject)->
			factoryType][((FLAG_POSITION*)pObject)->factoryInc];
			z = INT32_MAX - pie->texpage;
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

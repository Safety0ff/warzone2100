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
/** @file
 *  Functions for the label widget.
 */

#include "lib/framework/frame.h"
#include "widget.h"
#include "widgint.h"
#include "label.h"
#include "form.h"
#include "tip.h"
// FIXME Direct iVis implementation include!
#include "lib/ivis_opengl/textdraw.h"

W_LABINIT::W_LABINIT()
	: pText(NULL)
	, pTip(NULL)
	, FontID(font_regular)
{}

W_LABEL::W_LABEL(W_LABINIT const *init)
	: WIDGET(init, WIDG_LABEL)
	, FontID(init->FontID)
	, pTip(init->pTip)
{
	if (display == NULL)
	{
		display = labelDisplay;
	}

	aText[0] = '\0';
	if (init->pText)
	{
		sstrcpy(aText, init->pText);
	}
}


/* Create a button widget data structure */
W_LABEL* labelCreate(const W_LABINIT* psInit)
{
	/* Do some validation on the initialisation struct */
	if (psInit->style & ~(WLAB_PLAIN | WLAB_ALIGNLEFT |
						   WLAB_ALIGNRIGHT | WLAB_ALIGNCENTRE | WIDG_HIDDEN))
	{
		ASSERT( false, "Unknown button style" );
		return NULL;
	}

	/* Allocate the required memory */
	W_LABEL *psWidget = new W_LABEL(psInit);
	if (psWidget == NULL)
	{
		debug(LOG_FATAL, "labelCreate: Out of memory");
		abort();
		return NULL;
	}

	return psWidget;
}


/* Free the memory used by a button */
void labelFree(W_LABEL *psWidget)
{
	ASSERT( psWidget != NULL,
		"labelFree: Invalid label pointer" );

	delete psWidget;
}


/* label display function */
void labelDisplay(WIDGET *psWidget, UDWORD xOffset, UDWORD yOffset, PIELIGHT *pColours)
{
	SDWORD		fx,fy, fw;
	W_LABEL		*psLabel;
	enum iV_fonts FontID;

	psLabel = (W_LABEL *)psWidget;
	FontID = psLabel->FontID;

	iV_SetFont(FontID);
	iV_SetTextColour(pColours[WCOL_TEXT]);
	if (psLabel->style & WLAB_ALIGNCENTRE)
	{
  		fw = iV_GetTextWidth(psLabel->aText);
		fx = xOffset + psLabel->x + (psLabel->width - fw) / 2;
	}
	else if (psLabel->style & WLAB_ALIGNRIGHT)
	{
  		fw = iV_GetTextWidth(psLabel->aText);
		fx = xOffset + psLabel->x + psLabel->width - fw;
	}
	else
	{
		fx = xOffset + psLabel->x;
	}
  	fy = yOffset + psLabel->y + (psLabel->height - iV_GetTextLineSize())/2 - iV_GetTextAboveBase();
	iV_DrawText(psLabel->aText,fx,fy);
}

/* Respond to a mouse moving over a label */
void labelHiLite(W_LABEL *psWidget, W_CONTEXT *psContext)
{
	/* If there is a tip string start the tool tip */
	if (psWidget->pTip)
	{
		tipStart((WIDGET *)psWidget, psWidget->pTip, psContext->psScreen->TipFontID,
				 psContext->psForm->aColours,
				 psWidget->x + psContext->xOffset, psWidget->y + psContext->yOffset,
				 psWidget->width,psWidget->height);
	}
}


/* Respond to the mouse moving off a label */
void labelHiLiteLost(W_LABEL *psWidget)
{
	if (psWidget->pTip)
	{
		tipStop((WIDGET *)psWidget);
	}
}

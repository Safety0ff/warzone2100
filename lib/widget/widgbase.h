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
 *  Definitions for the basic widget types.
 */

#ifndef __INCLUDED_LIB_WIDGET_WIDGBASE_H__
#define __INCLUDED_LIB_WIDGET_WIDGBASE_H__

#include "lib/framework/frame.h"
#include "lib/ivis_opengl/piedef.h"
#include "lib/ivis_opengl/textdraw.h"

/* Button colours */
#define WBUTC_TEXT		0			// Colour for button text
#define WBUTC_BKGRND	1			// Colour for button background
#define WBUTC_BORDER1	2			// Colour for button border
#define WBUTC_BORDER2	3			// 2nd border colour
#define	WBUTC_HILITE	4			// Hilite colour


/* Forward definitions */
struct WIDGET;
struct W_CONTEXT;
struct W_FORM;
struct W_INIT;

/* The display function prototype */
typedef void (*WIDGET_DISPLAY)(WIDGET *psWidget, UDWORD xOffset, UDWORD yOffset, PIELIGHT *pColours);

/* The optional user callback function */
typedef void (*WIDGET_CALLBACK)(WIDGET *psWidget, W_CONTEXT *psContext);
typedef void (*WIDGET_AUDIOCALLBACK)(int AudioID);


/* The different base types of widget */
enum WIDGET_TYPE
{
	WIDG_FORM,
	WIDG_LABEL,
	WIDG_BUTTON,
	WIDG_EDITBOX,
	WIDG_BARGRAPH,
	WIDG_SLIDER,
};

/* The keys that can be used to press a button */
enum WIDGET_KEY
{
	WKEY_NONE,
	WKEY_PRIMARY,
	WKEY_SECONDARY,
};

/* The base widget data type */
struct WIDGET
{
	WIDGET(W_INIT const *init, WIDGET_TYPE type);
	virtual ~WIDGET() {}

	virtual void clicked(W_CONTEXT *, WIDGET_KEY = WKEY_PRIMARY) {}

	UDWORD                  formID;                 ///< ID of the widgets base form.
	UDWORD                  id;                     ///< The user set ID number for the widget. This is returned when e.g. a button is pressed.
	WIDGET_TYPE             type;                   ///< The widget type
	UDWORD                  style;                  ///< The style of the widget
	SWORD                   x, y;                   ///< The location of the widget
	UWORD                   width, height;          ///< The size of the widget
	WIDGET_DISPLAY          display;                ///< Display the widget
	WIDGET_CALLBACK         callback;               ///< User callback (if any)
	void *                  pUserData;              ///< Pointer to a user data block (if any)
	UDWORD                  UserData;               ///< User data (if any)

	WIDGET *                psNext;                 ///< Pointer to the next widget in the screen list
};


/* The screen structure which stores all info for a widget screen */
struct W_SCREEN
{
	W_FORM *         psForm;        ///< The root form of the screen
	WIDGET*          psFocus;       ///< The widget that has keyboard focus
	enum iV_fonts    TipFontID;     ///< ID of the IVIS font to use for tool tips.
	WIDGET*          psRetWidget;   ///< The widget to be returned by widgRunScreen
};

/* Context information to pass into the widget functions */
struct W_CONTEXT
{
	W_SCREEN	*psScreen;			// Parent screen of the widget
	W_FORM *        psForm;                 ///< Parent form of the widget
	SDWORD		xOffset,yOffset;	// Screen offset of the parent form
	SDWORD		mx,my;				// mouse position on the form
};

#endif // __INCLUDED_LIB_WIDGET_WIDGBASE_H__

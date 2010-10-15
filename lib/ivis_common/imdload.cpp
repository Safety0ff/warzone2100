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

/*!
 * \file imdload.c
 *
 * Load IMD (.pie) files
 */

#include <cmath>
#include <vector>
#include <set>
#include <algorithm>

#include "lib/framework/frame.h"
#include "lib/framework/string_ext.h"
#include "lib/framework/frameresource.h"
#include "lib/framework/fixedpoint.h"
#include "lib/ivis_opengl/piematrix.h"

#include "ivisdef.h" // for imd structures
#include "imd.h" // for imd structures
#include "rendmode.h"
#include "tex.h" // texture page loading

static void _imd_compute_bounds(iIMDShape *s);

struct _poly
{
	short indices[3];
	Vector2f texCoords[3];
	unsigned flags;

	/// Texture Animation Data
	unsigned frames;
	unsigned playbackRate;
	float width, height;
};

static BOOL AtEndOfFile(const char *CurPos, const char *EndOfFile)
{
	while ( *CurPos == 0x00 || *CurPos == 0x09 || *CurPos == 0x0a || *CurPos == 0x0d || *CurPos == 0x20 )
	{
		CurPos++;
		if (CurPos >= EndOfFile)
		{
			return true;
		}
	}

	if (CurPos >= EndOfFile)
	{
		return true;
	}
	return false;
}

/*!
 * Load shape level connectors
 * \param ppFileData Pointer to the data (usualy read from a file)
 * \param s Pointer to shape level
 * \return false on error (memory allocation failure/bad file format), true otherwise
 * \pre ppFileData loaded
 * \pre s allocated
 * \pre s->nconnectors set
 * \post s->connectors allocated
 */
static BOOL _imd_load_connectors(const char **ppFileData, iIMDShape *s)
{
	const char *pFileData = *ppFileData;
	int cnt;
	Vector3f *p = NULL, newVector = {0.0f, 0.0f, 0.0f};

	s->connectors = (Vector3f*)malloc(sizeof(Vector3f) * s->nconnectors);
	if (s->connectors == NULL)
	{
		debug(LOG_ERROR, "(_load_connectors) MALLOC fail");
		return false;
	}

	for (p = s->connectors; p < s->connectors + s->nconnectors; p++)
	{
		if (sscanf(pFileData, "%f %f %f%n", &newVector.x, &newVector.y, &newVector.z, &cnt) != 3)
		{
			debug(LOG_ERROR, "(_load_connectors) file corrupt -M");
			return false;
		}
		pFileData += cnt;
		*p = newVector;
	}

	*ppFileData = pFileData;

	return true;
}

static iIMDShape *_imd_new(void)
{
	iIMDShape *s = (iIMDShape*)malloc(sizeof(iIMDShape));
	if (s == NULL)
	{
		return NULL;
	}

	s->flags = 0;
	s->npoints = 0;
	s->npolys = 0;
	s->numFrames = 0;
	s->animInterval = 1;
	s->nconnectors = 0;

	s->points = NULL;
	s->polys = NULL;
//	s->normals = NULL;
	s->textureArrays = NULL;
	s->connectors = NULL;
	s->next = NULL;

	s->shadowEdgeList = NULL;
	s->nShadowEdges = 0;
	s->texpage = iV_TEX_INVALID;
	s->tcmaskpage = iV_TEX_INVALID;

	//s->max = {0.f, 0.f, 0.f};
	//s->min = {0.f, 0.f, 0.f};
	s->radius = 0;
	//s->ocen = {0.f, 0.f, 0.f};
	s->sradius = 0;
	s->radius = 0;

	return s;
}

struct __vertex_idx
{
	unsigned short pie_pos;
	unsigned short imd_pos;
};

inline bool _equal(float a, float b)
{
	return (std::abs(a - b) <= 1/128.f);
}

class _vertex_less
{
	Vector3f *m_points;
public:
	_vertex_less(){}
	_vertex_less(Vector3f *points):m_points(points){}
	void setPoints(Vector3f *points){m_points = points;}
	inline bool operator()(__vertex_idx a, __vertex_idx b)
	{
		Vector3f& v1 = m_points[a.pie_pos];
		Vector3f& v2 = m_points[b.pie_pos];
		if (_equal(v1.x, v2.x))
		{
			if (_equal(v1.y, v2.y))
			{
				if (_equal(v1.z, v2.z))
				{
					return false;
				}
				return v1.z < v2.z;
			}
			return v1.y < v2.y;
		}
		return v1.x < v2.x;
	}
};

static void _pie_uv_frame_calc(const _poly& in, unsigned index, unsigned frame, Vector2f& out)
{
	unsigned framesPerLine = 1;
	if (!(in.flags & iV_IMD_TEXANIM))
	{
		out.x = in.texCoords[index].x;
		out.y = in.texCoords[index].y;
		return;
	}
	else if (in.width >= 1/2048.f)
	{
		framesPerLine = 1 / in.width;
	}
	out.x = (frame % framesPerLine) * (in.width) + in.texCoords[index].x;
	out.y = (frame / framesPerLine) * (in.height) + in.texCoords[index].y;
}

/*!
 * Load shape levels recursively
 * \param ppFileData Pointer to the data (usualy read from a file)
 * \param FileDataEnd ???
 * \param nlevels Number of levels to load
 * \return pointer to iFSDShape structure (or NULL on error)
 * \pre ppFileData loaded
 * \post s allocated
 */
static iIMDShape *_imd_load_level(const char **ppFileData, const char *FileDataEnd, int nlevels, int pieVersion)
{
	const char *pFileName = GetLastResourceFilename();
	const char *pFileData = *ppFileData;
	char buffer[PATH_MAX] = {'\0'};
	iIMDShape *s = NULL;
	_vertex_less vl;
	Vector3f *points;
	std::vector<_poly> polys;
	std::vector<std::vector<Vector2f> > tmpTexArrays;
	Vector2f *tmpTexCoords;
	int cnt = 0, n = 0;
	unsigned i, j, k;

	if (nlevels == 0)
	{
		return NULL;
	}

	s = _imd_new();
	if (s == NULL)
	{
		return NULL;
	}

	if (sscanf(pFileData, "%s %d%n", buffer, &s->npoints, &cnt) != 2)
	{
		debug(LOG_ERROR, "%s: file corrupt", pFileName);
		return NULL;
	}
	pFileData += cnt;

	// Load the points
	if (strcmp(buffer, "POINTS") != 0)
	{
		debug(LOG_ERROR, "%s: expecting 'POINTS' directive", pFileName);
		return NULL;
	}

	if (s->npoints > iV_IMD_MAX_POINTS)
	{
		debug(LOG_ERROR, "%s: too many points in IMD", pFileName);
		return NULL;
	}

	points = (Vector3f*)malloc(sizeof(Vector3f) * s->npoints);
	if (points == NULL)
	{
		return NULL;
	}

	for (i = 0; i < s->npoints; ++i)
	{
		if (sscanf(pFileData, "%f %f %f%n", &points[i].x, &points[i].y, &points[i].z, &cnt) != 3)
		{
			debug(LOG_ERROR, "%s: file corrupt -K", pFileName);
			free(points);
			return NULL;
		}
		pFileData += cnt;
	}
	// Done loading points

	// Begin loading polygons
	if (sscanf(pFileData, "%s %u%n", buffer, &s->npolys, &cnt) != 2)
	{
		debug(LOG_ERROR, "%s: file corrupt", pFileName);
		free(points);
		return NULL;
	}
	pFileData += cnt;

	if (strcmp(buffer, "POLYGONS") != 0)
	{
		debug(LOG_ERROR,"%s: expected 'POLYGONS' directive", pFileName);
		free(points);
		return NULL;
	}

	polys.reserve(s->npolys * 2);

	/* Load the polygons, break them into triangles and store them in a temporary location
	 * for further processing before inserting them into iIMDShape.
	 */
	for (i = 0; i < s->npolys; ++i)
	{
		unsigned short pieces[pie_MAX_VERTICES_PER_POLYGON-3];
		Vector2f texPieces[pie_MAX_VERTICES_PER_POLYGON-3];
		unsigned int npnts;
		_poly poly;

		if (sscanf(pFileData, "%x %u%n", &poly.flags, &npnts, &cnt) != 2)
		{
			debug(LOG_ERROR, "%s: error loading flags and npoints [poly %u]", pFileName, i);
		}
		pFileData += cnt;

		if (sscanf(pFileData, "%hu %hu %hu%n", &poly.indices[0], &poly.indices[1], &poly.indices[2], &cnt) != 3)
		{
			debug(LOG_ERROR, "%s: failed reading indices of poly %u", pFileName, i);
			free(points);
			return NULL;
		}
		pFileData += cnt;

		for (j = 0; j < npnts -3; j++)
		{
			if (sscanf(pFileData, "%hu%n", &pieces[j], &cnt) != 1)
			{
				debug(LOG_ERROR, "failed poly %u. point %d", i, j);
				free(points);
				return NULL;
			}
			pFileData += cnt;
		}

		if (poly.flags & iV_IMD_TEXANIM)
		{
			if (sscanf(pFileData, "%u %u %f %f%n", &poly.frames, &poly.playbackRate, &poly.width, &poly.height, &cnt) != 4)
			{
				debug(LOG_ERROR, "%s: error reading texanim data [poly %u]", pFileName, i);
				free(points);
				return NULL;
			}
			pFileData += cnt;

			ASSERT(poly.width > 0, "%s: Non-positive tex anim width on poly %u", pFileName, i);
			ASSERT(poly.height > 0, "%s: Non-positive tex anim height on poly %u", pFileName, i);

			if (pieVersion == PIE_VER)
			{
				poly.width /= OLD_TEXTURE_SIZE_FIX;
				poly.height /= OLD_TEXTURE_SIZE_FIX;
			}
		}
		else
		{
			poly.playbackRate = 1;
			poly.frames = 1;
			poly.width = 0;
			poly.height = 0;
		}

		/* Must have same number of frames and same playback rate for all polygons */
		if (poly.frames > s->numFrames)
		{
			s->numFrames = poly.frames;
		}
		if (poly.playbackRate > s->animInterval)
		{
			s->animInterval = poly.playbackRate;
		}

		// PC texture coord routine
		if (poly.flags & iV_IMD_TEX)
		{
			for (j = 0; j < 3; j++)
			{
				if (sscanf(pFileData, "%f %f%n", &poly.texCoords[j].x, &poly.texCoords[j].y, &cnt) != 2)
				{
					debug(LOG_ERROR, "%s: error reading texcoords [poly %u]", pFileName, i);
					return NULL;
				}
				pFileData += cnt;

				if (pieVersion == PIE_VER)
				{
					poly.texCoords[j].x /= OLD_TEXTURE_SIZE_FIX;
					poly.texCoords[j].y /= OLD_TEXTURE_SIZE_FIX;
				}
			}
			for (j = 0; j < npnts -3; j++)
			{
				if (sscanf(pFileData, "%f %f%n", &texPieces[j].x, &texPieces[j].y, &cnt) != 2)
				{
					debug(LOG_ERROR, "%s: error reading texcoords [poly %u]", pFileName, i);
					return NULL;
				}
				pFileData += cnt;

				if (pieVersion == PIE_VER)
				{
					texPieces[j].x /= OLD_TEXTURE_SIZE_FIX;
					texPieces[j].y /= OLD_TEXTURE_SIZE_FIX;
				}
			}
		}

		polys.push_back(poly);
		for ( j = 0; j < npnts -3; j++)
		{
			poly.indices[1] = poly.indices[2];
			poly.texCoords[1] = poly.texCoords[2];
			poly.indices[2] = pieces[j];
			poly.texCoords[2] = texPieces[j];
			polys.push_back(poly);
		}
	}

	tmpTexArrays.resize(s->numFrames);
	tmpTexCoords = (Vector2f*)malloc(sizeof(Vector2f) * s->numFrames);

	vl.setPoints(points);
	{
		typedef std::multiset<__vertex_idx, _vertex_less> settype;
		settype set(vl);
		std::pair<settype::iterator, settype::iterator> equl_rng;
		settype::iterator it;
		__vertex_idx idx;

		for (i = 0; i < polys.size(); ++i)
		{
			for (j = 0; j < 3; ++j)
			{
				bool foundMatch = false;

				idx.pie_pos = polys[i].indices[j];

				for (k = 0; k < s->numFrames; ++k)
				{
					_pie_uv_frame_calc(polys[i], j, k, tmpTexCoords[k]);
				}

				for(equl_rng = set.equal_range(idx); equl_rng.first != equl_rng.second; ++equl_rng.first)
				{
					for (k = 0; k < s->numFrames; ++k)
					{
						if (std::abs(tmpTexArrays[k][equl_rng.first->imd_pos].x - tmpTexCoords[k].x) <= 1/2048.f)
						{
							if (std::abs(tmpTexArrays[k][equl_rng.first->imd_pos].y - tmpTexCoords[k].y) > 1/2048.f)
							{
								break;
							}
						}
						else
						{
							break;
						}
					}
					if (k >= s->numFrames)
					{
						polys[i].indices[j] = equl_rng.first->imd_pos;
						foundMatch = true;
						break;
					}
				}
				if (!foundMatch)
				{
					polys[i].indices[j] = idx.imd_pos = set.size();
					for (k = 0; k < s->numFrames; ++k)
					{
						tmpTexArrays[k].push_back(tmpTexCoords[k]);
					}
					set.insert(equl_rng.first, idx);
				}
			}
		}

		s->points = (Vector3f*)malloc(sizeof(Vector3f) * set.size());
		for (it = set.begin(); it != set.end(); ++it)
		{
			s->points[it->imd_pos] = points[it->pie_pos];
		}
		s->npoints = set.size();

		s->polys = (unsigned short*)malloc(sizeof(unsigned short) * polys.size() * 3);
		for (i = 0; i < polys.size(); ++i)
		{
			s->polys[i*3] = polys[i].indices[0];
			s->polys[i*3+1] = polys[i].indices[1];
			s->polys[i*3+2] = polys[i].indices[2];
		}
		s->npolys = polys.size();

		s->textureArrays = (Vector2f*)malloc(sizeof(Vector2f) * tmpTexArrays.size() * s->npoints);
		for (i = 0; i < tmpTexArrays.size(); ++i)
		{
			memcpy(&s->textureArrays[i * s->npoints], &tmpTexArrays[i][0], sizeof(Vector2f) * s->npoints);
		}
	}
	polys.clear();
	tmpTexArrays.clear();
	free(tmpTexCoords);
	free(points);
	_imd_compute_bounds(s);

	// NOW load optional stuff
	while (!AtEndOfFile(pFileData, FileDataEnd)) // check for end of file (give or take white space)
	{
		// Scans in the line ... if we don't get 2 parameters then quit
		if (sscanf(pFileData, "%s %d%n", buffer, &n, &cnt) != 2)
		{
			break;
		}
		pFileData += cnt;

		// check for next level
		if (strcmp(buffer, "LEVEL") == 0)
		{
			s->next = _imd_load_level(&pFileData, FileDataEnd, nlevels - 1, pieVersion);
		}
		else if (strcmp(buffer, "CONNECTORS") == 0)
		{
			// load connectors
			s->nconnectors = n;
			_imd_load_connectors( &pFileData, s );
		}
		else
		{
			debug(LOG_ERROR, "unexpected directive %s %d", buffer, n);
			break;
		}
	}

	*ppFileData = pFileData;

	return s;
}


/*!
 * Load ppFileData into a shape
 * \param ppFileData Data from the IMD file
 * \param FileDataEnd Endpointer
 * \return The shape, constructed from the data read
 */
// ppFileData is incremented to the end of the file on exit!
iIMDShape *iV_ProcessIMD( const char **ppFileData, const char *FileDataEnd )
{
	const char *pFileName = GetLastResourceFilename();
	const char *pFileData = *ppFileData;
	char buffer[PATH_MAX], texfile[PATH_MAX];
	int cnt, nlevels;
	iIMDShape *shape, *psShape;
	UDWORD level;
	int32_t imd_version;
	uint32_t imd_flags;
	BOOL bTextured = false;

	if (sscanf(pFileData, "%s %d%n", buffer, &imd_version, &cnt) != 2)
	{
		debug(LOG_ERROR, "%s bad version: (%s)", pFileName, buffer);
		assert(false);
		return NULL;
	}
	pFileData += cnt;

	if (strncmp(PIE_NAME, buffer, strlen(PIE_NAME)) !=0 )
	{
		debug(LOG_ERROR, "%s not an IMD file (%s %d)", pFileName, buffer, imd_version);
		return NULL;
	}

	//Now supporting version PIE_VER and PIE_FLOAT_VER files
	if (imd_version != PIE_VER && imd_version != PIE_FLOAT_VER)
	{
		debug(LOG_ERROR, "%s version %d not supported", pFileName, imd_version);
		return NULL;
	}

	/* Flags are ignored now. Reading them in just to pass the buffer. */
	if (sscanf(pFileData, "%s %x%n", buffer, &imd_flags, &cnt) != 2)
	{
		debug(LOG_ERROR, "%s bad flags: %s", pFileName, buffer);
		return NULL;
	}
	pFileData += cnt;

	/* This can be either texture or levels */
	if (sscanf(pFileData, "%s %d%n", buffer, &nlevels, &cnt) != 2)
	{
		debug(LOG_ERROR, "%s expecting TEXTURE or LEVELS: %s", pFileName, buffer);
		return NULL;
	}
	pFileData += cnt;

	// get texture page if specified
	if (strncmp(buffer, "TEXTURE", 7) == 0)
	{
		int i, pwidth, pheight;
		char ch, texType[PATH_MAX];

		/* the first parameter for textures is always ignored; which is why we ignore
		 * nlevels read in above */
		ch = *pFileData++;

		// Run up to the dot or till the buffer is filled. Leave room for the extension.
		for (i = 0; i < PATH_MAX-5 && (ch = *pFileData++) != '\0' && ch != '.'; ++i)
		{
 			texfile[i] = ch;
		}
		texfile[i] = '\0';

		if (sscanf(pFileData, "%s%n", texType, &cnt) != 1)
		{
			debug(LOG_ERROR, "%s texture info corrupt: %s", pFileName, buffer);
			return NULL;
		}
		pFileData += cnt;

		if (strcmp(texType, "png") != 0)
		{
			debug(LOG_ERROR, "%s: only png textures supported", pFileName);
			return NULL;
		}
		sstrcat(texfile, ".png");

		pie_MakeTexPageName(texfile);

		if (sscanf(pFileData, "%d %d%n", &pwidth, &pheight, &cnt) != 2)
		{
			debug(LOG_ERROR, "%s bad texture size: %s", pFileName, buffer);
			return NULL;
		}
		pFileData += cnt;

		/* Now read in LEVELS directive */
		if (sscanf(pFileData, "%s %d%n", buffer, &nlevels, &cnt) != 2)
		{
			debug(LOG_ERROR, "%s bad levels info: %s", pFileName, buffer);
			return NULL;
		}
		pFileData += cnt;

		bTextured = true;
	}

	if (strncmp(buffer, "LEVELS", 6) != 0)
	{
		debug(LOG_ERROR, "expecting 'LEVELS' directive (%s)", buffer);
		return NULL;
	}

	/* Read first LEVEL directive */
	if (sscanf(pFileData, "%s %d%n", buffer, &level, &cnt) != 2)
	{
		debug(LOG_ERROR, "file corrupt -J");
		return NULL;
	}
	pFileData += cnt;

	if (strncmp(buffer, "LEVEL", 5) != 0)
	{
		debug(LOG_ERROR, "expecting 'LEVEL' directive (%s)", buffer);
		return NULL;
	}

	shape = _imd_load_level(&pFileData, FileDataEnd, nlevels, imd_version);
	if (shape == NULL)
	{
		debug(LOG_ERROR, "%s unsuccessful", pFileName);
		return NULL;
	}

	// load texture page if specified
	if (bTextured)
	{
		int texpage = iV_GetTexture(texfile);

		if (texpage < 0)
		{
			debug(LOG_ERROR, "%s could not load tex page %s", pFileName, texfile);
			return NULL;
		}

		// assign tex page to levels
		for (psShape = shape; psShape != NULL; psShape = psShape->next)
		{
			psShape->texpage = texpage;
		}

		// check if model should use team colour mask
		if (imd_flags & iV_IMD_TCMASK)
		{
			pie_MakeTexPageTCMaskName(texfile);
			texpage = iV_GetTexture(texfile);

			if (texpage < 0)
			{
				ASSERT(false, "iV_ProcessIMD %s could not load tcmask %s", pFileName, texfile);
				debug(LOG_ERROR, "%s could not load tcmask %s", pFileName, texfile);
			}
			else
			{
				// Propagate settings through levels
				for (psShape = shape; psShape != NULL; psShape = psShape->next)
				{
					psShape->flags |= iV_IMD_TCMASK;
					psShape->tcmaskpage = texpage;
				}
			}
		}
	}

	*ppFileData = pFileData;

	return shape;
}

static void _imd_compute_bounds(iIMDShape *s)
{
	Vector3f *p = NULL;
	int32_t tempXMax, tempXMin, tempZMax, tempZMin;
	int32_t xmax, ymax, zmax;
	double dx, dy, dz, rad_sq, rad, old_to_p_sq, old_to_p, old_to_new;
	double xspan, yspan, zspan, maxspan;
	Vector3f dia1, dia2, cen;
	Vector3f vxmin = { 0, 0, 0 }, vymin = { 0, 0, 0 }, vzmin = { 0, 0, 0 },
	         vxmax = { 0, 0, 0 }, vymax = { 0, 0, 0 }, vzmax = { 0, 0, 0 };

	s->max.x = s->max.y = s->max.z = tempXMax = tempZMax = -FP12_MULTIPLIER;
	s->min.x = s->min.y = s->min.z = tempXMin = tempZMin = FP12_MULTIPLIER;

	vxmax.x = vymax.y = vzmax.z = -FP12_MULTIPLIER;
	vxmin.x = vymin.y = vzmin.z = FP12_MULTIPLIER;

	// set up bounding data for minimum number of vertices
	for (p = s->points; p < s->points + s->npoints; p++)
	{
		if (p->x > s->max.x)
		{
			s->max.x = p->x;
		}
		if (p->x < s->min.x)
		{
			s->min.x = p->x;
		}

		/* Biggest x coord so far within our height window? */
		if( p->x > tempXMax && p->y > DROID_VIS_LOWER && p->y < DROID_VIS_UPPER )
		{
			tempXMax = p->x;
		}

		/* Smallest x coord so far within our height window? */
		if( p->x < tempXMin && p->y > DROID_VIS_LOWER && p->y < DROID_VIS_UPPER )
		{
			tempXMin = p->x;
		}

		if (p->y > s->max.y)
		{
			s->max.y = p->y;
		}
		if (p->y < s->min.y)
		{
			s->min.y = p->y;
		}

		if (p->z > s->max.z)
		{
			s->max.z = p->z;
		}
		if (p->z < s->min.z)
		{
			s->min.z = p->z;
		}

		/* Biggest z coord so far within our height window? */
		if( p->z > tempZMax && p->y > DROID_VIS_LOWER && p->y < DROID_VIS_UPPER )
		{
			tempZMax = p->z;
		}

		/* Smallest z coord so far within our height window? */
		if( p->z < tempZMax && p->y > DROID_VIS_LOWER && p->y < DROID_VIS_UPPER )
		{
			tempZMin = p->z;
		}

		// for tight sphere calculations
		if (p->x < vxmin.x)
		{
			vxmin.x = p->x;
			vxmin.y = p->y;
			vxmin.z = p->z;
		}

		if (p->x > vxmax.x)
		{
			vxmax.x = p->x;
			vxmax.y = p->y;
			vxmax.z = p->z;
		}

		if (p->y < vymin.y)
		{
			vymin.x = p->x;
			vymin.y = p->y;
			vymin.z = p->z;
		}

		if (p->y > vymax.y)
		{
			vymax.x = p->x;
			vymax.y = p->y;
			vymax.z = p->z;
		}

		if (p->z < vzmin.z)
		{
			vzmin.x = p->x;
			vzmin.y = p->y;
			vzmin.z = p->z;
		}

		if (p->z > vzmax.z)
		{
			vzmax.x = p->x;
			vzmax.y = p->y;
			vzmax.z = p->z;
		}
	}

	// no need to scale an IMD shape (only FSD)
	xmax = MAX(s->max.x, -s->min.x);
	ymax = MAX(s->max.y, -s->min.y);
	zmax = MAX(s->max.z, -s->min.z);

	s->radius = MAX(xmax, MAX(ymax, zmax));
	s->sradius = sqrtf(xmax*xmax + ymax*ymax + zmax*zmax);

// START: tight bounding sphere

	// set xspan = distance between 2 points xmin & xmax (squared)
	dx = vxmax.x - vxmin.x;
	dy = vxmax.y - vxmin.y;
	dz = vxmax.z - vxmin.z;
	xspan = dx*dx + dy*dy + dz*dz;

	// same for yspan
	dx = vymax.x - vymin.x;
	dy = vymax.y - vymin.y;
	dz = vymax.z - vymin.z;
	yspan = dx*dx + dy*dy + dz*dz;

	// and ofcourse zspan
	dx = vzmax.x - vzmin.x;
	dy = vzmax.y - vzmin.y;
	dz = vzmax.z - vzmin.z;
	zspan = dx*dx + dy*dy + dz*dz;

	// set points dia1 & dia2 to maximally seperated pair
	// assume xspan biggest
	dia1 = vxmin;
	dia2 = vxmax;
	maxspan = xspan;

	if (yspan > maxspan)
	{
		maxspan = yspan;
		dia1 = vymin;
		dia2 = vymax;
	}

	if (zspan > maxspan)
	{
		maxspan = zspan;
		dia1 = vzmin;
		dia2 = vzmax;
	}

	// dia1, dia2 diameter of initial sphere
	cen.x = (dia1.x + dia2.x) / 2.;
	cen.y = (dia1.y + dia2.y) / 2.;
	cen.z = (dia1.z + dia2.z) / 2.;

	// calc initial radius
	dx = dia2.x - cen.x;
	dy = dia2.y - cen.y;
	dz = dia2.z - cen.z;

	rad_sq = dx*dx + dy*dy + dz*dz;
	rad = sqrt(rad_sq);

	// second pass (find tight sphere)
	for (p = s->points; p < s->points + s->npoints; p++)
	{
		dx = p->x - cen.x;
		dy = p->y - cen.y;
		dz = p->z - cen.z;
		old_to_p_sq = dx*dx + dy*dy + dz*dz;

		// do r**2 first
		if (old_to_p_sq>rad_sq)
		{
			// this point outside current sphere
			old_to_p = sqrt(old_to_p_sq);
			// radius of new sphere
			rad = (rad + old_to_p) / 2.;
			// rad**2 for next compare
			rad_sq = rad*rad;
			old_to_new = old_to_p - rad;
			// centre of new sphere
			cen.x = (rad*cen.x + old_to_new*p->x) / old_to_p;
			cen.y = (rad*cen.y + old_to_new*p->y) / old_to_p;
			cen.z = (rad*cen.z + old_to_new*p->z) / old_to_p;
		}
	}

	s->ocen = cen;

// END: tight bounding sphere
}
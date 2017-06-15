/*
 * CDDL HEADER START
 *
 * This file and its contents are supplied under the terms of the
 * Common Development and Distribution License ("CDDL"), version 1.0.
 * You may only use this file in accordance with the terms of version
 * 1.0 of the CDDL.
 *
 * A full copy of the text of the CDDL should have accompanied this
 * source.  A copy of the CDDL is also available via the Internet at
 * http://www.illumos.org/license/CDDL.
 *
 * CDDL HEADER END
 */
/*
 * Copyright 2017 Saso Kiselkov. All rights reserved.
 */

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "XPLMDataAccess.h"
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMPlanes.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"

#include "assert.h"
#include "avl.h"
#include "geom.h"
#include "log.h"
#include "helpers.h"
#include "snd_sys.h"
#include "types.h"
#include "xtcas.h"
#include "thread.h"

#include "xplane.h"

#define	FLOOP_INTVAL			1.0
#define	POS_UPDATE_INTVAL		0.1
#define	XTCAS_PLUGIN_NAME		"X-TCAS 1.0"
#define	XTCAS_PLUGIN_SIG		"skiselkov.xtcas.1.0"
#define	XTCAS_PLUGIN_DESCRIPTION \
	"Generic TCAS II v7.1 implementation for X-Plane"

#define	MAX_MP_PLANES	19
#define	MAX_DR_NAME_LEN	256

static bool_t intf_inited = B_FALSE;
static XPLMDataRef time_dr = NULL,
    baro_alt_dr = NULL,
    rad_alt_dr = NULL,
    lat_dr = NULL,
    lon_dr = NULL,
    plane_x_dr = NULL,
    plane_y_dr = NULL,
    plane_z_dr = NULL;

static XPLMDataRef mp_plane_x_dr[MAX_MP_PLANES],
    mp_plane_y_dr[MAX_MP_PLANES],
    mp_plane_z_dr[MAX_MP_PLANES];

static mutex_t acf_pos_lock;
static avl_tree_t acf_pos_tree;
static double last_pos_collected = 0;

static char plugindir[512] = { 0 };

static double xp_get_time(void *handle);
static void xp_get_my_acf_pos(void *handle, geo_pos3_t *pos, double *alt_agl);
static void xp_get_oth_acf_pos(void *handle, acf_pos_t **pos_p, size_t *num);
static bool_t xp_view_is_external(void *handle);

static const sim_intf_ops_t xp_intf_ops = {
	.handle = NULL,
	.get_time = xp_get_time,
	.get_my_acf_pos = xp_get_my_acf_pos,
	.get_oth_acf_pos = xp_get_oth_acf_pos,
	.update_contact = NULL,
	.update_RA = NULL,
	.update_RA_prediction = NULL
};

static const snd_intf_ops_t xp_snd_ops = {
	.handle = NULL,
	.sound_is_on = xp_view_is_external
};

static int
acf_pos_compar(const void *a, const void *b)
{
	const acf_pos_t *pa = a, *pb = b;

	if (pa->acf_id < pb->acf_id)
		return (-1);
	else if (pa->acf_id == pb->acf_id)
		return (0);
	else
		return (1);
}

static XPLMDataRef
find_dr_chk(XPLMDataTypeID type, const char *fmt, ...)
{
	char name[MAX_DR_NAME_LEN];
	va_list ap;

	va_start(ap, fmt);
	vsnprintf(name, sizeof (name), fmt, ap);
	va_end(ap);

	XPLMDataRef dr = XPLMFindDataRef(name);
	ASSERT(dr != NULL);
	ASSERT3U((XPLMGetDataRefTypes(dr) & type), ==, type);

	return (dr);
}

static void
sim_intf_init(void)
{
	time_dr = find_dr_chk(xplmType_Float,
	    "sim/time/total_running_time_sec");
	baro_alt_dr = find_dr_chk(xplmType_Float,
	    "sim/flightmodel/misc/h_ind");
	rad_alt_dr = find_dr_chk(xplmType_Float,
	    "sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot");
	lat_dr = find_dr_chk(xplmType_Double,
	    "sim/flightmodel/position/latitude");
	lon_dr = find_dr_chk(xplmType_Double,
	    "sim/flightmodel/position/longitude");

	plane_x_dr = find_dr_chk(xplmType_Double,
	    "sim/flightmodel/position/local_x");
	plane_y_dr = find_dr_chk(xplmType_Double,
	    "sim/flightmodel/position/local_y");
	plane_z_dr = find_dr_chk(xplmType_Double,
	    "sim/flightmodel/position/local_z");

	for (int i = 0; i < MAX_MP_PLANES; i++) {
		mp_plane_x_dr[i] = find_dr_chk(xplmType_Double,
		    "sim/multiplayer/position/plane%d_x", i + 1);
		mp_plane_y_dr[i] = find_dr_chk(xplmType_Double,
		    "sim/multiplayer/position/plane%d_y", i + 1);
		mp_plane_z_dr[i] = find_dr_chk(xplmType_Double,
		    "sim/multiplayer/position/plane%d_z", i + 1);
	}

	avl_create(&acf_pos_tree, acf_pos_compar, sizeof (acf_pos_t),
	    offsetof(acf_pos_t, tree_node));
	mutex_init(&acf_pos_lock);

	intf_inited = B_TRUE;
}

static void
sim_intf_fini(void)
{
	void *cookie = NULL;
	acf_pos_t *p;

	time_dr = NULL;
	baro_alt_dr = NULL;
	rad_alt_dr = NULL;
	lat_dr = NULL;
	lon_dr = NULL;

	plane_x_dr = NULL;
	plane_y_dr = NULL;
	plane_z_dr = NULL;

	memset(mp_plane_x_dr, 0, sizeof (mp_plane_x_dr));
	memset(mp_plane_y_dr, 0, sizeof (mp_plane_y_dr));
	memset(mp_plane_z_dr, 0, sizeof (mp_plane_z_dr));

	while ((p = avl_destroy_nodes(&acf_pos_tree, &cookie)) != NULL)
		free(p);
	avl_destroy(&acf_pos_tree);
	mutex_destroy(&acf_pos_lock);

	intf_inited = B_FALSE;
}

static int
acf_pos_collector(XPLMDrawingPhase phase, int before, void *ref)
{
	UNUSED(phase);
	UNUSED(before);
	UNUSED(ref);

	double now;

	/* grab updates only at a set interval */
	now = xp_get_time(NULL);
	if (last_pos_collected + POS_UPDATE_INTVAL > now)
		return (1);
	last_pos_collected = now;

	for (int i = 0; i < MAX_MP_PLANES; i++) {
		vect3_t local;
		geo_pos3_t world;
		avl_index_t where;
		acf_pos_t srch;
		acf_pos_t *pos;

		local.x = XPLMGetDatad(mp_plane_x_dr[i]);
		local.y = XPLMGetDatad(mp_plane_y_dr[i]);
		local.z = XPLMGetDatad(mp_plane_z_dr[i]);

		mutex_enter(&acf_pos_lock);

		srch.acf_id = (void *)(uintptr_t)(i + 1);
		pos = avl_find(&acf_pos_tree, &srch, &where);
		/*
		 * This is exceedingly unlikely, so it's "good enough" to use
		 * as an emptiness test.
		 */
		if (IS_ZERO_VECT3(local)) {
			if (pos != NULL) {
				avl_remove(&acf_pos_tree, pos);
				free(pos);
			}
		} else {
			if (pos == NULL) {
				pos = calloc(1, sizeof (*pos));
				pos->acf_id = (void *)(uintptr_t)(i + 1);
				avl_insert(&acf_pos_tree, pos, where);
			}
			XPLMLocalToWorld(local.x, local.y, local.z,
			    &world.lat, &world.lon, &world.elev);
			pos->pos = world;
		}

		mutex_exit(&acf_pos_lock);
	}

	return (1);
}

static float
floop_cb(float inElapsedSinceLastCall, float inElapsedTimeSinceLastFlightLoop,
    int counter, void *ref)
{
	UNUSED(inElapsedSinceLastCall);
	UNUSED(inElapsedTimeSinceLastFlightLoop);
	UNUSED(counter);
	UNUSED(ref);

	xtcas_run();

	return (FLOOP_INTVAL);
}

static double
xp_get_time(void *handle)
{
	UNUSED(handle);
	ASSERT(intf_inited);
	return (XPLMGetDataf(time_dr));
}

static void
xp_get_my_acf_pos(void *handle, geo_pos3_t *pos, double *alt_agl)
{
	UNUSED(handle);
	ASSERT(intf_inited);
	pos->lat = XPLMGetDatad(lat_dr);
	pos->lon = XPLMGetDatad(lon_dr);
	pos->elev = XPLMGetDataf(baro_alt_dr);
	*alt_agl = XPLMGetDataf(rad_alt_dr);
}

static void
xp_get_oth_acf_pos(void *handle, acf_pos_t **pos_p, size_t *num)
{
	size_t i;
	acf_pos_t *pos;

	UNUSED(handle);

	mutex_enter(&acf_pos_lock);
	*num = avl_numnodes(&acf_pos_tree);
	*pos_p = calloc(*num, sizeof (*pos));
	for (pos = avl_first(&acf_pos_tree), i = 0; pos != NULL;
	    pos = AVL_NEXT(&acf_pos_tree, pos), i++) {
		ASSERT3U(i, <, *num);
		memcpy(&(*pos_p)[i], pos, sizeof (*pos));
	}
	mutex_exit(&acf_pos_lock);
}

static bool_t
xp_view_is_external(void *handle)
{
	UNUSED(handle);
	return (B_FALSE);
}

static float
snd_sched_cb(float elapsed_since_last_call, float elapsed_since_last_floop,
    int counter, void *refcon)
{
	UNUSED(elapsed_since_last_call);
	UNUSED(elapsed_since_last_floop);
	UNUSED(counter);
	UNUSED(refcon);
	xtcas_snd_sys_run();
	return (-1.0);
}

PLUGIN_API int
XPluginStart(char *name, char *sig, char *desc)
{
	char *snd_dir;

	XPLMGetPluginInfo(XPLMGetMyID(), NULL, plugindir, NULL, NULL);

	strcpy(name, XTCAS_PLUGIN_NAME);
	strcpy(sig, XTCAS_PLUGIN_SIG);
	strcpy(desc, XTCAS_PLUGIN_DESCRIPTION);
	sim_intf_init();
	snd_dir = mkpathname(plugindir, "data", "male1", NULL);
	if (!xtcas_snd_sys_init(plugindir, &xp_snd_ops)) {
		free(snd_dir);
		return (0);
	}
	free(snd_dir);

	return (1);
}

PLUGIN_API void
XPluginStop(void)
{
	xtcas_snd_sys_fini();
	sim_intf_fini();
}

PLUGIN_API void
XPluginEnable(void)
{
	xtcas_init(&xp_intf_ops);
	XPLMRegisterFlightLoopCallback(floop_cb, FLOOP_INTVAL, NULL);
	XPLMRegisterDrawCallback(acf_pos_collector, xplm_Phase_Panel, 1, NULL);
	XPLMRegisterFlightLoopCallback(snd_sched_cb, -1.0, NULL);
}

PLUGIN_API void
XPluginDisable(void)
{
	XPLMUnregisterFlightLoopCallback(snd_sched_cb, NULL);
	XPLMUnregisterDrawCallback(acf_pos_collector, xplm_Phase_Panel,
	    1, NULL);
	XPLMUnregisterFlightLoopCallback(floop_cb, NULL);
	xtcas_fini();
}

PLUGIN_API void
XPluginReceiveMessage(XPLMPluginID from, int msg, void *param)
{
	UNUSED(from);
	UNUSED(msg);
	UNUSED(param);
}

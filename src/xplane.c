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
 * Copyright 2016 Saso Kiselkov. All rights reserved.
 */

#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include "XPLMDataAccess.h"
#include "XPLMPlanes.h"
#include "XPLMPlugin.h"
#include "XPLMProcessing.h"

#include "log.h"
#include "helpers.h"
#include "types.h"
#include "xtcas.h"

#include "xplane.h"

#define	FLOOP_INTVAL			1.0
#define	XTCAS_PLUGIN_NAME		"X-TCAS 1.0"
#define	XTCAS_PLUGIN_SIG		"skiselkov.xtcas.1.0"
#define	XTCAS_PLUGIN_DESCRIPTION \
	"Generic TCAS II v7.1 implementation for X-Plane"

static bool_t intf_inited = B_FALSE;
XPLMDataRef time_dr = NULL,
    baro_alt_dr = NULL,
    rad_alt_dr = NULL,
    lat_dr = NULL,
    lon_dr = NULL;

static XPLMDataRef
find_dr_chk(const char *name, XPLMDataTypeID type)
{
	XPLMDataRef dr = XPLMFindDataRef(name);
	assert(dr != NULL);
	assert((XPLMGetDataRefTypes(dr) & type) == type);
	return (dr);
}

static void
sim_intf_init(void)
{
	time_dr = find_dr_chk("sim/time/total_running_time_sec",
	    xplmType_Float);
	baro_alt_dr = find_dr_chk("sim/flightmodel/misc/h_ind",
	    xplmType_Float);
	rad_alt_dr = find_dr_chk("sim/cockpit2/gauges/indicators/"
	    "radio_altimeter_height_ft_pilot", xplmType_Float);
	lat_dr = find_dr_chk("sim/flightmodel/position/latitude",
	    xplmType_Double);
	lon_dr = find_dr_chk("sim/flightmodel/position/longitude",
	    xplmType_Double);
	intf_inited = B_TRUE;
}

static void
sim_intf_fini(void)
{
	time_dr = NULL;
	baro_alt_dr = NULL;
	rad_alt_dr = NULL;
	lat_dr = NULL;
	lon_dr = NULL;
	intf_inited = B_FALSE;
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

double
xtcas_get_time(void)
{
	assert(intf_inited);
	return (XPLMGetDataf(time_dr));
}

void
xtcas_get_acf_pos(const void *acf_id, geo_pos3_t *pos, double *alt_agl)
{
	assert(intf_inited);
	assert(acf_id == MY_ACF_ID);
	pos->lat = XPLMGetDatad(lat_dr);
	pos->lon = XPLMGetDatad(lon_dr);
	pos->elev = XPLMGetDataf(baro_alt_dr);
	*alt_agl = XPLMGetDataf(rad_alt_dr);
}

void
xtcas_get_acf_ids(void ***id_list, size_t *count)
{
	XPLMPluginID controller;
	int total, active;

	XPLMCountAircraft(&total, &active, &controller);

	xtcas_log("get_acf_ids: %d / %d\n", total, active);
	*id_list = NULL;
	*count = 0;
}

PLUGIN_API int
XPluginStart(char *name, char *sig, char *desc)
{
	strcpy(name, XTCAS_PLUGIN_NAME);
	strcpy(sig, XTCAS_PLUGIN_SIG);
	strcpy(desc, XTCAS_PLUGIN_DESCRIPTION);
	sim_intf_init();

	return (1);
}

PLUGIN_API void
XPluginStop(void)
{
	sim_intf_fini();
}

PLUGIN_API void
XPluginEnable(void)
{
	xtcas_init();
	XPLMRegisterFlightLoopCallback(floop_cb, FLOOP_INTVAL, NULL);
}

PLUGIN_API void
XPluginDisable(void)
{
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

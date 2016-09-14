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
#include "types.h"

#include "xplane.h"

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

void
xtcas_sim_intf_init(void)
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

void
xtcas_sim_intf_fini(void)
{
	time_dr = NULL;
	baro_alt_dr = NULL;
	rad_alt_dr = NULL;
	lat_dr = NULL;
	lon_dr = NULL;
	intf_inited = B_FALSE;
}

double
xtcas_get_time(void)
{
	assert(intf_inited);
	return (XPLMGetDataf(time_dr));
}

void
xtcas_get_aircraft_pos(void *acf_ID, geo_pos3_t *pos, double *alt_agl)
{
	assert(intf_inited);
	assert(acf_ID == MY_ACF_ID);
	pos->lat = XPLMGetDatad(lat_dr);
	pos->lon = XPLMGetDatad(lon_dr);
	pos->elev = XPLMGetDataf(baro_alt_dr);
	*alt_agl = XPLMGetDataf(rad_alt_dr);
}

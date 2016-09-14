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

#include "helpers.h"

#include "SL.h"
#include "geom.h"
#include "pos.h"
#include "xplane.h"

#include "xtcas.h"

typedef enum {
	RA_clb,
	RA_des,
	RA_cross_clb,
	RA_cross_des,
	RA_level_off,
	RA_clb_rev,
	RA_des_rev,
	RA_clb_more,
	RA_des_more,
	RA_maint_clb,
	RA_maint_des,
	RA_maint_cross_clb,
	RA_maint_cross_des,
	RA_monitor_vs,
	RA_clear
} RA_type_t;

typedef struct RA {
	RA_type_t type;
	int min_vs, max_vs;
} RA_t;


typedef struct tcas_acf_pos {
	obj_pos_t pos;
	double gs, d_gs;
	double trk, d_trk;
	double vvel;
} tcas_acf_pos_t;

tcas_acf_pos_t my_pos;

static bool_t
update_my_position(double t)
{
	geo_pos3_t new_pos;
	double new_agl;

	xtcas_get_aircraft_pos(MY_ACF_ID, &new_pos, &new_agl);
	xtcas_obj_pos_update(&my_pos.pos, t, new_pos, new_agl);
	if (!xtcas_obj_pos_get_gs(&my_pos.pos, &my_pos.gs, &my_pos.d_gs) ||
	    !xtcas_obj_pos_get_trk(&my_pos.pos, &my_pos.trk, &my_pos.d_trk) ||
	    !xtcas_obj_pos_get_vvel(&my_pos.pos, &my_pos.vvel, NULL))
		return (B_FALSE);

	return (B_TRUE);
}

static void
update_bogie_positions(double t)
{
	UNUSED(t);
}

void
xtcas_run(void)
{
	const SL_t *sl;
	double t = xtcas_get_time();

	if (!update_my_position(t))
		return;
	update_bogie_positions(t);

	sl = xtcas_SL_select(CUR_OBJ_ALT_MSL(&my_pos.pos),
	    CUR_OBJ_ALT_AGL(&my_pos.pos));
}

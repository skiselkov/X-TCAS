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

#include "assert.h"
#include "helpers.h"
#include "pos.h"

#define	STEP_BACK(step)	((step) == 0 ? (NUM_POS_STEPS - 1) : (step) - 1)

/*
 * Puts a position update into `pos'. The time of the update is `t'.
 */
void
xtcas_obj_pos_update(obj_pos_t *pos, double t, geo_pos3_t upd, double rad_alt)
{
	unsigned next_step = (pos->latest_step + 1) % NUM_POS_STEPS;

	if (pos->populated_steps > 0)
		ASSERT3F(pos->time[pos->latest_step], <, t);

	pos->time[next_step] = t;
	pos->rad_alt[next_step] = rad_alt;
	pos->pos[next_step] = upd;
	if (pos->populated_steps < NUM_POS_STEPS)
		pos->populated_steps++;
	pos->latest_step = next_step;
}

/*
 * Given an object's position, calculate its groundspeed/velocity heading and
 * first derivative.
 */
bool_t
xtcas_obj_pos_get_gs(const obj_pos_t *pos, double *gs)
{
	geo_pos3_t p1_2d, p2_2d;
	vect3_t p1, p2;
	double dt1;

	/*
	 * Strip the elevation part from the positions to transform the
	 * groundspeed readout into a true along-the-ground value, instead
	 * of incorporating the vertical velocity portion.
	 */
	p1_2d = pos->pos[pos->latest_step];
	p1_2d.elev = 0;
	p2_2d = pos->pos[STEP_BACK(pos->latest_step)];
	p2_2d.elev = 0;

	p1 = geo2ecef(p1_2d, &wgs84);
	p2 = geo2ecef(p2_2d, &wgs84);
	dt1 = pos->time[pos->latest_step] -
	    pos->time[STEP_BACK(pos->latest_step)];
	if (gs != NULL) {
		if (pos->populated_steps >= 2) {
			ASSERT3F(dt1, >, 0.0);
			*gs = vect3_abs(vect3_sub(p1, p2)) / dt1;
		} else {
			return (B_FALSE);
		}
	}

	return (B_TRUE);
}

/*
 * Given an object's position, calculate its true track heading and first
 * derivative.
 */
bool_t
xtcas_obj_pos_get_trk(const obj_pos_t *pos, double *trk)
{
	fpp_t fpp;
	geo_pos3_t p1 = pos->pos[pos->latest_step];
	geo_pos3_t p2 = pos->pos[STEP_BACK(pos->latest_step)];
	vect2_t tp1, tp2;

	fpp = ortho_fpp_init(GEO3_TO_GEO2(p2), 0, &wgs84, B_FALSE);
	tp1 = geo2fpp(GEO3_TO_GEO2(p1), &fpp);
	tp2 = geo2fpp(GEO3_TO_GEO2(p2), &fpp);

	if (trk != NULL) {
		if (pos->populated_steps >= 2)
			*trk = dir2hdg(vect2_sub(tp1, tp2));
		else
			return (B_FALSE);
	}
	return (B_TRUE);
}

/*
 * Given an object's position, calculate its vertical velocity and first
 * derivative.
 */
bool_t
xtcas_obj_pos_get_vvel(const obj_pos_t *pos, double *vvel)
{
	double e1, e2, e3;
	double dt1 = pos->time[pos->latest_step] -
	    pos->time[STEP_BACK(pos->latest_step)];

	e1 = pos->pos[pos->latest_step].elev;
	e2 = pos->pos[STEP_BACK(pos->latest_step)].elev;
	e3 = pos->pos[STEP_BACK(STEP_BACK(pos->latest_step))].elev;

	if (vvel != NULL) {
		if (pos->populated_steps >= 2) {
			ASSERT3F(dt1, >, 0.0);
			*vvel = (e1 - e2) / dt1;
		} else {
			return (B_FALSE);
		}
	}
	return (B_TRUE);
}

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

#ifndef	_XTCAS_POS_H_
#define	_XTCAS_POS_H_

#include "geom.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Position tracking funtions.
 */

#define NUM_POS_STEPS 3
typedef struct obj_pos {
	unsigned latest_step;
	unsigned populated_steps;
	double time[NUM_POS_STEPS];
	double rad_alt[NUM_POS_STEPS];
	geo_pos3_t pos[NUM_POS_STEPS];
} obj_pos_t;

void xtcas_pos_update(obj_pos_t *pos, double t, geo_pos3_t upd, double rad_alt);
void xtcas_pos_get_gs(const obj_pos_t *pos, double *gs, double *d_gs);
void xtcas_pos_get_trk(const obj_pos_t *pos, double *trk, double *d_trk);
void xtcas_pos_get_vvel(const obj_pos_t *pos, double *vvel, double *d_vvel);

#ifdef __cplusplus
}
#endif

#endif	/* _XTCAS_POS_H_ */

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

#ifndef	_XTCAS_XPLANE_H_
#define	_XTCAS_XPLANE_H_

#include "geom.h"

#ifdef __cplusplus
extern "C" {
#endif

#define	MY_ACF_ID	(NULL)

void xtcas_sim_intf_init(void);
void xtcas_sim_intf_fini(void);
double xtcas_get_time(void);
void xtcas_get_aircraft_pos(void *acf_ID, geo_pos3_t *pos, double *alt_agl);

#ifdef __cplusplus
}
#endif

#endif	/* _XTCAS_XPLANE_H_ */

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

#include "SL.h"
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

void
xtcas_run(void)
{
//	const SL_t *sl;
}

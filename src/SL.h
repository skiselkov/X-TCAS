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

#ifndef _XTCAS_SL_H_
#define _XTCAS_SL_H_

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * These maximum parameters constrain the CPA search algorithm (if a CPA
 * is over them, we don't even examine it), but they depend on the exact
 * SL values used. Basically, the maximum values are designed so as to
 * be at least 2x as much as the largest distance & tau in the highest SL,
 * but to also efficiently cull away as many CPAs as possible, so we don't
 * have to waste time resolving them.
 */
enum {
	CPA_MAX_T =	120,	/* seconds */
	CPA_MAX_DIST =	5000	/* meters */
};

#define	INHIBIT_AUDIO		FEET2MET(500)
#define	INHIBIT_DES_RA		FEET2MET(1100)
#define	INHIBIT_INC_DESC_RA	FEET2MET(1550)
#define	INHIBIT_NO_ALT_RPTG_ACF	FEET2MET(15500)
#define	INHIBIT_CLB_RA		FEET2MET(48000)

/* See SL.c for an explanation of these */
typedef struct SL {
	unsigned	SL_id;		/* numeric ID */
	bool_t		agl;
	double		alt_min;	/* meters */
	double		alt_max;	/* meters */
	double		hyst_down;	/* meters */
	double		hyst_up;	/* meters */
	double		tau_TA;		/* seconds */
	double		tau_RA;		/* seconds */
	double		dmod_TA;	/* meters */
	double		dmod_RA;	/* meters */
	double		zthr_TA;	/* meters */
	double		zthr_RA;	/* meters */
	double		alim_RA;	/* meters */
} SL_t;

const SL_t *xtcas_SL_select(unsigned prev_SL_id, double alt_msl,
    double alt_agl, unsigned force_select_SL);

#ifdef __cplusplus
}
#endif

#endif /* _XTCAS_SL_H_ */

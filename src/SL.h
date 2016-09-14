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

#ifdef __cplusplus
extern "C" {
#endif

typedef struct SL {
	int agl;
	unsigned alt_min;
	unsigned alt_max;
	unsigned tau_TA;
	unsigned tau_RA;
	double dmod_TA;
	double dmod_RA;
	unsigned zthr_TA;
	unsigned zthr_RA;
	unsigned alim_RA;
} SL_t;

const SL_t *xtcas_SL_select(unsigned alt_msl, unsigned alt_agl);

#ifdef __cplusplus
}
#endif

#endif /* _XTCAS_SL_H_ */

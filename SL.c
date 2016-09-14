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

#include "SL.h"

static const SL_t SL_table[] = {
    {   /* SL2 */
	.agl = 1, .alt_min = 0, .alt_max = 1000,
	.tau_TA = 20, .tau_RA = 0, .dmod_TA = 0.30, .dmod_RA = 0.0,
	.zthr_TA = 850, .zthr_RA = 0, .alim_RA = 0
    },
    {   /* SL3 */
	.agl = 1, .alt_min = 1000, .alt_max = 2350,
	.tau_TA = 25, .tau_RA = 15, .dmod_TA = 0.33, .dmod_RA = 0.20,
	.zthr_TA = 850, .zthr_RA = 600, .alim_RA = 300
    },
    {   /* SL4 */
	.agl = 0, .alt_min = 0, .alt_max = 5000,
	.tau_TA = 30, .tau_RA = 20, .dmod_TA = 0.48, .dmod_RA = 0.35,
	.zthr_TA = 850, .zthr_RA = 600, .alim_RA = 300
    },
    {   /* SL5 */
	.agl = 0, .alt_min = 5000, .alt_max = 10000,
	.tau_TA = 40, .tau_RA = 25, .dmod_TA = 0.75, .dmod_RA = 0.55,
	.zthr_TA = 850, .zthr_RA = 600, .alim_RA = 350
    },
    {   /* SL6 */
	.agl = 0, .alt_min = 10000, .alt_max = 20000,
	.tau_TA = 45, .tau_RA = 30, .dmod_TA = 1.00, .dmod_RA = 0.80,
	.zthr_TA = 850, .zthr_RA = 600, .alim_RA = 400
    },
    {   /* SL7 below 42000 ft */
	.agl = 0, .alt_min = 20000, .alt_max = 42000,
	.tau_TA = 48, .tau_RA = 35, .dmod_TA = 1.30, .dmod_RA = 1.10,
	.zthr_TA = 850, .zthr_RA = 700, .alim_RA = 600
    },
    {   /* SL7 above 42000 ft */
	.agl = 0, .alt_min = 42000, .alt_max = -1u,
	.tau_TA = 48, .tau_RA = 35, .dmod_TA = 1.30, .dmod_RA = 1.10,
	.zthr_TA = 1200, .zthr_RA = 800, .alim_RA = 700
    },
    {
	.alt_min = 0, .alt_max = 0
    }
};

const SL_t *
xtcas_SL_select(unsigned alt_msl, unsigned alt_agl)
{
	for (int i = 0; SL_table[i].alt_min != SL_table[i].alt_max; i++) {
		const SL_t *sl = &SL_table[i];
		if ((sl->agl && alt_agl >= sl->alt_min &&
		    alt_agl < sl->alt_max) || (!sl->agl &&
		    alt_msl >= sl->alt_min && alt_msl < sl->alt_max))
			return (sl);
	}
	return (NULL);
}

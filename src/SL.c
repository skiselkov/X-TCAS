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

#include <stdlib.h>
#include <math.h>

#include <acfutils/assert.h>
#include <acfutils/helpers.h>
#include <acfutils/perf.h>

#include "SL.h"

/*
 * The sensitivity level table provides an altitude-dependent method of
 * selecting the collision avoidance and resolution parameters. This is
 * part of the TCAS II v7.1 specification.
 *
 * The fields in this table have the following meanings:
 *	SL_id: Numeric sensitivity level identifier.
 *	agl: Defines if the SL is based on radio altitude above ground
 *		level or on barometric altitude
 *	alt_min & alt_max: Define the vertical boundaries for when this
 *		SL should be selected. If agl=true, alt_min & alt_max
 *		are height AGL, otherwise they're baro altitude.
 *	hyst_down: hysteresis value subtracted from alt_min if this SL was
 *		previously selected.
 *	hyst_up: hysteresis value added to alt_max if this SL was previously
 *		selected.
 *	tau_TA & tau_RA: The time-to-go to closest point of approach
 *		(CPA) before issuing a Traffic Advisory (TA) or
 *		Resolution Advisory (RA).
 *	dmod_TA & dmod_RA: The horizontal radius of our protected
 *		volume. If the intruder's CPA lies within this volume,
 *		or the intruder itself crosses into this volume, we
 *		either issue a TA or RA.
 *	zthr_TA & zthr_RA: The vertical size of our protected volume
 *		(centered on our aircraft). For a TA or RA to be issued,
 *		both the horizontal and vertical range boundary rules
 *		must be met.
 *	alim_RA: TCAS normally attempts to select an RA which provides
 *		the greatest vertical separation. However, it also wants
 *		to select an RA which DOESN'T cross the intruder's
 *		altitude before reaching CPA. If a non-crossing RA
 *		provides at least alim_RA vertical separation at CPA, it
 *		is selected. Otherwise the RA which provides the
 *		greatest vertical separation is selected.
 */

/*
 * When configured as a TCAS I GTS 820 system, we modify this table to
 * only contain the 'A' and 'B' sensitivity levels that correspond with
 * the GTS 820 SL values.
 */

#if	GTS820_MODE

#define	NUM_SL		3
#define	FALLBACK_SL	1

static const SL_t SL_table[NUM_SL] = {
    {	/* Stand-by mode */
	.SL_id =	1,
	.agl =		B_TRUE,
	.alt_min =	-INFINITY,
	.alt_max =	FEET2MET(50),
	.hyst_down =	0,
	.hyst_up =	FEET2MET(10),
	.tau_TA =	0,
	.tau_RA =	0,
	.dmod_TA =	0,
	.dmod_RA =	0,
	.zthr_TA =	0,
	.zthr_RA =	0,
	.alim_RA =	0
    },
    {	/* SL-A */
	.SL_id =	2,
	.agl =		B_TRUE,
	.alt_min =	FEET2MET(50),
	.alt_max =	FEET2MET(2000),
	.hyst_down =	FEET2MET(10),
	.hyst_up =	FEET2MET(100),
	.tau_TA =	20,
	.tau_RA =	0,
	.dmod_TA =	NM2MET(0.20),
	.dmod_RA =	0.0,
	.zthr_TA =	FEET2MET(600),
	.zthr_RA =	0,
	.alim_RA =	0,
	.gear_test =	GEAR_TEST_DOWN
    },
    {	/* SL-B */
	.SL_id =	3,
	.agl =		B_TRUE,
	.alt_min =	FEET2MET(2000),
	.alt_max =	INFINITY,
	.hyst_down =	FEET2MET(100),
	.hyst_up =	FEET2MET(100),
	.tau_TA =	30,
	.tau_RA =	0,
	.dmod_TA =	NM2MET(0.55),
	.dmod_RA =	0.0,
	.zthr_TA =	FEET2MET(800),
	.zthr_RA =	0,
	.alim_RA =	0,
	.gear_test =	GEAR_TEST_UP
    }
};

#else	/* !GTS820_MODE */
/*
 * TCAS II v7.1 mode.
 */
#define	NUM_SL		8
#define	FALLBACK_SL	3
static const SL_t SL_table[NUM_SL] = {
    {	/* SL1 */
	.SL_id =	1,
	.agl =		B_TRUE,
	.alt_min =	-INFINITY,
	.alt_max =	FEET2MET(50),
	.hyst_down =	0,
	.hyst_up =	FEET2MET(10),
	.tau_TA =	0,
	.tau_RA =	0,
	.dmod_TA =	0,
	.dmod_RA =	0,
	.zthr_TA =	0,
	.zthr_RA =	0,
	.alim_RA =	0
    },
    {	/* SL2 */
	.SL_id =	2,
	.agl =		B_TRUE,
	.alt_min =	FEET2MET(50),
	.alt_max =	FEET2MET(1000),
	.hyst_down =	FEET2MET(10),
	.hyst_up =	FEET2MET(100),
	.tau_TA =	20,
	.tau_RA =	0,
	.dmod_TA =	NM2MET(0.30),
	.dmod_RA =	0.0,
	.zthr_TA =	FEET2MET(850),
	.zthr_RA =	0,
	.alim_RA =	0
    },
    {	/* SL3 */
	.SL_id =	3,
	.agl =		B_TRUE,
	.alt_min =	FEET2MET(1000),
	.alt_max =	FEET2MET(2350),
	.hyst_down =	FEET2MET(100),
	.hyst_up =	FEET2MET(200),
	.tau_TA =	25,
	.tau_RA =	15,
	.dmod_TA =	NM2MET(0.33),
	.dmod_RA =	NM2MET(0.20),
	.zthr_TA =	FEET2MET(850),
	.zthr_RA =	FEET2MET(600),
	.alim_RA =	FEET2MET(300)
    },
    {	/* SL4 */
	.SL_id =	4,
	.agl =		B_FALSE,
	.alt_min =	-INFINITY,	/* fallback in case RA is invalid */
	.alt_max =	FEET2MET(5000),
	.hyst_down =	0,
	.hyst_up =	FEET2MET(500),
	.tau_TA =	30,
	.tau_RA =	20,
	.dmod_TA =	NM2MET(0.48),
	.dmod_RA =	NM2MET(0.35),
	.zthr_TA =	FEET2MET(850),
	.zthr_RA =	FEET2MET(600),
	.alim_RA =	FEET2MET(300)
    },
    {	/* SL5 */
	.SL_id =	5,
	.agl =		B_FALSE,
	.alt_min =	FEET2MET(5000),
	.alt_max =	FEET2MET(10000),
	.hyst_down =	FEET2MET(500),
	.hyst_up =	FEET2MET(500),
	.tau_TA =	40,
	.tau_RA =	25,
	.dmod_TA =	NM2MET(0.75),
	.dmod_RA =	NM2MET(0.55),
	.zthr_TA =	FEET2MET(850),
	.zthr_RA =	FEET2MET(600),
	.alim_RA =	FEET2MET(350)
    },
    {	/* SL6 */
	.SL_id =	6,
	.agl =		B_FALSE,
	.alt_min =	FEET2MET(10000),
	.alt_max =	FEET2MET(20000),
	.hyst_down =	FEET2MET(500),
	.hyst_up =	FEET2MET(500),
	.tau_TA =	45,
	.tau_RA =	30,
	.dmod_TA =	NM2MET(1.00),
	.dmod_RA =	NM2MET(0.80),
	.zthr_TA =	FEET2MET(850),
	.zthr_RA =	FEET2MET(600),
	.alim_RA =	FEET2MET(400)
    },
    {	/* SL7 below 42000 ft */
	.SL_id =	7,
	.agl =		B_FALSE,
	.alt_min =	FEET2MET(20000),
	.alt_max =	FEET2MET(42000),
	.hyst_down =	FEET2MET(500),
	.hyst_up =	FEET2MET(500),
	.tau_TA =	48,
	.tau_RA =	35,
	.dmod_TA =	NM2MET(1.30),
	.dmod_RA =	NM2MET(1.10),
	.zthr_TA =	FEET2MET(850),
	.zthr_RA =	FEET2MET(700),
	.alim_RA =	FEET2MET(600)
    },
    {	/* SL7 above 42000 ft */
	.SL_id =	8,
	.agl =		B_FALSE,
	.alt_min =	FEET2MET(42000),
	.alt_max =	INFINITY,
	.hyst_down =	FEET2MET(500),
	.hyst_up =	FEET2MET(500),
	.tau_TA =	48,
	.tau_RA =	35,
	.dmod_TA =	NM2MET(1.30),
	.dmod_RA =	NM2MET(1.10),
	.zthr_TA =	FEET2MET(1200),
	.zthr_RA =	FEET2MET(800),
	.alim_RA =	FEET2MET(700)
    }
};

#endif	/* !GTS820_MODE */

/*
 * Selects the appropriate TCAS sensitivity level based on previously selected
 * sensitivity level (prev_SL_id), altitude AMSL (alt_msl) and altitude AGL
 * (alt_agl). If force_select_SL is non-zero, this forcibly selects the given
 * SL based on SL_id (must be between 1 and 8 inclusive).
 */
const SL_t *
xtcas_SL_select(unsigned prev_SL_id, double alt_msl, double alt_agl,
    unsigned force_select_SL, bool_t gear_ext)
{
	if (force_select_SL != 0) {
		VERIFY3U(force_select_SL, >=, 1);
		VERIFY3U(force_select_SL, <=, 8);
		return (&SL_table[force_select_SL - 1]);
	}
	/*
	 * In case we have no altitude data, fall back to the default SL.
	 */
	if (!is_valid_alt_m(alt_msl))
		return (&SL_table[FALLBACK_SL]);
	for (int i = 0; i < NUM_SL; i++) {
		const SL_t *sl = &SL_table[i];
		double min, max;

		if (prev_SL_id == sl->SL_id) {
			min = sl->alt_min - sl->hyst_down;
			max = sl->alt_max + sl->hyst_up;
		} else {
			min = sl->alt_min;
			max = sl->alt_max;
		}
		if ((sl->agl && !isnan(alt_agl) && alt_agl >= min &&
		    alt_agl < max) ||
		    (!sl->agl && alt_msl >= min && alt_msl < max) ||
		    (sl->gear_test == GEAR_TEST_DOWN && gear_ext) ||
		    (sl->gear_test == GEAR_TEST_UP && !gear_ext))
			return (sl);
	}
	VERIFY_FAIL();
}

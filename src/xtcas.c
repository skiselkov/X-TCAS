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
#include <string.h>
#include <stddef.h>

#include <acfutils/assert.h>
#include <acfutils/avl.h>
#include <acfutils/helpers.h>
#include <acfutils/geom.h>
#include <acfutils/list.h>
#include <acfutils/log.h>
#include <acfutils/math.h>
#include <acfutils/perf.h>
#include <acfutils/safe_alloc.h>
#include <acfutils/thread.h>
#include <acfutils/time.h>

#include "dbg_log.h"
#include "pos.h"
#ifndef	XTCAS_NO_AUDIO
#include "snd_sys.h"
#endif
#include "SL.h"
#include "xtcas.h"

#define	ALT_ROUND_MUL		FPM2MPS(100)	/* altitude rouding multiple */
#define	EQ_ALT_THRESH		FEET2MET(100)	/* equal altitude threshold */
#define	D_VVEL_MAN_THRESH	FPM2MPS(200)	/* m/s^2 */
#define	D_VVEL_MAN_TIME		FPM2MPS(6)	/* seconds */
#define	NUM_RA_INFOS		26

#define	WORKER_LOOP_INTVAL	1		/* seconds */
#define	WORKER_LOOP_INTVAL_US	((uint64_t)SEC2USEC(WORKER_LOOP_INTVAL))
#define	STATE_CHG_DELAY		SEC2USEC(4)	/* microseconds */
#define	EARTH_G			9.81		/* m.s^-2 */
#define	INITIAL_RA_D_VVEL	(EARTH_G / 4)	/* 1/4 g */
#define	INITIAL_RA_DELAY	5.0		/* seconds */
#define	SUBSEQ_RA_D_VVEL	(EARTH_G / 3)	/* 1/3 g */
#define	SUBSEQ_RA_DELAY		2.5		/* seconds */
#define	CROSSING_RA_PENALTY	0.125		/* multiplier */
#define	REVERSAL_RA_PENALTY	0.125		/* multiplier */

#define	FALSE_CTC_SUPPRESS_GS	2		/* m/s */

#define	LONG_VERT_FILTER	FEET2MET(9900)	/* Used for the ABV and BLW */
#define	NORM_VERT_FILTER	FEET2MET(2700)	/* vertical filter modes */

#define	OTH_TFC_DIST_THRESH		NM2MET(40)
#define	PROX_DIST_THRESH		NM2MET(6)
#define	PROX_ALT_THRESH			FEET2MET(1200)
#define	ON_GROUND_AGL_THRESH		FEET2MET(380)
#define	ON_GROUND_AGL_CHK_THRESH	FEET2MET(1700)
#define	INF_VS				FPM2MPS(100000)
#define	APCH_SPD_THRESH			KT2MPS(40)
#define	CLEARING_CLIMB_RATE		FPM2MPS(1000)

#if	GTS820_MODE
#define	TA_THREAT_CANCEL_DELAY		SEC2USEC(8)
#else	/* !GTS820_MODE */
#define	TA_THREAT_CANCEL_DELAY		SEC2USEC(3)
#endif	/* !GTS820_MODE */

#if	GTS820_MODE
#define	NUM_TEST_CTC			3
#else
#define	NUM_TEST_CTC			4
#endif

/*
 * When checking if an RA hint is still active, we boost the TA horizontal
 * and vertical volume by these factors. This helps us to avoid issuing a
 * CLEAR OF CONFLICT too early, when returning to the original path could
 * trigger a second RA.
 */
#define	HINT_H_INCR_FACT		1.1
#define	HINT_V_INCR_FACT		1.7

#define	TCAS_TEST_DUR			8	/* seconds */

#define	PRINTF_ACF_FMT "rdy:%d  alt_rptg:%d  pos:%3.04f/%2.4f/%4.1f  " \
	"agl:%.0f  gs:%.1f  trk:%.0f  trk_v:%.2fx%.2f  vvel:%.1f  ongnd:%d"
#define	PRINTF_ACF_ARGS(acf) \
	(acf)->trend_data_ready, (acf)->alt_rptg, (acf)->cur_pos.lat, \
	(acf)->cur_pos.lon, (acf)->cur_pos.elev, (acf)->agl, (acf)->gs, \
	(acf)->trk, (acf)->trk_v.x, (acf)->trk_v.y, (acf)->vvel, \
	(acf)->on_ground

#define	PRINTF_RI_FMT "%s/%s/%s/%s|%.1f=%.1f->%.1f=%.1f"
#define	PRINTF_RI_ARGS(ri) RA_msg2str((ri)->msg), RA_type2str((ri)->type), \
	RA_sense2str((ri)->sense), RA_cross2str((ri)->cross), \
	MPS2FPM((ri)->vs.in.min), MPS2FPM((ri)->vs.in.max), \
	MPS2FPM((ri)->vs.out.min), MPS2FPM((ri)->vs.out.max)

#define	PRINTF_RA_FMT PRINTF_RI_FMT \
	"\\/rev:%d cross:%d zthr:%d alim:%d min_sep:%.1f corr:%.1f"
#define	PRINTF_RA_ARGS(ra) PRINTF_RI_ARGS((ra)->info), (ra)->reversal, \
	(ra)->crossing, (ra)->zthr_achieved, (ra)->alim_achieved, \
	(ra)->min_sep, (ra)->vs_corr_reqd

typedef struct cpa cpa_t;

typedef struct tcas_acf {
	void	*acf_id;	/* identifier - used for locating in tree */
	obj_pos_t pos_upd;	/* position updates */
	geo_pos3_t cur_pos;	/* current position */
	vect3_t	cur_pos_3d;	/* current position relative to our aircraft */
	bool_t	alt_rptg;	/* target altitude is available */
	double	agl;		/* altitude above ground level in meters */
	double	gs;		/* true groundspeed (horizontal) in m/s */
	double	hdg;		/* true heading in degrees */
	double	trk;		/* true track in degrees */
	vect2_t	trk_v;		/* true track as speed vector at gs */
	double	vvel;		/* computed vertical velocity in m/s */
	double	d_vvel;		/* change in vvel in m/s^2 */
	bool_t	trend_data_ready; /* indicates if gs/trk/vvel is available */
	bool_t	up_to_date;	/* used for efficient position updates */
	bool_t	on_ground;	/* on-ground condition */
	bool_t	gear_ext;	/* gear is extended */
	bool_t	custom_gear_ext;/* host provides gear extended readings */
	bool_t	has_RA;		/* has radio altimeter? */
	bool_t	custom_RA;	/* host provides custom RA readings */
	bool_t	has_WOW;	/* has weight-on-wheels switch? */
	bool_t	custom_WOW;	/* host provides custom WOW readings */
	cpa_t	*cpa;		/* CPA this aircraft participates in */
	bool_t	slow_closure;	/* for RA threats that are closing in slow */
	tcas_threat_t	threat;	/* type of TCAS threat */
	uint64_t ta_time;	/* time when we became a TA threat */

	avl_node_t	node;		/* used by other_acf_glob tree */
	list_node_t	new_TA_node;	/* used by new_TA_threat list */
} tcas_acf_t;

typedef enum {
	RA_CROSS_REQ,
	RA_CROSS_REJ,
	RA_CROSS_ANY
} tcas_RA_cross_t;

typedef struct {
	double min;
	double max;
} vs_band_t;

typedef struct {
	tcas_msg_t	msg, rev_msg;
	bool_t		initial;
	bool_t		subseq;
	bool_t		chk_black;
	tcas_RA_sense_t	sense;
	tcas_RA_type_t	type;
	tcas_RA_cross_t	cross;
	struct {
		vs_band_t	in;
		vs_band_t	out;
		vs_band_t	red_lo;
		vs_band_t	red_hi;
	} vs;
} tcas_RA_info_t;

typedef struct {
	const tcas_RA_info_t	*info;
	avl_tree_t		*cpas;
	const SL_t		*sl;
	bool_t			reversal;	/* sense reversal */
	bool_t			crossing;	/* crossing intruder's alt */
	bool_t			zthr_achieved; /* min_sep at least ALIM_TA */
	bool_t			alim_achieved; /* min_sep at least ALIM_RA */
	double			min_sep;	/* Minimum among seps */
	double			vs_corr_reqd;	/* required VS correction */
	avl_node_t		node;
} tcas_RA_t;

typedef struct {
	void		*acf_id;
	tcas_threat_t	level;
	bool_t		slow_closure;
	avl_node_t	node;
} tcas_RA_hint_t;

typedef struct {
	tcas_adv_t	adv_state;
	tcas_RA_t	*ra;
	double		initial_ra_vs;	/* VS when first RA was issued */
	uint64_t	change_t;	/* microclock() timestamp */
	tcas_mode_t	mode;
	tcas_filter_t	filter;

	mutex_t		test_lock;
	bool_t		test_in_prog;
	double		test_start_time;
} tcas_state_t;

struct cpa {
	double		d_t;		/* seconds from now */

	vect3_t		pos_a;		/* meters */
	vect3_t		pos_b;		/* meters */
	double		d_h;		/* meters */
	double		d_v;		/* meters */

	tcas_acf_t	*acf_a;
	tcas_acf_t	*acf_b;

	avl_node_t	node;
	avl_node_t	ra_node;
};

static const tcas_RA_info_t RA_info[NUM_RA_INFOS] = {

/* Preventive climbing RAs */
    {	/* MONITOR VERTICAL SPEED */
	.msg = RA_MSG_MONITOR_VS, .rev_msg = RA_MSG_MONITOR_VS,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_UPWARD,
	.type = RA_TYPE_PREVENTIVE, .cross = RA_CROSS_ANY,
	.chk_black = B_FALSE, .vs = {
	    .in = {0, INF_VS}, .out = {0, INF_VS},
	    .red_lo = {-INF_VS, 0}, .red_hi = {0, 0}
	}
    },
    {	/* MONITOR VERTICAL SPEED */
	.msg = RA_MSG_MONITOR_VS, .rev_msg = RA_MSG_MONITOR_VS,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_UPWARD,
	.type = RA_TYPE_PREVENTIVE, .cross = RA_CROSS_ANY,
	.chk_black = B_FALSE, .vs = {
	    .in = {FPM2MPS(-500), INF_VS}, .out = {FPM2MPS(-500), INF_VS},
	    .red_lo = {-INF_VS, FPM2MPS(-500)},
	    .red_hi = {FPM2MPS(-500), FPM2MPS(-500)}
	}
    },
    {	/* MONITOR VERTICAL SPEED */
	.msg = RA_MSG_MONITOR_VS, .rev_msg = RA_MSG_MONITOR_VS,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_UPWARD,
	.type = RA_TYPE_PREVENTIVE, .cross = RA_CROSS_ANY,
	.chk_black = B_FALSE, .vs = {
	    .in = {FPM2MPS(-1000), INF_VS}, .out = {FPM2MPS(-1000), INF_VS},
	    .red_lo = {-INF_VS, FPM2MPS(-1000)},
	    .red_hi = {FPM2MPS(-1000), FPM2MPS(-1000)}
	}
    },
    {	/* MONITOR VERTICAL SPEED */
	.msg = RA_MSG_MONITOR_VS, .rev_msg = RA_MSG_MONITOR_VS,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_UPWARD,
	.type = RA_TYPE_PREVENTIVE, .cross = RA_CROSS_ANY,
	.chk_black = B_FALSE, .vs = {
	    .in = {FPM2MPS(-2000), INF_VS}, .out = {FPM2MPS(-2000), INF_VS},
	    .red_lo = {-INF_VS, FPM2MPS(-2000)},
	    .red_hi = {FPM2MPS(-2000), FPM2MPS(-2000)}
	}
    },

/* Preventive descending RAs */
    {	/* MONITOR VERTICAL SPEED */
	.msg = RA_MSG_MONITOR_VS, .rev_msg = RA_MSG_MONITOR_VS,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_DOWNWARD,
	.type = RA_TYPE_PREVENTIVE, .cross = RA_CROSS_ANY,
	.chk_black = B_FALSE, .vs = {
	    .in = {-INF_VS, 0}, .out = { -INF_VS, 0},
	    .red_lo = {0, 0}, .red_hi = {0, INF_VS}
	}
    },
    {	/* MONITOR VERTICAL SPEED */
	.msg = RA_MSG_MONITOR_VS, .rev_msg = RA_MSG_MONITOR_VS,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_DOWNWARD,
	.type = RA_TYPE_PREVENTIVE, .cross = RA_CROSS_ANY,
	.chk_black = B_FALSE, .vs = {
	    .in = {-INF_VS, FPM2MPS(500)}, .out = {-INF_VS, FPM2MPS(500)},
	    .red_lo = {FPM2MPS(500), FPM2MPS(500)},
	    .red_hi = {FPM2MPS(500), INF_VS}
	}
    },
    {	/* MONITOR VERTICAL SPEED */
	.msg = RA_MSG_MONITOR_VS, .rev_msg = RA_MSG_MONITOR_VS,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_DOWNWARD,
	.type = RA_TYPE_PREVENTIVE, .cross = RA_CROSS_ANY,
	.chk_black = B_FALSE, .vs = {
	    .in = {-INF_VS, FPM2MPS(1000)}, .out = {-INF_VS, FPM2MPS(1000)},
	    .red_lo = {FPM2MPS(1000), FPM2MPS(1000)},
	    .red_hi = {FPM2MPS(1000), INF_VS}
	}
    },
    {	/* MONITOR VERTICAL SPEED */
	.msg = RA_MSG_MONITOR_VS, .rev_msg = RA_MSG_MONITOR_VS,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_DOWNWARD,
	.type = RA_TYPE_PREVENTIVE, .cross = RA_CROSS_ANY,
	.chk_black = B_FALSE, .vs = {
	    .in = {-INF_VS, FPM2MPS(2000)}, .out = {-INF_VS, FPM2MPS(2000)},
	    .red_lo = {FPM2MPS(2000), FPM2MPS(2000)},
	    .red_hi = {FPM2MPS(2000), INF_VS}
	}
    },

/* Preventive level RA */
    {	/* MONITOR VERTICAL SPEED */
	.msg = RA_MSG_MONITOR_VS, .rev_msg = RA_MSG_MONITOR_VS,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_LEVEL_OFF,
	.type = RA_TYPE_PREVENTIVE, .cross = RA_CROSS_ANY,
	.chk_black = B_FALSE, .vs = {
	    .in = {-FPM2MPS(-100), FPM2MPS(100)},
	    .out = {FPM2MPS(-100), FPM2MPS(100)},
	    .red_lo = {-INF_VS, FPM2MPS(-100)},
	    .red_hi = {FPM2MPS(100), INF_VS}
	}
    },

/* Corrective level-off RAs */
    {	/* LEVEL OFF */
	.msg = RA_MSG_LEVEL_OFF, .rev_msg = RA_MSG_LEVEL_OFF,
	.initial = B_TRUE, .subseq = B_TRUE, .sense = RA_SENSE_LEVEL_OFF,
	.type = RA_TYPE_CORRECTIVE, .cross = RA_CROSS_REJ,
	.chk_black = B_TRUE, .vs = {
	    .in = {-INF_VS, INF_VS}, .out = {FPM2MPS(-250), FPM2MPS(250)},
	    .red_lo = {-INF_VS, -FPM2MPS(-250)},
	    .red_hi = {FPM2MPS(250), INF_VS}
	}
    },
    {	/* LEVEL OFF */
	.msg = RA_MSG_LEVEL_OFF, .rev_msg = RA_MSG_LEVEL_OFF,
	.initial = B_TRUE, .subseq = B_TRUE, .sense = RA_SENSE_LEVEL_OFF,
	.type = RA_TYPE_CORRECTIVE, .cross = RA_CROSS_REJ,
	.chk_black = B_TRUE, .vs = {
	    .in = {-INF_VS, INF_VS}, .out = {FPM2MPS(-300), 0},
	    .red_lo = {FPM2MPS(-300), FPM2MPS(-300)}, .red_hi = {0, INF_VS}
	}
    },
    {	/* LEVEL OFF */
	.msg = RA_MSG_LEVEL_OFF, .rev_msg = RA_MSG_LEVEL_OFF,
	.initial = B_TRUE, .subseq = B_TRUE, .sense = RA_SENSE_LEVEL_OFF,
	.type = RA_TYPE_CORRECTIVE, .cross = RA_CROSS_REJ,
	.chk_black = B_TRUE, .vs = {
	    .in = {-INF_VS, INF_VS}, .out = {0, FPM2MPS(300)},
	    .red_lo = {-INF_VS, 0}, .red_hi = {FPM2MPS(300), FPM2MPS(300)}
	}
    },

/* Corrective climbing RAs */
    {	/* MAINTAIN VERTICAL SPEED, CROSSING MAINTAIN - large VS range */
	.msg = RA_MSG_MAINT_VS_CROSS, .rev_msg = -1,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_UPWARD,
	.type = RA_TYPE_CORRECTIVE, .cross = RA_CROSS_REQ,
	.chk_black = B_FALSE, .vs = {
	    .in = {FPM2MPS(1500), FPM2MPS(4400)},
	    .out = {FPM2MPS(1500), FPM2MPS(4400)},
	    .red_lo = {-INF_VS, FPM2MPS(1500)},
	    .red_hi = {FPM2MPS(4400), INF_VS}
	}
    },
    {	/* MAINTAIN VERTICAL SPEED, MAINTAIN - large VS range */
	.msg = RA_MSG_MAINT_VS, .rev_msg = -1,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_UPWARD,
	.type = RA_TYPE_CORRECTIVE, .cross = RA_CROSS_REJ,
	.chk_black = B_FALSE, .vs ={
	    .in = {FPM2MPS(1500), FPM2MPS(4400)},
	    .out = {FPM2MPS(1500), FPM2MPS(4400)},
	    .red_lo = {-INF_VS, FPM2MPS(1500)},
	    .red_hi = {FPM2MPS(4400), INF_VS}
	}
    },
    {	/* MAINTAIN VERTICAL SPEED,CROSSING MAINTAIN */
	.msg = RA_MSG_MAINT_VS_CROSS, .rev_msg = -1,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_UPWARD,
	.type = RA_TYPE_CORRECTIVE, .cross = RA_CROSS_REQ,
	.chk_black = B_FALSE, .vs = {
	    .in = {FPM2MPS(1500), FPM2MPS(2000)},
	    .out = {FPM2MPS(1500), FPM2MPS(2000)},
	    .red_lo = {-INF_VS, FPM2MPS(1500)},
	    .red_hi = {FPM2MPS(2000), INF_VS}
	}
    },
    {	/* MAINTAIN VERTICAL SPEED, MAINTAIN */
	.msg = RA_MSG_MAINT_VS, .rev_msg = -1,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_UPWARD,
	.type = RA_TYPE_CORRECTIVE, .cross = RA_CROSS_REJ,
	.chk_black = B_FALSE, .vs = {
	    .in = {FPM2MPS(1500), FPM2MPS(2000)},
	    .out = {FPM2MPS(1500), FPM2MPS(2000)},
	    .red_lo = {-INF_VS, FPM2MPS(1500)},
	    .red_hi = {FPM2MPS(2000), INF_VS}
	}
    },
    {	/* CLIMB, CLIMB */
	.msg = RA_MSG_CLB, .rev_msg = RA_MSG_CLB_NOW,
	.initial = B_TRUE, .subseq = B_TRUE, .sense = RA_SENSE_UPWARD,
	.type = RA_TYPE_CORRECTIVE, .cross = RA_CROSS_REJ,
	.chk_black = B_FALSE, .vs = {
	    .in = {-INF_VS, INF_VS}, .out = {FPM2MPS(1500), FPM2MPS(2000)},
	    .red_lo = {-INF_VS, FPM2MPS(1500)},
	    .red_hi = {FPM2MPS(2000), FPM2MPS(2000)}
	}
    },
    {	/* CLIMB, CROSSING CLIMB */
	.msg = RA_MSG_CLB_CROSS, .rev_msg = RA_MSG_CLB_NOW,
	.initial = B_TRUE, .subseq = B_TRUE, .sense = RA_SENSE_UPWARD,
	.type = RA_TYPE_CORRECTIVE, .cross = RA_CROSS_REQ,
	.chk_black = B_FALSE, .vs = {
	    .in = {-INF_VS, INF_VS}, .out = {FPM2MPS(1500), FPM2MPS(2000)},
	    .red_lo = {-INF_VS, FPM2MPS(1500)},
	    .red_hi = {FPM2MPS(2000), FPM2MPS(2000)}
	}
    },
    {	/* INCREASE CLIMB */
	.msg = RA_MSG_CLB_MORE, .rev_msg = -1,
	.initial = B_FALSE, .subseq = B_TRUE, .sense = RA_SENSE_UPWARD,
	.type = RA_TYPE_CORRECTIVE, .cross = RA_CROSS_ANY,
	.chk_black = B_FALSE, .vs = {
	    .in = {FPM2MPS(500), INF_VS}, .out = {FPM2MPS(2500), FPM2MPS(4400)},
	    .red_lo = {-INF_VS, FPM2MPS(2500)},
	    .red_hi = {FPM2MPS(4400), FPM2MPS(4400)}
	}
    },


/* Corrective descending RAs */
    {	/* MAINTAIN VERTICAL SPEED, CROSSING MAINTAIN - large VS range */
	.msg = RA_MSG_MAINT_VS_CROSS, .rev_msg = -1,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_DOWNWARD,
	.type = RA_TYPE_PREVENTIVE, .cross = RA_CROSS_REQ,
	.chk_black = B_FALSE, .vs = {
	    .in = {FPM2MPS(-4400), FPM2MPS(-1500)},
	    .out = {FPM2MPS(-4400), FPM2MPS(-1500)},
	    .red_lo = {FPM2MPS(-4400), FPM2MPS(-4400)},
	    .red_hi = {FPM2MPS(-1500), INF_VS}
	}
    },
    {	/* MAINTAIN VERTICAL SPEED, MAINTAIN - large VS range */
	.msg = RA_MSG_MAINT_VS, .rev_msg = -1,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_DOWNWARD,
	.type = RA_TYPE_PREVENTIVE, .cross = RA_CROSS_REJ,
	.chk_black = B_FALSE, .vs = {
	    .in = {FPM2MPS(-4400), FPM2MPS(-1500)},
	    .out = {FPM2MPS(-4400), FPM2MPS(-1500)},
	    .red_lo = {FPM2MPS(-4400), FPM2MPS(-4400)},
	    .red_hi = {FPM2MPS(-1500), INF_VS}
	}
    },
    {	/* MAINTAIN VERTICAL SPEED, CROSSING MAINTAIN - high VS range */
	.msg = RA_MSG_MAINT_VS_CROSS, .rev_msg = -1,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_DOWNWARD,
	.type = RA_TYPE_PREVENTIVE, .cross = RA_CROSS_REQ,
	.chk_black = B_FALSE, .vs = {
	    .in = {FPM2MPS(-2000), FPM2MPS(-1500)},
	    .out = {FPM2MPS(-2000), FPM2MPS(-1500)},
	    .red_lo = {-INF_VS, FPM2MPS(-2000)},
	    .red_hi = {FPM2MPS(-1500), INF_VS}
	}
    },
    {	/* MAINTAIN VERTICAL SPEED, MAINTAIN */
	.msg = RA_MSG_MAINT_VS, .rev_msg = -1,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_DOWNWARD,
	.type = RA_TYPE_PREVENTIVE, .cross = RA_CROSS_REJ,
	.chk_black = B_FALSE, .vs = {
	    .in = {FPM2MPS(-2000), FPM2MPS(-1500)},
	    .out = {FPM2MPS(-2000), FPM2MPS(-1500)},
	    .red_lo = {-INF_VS, FPM2MPS(-2000)},
	    .red_hi = {FPM2MPS(-1500), INF_VS}
	}
    },
    {	/* DESCEND, DESCEND */
	.msg = RA_MSG_DES, .rev_msg = RA_MSG_DES_NOW,
	.initial = B_TRUE, .subseq = B_TRUE, .sense = RA_SENSE_DOWNWARD,
	.type = RA_TYPE_CORRECTIVE, .cross = RA_CROSS_REJ,
	.chk_black = B_FALSE, .vs = {
	    .in = {-INF_VS, INF_VS}, .out = {FPM2MPS(-2000), FPM2MPS(-1500)},
	    .red_lo = {FPM2MPS(-2000), FPM2MPS(-2000)},
	    .red_hi = {FPM2MPS(-1500), INF_VS}
	}
    },
    {	/* DESCEND, CROSSING DESCEND */
	.msg = RA_MSG_DES_CROSS, .rev_msg = RA_MSG_DES_NOW,
	.initial = B_TRUE, .subseq = B_TRUE, .sense = RA_SENSE_DOWNWARD,
	.type = RA_TYPE_CORRECTIVE, .cross = RA_CROSS_REQ,
	.chk_black = B_FALSE, .vs = {
	    .in = {-INF_VS, INF_VS}, .out = {FPM2MPS(-2000), FPM2MPS(-1500)},
	    .red_lo = {FPM2MPS(-2000), FPM2MPS(-2000)},
	    .red_hi = {FPM2MPS(-1500), INF_VS}
	}
    },
    {	/* INCREASE DESCENT */
	.msg = RA_MSG_DES_MORE, .rev_msg = -1,
	.initial = B_FALSE, .subseq = B_TRUE, .sense = RA_SENSE_DOWNWARD,
	.type = RA_TYPE_CORRECTIVE, .cross = RA_CROSS_ANY,
	.chk_black = B_FALSE, .vs = {
	    .in = {-INF_VS, FPM2MPS(-500)},
	    .out = {FPM2MPS(-4400), FPM2MPS(-2500)},
	    .red_lo = {-INF_VS, FPM2MPS(-4400)},
	    .red_hi = {FPM2MPS(-2500), INF_VS}
	}
    }
};

static mutex_t acf_lock;
static tcas_acf_t my_acf_glob = {
    .has_RA = B_TRUE,
    .has_WOW = B_TRUE
};
static avl_tree_t other_acf_glob;
static double last_collect_t = 0;
static tcas_state_t tcas_state;
static bool_t inited = B_FALSE;
static int xtcas_SL = 0;

static condvar_t worker_cv;
static thread_t worker_thr;
static mutex_t worker_lock;
static bool_t worker_shutdown = B_FALSE;

static const sim_intf_input_ops_t *in_ops = NULL;
static const sim_intf_output_ops_t *out_ops = NULL;

static const char *
RA_msg2str(tcas_msg_t msg)
{
	switch (msg) {
	case RA_MSG_CLB:
		return "CLB";
	case RA_MSG_CLB_CROSS:
		return "CLBCROSS";
	case RA_MSG_CLB_MORE:
		return "INCCLB";
	case RA_MSG_CLB_NOW:
		return "CLBNOW";
	case RA_MSG_CLEAR:
		return "CLR";
	case RA_MSG_DES:
		return "DES";
	case RA_MSG_DES_CROSS:
		return "DESCROSS";
	case RA_MSG_DES_MORE:
		return "INCDES";
	case RA_MSG_DES_NOW:
		return "DESNOW";
	case RA_MSG_MONITOR_VS:
		return "MONVS";
	case RA_MSG_MAINT_VS:
		return "MAINTVS";
	case RA_MSG_MAINT_VS_CROSS:
		return "MAINTVSCROSS";
	case RA_MSG_LEVEL_OFF:
		return "LEVELOFF";
	case RA_MSG_TFC:
		return "TFC";
	default:
		return "<invalid>";
	}
}

const char *
xtcas_RA_msg2text(tcas_msg_t msg)
{
	switch (msg) {
	case RA_MSG_CLB:
		return "CLIMB CLIMB";
	case RA_MSG_CLB_CROSS:
		return "CLIMB CROSSING CLIMB";
	case RA_MSG_CLB_MORE:
		return "INCREASE CLIMB";
	case RA_MSG_CLB_NOW:
		return "CLIMB, CLIMB NOW";
	case RA_MSG_CLEAR:
		return "CLEAR OF CONFLICT";
	case RA_MSG_DES:
		return "DESCEND DESCEND";
	case RA_MSG_DES_CROSS:
		return "DESCEND CROSSING DESCEND";
	case RA_MSG_DES_MORE:
		return "INCREASE DESCENT";
	case RA_MSG_DES_NOW:
		return "DESCEND, DESCEND NOW";
	case RA_MSG_MONITOR_VS:
		return "MONITOR VERTICAL SPEED";
	case RA_MSG_MAINT_VS:
		return "MAINTAIN VERTICAL SPEED, MAINTAIN";
	case RA_MSG_MAINT_VS_CROSS:
		return "MAINTAIN VERTICAL SPEED, CROSSING MAINTAIN";
	case RA_MSG_LEVEL_OFF:
		return "LEVEL OFF";
	case RA_MSG_TFC:
		return "TRAFFIC TRAFFIC";
	default:
		return "";
	}
}

static const char *
RA_type2str(tcas_RA_type_t type)
{
	if (type == RA_TYPE_CORRECTIVE)
		return "CORR";
	return "PREV";
}

static const char *
RA_sense2str(tcas_RA_sense_t sense)
{
	switch (sense) {
	case RA_SENSE_UPWARD:
		return "UP";
	case RA_SENSE_LEVEL_OFF:
		return "LVL";
	default:
		return "DN";
	}
}

static const char *
RA_cross2str(tcas_RA_cross_t cross)
{
	switch (cross) {
	case RA_CROSS_REQ:
		return "REQ";
	case RA_CROSS_REJ:
		return "REJ";
	default:
		return "ANY";
	}
}

static int
acf_compar(const void *a, const void *b)
{
	const tcas_acf_t *acf_a = a, *acf_b = b;

	if (acf_a->acf_id < acf_b->acf_id)
		return (-1);
	else if (acf_a->acf_id == acf_b->acf_id)
		return (0);
	else
		return (1);
}

static int
RA_hint_compar(const void *ha, const void *hb)
{
	const tcas_RA_hint_t *a = ha, *b = hb;

	if (a->acf_id < b->acf_id)
		return (-1);
	if (a->acf_id > b->acf_id)
		return (1);
	return (0);
}

static void
update_my_position(double t)
{
	double agl;
	bool_t on_ground, gear_ext;

	in_ops->get_my_acf_pos(in_ops->handle, &my_acf_glob.cur_pos,
	    &agl, &my_acf_glob.hdg, &gear_ext, &on_ground);
	if (!my_acf_glob.custom_RA)
		my_acf_glob.agl = agl;
	if (!my_acf_glob.custom_WOW)
		my_acf_glob.on_ground = on_ground;
	if (!my_acf_glob.custom_gear_ext)
		my_acf_glob.gear_ext = gear_ext;
	my_acf_glob.cur_pos_3d = VECT3(0, 0, my_acf_glob.cur_pos.elev);
	xtcas_obj_pos_update(&my_acf_glob.pos_upd, t, my_acf_glob.cur_pos,
	    my_acf_glob.agl);
	my_acf_glob.trend_data_ready = (
	    xtcas_obj_pos_get_gs(&my_acf_glob.pos_upd, &my_acf_glob.gs) &&
	    xtcas_obj_pos_get_trk(&my_acf_glob.pos_upd, &my_acf_glob.trk) &&
	    xtcas_obj_pos_get_vvel(&my_acf_glob.pos_upd, &my_acf_glob.vvel,
	    &my_acf_glob.d_vvel));
	if (my_acf_glob.trend_data_ready) {
		my_acf_glob.trk_v = vect2_set_abs(hdg2dir(my_acf_glob.trk),
		    my_acf_glob.gs);
	}

	/*
	 * If we don't have an RA, invalidate our height. This makes
	 * any check involving our height fail.
	 */
	if (!my_acf_glob.has_RA)
		my_acf_glob.agl = NAN;
	if (!my_acf_glob.has_WOW)
		my_acf_glob.on_ground = B_FALSE;

	dbg_log(contact, 1, "my_pos: " PRINTF_ACF_FMT,
	    PRINTF_ACF_ARGS(&my_acf_glob));
}

/*
 * Updates the position of bogies (other aircraft). This calls into the
 * sim interface to grab new aircraft position data. It then computes the
 * deltas
 */
static void
update_bogie_positions(double t, geo_pos3_t my_pos, double my_alt_agl)
{
	acf_pos_t *pos;
	size_t count;
	fpp_t fpp = ortho_fpp_init(GEO3_TO_GEO2(my_pos), 0, &wgs84, B_FALSE);
	double gnd_level = (my_alt_agl <= ON_GROUND_AGL_CHK_THRESH) ?
	    (my_pos.elev - my_alt_agl) : MIN_ELEV;
	tcas_filter_t filter = tcas_state.filter;
	tcas_mode_t mode = tcas_state.mode;

	in_ops->get_oth_acf_pos(in_ops->handle, &pos, &count);
	dbg_log(contact, 3, "received %d contacts from sim", (int)count);

	/* walk the tree and mark all acf as out-of-date */
	for (tcas_acf_t *acf = avl_first(&other_acf_glob); acf != NULL;
	    acf = AVL_NEXT(&other_acf_glob, acf))
		acf->up_to_date = B_FALSE;

	for (size_t i = 0; i < count; i++) {
		avl_index_t where;
		tcas_acf_t srch = { .acf_id = pos[i].acf_id };
		tcas_acf_t *acf = avl_find(&other_acf_glob, &srch, &where);
		vect2_t proj;

		/*
		 * Apply our vertical detection filter (ALL/ABV/BLW). The
		 * THRT display filter is applied in the threat level
		 * assignment function.
		 */
		switch (filter) {
		case TCAS_FILTER_ABV:
			if (pos[i].pos.elev > my_pos.elev + LONG_VERT_FILTER ||
			    pos[i].pos.elev < my_pos.elev - NORM_VERT_FILTER)
				continue;
			break;
		case TCAS_FILTER_BLW:
			if (pos[i].pos.elev > my_pos.elev + NORM_VERT_FILTER ||
			    pos[i].pos.elev < my_pos.elev - LONG_VERT_FILTER)
				continue;
			break;
		case TCAS_FILTER_EXP:
			if (pos[i].pos.elev > my_pos.elev + LONG_VERT_FILTER ||
			    pos[i].pos.elev < my_pos.elev - LONG_VERT_FILTER)
				continue;
			break;
		default:
			if (pos[i].pos.elev > my_pos.elev + NORM_VERT_FILTER ||
			    pos[i].pos.elev < my_pos.elev - NORM_VERT_FILTER)
				continue;
			break;
		}
		if (mode == TCAS_MODE_STBY || (isnan(pos[i].pos.elev) &&
		    my_pos.elev > INHIBIT_NO_ALT_RPTG_ACF))
			continue;

		if (acf == NULL) {
			acf = safe_calloc(1, sizeof (*acf));
			acf->acf_id = pos[i].acf_id;
			acf->agl = NAN;
			avl_insert(&other_acf_glob, acf, where);
		}
		acf->alt_rptg = !isnan(pos[i].pos.elev);
		acf->cur_pos = pos[i].pos;
		xtcas_obj_pos_update(&acf->pos_upd, t, acf->cur_pos, -1);
		proj = geo2fpp(GEO3_TO_GEO2(acf->cur_pos), &fpp);
		acf->cur_pos_3d = VECT3(proj.x, proj.y, acf->cur_pos.elev);
		if (vect2_abs(VECT3_TO_VECT2(acf->cur_pos_3d)) >
		    OTH_TFC_DIST_THRESH) {
			/* Don't bother if the traffic is too far away */
			continue;
		}
		acf->trend_data_ready = (
		    xtcas_obj_pos_get_gs(&acf->pos_upd, &acf->gs) &&
		    xtcas_obj_pos_get_trk(&acf->pos_upd, &acf->trk) &&
		    xtcas_obj_pos_get_vvel(&acf->pos_upd, &acf->vvel,
		    &my_acf_glob.d_vvel));
		acf->trk_v = (acf->trend_data_ready) ?
		    vect2_set_abs(hdg2dir(acf->trk), acf->gs) : NULL_VECT2;
		if (pos->on_ground) {
			acf->on_ground = B_TRUE;
		} else {
			/*
			 * If not already declared as being on-ground via
			 * Mode S, try to determine that using our RA height.
			 */
			acf->on_ground = (acf->alt_rptg &&
			    acf->cur_pos.elev < gnd_level +
			    ON_GROUND_AGL_THRESH);
		}
		/* mark acf as up-to-date */
		acf->up_to_date = B_TRUE;

		dbg_log(contact, 2, "bogie %p " PRINTF_ACF_FMT, acf->acf_id,
		    PRINTF_ACF_ARGS(acf));
	}

	/* walk the tree again and remove out-of-date aircraft */
	for (tcas_acf_t *acf = avl_first(&other_acf_glob), *acf_next = NULL;
	    acf != NULL; acf = acf_next) {
		acf_next = AVL_NEXT(&other_acf_glob, acf);
		if (!acf->up_to_date) {
			dbg_log(contact, 2, "bogie %p contact lost",
			    acf->acf_id);
			if (out_ops != NULL) {
				out_ops->delete_contact(out_ops->handle,
				    acf->acf_id);
			}
			avl_remove(&other_acf_glob, acf);
			free(acf);
		}
	}

	dbg_log(contact, 1, "total bogies: %lu", avl_numnodes(&other_acf_glob));

	free(pos);
}

/*
 * Copies all of the global aircraft position info (our aircraft and other
 * aircraft) to create a thread-local copy. `my_acf_copy' will be populated
 * with our aircraft's position info, while `other_acf_copy' will be
 * populated with the state of all other aircraft. Neither may be NULL.
 * If `test' is set, the intruder contact tree is populated with TCAS test
 * contacts instead of the real contacts.
 */
static void
copy_acf_state(tcas_acf_t *my_acf_copy, avl_tree_t *other_acf_copy, bool_t test)
{
	ASSERT(my_acf_copy != NULL);
	ASSERT(other_acf_copy != NULL);

	avl_create(other_acf_copy, acf_compar, sizeof (tcas_acf_t),
	    offsetof(tcas_acf_t, node));

	mutex_enter(&acf_lock);

	memcpy(my_acf_copy, &my_acf_glob, sizeof (*my_acf_copy));

	if (!test) {
		for (tcas_acf_t *acf = avl_first(&other_acf_glob); acf != NULL;
		    acf = AVL_NEXT(&other_acf_glob, acf)) {
			tcas_acf_t *acf_copy = safe_calloc(1, sizeof (*acf));
			memcpy(acf_copy, acf, sizeof (*acf));
			avl_add(other_acf_copy, acf_copy);
		}
	} else {
		/*
		 * The TCAS test pattern consists of 4 contacts arranged as
		 * follows:
		 * contact 1: 2NM left and 3NM ahead of our aircraft, 1000 ft
		 *	above, neither climbing nor descending. Classified as
		 *	other traffic (empty diamond).
		 * contact 2: 2NM right and 3NM ahead of our aircraft, 1000 ft
		 *	below, descending. Classificied as proximate traffic
		 *	(filled diamond).
		 * contact 3: 2NM left, 200 ft below, climbing, classified as
		 *	a TRAFFIC threat (solid yellow circle).
		 * contact 4: 2NM right, 200 ft above, flying level,
		 *	classified as an RA threat (solid red square).
		 * When we're built in GTS820 mode, contact 4 will be absent.
		 */

#define	ADD_TEST_CONTACT(id, x_nm, y_nm, rel_alt_ft, trend, threat_lvl) \
	do { \
		tcas_acf_t *acf = safe_calloc(1, sizeof (*acf)); \
		vect2_t v = vect2_rot(VECT2(NM2MET(x_nm), NM2MET(y_nm)), \
		    my_acf_copy->hdg); \
		acf->acf_id = (void *)id; \
		acf->cur_pos_3d = VECT3(v.x, v.y, \
		    my_acf_copy->cur_pos.elev + FEET2MET(rel_alt_ft)); \
		acf->alt_rptg = B_TRUE; \
		acf->vvel = trend; \
		acf->trend_data_ready = B_TRUE; \
		acf->up_to_date = B_TRUE; \
		acf->threat = threat_lvl; \
		avl_add(other_acf_copy, acf); \
	} while (0)

		ADD_TEST_CONTACT(1, -2, 3, 1000, 0, OTH_THREAT);
		ADD_TEST_CONTACT(2, 2, 3, -1000, -1000, PROX_THREAT);
		ADD_TEST_CONTACT(3, -2, 0, -200, 1000, TA_THREAT);
#if	!GTS820_MODE
		ADD_TEST_CONTACT(4, 2, 0, 200, 0, RA_THREAT_CORR);
#endif

#undef	ADD_TEST_CONTACT
	}

	mutex_exit(&acf_lock);
}

static void
destroy_acf_state(avl_tree_t *other_acf_copy)
{
	void *cookie = NULL;
	tcas_acf_t *acf;

	/*
	 * We will back up the threat level to the original tcas_acf_t
	 * object, since we need it in GTS820 mode to determine if a new
	 * TA threat has come up.
	 */
	mutex_enter(&acf_lock);

	while ((acf = avl_destroy_nodes(other_acf_copy, &cookie)) != NULL) {
		tcas_acf_t *orig_acf;

		ASSERT3P(acf->cpa, ==, NULL);
		orig_acf = avl_find(&other_acf_glob, acf, NULL);
		if (orig_acf != NULL) {
			orig_acf->threat = acf->threat;
			orig_acf->ta_time = acf->ta_time;
		}
		free(acf);
	}

	mutex_exit(&acf_lock);

	avl_destroy(other_acf_copy);
}

/*
 * This is the comparator for the CPA tree. We sort CPAs by time order
 * first and then by aircraft pointer value.
 */
static int
cpa_compar(const void *a, const void *b)
{
	const cpa_t *ca = a, *cb = b;

	ASSERT3P(ca->acf_a->acf_id, ==, cb->acf_a->acf_id);

	if (ca->d_t < cb->d_t) {
		return (-1);
	} else if (ca->d_t == cb->d_t) {
		if (ca->acf_b->acf_id < cb->acf_b->acf_id) {
			return (-1);
		} else if (ca->acf_b->acf_id == cb->acf_b->acf_id) {
			return (0);
		} else {
			return (1);
		}
	} else {
		return (1);
	}
}

/*
 * Creates a cpa_t.
 * @param d_t Time from now until the CPA.
 * @param acf_a Our aircraft involved in the CPA.
 * @param acf_b Intruder aircraft involved in the CPA.
 * @param pos_a Our aircraft's position at CPA.
 * @param pos_b Intruder aircraft's position at CPA.
 */
static cpa_t *
make_cpa(double d_t, tcas_acf_t *acf_a, tcas_acf_t *acf_b,
    vect3_t pos_a, vect3_t pos_b)
{
	cpa_t *cpa = safe_calloc(1, sizeof (*cpa));

	cpa->d_t = d_t;

	cpa->pos_a = pos_a;
	cpa->pos_b = pos_b;
	cpa->d_h = vect2_abs(vect2_sub(VECT2(pos_a.x, pos_a.y),
	    VECT2(pos_b.x, pos_b.y)));
	cpa->d_v = ABS(pos_a.z - pos_b.z);

	cpa->acf_a = acf_a;
	cpa->acf_b = acf_b;
	ASSERT3P(acf_b->cpa, ==, NULL);
	acf_b->cpa = cpa;

	return (cpa);
}
/*
 * Given our and all other aircraft's positions, directions, speeds and
 * vertical velocities, compute where the closest points of approach are
 * going to occur.
 *
 * The algorithm works as follows:
 * 1) We first construct a stereographic projection at our aircraft's
 *	current position. This forms the horizontal zero coordinates of a
 *	3D space (z coordinate at sea level) in which we will conduct all
 *	of our CPA computation.
 * 2) Construct our 3D coordinates (by definition [0, 0, elev]) and our
 *	3D velocity vector (based on our track, ground & vertical speed).
 * 3) For each potential intruder aircraft (bogie), transform their
 *	position into our fpp space and construct their 3D position and
 *	3D velocity vector.
 * 4) For each bogie, compute a relative velocity vector with respect to
 *	our velocity vector. This, in combination with the bogie's position
 *	vector (which is relative to our current position) in effect gives
 *	us the bogie's position as a straight line through the reference
 *	frame of our aircraft's movement through space.
 * 5) At any given point, the bogie's distance to us is defined by the
 *	following simple 3-space vector length relationship:
 *	l = sqrt((x + v_x * t)^2 + (y + v_y * t)^2 + (z + v_z * t)^2)
 *	where:
 *	x, y, z: are the bogie's initial position relative to us
 *	v_x, v_y, v_z: are the bogie's relative velocity vector components
 *	t: is the time relative to now
 *	This distance equation describes a quadratic function.
 * 6) To determine the closest point of approach, we only need to know
 *	for what value of 't' the function reaches its lowest point, i.e.
 *	where the derivative of this function passes through 0.
 * 7) The derivative of the right-hand side of that equation, solved for
 *	for t is:
 *	t = (-(x * v_x) - (y * v_y) - (z * v_z)) / (v_x^2 + v_y^2 + v_z^2)
 * 8) Given the value 't', we know when and can simply compute where the
 *	CPA occurs (or occurred). If 't' is negative (CPA has already
 *	occurred) or too high (CPA is too far in the future), or the
 *	separation at CPA exceeds a threshold value, we simply don't
 *	generate a cpa_t record for that particular aircraft at all.
 */
static void
compute_CPAs(avl_tree_t *cpas, tcas_acf_t *my_acf, avl_tree_t *other_acf)
{
	vect3_t my_pos_3d = my_acf->cur_pos_3d;
	vect3_t my_vel = VECT3(my_acf->trk_v.x, my_acf->trk_v.y, my_acf->vvel);

	avl_create(cpas, cpa_compar, sizeof (cpa_t), offsetof(cpa_t, node));

	for (tcas_acf_t *acf = avl_first(other_acf); acf != NULL;
	    acf = AVL_NEXT(other_acf, acf)) {
		vect2_t dir;
		vect3_t rel_pos_3d, vel, rel_vel, cpa_pos, my_cpa_pos;
		double t_cpa;
		cpa_t *cpa;

		/*
		 * Don't compute CPAs for contacts that either:
		 * 1) Don't have any trend data available yet.
		 * 2) Ground speed is zero (false contact).
		 * 3) Fall outside of our maximum vertical filter boundaries.
		 */
		if (!acf->trend_data_ready || !my_acf->trend_data_ready ||
		    acf->gs < FALSE_CTC_SUPPRESS_GS || ABS(acf->cur_pos.elev -
		    my_acf->cur_pos.elev) > LONG_VERT_FILTER)
			continue;

		rel_pos_3d = VECT3(acf->cur_pos_3d.x, acf->cur_pos_3d.y,
		    acf->cur_pos_3d.z - my_pos_3d.z);
		dir = acf->trk_v;
		vel = VECT3(dir.x, dir.y, acf->vvel);
		rel_vel = vect3_sub(vel, my_vel);
		if (!IS_ZERO_VECT3(rel_vel)) {
			t_cpa = floor((-(rel_pos_3d.x * rel_vel.x) -
			    (rel_pos_3d.y * rel_vel.y) -
			    (rel_pos_3d.z * rel_vel.z)) /
			    (POW2(rel_vel.x) + POW2(rel_vel.y) +
			    POW2(rel_vel.z)));
			/*
			 * If CPA is in the past, the current position is the
			 * CPA.
			 */
			t_cpa = MAX(t_cpa, 0);
		} else {
			t_cpa = 0;
		}

		cpa_pos = vect3_add(acf->cur_pos_3d, vect3_scmul(vel, t_cpa));
		my_cpa_pos = vect3_add(my_pos_3d, vect3_scmul(my_vel, t_cpa));

		cpa = make_cpa(t_cpa, my_acf, acf, my_cpa_pos, cpa_pos);
		dbg_log(cpa, 1, "bogie %p cpa d_t:%.1f pos_a:%.0fx%.0fx%.0f "
		    "pos_b:%.0fx%.0fx%.0f d_h:%.0f d_v:%.0f",
		    acf->acf_id, cpa->d_t,
		    cpa->pos_a.x, cpa->pos_a.y, cpa->pos_a.z,
		    cpa->pos_b.x, cpa->pos_b.y, cpa->pos_b.z,
		    cpa->d_h, cpa->d_v);
		avl_add(cpas, cpa);
	}
}

static void
destroy_CPAs(avl_tree_t *cpas)
{
	void *cookie = NULL;
	cpa_t *cpa;
	while ((cpa = avl_destroy_nodes(cpas, &cookie)) != NULL) {
		cpa->acf_b->cpa = NULL;
		free(cpa);
	}
	avl_destroy(cpas);
}

/*
 * Given an intruder aircraft and the current SL, assigns the threat level
 * (tcas_threat_t) to that aircraft. Arguments:
 * @param my_pos_3d Our current position in 3-space vector coordinates.
 * @param oacf The other aircraft contact to which to assign a threat level.
 * @param sl The current sensitivity level selected for TCAS
 *	(see xtcas_SL_select).
 * @param RA_hints A set of external RA-threat hints. When an aircraft
 *	is initially declared an RA threat, it is marked in this tree
 *	to prevent degrading it to a lower threat during maneuvers.
 * @param filter The currently active TCAS altitude filter (see tcas_filter_t).
 *
 * The order of threat assignments here is important. We go from most serious
 * to least serious:
 * 1) If an intruder violates our RA protected volume or RA tau, they are
 *    placed at the highest level of RA_THREAT_CORR.
 * 2) If an intruder didn't satisfy condition 1, we look for a previous
 *    RA threat hint, so we might still declare them RA_THREAT_CORR or
 *    RA_THREAT_PREV.
 * 3) If there is no hint, we evaluate the protected volume and tau space
 *    for preventive threatds (same horizontal space, vertical expanded
 *    to zthr_TA) and assign RA_THREAT_PREV if we find something.
 * 4) If conditions 1-3 aren't met, the intruder is definitely NOT an RA
 *    threat. Evaluate the TA protection volume and tau and assign TA_THREAT.
 * 5) If 1-4 aren't met, the intruder is not a threat, so just evaluate
 *    range & altitude to determine proximate traffic and assign PROX_THREAT.
 * 6) Lastly, any other intruder is other traffic and designated OTH_THREAT.
 */
static void
assign_threat_level(tcas_acf_t *my_acf, tcas_acf_t *oacf, const SL_t *sl,
    avl_tree_t *RA_hints, tcas_filter_t filter, uint64_t now)
{
	double d_h = vect2_abs(vect2_sub(VECT3_TO_VECT2(oacf->cur_pos_3d),
	    VECT3_TO_VECT2(my_acf->cur_pos_3d)));
	double d_v = ABS(my_acf->cur_pos_3d.z - oacf->cur_pos_3d.z);
	cpa_t *cpa = oacf->cpa;
	double filter_min = my_acf->cur_pos_3d.z;
	double filter_max = my_acf->cur_pos_3d.z;
	bool_t vert_filter = B_TRUE;
	tcas_RA_hint_t srch = { .acf_id = oacf->acf_id };
	tcas_RA_hint_t *hint;

	/*
	 * Check if the altitude filter has been satisfied. Non-altitude-
	 * reporting aircraft cannot become RA threats.
	 */
	filter_min -= (filter == TCAS_FILTER_ABV ? NORM_VERT_FILTER :
	    LONG_VERT_FILTER);
	filter_max += (filter == TCAS_FILTER_BLW ? NORM_VERT_FILTER :
	    LONG_VERT_FILTER);
	if (oacf->alt_rptg) {
		vert_filter = (oacf->cur_pos.elev >= filter_min &&
		    oacf->cur_pos.elev <= filter_max);
	}

	hint = avl_find(RA_hints, &srch, NULL);
	if (cpa != NULL && (!oacf->on_ground || hint != NULL)) {
		double dist = vect2_abs(vect2_sub(VECT3_TO_VECT2(
		    my_acf->cur_pos_3d), VECT3_TO_VECT2(oacf->cur_pos_3d)));
		double r_vel = ((cpa->d_t == 0) ? 1 : -1) *
		    vect2_abs(vect2_sub(my_acf->trk_v, oacf->trk_v));
		double r_alt = ABS(my_acf->cur_pos_3d.z - oacf->cur_pos_3d.z);

		/*
		 * Fast RA-corrective threat iff:
		 * 1) reporting an altitude AND
		 * 2) NOT on the ground AND
		 * 3) CPA d_v is below ALIM (thus a correction is required)
		 *     AND
		 * 4) its CPA point violates the RA-protected volume AND
		 * 5) time to CPA is <= tau_RA.
		 * 6) we are actually closing in
		 */
		if (oacf->alt_rptg && !oacf->on_ground &&
		    cpa->d_v <= sl->alim_RA && cpa->d_h <= sl->dmod_RA &&
		    cpa->d_t <= sl->tau_RA && cpa->d_t > 0) {
			dbg_log(threat, 1, "bogie %p RA_CORR(fast) "
			    "cpa->d_v: %.0f <= %.0f cpa->d_h: %.0f <= %.0f "
			    "d_t: %.1f", oacf->acf_id, cpa->d_v, sl->alim_RA,
			    cpa->d_h, sl->dmod_RA, cpa->d_t);
			oacf->threat = RA_THREAT_CORR;
			oacf->slow_closure = B_FALSE;
			return;
		}
		/*
		 * Slow RA-corrective threat iff:
		 * 1) reporting an altitude AND
		 * 2) NOT on the ground AND
		 * 2) its present position violates the RA-protected volume AND
		 * 3a) EITHER the relative velocity is below the approaching
		 *    threshold (i.e. intruder is approaching us),
		 * 3b) OR intruder is moving away so slowly, that they won't
		 *    clear our the horizontal boundaries of our protected
		 *    volume in less than 5 seconds.
		 * The vertical filter is NOT applied to RAs.
		 */
		if (oacf->alt_rptg && !oacf->on_ground && d_v <= sl->alim_RA &&
		    d_h <= sl->dmod_RA && (r_vel <= APCH_SPD_THRESH ||
		    (sl->dmod_RA - d_h) / r_vel > INITIAL_RA_DELAY)) {
			dbg_log(threat, 1, "bogie %p RA_CORR(slow) d_v: "
			    "%.0f <= %.0f d_h: %.0f <= %.0f d_t: %.1f",
			    oacf->acf_id, d_v, sl->alim_RA, d_h, sl->dmod_RA,
			    cpa->d_t);
			oacf->threat = RA_THREAT_CORR;
			oacf->slow_closure = B_TRUE;
			return;
		}
		/*
		 * If the previous filter didn't find a corrective threat, we
		 * might still derive an RA threat level from a previous hint.
		 * However, only do so if:
		 * 1) The target is reporting altitude (if they stop
		 *    reporting, we need to drop the RA).
		 * 2) The current position violates the TA-protected volume
		 *    boosted by 20% in both vertical & horizontal extent
		 *    (to avoid issuing a TA against an RA threat).
		 * 3) We're closing in, or if we're moving away, they're
		 *    so close that if we started a descent or climb after
		 *    declaring clear of conflict, we might trigger a second
		 *    TA or RA.
		 */
		if (hint != NULL && oacf->alt_rptg &&
		    (cpa->d_h <= sl->dmod_TA * HINT_H_INCR_FACT ||
		    d_h <= sl->dmod_TA * HINT_H_INCR_FACT) &&
		    (cpa->d_v <= sl->zthr_TA * HINT_V_INCR_FACT ||
		    d_v <= sl->zthr_TA * HINT_V_INCR_FACT) &&
		    ((r_vel <= APCH_SPD_THRESH) ||
		    ((sl->dmod_TA - dist) / ABS(r_vel) >
		    (r_alt - sl->zthr_TA) / CLEARING_CLIMB_RATE))) {
			/* Hints cannot exist on initial RAs */
			ASSERT3U(tcas_state.adv_state, ==, ADV_STATE_RA);
			dbg_log(threat, 1, "bogie %p RA_HINT(%d,%d) "
			    "d_t: %.1f > 0 r_vel: %.1f alt_rptg: %d "
			    "d_h: %.0f|%.0f <= %.0f && d_v: %.0f|%.0f <= %.0f",
			    oacf->acf_id, hint->level, hint->slow_closure,
			    cpa->d_t, r_vel, oacf->alt_rptg, cpa->d_h, d_h,
			    sl->dmod_TA * HINT_H_INCR_FACT, cpa->d_v, d_v,
			    sl->zthr_TA * HINT_V_INCR_FACT);
			ASSERT3U(hint->level, >=, RA_THREAT_PREV);
			oacf->threat = hint->level;
			oacf->slow_closure = hint->slow_closure;
			return;
		}
		/*
		 * An RA-preventive threat is the same as an RA-corrective
		 * threat, but the protected volume computation is modified
		 * such that the horizontal extent remains the same, but
		 * the vertical volume is on the RA boundaries. This allows
		 * us to issue a preventive RA to prevent the aircraft from
		 * entering the RA vertical extent.
		 */
		if (oacf->alt_rptg && !oacf->on_ground &&
		    cpa->d_v <= sl->zthr_RA && cpa->d_h <= sl->dmod_RA &&
		    cpa->d_t <= sl->tau_RA && cpa->d_t > 0) {
			dbg_log(threat, 1, "bogie %p RA_PREV(fast) d_v: "
			    "%.0f <= %.0f d_h: %.0f <= %.0f d_t: %.0f",
			    oacf->acf_id, cpa->d_v, sl->zthr_RA, cpa->d_h,
			    sl->dmod_RA, cpa->d_t);
			oacf->threat = RA_THREAT_PREV;
			oacf->slow_closure = B_FALSE;
			return;
		}
		/*
		 * The preventive version of the slow approach corrective RA.
		 */
		if (oacf->alt_rptg && !oacf->on_ground && d_v <= sl->zthr_RA &&
		    d_h <= sl->dmod_RA && (r_vel < APCH_SPD_THRESH ||
		    (sl->dmod_RA - d_h) / r_vel > INITIAL_RA_DELAY)) {
			dbg_log(threat, 1, "bogie %p RA_PREV(slow) d_v: "
			    "%.0f <= %.0f d_h: %.0f <= %.0f d_t: %.0f",
			    oacf->acf_id, d_v, sl->zthr_RA, d_h, sl->dmod_RA,
			    cpa->d_t);
			oacf->threat = RA_THREAT_PREV;
			oacf->slow_closure = B_TRUE;
			return;
		}

		/*
		 * Fast TA threat if:
		 * 1) The vertical filter is satisfied AND
		 * 2) The aircraft is NOT declared to be on the ground AND
		 * 3) horiz separation at CPA violates protected volume AND
		 * 4) the aircraft is NOT reporting altitude, OR vert
		 *    separation at CPA violates protected volume
		 * 5) time to CPA <= tau_TA
		 * 6) relative velocity indicates we're approaching
		 */
		if (vert_filter && !oacf->on_ground &&
		    cpa->d_h <= sl->dmod_TA && (!oacf->alt_rptg ||
		    cpa->d_v <= sl->zthr_TA) && cpa->d_t <= sl->tau_TA &&
		    r_vel < APCH_SPD_THRESH) {
			dbg_log(threat, 1, "bogie %p TA(fast)", oacf->acf_id);
			oacf->threat = TA_THREAT;
			return;
		}
		/*
		 * Slow TA threat if:
		 * 1) The vertical filter is satisfied AND
		 * 2) The aircraft is NOT declared to be on the ground AND
		 * 3) horiz separation NOW violates protected volume AND
		 * 4) the relative velocity is below the approaching
		 *    threshold (i.e. intruder is approaching us)
		 * 5) the aircraft is NOT reporting altitude, OR vert
		 *    separation NOW violates protected volume
		 */
		if (vert_filter && !oacf->on_ground &&
		    d_h <= sl->dmod_TA && r_vel < APCH_SPD_THRESH &&
		    (!oacf->alt_rptg || d_v <= sl->zthr_TA)) {
			dbg_log(threat, 1, "bogie %p TA(slow)", oacf->acf_id);
			oacf->threat = TA_THREAT;
			return;
		}
	}

	/*
	 * Don't degrade TAs too quickly to avoid duplicate "Traffic" calls.
	 */
	if (now - oacf->ta_time < TA_THREAT_CANCEL_DELAY) {
		dbg_log(threat, 1, "bogie %p TA(delay)", oacf->acf_id);
		oacf->threat = TA_THREAT;
		return;
	}

	/*
	 * Proximate traffic iff:
	 * 1) horiz separation within prox traffic dist thresh, AND
	 * 2a) target doesn't report altitude, OR
	 * 2b) vert separation within prox traffic altitude threshold
	 */
	if (d_h <= PROX_DIST_THRESH && !oacf->on_ground &&
	    (!oacf->alt_rptg || d_v <= PROX_ALT_THRESH)) {
		dbg_log(threat, 1, "bogie %p PROX", oacf->acf_id);
		oacf->threat = PROX_THREAT;
		return;
	}

	dbg_log(threat, 1, "bogie %p OTH", oacf->acf_id);
	oacf->threat = OTH_THREAT;
}

static const tcas_RA_t *
least_departing_RA(const tcas_RA_t *a, const tcas_RA_t *b)
{
	const tcas_acf_t *my_acf = ((cpa_t *)avl_first(a->cpas))->acf_a;
	double init_vs = roundmul(tcas_state.initial_ra_vs, ALT_ROUND_MUL);
	double cur_vs = roundmul(my_acf->vvel, ALT_ROUND_MUL);
	double d_vs_a = fabs(init_vs - (cur_vs + a->vs_corr_reqd));
	double d_vs_b = fabs(init_vs - (cur_vs + b->vs_corr_reqd));

	if (d_vs_a < d_vs_b)
		return (a);
	if (d_vs_a > d_vs_b)
		return (b);
	return (NULL);
}

static int
ra_compar_normal(const void *ra_a, const void *ra_b)
{
	const tcas_RA_t *a = ra_a, *b = ra_b;
	const tcas_RA_t *x;

	/* RAs can only be sorted if they refer to the same encounter */
	ASSERT3P(a->cpas, ==, b->cpas);
	ASSERT3P(a->sl, ==, b->sl);

	/* if the RAs point to the same info, they must be the same RA */
	if (a->info == b->info) {
		ASSERT(memcmp(a, b, offsetof(tcas_RA_t, node)) == 0);
		return (0);
	}

	/*
	 * If the encounter is beyond tau_RA, then we were triggered by
	 * the dmod_RA or zthr_RA filter. In that case, we want to prefer
	 * RAs which provide zthr_RA separation so that we get out of the
	 * encounter as soon as possible.
	 */
	if (((cpa_t *)avl_first(a->cpas))->d_t > a->sl->tau_RA) {
		if (a->zthr_achieved && !b->zthr_achieved)
			return (-1);
		if (!a->zthr_achieved && b->zthr_achieved)
			return (1);
	}

	/* if one RA provides ALIM_RA and the other not, that's sufficient */
	if (a->alim_achieved && !b->alim_achieved)
		return (-1);
	if (!a->alim_achieved && b->alim_achieved)
		return (1);

	/*
	 * If neither achieves ALIM, we pick the one with the greatest
	 * minimum separation from any contact.
	 */
	if (!a->alim_achieved && !b->alim_achieved) {
		double sep_a = MAX(a->min_sep, 0), sep_b = MAX(b->min_sep, 0);
		if (sep_a == sep_b && a->min_sep >= 0 && b->min_sep >= 0) {
			/*
			 * If our minimum seps are equal, that means we have
			 * too little time to perform the full VS correction
			 * maneuver. In that case try to pick the RA that
			 * performs the greatest VS change - maximizing the
			 * chance the pilot will react faster than our
			 * pessimistic estimate and pull us away hard.
			 */
			if (ABS(a->vs_corr_reqd) > ABS(b->vs_corr_reqd))
				return (-1);
			else
				return (1);
		}
		if (sep_a > sep_b)
			return (-1);
		else
			return (1);
	}

	/*
	 * From here on out both RAs satisfy the ALIM condition, so we need
	 * to just pick the best one from them.
	 */

	/* prefer non-sense-reversing RAs */
	if (!a->reversal && b->reversal)
		return (-1);
	if (a->reversal && !b->reversal)
		return (1);

	/* prefer non-crossing RAs */
	if (!a->crossing && b->crossing)
		return (-1);
	if (a->crossing && !b->crossing)
		return (1);

	/* pick the RA departing least from our original VS */
	x = least_departing_RA(a, b);
	if (x == a)
		return (-1);
	if (x == b)
		return (1);

	/* pick the RA requiring the minimum VS correction */
	if (ABS(a->vs_corr_reqd) < ABS(b->vs_corr_reqd))
		return (-1);
	if (ABS(a->vs_corr_reqd) > ABS(b->vs_corr_reqd))
		return (1);

	/* pick based on minimum separation achieved */
	if (a->min_sep > b->min_sep)
		return (-1);
	if (a->min_sep < b->min_sep)
		return (1);

	/* prefer preventive RAs over corrective RAs */
	if (a->info->type == RA_TYPE_PREVENTIVE &&
	    b->info->type == RA_TYPE_CORRECTIVE)
		return (-1);
	if (a->info->type == RA_TYPE_CORRECTIVE &&
	    b->info->type == RA_TYPE_PREVENTIVE)
		return (1);

	/* final choice - simply pick RA info that comes first */
	if (a->info < b->info)
		return (-1);
	else
		return (1);
}

static int
ra_compar_slow(const void *ra_a, const void *ra_b)
{
	const tcas_RA_t *a = ra_a, *b = ra_b;
	const tcas_RA_t *x;

	if (a->info == b->info) {
		ASSERT(memcmp(a, b, offsetof(tcas_RA_t, node)) == 0);
		return (0);
	}

	if (a->crossing && b->crossing) {
		if (a->info < b->info)
			return (-1);
		else
			return (1);
	}
	if (!a->crossing && b->crossing)
		return (-1);
	if (a->crossing && !b->crossing)
		return (1);

	x = least_departing_RA(a, b);
	if (x == a)
		return (-1);
	if (x == b)
		return (1);

	/* pick the RA requiring the minimum VS correction */
	if (ABS(a->vs_corr_reqd) < ABS(b->vs_corr_reqd))
		return (-1);
	if (ABS(a->vs_corr_reqd) > ABS(b->vs_corr_reqd))
		return (1);

	/* final choice - simply pick RA info that comes first */
	if (a->info < b->info)
		return (-1);
	else
		return (1);
}

static double
predict_elev_at_CPA(const tcas_acf_t *my_acf, double cpa_t, double delay_t,
    double accel, double vsr)
{
	double man_t, rmng_t, result;

	delay_t = MIN(cpa_t, delay_t);
	if (vsr < my_acf->vvel)
		accel = -accel;
	man_t = MIN(cpa_t - delay_t, (vsr - my_acf->vvel) / accel);
	rmng_t = cpa_t - (delay_t + man_t);
	/*
	 * The elevation change consists of the following segments:
	 * 1) a straight segment at the aircraft's original VVEL for the
	 *	duration of delay_t:
	 *	d1 = vs1 * delay_t
	 * 2) an accelerating maneuver segment at the acceleration value,
	 *	which takes a computed amount of time to reach the new
	 *	target VSR:
	 *	d2 = vs1 * man_t + 0.5 * accel * man_t^2
	 * 3) a final straight line segment at the new VSR for the remaining
	 *	time between the end of delay_t + man_t until cpa_t (rmng_t):
	 *	d3 = vsr * rmng_t
	 * We can simplify the first and second portions to obtain:
	 *	d12 = vs1 * (delay_t + man_t) + 0.5 * accel * man_t^2
	 */
	result = my_acf->cur_pos.elev + roundmul(my_acf->vvel, ALT_ROUND_MUL) *
	    (delay_t + man_t) + 0.5 * accel * POW2(man_t) + vsr * rmng_t;
	return (result);
}

static double
compute_separation(const tcas_acf_t *my_acf, cpa_t *cpa,
    const tcas_RA_info_t *ri, double delay_t, double accel)
{
	double elev_min = predict_elev_at_CPA(my_acf, cpa->d_t,
	    ri->vs.out.min > my_acf->vvel ? delay_t : 0, accel, ri->vs.out.min);
	double elev_max = predict_elev_at_CPA(my_acf, cpa->d_t,
	    ri->vs.out.max < my_acf->vvel ? delay_t : 0, accel, ri->vs.out.max);
	double sep;

	switch (ri->sense) {
	case RA_SENSE_UPWARD:
		sep = elev_min - cpa->pos_b.z;
		break;
	case RA_SENSE_LEVEL_OFF:
		sep = MAX(elev_min - cpa->pos_b.z, cpa->pos_b.z - elev_max);
		break;
	default:
		sep = (cpa->pos_b.z - elev_max);
		break;
	}

	dbg_log(cpa, 3, "acf %p %s s:%s d_t:%.1f vsmin:%.0f vsmax:%.0f "
	    "omin:%.0f omax:%.0f sep:%f", cpa->acf_b->acf_id,
	    RA_msg2str(ri->msg), RA_sense2str(ri->sense), cpa->d_t,
	    ri->vs.out.min, ri->vs.out.max, elev_min, elev_max, sep);

	return (sep);
}

/*
 * Check for incompatible sequences of RA message annunciations. Sometimes
 * we want to prevent calling out an RA even if it has changed. For example,
 * when weakening a preventive RA with the same messages, don't call out
 * the weakening action - it's already fine.
 * Returns the actual RA message which should be played, or RA_MSG_NONE.
 */
static tcas_msg_t
RA_msg_sequence_check(tcas_msg_t prev_msg, tcas_msg_t next_msg)
{
	/* Prevent repeating the same RA */
	if (prev_msg == next_msg)
		return (-1);
	return (next_msg);
}

static bool_t
check_black_band(const tcas_RA_info_t *ri, const tcas_acf_t *my_acf,
    avl_tree_t *cpas)
{
	double vs;

	if (ri->vs.red_lo.min != ri->vs.red_lo.max) {
		/* black band is ABOVE green band */
		vs = ri->vs.out.max;
	} else {
		/* black band is BELOW green band */
		ASSERT3F(ri->vs.red_hi.min, !=, ri->vs.red_hi.max);
		vs = ri->vs.out.min;
	}

	for (cpa_t *cpa = avl_first(cpas); cpa != NULL;
	    cpa = AVL_NEXT(cpas, cpa)) {
		double elev = my_acf->cur_pos.elev + cpa->d_t * vs;

		if ((vs <= 0 && cpa->pos_b.z <= elev) ||
		    (vs >= 0 && cpa->pos_b.z >= elev))
			return (B_FALSE);
	}

	return (B_TRUE);
}

static tcas_RA_t *
ra_construct(const tcas_acf_t *my_acf, const tcas_RA_info_t *ri,
    avl_tree_t *cpas, const SL_t *sl, double delay_t, double accel,
    bool_t reversal)
{
	tcas_RA_t *ra = safe_calloc(1, sizeof (*ra));

	ra->info = ri;
	ra->cpas = cpas;
	ra->sl = sl;
	ra->reversal = reversal;
	ra->min_sep = 1e10;
	for (cpa_t *cpa = avl_first(cpas); cpa != NULL;
	    cpa = AVL_NEXT(cpas, cpa)) {
		double sep = compute_separation(my_acf, cpa, ri, delay_t,
		    accel);
		ra->min_sep = MIN(sep, ra->min_sep);
		/*
		 * An encounter is crossing if either:
		 * 1)
		 *	a: the RA sense is upward
		 *	b: currently we are below the intruder
		 * 2)
		 *	a: the RA sense is downward
		 *	b: currently we are above the intruder
		 */
		ra->crossing |= ((ri->sense == RA_SENSE_UPWARD &&
		    cpa->acf_b->cur_pos_3d.z - cpa->acf_a->cur_pos_3d.z >
		    EQ_ALT_THRESH) ||
		    (ri->sense == RA_SENSE_DOWNWARD &&
		    cpa->acf_a->cur_pos_3d.z - cpa->acf_b->cur_pos_3d.z >
		    EQ_ALT_THRESH));

	}
	ASSERT(isfinite(ra->min_sep));
	ra->alim_achieved = (ra->min_sep >= sl->alim_RA);
	/*
	 * min_sep is computed with a maneuver in mind. We want to
	 * nullify that and provide some buffer so that another RA
	 * won't be triggered. So we use the TA zthr instead of RA.
	 */
	ra->zthr_achieved = (ra->min_sep >= sl->zthr_TA);

	return (ra);
}

static void
CAS_logic_normal(const tcas_acf_t *my_acf, const tcas_RA_t *prev_ra,
    avl_tree_t *cpas, const SL_t *sl, bool_t prev_only, avl_tree_t *prio)
{
	bool_t initial = (prev_ra == NULL);
	double delay_t = (initial ? INITIAL_RA_DELAY : SUBSEQ_RA_DELAY);
	double accel = (initial ? INITIAL_RA_D_VVEL : SUBSEQ_RA_D_VVEL);

	avl_create(prio, ra_compar_normal, sizeof (tcas_RA_t),
	    offsetof(tcas_RA_t, node));

	for (int i = 0; i < NUM_RA_INFOS; i++) {
		const tcas_RA_info_t *ri = &RA_info[i];
		double agl_at_cpa = my_acf->agl - (my_acf->cur_pos.elev -
		    ((cpa_t *)avl_last(cpas))->pos_a.z);
		tcas_msg_t prev_msg = (prev_ra != NULL ?
		    prev_ra->info->msg : -1u);
		tcas_RA_sense_t prev_sense = (prev_ra != NULL ?
		    prev_ra->info->sense : RA_SENSE_LEVEL_OFF);
		bool_t reversal = (prev_ra != NULL && prev_sense != ri->sense);
		tcas_RA_type_t prev_type = (prev_ra != NULL ?
		    prev_ra->info->type : -1u);
		tcas_msg_t msg = ri->msg;
		double penalty = 0;
		tcas_RA_t *ra;

		ASSERT3U(ri->msg, >=, 0);
		ASSERT3U(ri->msg, <, RA_NUM_MSGS);

		/* ri must allow initial/subseq ann as necessary */
		if ((initial && !ri->initial) || (!initial && !ri->subseq)) {
			dbg_log(ra, 4, "CULLRI(norm) init/subseq "
			    PRINTF_RI_FMT, PRINTF_RI_ARGS(ri));
			continue;
		}
		/* ri must satisfy input VS condition */
		if (my_acf->vvel < ri->vs.in.min ||
		    my_acf->vvel > ri->vs.in.max) {
			dbg_log(ra, 4, "CULLRI(norm) in.vs " PRINTF_RI_FMT,
			    PRINTF_RI_ARGS(ri));
			continue;
		}
		/* if performing a sense reversal, ri must allow it */
		if (reversal && (int)ri->rev_msg == -1) {
			dbg_log(ra, 4, "CULLRI(norm) REV " PRINTF_RI_FMT,
			    PRINTF_RI_ARGS(ri));
			continue;
		}
		/* select preventive-only RAs if requested */
		if (prev_only && ri->type != RA_TYPE_PREVENTIVE) {
			dbg_log(ra, 4, "CULLRI(norm) PREV " PRINTF_RI_FMT,
			    PRINTF_RI_ARGS(ri));
			continue;
		}
		/* above FL480 we want to inhibit climb RAs. */
		if (my_acf->cur_pos.elev > INHIBIT_CLB_RA &&
		    ri->sense == RA_SENSE_UPWARD) {
			dbg_log(ra, 4, "CULLRI(norm) CLB(FL480/NOW) "
			    PRINTF_RI_FMT, PRINTF_RI_ARGS(ri));
			continue;
		}
		/* inhibit INCREASE DESCENT RAs below 1550ft AGL. */
		if (my_acf->agl < INHIBIT_INC_DES_AGL &&
		    msg == RA_MSG_DES_MORE) {
			dbg_log(ra, 4, "CULLRI(norm) INCDES(1550/NOW) "
			    PRINTF_RI_FMT, PRINTF_RI_ARGS(ri));
			continue;
		}
		/* inhibit all DESCEND RAs below 1100ft AGL. */
		if (my_acf->agl < INHIBIT_DES_AGL &&
		    ri->sense == RA_SENSE_DOWNWARD) {
			dbg_log(ra, 4, "CULLRI(norm) DES(1100) "
			    PRINTF_RI_FMT, PRINTF_RI_ARGS(ri));
			continue;
		}
		/*
		 * This is a predictive version of the conditions above, but
		 * using AGL at the first CPA, rather than AGL NOW.
		 */
		if ((msg == RA_MSG_DES || msg == RA_MSG_DES_CROSS) &&
		    agl_at_cpa < INHIBIT_DES_AGL) {
			dbg_log(ra, 4, "CULLRI(norm) DES(1100/PRED) "
			    PRINTF_RI_FMT, PRINTF_RI_ARGS(ri));
			continue;
		}
		/* Don't issue DES MORE below 1550ft */
		if (msg == RA_MSG_DES_MORE &&
		    agl_at_cpa < INHIBIT_INC_DES_AGL) {
			dbg_log(ra, 4, "CULLRI(norm) DESMORE(1550/PRED) "
			    PRINTF_RI_FMT, PRINTF_RI_ARGS(ri));
			continue;
		}
		/*
		 * Pick an RA that makes sense from a sequencing POV. We
		 * want to prevent a CLB MORE RA from reducing to a
		 * regular CLB or CROSS CLB RA. In those cases, we want
		 * to either LEVEL OFF or perform a sense reversal.
		 */
		if (prev_msg == RA_MSG_CLB_MORE && (msg == RA_MSG_CLB ||
		    msg == RA_MSG_CLB_CROSS)) {
			dbg_log(ra, 4, "CULLRI(norm) CLBMORE->CLB "
			    PRINTF_RI_FMT, PRINTF_RI_ARGS(ri));
			continue;
		}
		/* Same as above, but for DES_MORE */
		if (prev_msg == RA_MSG_DES_MORE && (msg == RA_MSG_DES ||
		    msg == RA_MSG_DES_CROSS)) {
			dbg_log(ra, 4, "CULLRI(norm) DESMORE->DES "
			    PRINTF_RI_FMT, PRINTF_RI_ARGS(ri));
			continue;
		}
		if ((msg == RA_MSG_CLB_MORE || msg == RA_MSG_DES_MORE) &&
		    prev_sense != ri->sense) {
			dbg_log(ra, 4, "CULLRI(norm) sense check -> *MORE "
			    PRINTF_RI_FMT, PRINTF_RI_ARGS(ri));
			continue;
		}
		/*
		 * Avoid issuing a LEVEL OFF RA in response to a preventive
		 * RA (MONITOR VS). LEVEL OFF is only allowed in response to
		 * a corrective RA.
		 */
		if (msg == RA_MSG_LEVEL_OFF &&
		    prev_type == RA_TYPE_PREVENTIVE) {
			dbg_log(ra, 4, "CULLRI(norm) PREV -> LEVEL OFF "
			    PRINTF_RI_FMT, PRINTF_RI_ARGS(ri));
			continue;
		}
		/*
		 * LEVEL OFF RAs must check that there's no RA CPA in their
		 * black band, to avoid causing a LEVEL OFF into an intruder.
		 */
		if (ri->chk_black && !check_black_band(ri, my_acf, cpas)) {
			dbg_log(ra, 4, "CULLRI(norm) LEVEL_OFF(BB) "
			    PRINTF_RI_FMT, PRINTF_RI_ARGS(ri));
			continue;
		}

		ra = ra_construct(my_acf, ri, cpas, sl, delay_t, accel,
		    reversal);
		if (ra->crossing)
			penalty += CROSSING_RA_PENALTY;
		if (ra->reversal)
			penalty += CROSSING_RA_PENALTY;
		ra->min_sep -= ABS(ra->min_sep) * penalty;
		if (my_acf->vvel < ri->vs.out.min) {
			ra->vs_corr_reqd = roundmul(ri->vs.out.min -
			    my_acf->vvel, ALT_ROUND_MUL);
		} else if (my_acf->vvel > ri->vs.out.max) {
			ra->vs_corr_reqd = roundmul(ri->vs.out.max -
			    my_acf->vvel, ALT_ROUND_MUL);
		}

		/* Honor the RI's crossing restriction. */
		if ((ri->cross == RA_CROSS_REQ && !ra->crossing) ||
		    (ri->cross == RA_CROSS_REJ && ra->crossing)) {
			dbg_log(ra, 4, "CULLRA(norm) cross restr "
			    PRINTF_RA_FMT, PRINTF_RA_ARGS(ra));
			free(ra);
			continue;
		}
		/* Don't accept a preventive RA which doesn't give ALIM. */
		if (prev_only && !ra->alim_achieved) {
			dbg_log(ra, 4, "CULLRA(norm) PREV(w/o alim) "
			    PRINTF_RA_FMT, PRINTF_RA_ARGS(ra));
			free(ra);
			continue;
		}
		dbg_log(ra, 4, "ADD(norm) " PRINTF_RA_FMT, PRINTF_RA_ARGS(ra));
		avl_add(prio, ra);
	}
}

static void
CAS_logic_slow(const tcas_acf_t *my_acf, const tcas_RA_t *prev_ra,
    avl_tree_t *cpas, const SL_t *sl, avl_tree_t *prio)
{
	bool_t initial = (prev_ra == NULL);
	double delay_t = (initial ? INITIAL_RA_DELAY : SUBSEQ_RA_DELAY);
	double accel = (initial ? INITIAL_RA_D_VVEL : SUBSEQ_RA_D_VVEL);

	avl_create(prio, ra_compar_slow, sizeof (tcas_RA_t),
	    offsetof(tcas_RA_t, node));

	for (int i = 0; i < NUM_RA_INFOS; i++) {
		const tcas_RA_info_t *ri = &RA_info[i];
		tcas_msg_t prev_msg = (prev_ra != NULL ?
		    prev_ra->info->msg : -1u);
		tcas_RA_sense_t prev_sense = (prev_ra != NULL ?
		    prev_ra->info->sense : RA_SENSE_LEVEL_OFF);
		bool_t reversal = (prev_sense != RA_SENSE_LEVEL_OFF &&
		    ri->sense != RA_SENSE_LEVEL_OFF && prev_sense != ri->sense);
		tcas_msg_t msg = ri->msg;
		tcas_RA_t *ra;

		ASSERT3U(ri->msg, >=, 0);
		ASSERT3U(ri->msg, <, RA_NUM_MSGS);

		/* ri must allow initial/subseq ann as necessary */
		if ((initial && !ri->initial) || (!initial && !ri->subseq)) {
			dbg_log(ra, 4, "CULLRI(slow) init/subseq "
			    PRINTF_RI_FMT, PRINTF_RI_ARGS(ri));
			continue;
		}
		/* ri must satisfy input VS condition */
		if (my_acf->vvel < ri->vs.in.min ||
		    my_acf->vvel > ri->vs.in.max) {
			dbg_log(ra, 4, "CULLRI(norm) in.vs " PRINTF_RI_FMT,
			    PRINTF_RI_ARGS(ri));
			continue;
		}
		/* if performing a sense reversal, ri must allow it */
		if (reversal && (int)ri->rev_msg == -1) {
			dbg_log(ra, 4, "CULLRI(slow) REV "
			    PRINTF_RI_FMT, PRINTF_RI_ARGS(ri));
			continue;
		}
		/* Above FL480 we want to inhibit climb RAs. */
		if (my_acf->cur_pos.elev > INHIBIT_CLB_RA &&
		    ri->sense == RA_SENSE_UPWARD) {
			dbg_log(ra, 4, "CULLRI(slow) CLB(FL480/NOW) "
			    PRINTF_RI_FMT, PRINTF_RI_ARGS(ri));
			continue;
		}
		/* Inhibit INCREASE DESCENT RAs below 1550ft AGL. */
		if(my_acf->agl < INHIBIT_INC_DES_AGL &&
		    msg == RA_MSG_DES_MORE) {
			dbg_log(ra, 4, "CULLRI(slow) INCDES(1550/NOW) "
			    PRINTF_RI_FMT, PRINTF_RI_ARGS(ri));
			continue;
		}
		/* Inhibit all DESCEND RAs below 1100ft AGL. */
		if (my_acf->agl < INHIBIT_DES_AGL &&
		    ri->sense == RA_SENSE_DOWNWARD) {
			dbg_log(ra, 4, "CULLRI(slow) DES(1100) "
			    PRINTF_RI_FMT, PRINTF_RI_ARGS(ri));
			continue;
		}
		/*
		 * Pick an RA that makes sense from a sequencing POV. We
		 * want to prevent a CLB MORE RA from reducing to a
		 * regular CLB or CROSS CLB RA. In those cases, we want
		 *  to either LEVEL OFF or perform a sense reversal.
		 */
		if (prev_msg == RA_MSG_CLB_MORE && (msg == RA_MSG_CLB ||
		    msg == RA_MSG_CLB_CROSS)) {
			dbg_log(ra, 4, "CULLRI(slow) CLBMORE->CLB "
			    PRINTF_RI_FMT, PRINTF_RI_ARGS(ri));
			continue;
		}
		/* Same as above, but for DES_MORE */
		if (prev_msg == RA_MSG_DES_MORE && (msg == RA_MSG_DES ||
		    msg == RA_MSG_DES_CROSS)) {
			dbg_log(ra, 4, "CULLRI(norm) DESMORE->DES "
			    PRINTF_RI_FMT, PRINTF_RI_ARGS(ri));
			continue;
		}
		/* Inhibit PREVENTIVE or LEVEL OFF RAs for slow encounters */
		if (ri->type == RA_TYPE_PREVENTIVE ||
		    ri->sense == RA_SENSE_LEVEL_OFF) {
			dbg_log(ra, 4, "CULLRI(slow) BADRA "
			    PRINTF_RI_FMT, PRINTF_RI_ARGS(ri));
			continue;
		}

		ra = ra_construct(my_acf, ri, cpas, sl, delay_t, accel,
		    reversal);
		if (my_acf->vvel < ri->vs.out.min) {
			ra->vs_corr_reqd = roundmul(ri->vs.out.min -
			    my_acf->vvel, ALT_ROUND_MUL);
		} else if (my_acf->vvel > ri->vs.out.max) {
			ra->vs_corr_reqd = roundmul(ri->vs.out.max -
			    my_acf->vvel, ALT_ROUND_MUL);
		}

		dbg_log(ra, 4, "ADD(slow) " PRINTF_RA_FMT, PRINTF_RA_ARGS(ra));
		avl_add(prio, ra);
	}
}

static tcas_RA_t *
CAS_logic(const tcas_acf_t *my_acf, const tcas_RA_t *prev_ra, avl_tree_t *cpas,
    const SL_t *sl, bool_t prev_only, bool_t slow_closure)
{
	bool_t initial = (prev_ra == NULL);
	const cpa_t *last_cpa = avl_last(cpas);
	avl_tree_t prio;
	tcas_RA_t *ra, *xra;
	void *cookie;

	ASSERT(last_cpa != NULL);

	/*
	 * Don't try subsequent RAs if we're too close to the last CPA anyway.
	 */
	if (!initial && last_cpa->d_t < SUBSEQ_RA_DELAY)
		return (NULL);

	if (!slow_closure) {
		CAS_logic_normal(my_acf, prev_ra, cpas, sl, prev_only, &prio);
		/*
		 * We are not guaranteed to find a suitable preventive RA if
		 * the preventive RA has vertical speed ranges that might cause
		 * us to dip into the alim threshold. In those cases, try to
		 * look for a corrective RA instead.
		 */
		if (prev_only && avl_numnodes(&prio) == 0) {
			avl_destroy(&prio);
			CAS_logic_normal(my_acf, prev_ra, cpas, sl, B_FALSE,
			    &prio);
		}
	} else {
		CAS_logic_slow(my_acf, prev_ra, cpas, sl, &prio);
	}

	ASSERT(avl_numnodes(&prio) != 0 || !initial);

	if (xtcas_dbg.ra >= 2) {
		for (tcas_RA_t *ra = avl_first(&prio); ra != NULL;
		    ra = AVL_NEXT(&prio, ra)) {
			dbg_log(ra, 2, "AVAIL " PRINTF_RA_FMT,
			    PRINTF_RA_ARGS(ra));
		}
	}

	ra = avl_first(&prio);
	if (ra != NULL)
		avl_remove(&prio, ra);
	cookie = NULL;
	while ((xra = avl_destroy_nodes(&prio, &cookie)) != NULL)
		free(xra);
	avl_destroy(&prio);

	/*
	 * Now that we have an RA, we need to determine if it's sensible
	 * given the previously issued RA.
	 */
	if (prev_ra != NULL && ra != NULL) {
		/* Don't re-issue the same RA type with unchanged limits. */
		if (ra->info->type == prev_ra->info->type &&
		    ra->info->vs.out.min == prev_ra->info->vs.out.min &&
		    ra->info->vs.out.max == prev_ra->info->vs.out.max) {
			free(ra);
			return (NULL);
		}
	}

	return (ra);
}

static void
destroy_RA_hints(avl_tree_t *RA_hints)
{
	void *cookie = NULL;
	tcas_RA_hint_t *hint;
	while ((hint = avl_destroy_nodes(RA_hints, &cookie)) != NULL)
		free(hint);
}

static void
construct_RA_hints(avl_tree_t *RA_hints, avl_tree_t *RA_cpas)
{
	ASSERT(tcas_state.adv_state == ADV_STATE_RA ||
	    avl_numnodes(RA_cpas) == 0);
	for (cpa_t *cpa = avl_first(RA_cpas); cpa != NULL;
	    cpa = AVL_NEXT(RA_cpas, cpa)) {
		tcas_RA_hint_t *hint = safe_calloc(1, sizeof (*hint));
		hint->acf_id = cpa->acf_b->acf_id;
		hint->level = cpa->acf_b->threat;
		ASSERT3U(hint->level, >=, RA_THREAT_PREV);
		hint->slow_closure = cpa->acf_b->slow_closure;
		avl_add(RA_hints, hint);
	}
}

#if	GTS820_MODE

/*
 * Builds and plays the messages for a particular set of new TA threats using
 * the GTS820 annunciation method ("Traffic", "<relative bearing>",
 * "<high|low|same altitude>", "<distance in NM>").
 */
static void
gts820_TA_play_msg(const tcas_acf_t *my_acf, const list_t *new_TA_threats)
{
	vect2_t my_pos_2d = VECT3_TO_VECT2(my_acf->cur_pos_3d);
	tcas_msg_t *msgs;
	unsigned i, n;

	CTASSERT(GTS820_MSG_12CLK - GTS820_MSG_1CLK == 11);
	CTASSERT(GTS820_MSG_P10NM - GTS820_MSG_M1NM == 11);

	i = 0;
	n = (4 * list_count(new_TA_threats)) + 1;
	msgs = safe_calloc(n, sizeof (*msgs));
	msgs[n - 1] = -1u;

	for (tcas_acf_t *acf = list_head(new_TA_threats); acf != NULL;
	    acf = list_next(new_TA_threats, acf)) {
		vect2_t pos_2d = VECT3_TO_VECT2(acf->cur_pos_3d);
		vect2_t d_pos_2d = vect2_sub(pos_2d, my_pos_2d);
		double rdist = vect2_abs(d_pos_2d);
		double rbrg = (rdist > 0 ? normalize_hdg(dir2hdg(d_pos_2d) -
		    my_acf->hdg) : 0);
		double ralt = acf->cur_pos_3d.z - my_acf->cur_pos_3d.z;
		int sector = clampi(normalize_hdg(rbrg - (360 / 24)) /
		    (360 / 12), 0, 11);
		tcas_msg_t sector_msg = GTS820_MSG_1CLK + sector;
		tcas_msg_t alt_msg, dist_msg;
		int dist_nm;

		if (ralt > FEET2MET(300))
			alt_msg = GTS820_MSG_HIGH;
		else if (ralt < FEET2MET(-300))
			alt_msg = GTS820_MSG_LOW;
		else
			alt_msg = GTS820_MSG_SAME_ALT;
		dist_nm = clampi(MET2NM(rdist), 0, 11);
		dist_msg = GTS820_MSG_M1NM + dist_nm;

		msgs[i++] = RA_MSG_TFC;
		msgs[i++] = sector_msg;
		msgs[i++] = alt_msg;
		msgs[i++] = dist_msg;
	}

	xtcas_play_msgs(msgs);

	free(msgs);
}

#endif	/* GTS820_MODE */

static void
resolve_CPAs(tcas_acf_t *my_acf, avl_tree_t *other_acf, avl_tree_t *cpas,
    const SL_t *sl, avl_tree_t *RA_hints, uint64_t now)
{
	bool_t TA_found = B_FALSE;
	bool_t RA_prev_found = B_FALSE;
	bool_t RA_corr_found = B_FALSE;
	bool_t slow_closure_only = B_TRUE;
	avl_tree_t RA_cpas;
	void *cookie;
	list_t new_TA_threats;

	/*
	 * List of newly discovered TA threats during this cycle. This is
	 * used by the GTS820 mode to annunciate individual new TA threats.
	 */
	list_create(&new_TA_threats, sizeof (tcas_acf_t),
	    offsetof(tcas_acf_t, new_TA_node));

	/* Re-assign threat level as necessary. */
	for (tcas_acf_t *acf = avl_first(other_acf); acf != NULL;
	    acf = AVL_NEXT(other_acf, acf)) {
		bool_t non_TA = (acf->threat < TA_THREAT);

		assign_threat_level(my_acf, acf, sl, RA_hints,
		    tcas_state.filter, now);

		TA_found |= (acf->threat == TA_THREAT);
		RA_prev_found |= (acf->threat == RA_THREAT_PREV);
		RA_corr_found |= (acf->threat == RA_THREAT_CORR);
		if (acf->threat == RA_THREAT_PREV ||
		    acf->threat == RA_THREAT_CORR)
			slow_closure_only &= acf->slow_closure;
		if (non_TA && acf->threat >= TA_THREAT) {
			list_insert_tail(&new_TA_threats, acf);
			acf->ta_time = now;
		}
	}

	avl_create(&RA_cpas, cpa_compar, sizeof (cpa_t),
	    offsetof(cpa_t, ra_node));

	if (RA_corr_found || RA_prev_found) {
		/*
		 * We've located RA threats (either from direct CPAs or via
		 * RA_hints or via a violation of dmod+zthr for preventive
		 * encounters).
		 */
		tcas_RA_t *ra;
		double d_t = NAN;

		for (cpa_t *cpa = avl_first(cpas); cpa != NULL;
		    cpa = AVL_NEXT(cpas, cpa)) {
			if (cpa->acf_b->threat <= TA_THREAT)
				continue;
			if (isnan(d_t))
				d_t = cpa->d_t;
			ASSERT3F(d_t, <=, cpa->d_t);
			if (d_t <= cpa->d_t + INITIAL_RA_DELAY)
				avl_add(&RA_cpas, cpa);
		}
		ASSERT(!isnan(d_t));
		ASSERT(avl_numnodes(&RA_cpas) != 0);

		dbg_log(ra, 1, "resolve_CPAs: RA  count:%lu  adv_state:%d  "
		    "elapsed:%.0f", avl_numnodes(&RA_cpas),
		    tcas_state.adv_state,
		    (now - tcas_state.change_t) / 1000000.0);

		ra = CAS_logic(my_acf, tcas_state.ra, &RA_cpas, sl,
		    /*
		     * A preventive RA is only guaranteed to be found when
		     * climbing/descending below the maximum preventive RA
		     * vertical rate value.
		     */
		    RA_prev_found && !RA_corr_found &&
		    ABS(my_acf->vvel) < FPM2MPS(2000), slow_closure_only);
		/* On initial annunciation, we must ALWAYS issue an RA */
		ASSERT(ra != NULL || tcas_state.ra != NULL);

		if (tcas_state.adv_state != ADV_STATE_RA) {
			/* memorize what VS we started at */
			tcas_state.initial_ra_vs = my_acf->vvel;
		}

		if (ra != NULL) {
			dbg_log(ra, 1, "SELECTED RA: " PRINTF_RA_FMT,
			    PRINTF_RA_ARGS(ra));
		}

		if (ra != NULL) {
			bool_t inhibit_audio;
			if (out_ops != NULL &&
			    out_ops->update_RA_prediction != NULL) {
				out_ops->update_RA_prediction(out_ops->handle,
				    ra->info->msg, ra->info->type,
				    ra->info->sense, ra->crossing,
				    ra->reversal, ra->min_sep);
			}
			if (now - tcas_state.change_t >= STATE_CHG_DELAY) {
				tcas_msg_t prev_msg = -1;
				tcas_msg_t msg;
				const tcas_RA_info_t *ri = ra->info;
				double min_green = 0, max_green = 0;
				if (tcas_state.ra != NULL) {
					prev_msg = tcas_state.ra->info->msg;
					free(tcas_state.ra);
				}
				tcas_state.ra = ra;
				tcas_state.change_t = now;
				tcas_state.adv_state = ADV_STATE_RA;
				/* Filter out pointless annunciations */
				msg = RA_msg_sequence_check(prev_msg,
				    ra->reversal ? ra->info->rev_msg :
				    ra->info->msg);
#if	GTS820_MODE
				if (!isnan(my_acf->agl)) {
					inhibit_audio = (my_acf->agl <
					    INHIBIT_AUDIO || my_acf->on_ground);
				} else {
					inhibit_audio = my_acf->gear_ext;
				}
#else	/* !GTS820_MODE */
				/*
				 * In case the RA fails, we use the strut
				 * switch.
				 */
				inhibit_audio = (my_acf->agl < INHIBIT_AUDIO ||
				    my_acf->on_ground);
#endif	/* !GTS820_MODE */
				if ((int)msg != -1 && !inhibit_audio) {
#ifndef	XTCAS_NO_AUDIO
					xtcas_play_msg(msg);
#endif
					if (out_ops != NULL &&
					    out_ops->play_audio_msg != NULL) {
						out_ops->play_audio_msg(
						    out_ops->handle,
						    msg);
					}
				}
				if (ra->info->type == RA_TYPE_CORRECTIVE) {
					min_green = ra->info->vs.out.min;
					max_green = ra->info->vs.out.max;
				}
				if (out_ops != NULL) {
					out_ops->update_RA(out_ops->handle,
					    ADV_STATE_RA, msg, ri->type,
					    ri->sense, ra->crossing,
					    ra->reversal, ra->min_sep,
					    min_green, max_green,
					    ra->info->vs.red_lo.min,
					    ra->info->vs.red_lo.max,
					    ra->info->vs.red_hi.min,
					    ra->info->vs.red_hi.max);
				}
			} else {
				/*
				 * Avoid attempting to generate RA hints yet,
				 * since we have transitioned to a TRAFFIC or
				 * clear-of-conflict state too recently. On
				 * next cycle, we will retry.
				 */
				cookie = NULL;
				while ((avl_destroy_nodes(&RA_cpas, &cookie)) !=
				    NULL)
					;
				free(ra);
			}
		}
	} else if (TA_found) {
		/*
		 * TRAFFIC is annunciated only from the NONE state and after
		 * the minimum state change delay has passed. This prevents
		 * annunciating TRAFFIC after an RA has been resolved, but
		 * while the traffic still falls into the TA range.
		 */
		dbg_log(tcas, 1, "resolve_CPAs: TA  adv_state:%d  "
		    "elapsed:%.0f", tcas_state.adv_state,
		    (now - tcas_state.change_t) / 1000000.0);
		if (
#if	GTS820_MODE
		    list_count(&new_TA_threats) != 0
#else	/* !GTS820_MODE */
		    tcas_state.adv_state < ADV_STATE_TA &&
		    now - tcas_state.change_t >= STATE_CHG_DELAY
#endif	/* !GTS820_MODE */
		    ) {
			if (my_acf->agl > INHIBIT_AUDIO || isnan(my_acf->agl)) {
#if	GTS820_MODE
				gts820_TA_play_msg(my_acf, &new_TA_threats);
#else	/* !GTS820_MODE */
#ifndef	XTCAS_NO_AUDIO
				xtcas_play_msg(RA_MSG_TFC);
#endif
				if (out_ops != NULL &&
				    out_ops->play_audio_msg != NULL) {
					out_ops->play_audio_msg(out_ops->handle,
					    RA_MSG_TFC);
				}
#endif	/* !GTS820_MODE */
			}
			free(tcas_state.ra);
			tcas_state.ra = NULL;
			tcas_state.initial_ra_vs = NAN;
			tcas_state.adv_state = ADV_STATE_TA;
			if (out_ops != NULL) {
				out_ops->update_RA(out_ops->handle,
				    ADV_STATE_TA, RA_MSG_TFC, -1, -1, B_FALSE,
				    B_FALSE, 0, 0, 0, 0, 0, 0, 0);
			}
		}
	} else if (tcas_state.adv_state != ADV_STATE_NONE &&
	    now - tcas_state.change_t >= STATE_CHG_DELAY) {
		dbg_log(tcas, 1, "resolve_CPAs: NONE  adv_state:%d  "
		    "elapsed:%.0f", tcas_state.adv_state,
		    (now - tcas_state.change_t) / 1000000.0);
		if (tcas_state.adv_state == ADV_STATE_RA) {
#ifndef	XTCAS_NO_AUDIO
			xtcas_play_msg(RA_MSG_CLEAR);
#endif	/* !defined(XTCAS_NO_AUDIO) */
			if (out_ops != NULL &&
			    out_ops->play_audio_msg != NULL) {
				out_ops->play_audio_msg(out_ops->handle,
				    RA_MSG_CLEAR);
			}
		}
		free(tcas_state.ra);
		if (out_ops != NULL) {
			out_ops->update_RA(out_ops->handle, ADV_STATE_NONE,
			    RA_MSG_CLEAR, -1, -1, B_FALSE, B_FALSE,
			    0, 0, 0, 0, 0, 0, 0);
		}
		tcas_state.ra = NULL;
		tcas_state.initial_ra_vs = NAN;
		tcas_state.adv_state = ADV_STATE_NONE;
		tcas_state.change_t = now;
	}

	/*
	 * Reconstruct the RA hints so we know which contacts need to be
	 * hard-marked as RA threats next time.
	 */
	destroy_RA_hints(RA_hints);
	construct_RA_hints(RA_hints, &RA_cpas);

	cookie = NULL;
	while ((avl_destroy_nodes(&RA_cpas, &cookie)) != NULL)
		;
	avl_destroy(&RA_cpas);

	while (list_remove_head(&new_TA_threats) != NULL)
		;
	list_destroy(&new_TA_threats);
}

static void
update_contacts(tcas_acf_t *my_acf, avl_tree_t *other_acf, bool_t test)
{
	vect2_t my_pos_2d = VECT3_TO_VECT2(my_acf->cur_pos_3d);

	/*
	 * Badly behaved multiplayer plugins such as XSquawkBox tend not
	 * to delete unused multiplayer aircraft, so they just sit in
	 * space, stationary. Detect and remove those.
	 */
	for (tcas_acf_t *acf = avl_first(other_acf); acf != NULL;
	    acf = AVL_NEXT(other_acf, acf)) {
		if (acf->gs < FALSE_CTC_SUPPRESS_GS && !test &&
		    out_ops != NULL)
			out_ops->delete_contact(out_ops->handle, acf->acf_id);
	}

	if (tcas_state.filter == TCAS_FILTER_THRT &&
	    tcas_state.adv_state == ADV_STATE_NONE && !test) {
		for (tcas_acf_t *acf = avl_first(other_acf); acf != NULL;
		    acf = AVL_NEXT(other_acf, acf)) {
			if (out_ops != NULL) {
				out_ops->delete_contact(out_ops->handle,
				    acf->acf_id);
			}
		}
	} else {
		for (tcas_acf_t *acf = avl_first(other_acf); acf != NULL;
		    acf = AVL_NEXT(other_acf, acf)) {
			if (out_ops == NULL)
				continue;
			if (!acf->on_ground) {
				vect2_t pos_2d =
				    VECT3_TO_VECT2(acf->cur_pos_3d);
				vect2_t d_pos_2d = vect2_sub(pos_2d, my_pos_2d);
				double rdist = vect2_abs(d_pos_2d);
				double rbrg = (rdist > 0 ?
				    normalize_hdg(dir2hdg(d_pos_2d) -
				    my_acf->hdg) : 0);
				double ralt = acf->cur_pos_3d.z -
				    my_acf->cur_pos_3d.z;
				out_ops->update_contact(out_ops->handle,
				    acf->acf_id, rbrg, rdist, ralt,
				    acf->trend_data_ready ? acf->vvel : NAN,
				    acf->trend_data_ready ? acf->trk : NAN,
				    acf->trend_data_ready ? acf->gs : NAN,
				    acf->threat);
			} else {
				out_ops->delete_contact(out_ops->handle,
				    acf->acf_id);
			}
		}
	}
}

static void
main_loop(void *ignored)
{
	const SL_t *sl = NULL;
	double last_t = in_ops->get_time(in_ops->handle);
	avl_tree_t RA_hints;

	thread_set_name("X-TCAS");

	dbg_log(tcas, 4, "main_loop: entry (%.1f)", last_t);

	UNUSED(ignored);
	ASSERT(inited);

	avl_create(&RA_hints, RA_hint_compar, sizeof (tcas_RA_hint_t),
	    offsetof(tcas_RA_hint_t, node));

	mutex_enter(&worker_lock);
	for (double now = microclock(); !worker_shutdown; now = microclock()) {
		tcas_acf_t my_acf;
		avl_tree_t other_acf, cpas;
		double now_t = in_ops->get_time(in_ops->handle);
		bool_t test;

		dbg_log(tcas, 4, "main_loop: start (%.1f)", now_t);

		/* If sim time hasn't advanced, we're paused. */
		if (last_t >= now_t) {
			dbg_log(tcas, 3, "main_loop: time hasn't progressed "
			    "or STBY mode set (%d)", tcas_state.mode);
			cv_timedwait(&worker_cv, &worker_lock,
			    now + WORKER_LOOP_INTVAL_US);
			continue;
		}

		mutex_enter(&tcas_state.test_lock);

		if (tcas_state.test_in_prog) {
			if (isnan(tcas_state.test_start_time)) {
				tcas_state.test_start_time = now_t;

				if (out_ops != NULL) {
					/*
					 * During a system test, we give a
					 * normal climb indication on the PFD.
					 */
#if	GTS820_MODE
					out_ops->update_RA(out_ops->handle,
					    ADV_STATE_TA, RA_MSG_TFC,
					    -1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0);
#elif	VSI_DRAW_MODE
					out_ops->update_RA(out_ops->handle,
					    ADV_STATE_RA, RA_MSG_CLB,
					    RA_TYPE_CORRECTIVE, RA_SENSE_UPWARD,
					    B_FALSE, B_FALSE, 0, FPM2MPS(0),
					    FPM2MPS(300),
					    -INF_VS, FPM2MPS(0),
					    FPM2MPS(2000), INF_VS);
#else	/* !VSI_DRAW_MODE */
					out_ops->update_RA(out_ops->handle,
					    ADV_STATE_RA, RA_MSG_CLB,
					    RA_TYPE_CORRECTIVE, RA_SENSE_UPWARD,
					    B_FALSE, B_FALSE, 0,
					    0, FPM2MPS(300),
					    -INF_VS, 0, FPM2MPS(1500), INF_VS);
#endif	/* !VSI_DRAW_MODE */
				}
			} else if (now_t - tcas_state.test_start_time >
			    TCAS_TEST_DUR) {
				/* Remove the fake test contacts */
				if (out_ops != NULL) {
					for (uintptr_t i = 1; i <= NUM_TEST_CTC;
					    i++) {
						out_ops->delete_contact(
						    out_ops->handle, (void *)i);
					}
					out_ops->update_RA(out_ops->handle,
					    ADV_STATE_NONE, RA_MSG_CLEAR, -1,
					    -1, B_FALSE, B_FALSE, 0, 0, 0, 0,
					    0, 0, 0);
				}
				tcas_state.test_start_time = NAN;
				tcas_state.test_in_prog = B_FALSE;
#ifndef	XTCAS_NO_AUDIO
				xtcas_play_msg(TCAS_TEST_PASS);
#endif
				if (out_ops != NULL &&
				    out_ops->play_audio_msg != NULL) {
					out_ops->play_audio_msg(out_ops->handle,
					    TCAS_TEST_PASS);
				}
			}
		}
		test = tcas_state.test_in_prog;

		mutex_exit(&tcas_state.test_lock);

		last_t = now_t;

		/*
		 * We'll create a local copy of all aircraft positions so
		 * we don't have to hold acf_lock throughout.
		 */
		copy_acf_state(&my_acf, &other_acf, test);

		/*
		 * Based on our altitudes, determine the sensitivity level.
		 * SL change is prevented while in an RA to avoid excessive
		 * RA switching. TA-only mode always selects SL2.
		 */
		if (sl == NULL || tcas_state.adv_state != ADV_STATE_RA) {
			sl = xtcas_SL_select(sl != NULL ? sl->SL_id : 1,
			    my_acf.cur_pos.elev, my_acf.agl,
#if	GTS820_MODE
			    0,
#else
			    tcas_state.mode == TCAS_MODE_TAONLY ? 2 : 0,
#endif
			    !my_acf.has_RA && my_acf.gear_ext);
			dbg_log(sl, 1, "SL: %d", sl->SL_id);
			xtcas_SL = sl->SL_id;
		}

		/*
		 * Determine the CPA for each bogie and place them in the
		 * correct time order.
		 */
		compute_CPAs(&cpas, &my_acf, &other_acf);

		/*
		 * Enter the resolution phase and check if we need to do
		 * anything. This is the main function where we issue TAs
		 * and RAs.
		 * In the test case, the contacts are already generated with
		 * the appropriate threat levels assigned, so we don't need
		 * to do any more resolution.
		 */
		if (!test) {
			resolve_CPAs(&my_acf, &other_acf, &cpas, sl,
			    &RA_hints, now);
		}

		/*
		 * Update the avionics on the threat status of all the
		 * contacts that we have.
		 */
		update_contacts(&my_acf, &other_acf, test);

		destroy_CPAs(&cpas);

		/*
		 * Dispose of the local position copy.
		 */
		destroy_acf_state(&other_acf);

		dbg_log(tcas, 5, "main_loop: end");

		/*
		 * Jump forward at fixed intervals to guarantee our
		 * execution schedule.
		 */
		do {
			cv_timedwait(&worker_cv, &worker_lock,
			    now + WORKER_LOOP_INTVAL_US);
		} while (microclock() < now + WORKER_LOOP_INTVAL_US &&
		    !worker_shutdown);
	}
	mutex_exit(&worker_lock);

	dbg_log(tcas, 4, "shutdown");

	destroy_RA_hints(&RA_hints);
}

void
xtcas_run(void)
{
	double t = in_ops->get_time(in_ops->handle);

	dbg_log(tcas, 4, "run: %.1f", t);

	ASSERT(inited);

	/* protection in case the sim is paused */
	if (t < last_collect_t + WORKER_LOOP_INTVAL) {
		dbg_log(tcas, 5, "run: not enough time has elapsed "
		    "(t: %.1f last: %.1f)", t, last_collect_t);
		return;
	}
	last_collect_t = t;

	mutex_enter(&acf_lock);
	update_my_position(t);
	update_bogie_positions(t, my_acf_glob.cur_pos, my_acf_glob.agl);

	if (!my_acf_glob.trend_data_ready) {
		mutex_exit(&acf_lock);
		return;
	}
	mutex_exit(&acf_lock);
}

void
xtcas_init(const sim_intf_input_ops_t *intf_input_ops,
    const sim_intf_output_ops_t *intf_output_ops)
{
	dbg_log(tcas, 1, "init");

	inited = B_TRUE;
	worker_shutdown = B_FALSE;

	ASSERT(intf_input_ops != NULL);
	ASSERT(intf_input_ops->get_time != NULL);
	ASSERT(intf_input_ops->get_my_acf_pos != NULL);
	ASSERT(intf_input_ops->get_oth_acf_pos != NULL);
	if (intf_output_ops != NULL) {
		ASSERT(intf_output_ops->update_contact != NULL);
		ASSERT(intf_output_ops->delete_contact != NULL);
		ASSERT(intf_output_ops->update_RA != NULL);
	}

	memset(&my_acf_glob, 0, sizeof (my_acf_glob));
	avl_create(&other_acf_glob, acf_compar,
	    sizeof (tcas_acf_t), offsetof(tcas_acf_t, node));
	mutex_init(&acf_lock);

	memset(&tcas_state, 0, sizeof (tcas_state));
	tcas_state.initial_ra_vs = NAN;
	mutex_init(&tcas_state.test_lock);
	tcas_state.test_start_time = NAN;

	in_ops = intf_input_ops;
	out_ops = intf_output_ops;

	mutex_init(&worker_lock);
	cv_init(&worker_cv);
	VERIFY(thread_create(&worker_thr, main_loop, NULL));
}

void
xtcas_fini(void)
{
	dbg_log(tcas, 1, "fini");

	mutex_enter(&worker_lock);
	worker_shutdown = B_TRUE;
	cv_broadcast(&worker_cv);
	mutex_exit(&worker_lock);
	thread_join(&worker_thr);

	destroy_acf_state(&other_acf_glob);

	mutex_destroy(&tcas_state.test_lock);
	mutex_destroy(&acf_lock);

	cv_destroy(&worker_cv);
	mutex_destroy(&worker_lock);

	free(tcas_state.ra);

	inited = B_FALSE;
}

void
xtcas_set_mode(tcas_mode_t mode)
{
	tcas_state.mode = mode;
}

tcas_mode_t
xtcas_get_mode(void)
{
	return (tcas_state.mode);
}

tcas_mode_t
xtcas_get_mode_act(void)
{
	if (tcas_state.mode == TCAS_MODE_TARA && xtcas_get_SL() <= 2)
		return (TCAS_MODE_TAONLY);
	return (tcas_state.mode);
}

void
xtcas_set_filter(tcas_filter_t filter)
{
	tcas_state.filter = filter;
}

tcas_filter_t
xtcas_get_filter(void)
{
	return (tcas_state.filter);
}

int
xtcas_get_SL(void)
{
	return (xtcas_SL);
}

void
xtcas_set_has_RA(bool_t flag)
{
	if (inited)
		mutex_enter(&acf_lock);
	my_acf_glob.has_RA = flag;
	if (inited)
		mutex_exit(&acf_lock);
}

void
xtcas_set_has_WOW(bool_t flag)
{
	if (inited)
		mutex_enter(&acf_lock);
	my_acf_glob.has_WOW = flag;
	if (inited)
		mutex_exit(&acf_lock);
}

void
xtcas_set_RA(double agl_hgt_m)
{
	if (inited)
		mutex_enter(&acf_lock);
	my_acf_glob.has_RA = !isnan(agl_hgt_m);
	my_acf_glob.agl = agl_hgt_m;
	my_acf_glob.custom_RA = B_TRUE;
	if (inited)
		mutex_exit(&acf_lock);
}

void
xtcas_set_WOW(bool_t on_ground)
{
	if (inited)
		mutex_enter(&acf_lock);
	my_acf_glob.has_WOW = B_TRUE;
	my_acf_glob.on_ground = on_ground;
	my_acf_glob.custom_WOW = B_TRUE;
	if (inited)
		mutex_exit(&acf_lock);
}

void
xtcas_set_gear_ext(bool_t gear_ext)
{
	if (inited)
		mutex_enter(&acf_lock);
	my_acf_glob.gear_ext = gear_ext;
	my_acf_glob.custom_gear_ext = B_TRUE;
	if (inited)
		mutex_exit(&acf_lock);
}

static bool_t
xtcas_test_check(const char **reason)
{
	ASSERT(reason != NULL);

	if (tcas_state.mode != TCAS_MODE_STBY) {
		*reason = "TCAS mode not STBY";
		return (B_FALSE);
	}
	if (!my_acf_glob.has_RA) {
		*reason = "RA INOP";
		return (B_FALSE);
	}
	return (B_TRUE);
}

void
xtcas_test(bool_t force_fail)
{
	const char *reason = NULL;

	if (force_fail) {
#ifndef	XTCAS_NO_AUDIO
		xtcas_play_msg(TCAS_TEST_FAIL);
#endif
		if (out_ops != NULL && out_ops->play_audio_msg != NULL) {
			out_ops->play_audio_msg(out_ops->handle,
			    TCAS_TEST_FAIL);
		}
		return;
	}
	if (!xtcas_test_check(&reason)) {
		logMsg("TCAS test fail: %s", reason);
#ifndef	XTCAS_NO_AUDIO
		xtcas_play_msg(TCAS_TEST_FAIL);
#endif
		if (out_ops != NULL && out_ops->play_audio_msg != NULL) {
			out_ops->play_audio_msg(out_ops->handle,
			    TCAS_TEST_FAIL);
		}
		return;
	}
	mutex_enter(&tcas_state.test_lock);
	tcas_state.test_in_prog = B_TRUE;
	mutex_exit(&tcas_state.test_lock);
}

bool_t
xtcas_test_is_in_prog(void)
{
	return (tcas_state.test_in_prog);
}

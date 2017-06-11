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

#include "assert.h"
#include "avl.h"
#include "helpers.h"
#include "geom.h"
#include "log.h"
#include "math.h"
#include "pos.h"
#include "snd_sys.h"
#include "SL.h"
#include "thread.h"
#include "time.h"

#include "xtcas.h"

#define	ALT_ROUND_MUL		FEET2MET(100)	/* altitude rouding multiple */
#define	EQ_ALT_THRESH		FEET2MET(100)	/* equal altitude threshold */
#define	LEVEL_VVEL_THRESH	FPM2MPS(300)
#define	NUM_RA_INFOS		24

#define	WORKER_LOOP_INTVAL	SEC2USEC(1)	/* microseconds */
#define	STATE_CHG_DELAY		SEC2USEC(4)	/* microseconds */
#define	EARTH_G			9.81		/* m.s^-2 */
#define	INITIAL_RA_D_VVEL	(EARTH_G / 4)	/* 1/4 g */
#define	INITIAL_RA_DELAY	5.0		/* seconds */
#define	SUBSEQ_RA_D_VVEL	(EARTH_G / 3)	/* 1/3 g */
#define	SUBSEQ_RA_DELAY		2.5		/* seconds */

#define	NORM_VERT_FILTER	FEET2MET(9900)	/* Used for the ABV and BLW */
#define	SHORT_VERT_FILTER	FEET2MET(2700)	/* vertical filter modes */

#define	OTH_TFC_DIST_THRESH		NM2MET(40)
#define	PROX_DIST_THRESH		NM2MET(6)
#define	PROX_ALT_THRESH			FEET2MET(1200)
#define	ON_GROUND_AGL_CHK_THRESH	FEET2MET(1700)

#define	PRINTF_ACF_FMT "alt_rptg:%d  pos:%3.04f/%2.4f/%4.1f  agl:%.0f  " \
	"gs:%.1f  trk:%.0f  trk_v:%.2fx%.2f  vvel:%.1f  ongnd:%d"
#define	PRINTF_ACF_ARGS(acf) \
	(acf)->alt_rptg, (acf)->cur_pos.lat, (acf)->cur_pos.lon, \
	(acf)->cur_pos.elev, (acf)->agl, (acf)->gs, (acf)->trk, \
	(acf)->trk_v.x, (acf)->trk_v.y, (acf)->vvel, (acf)->on_ground

typedef enum {
	OTH_THREAT,		/* other traffic, empty diamond */
	PROX_THREAT,		/* proximate traffic, filled diamond */
	TA_THREAT,		/* traffic advisory, filled yellow circle */
	RA_THREAT		/* resolution advisory, filled red square */
} threat_t;

typedef struct cpa cpa_t;

typedef struct tcas_acf {
	void	*acf_id;	/* identifier - used for locating in tree */
	obj_pos_t pos_upd;	/* position updates */
	geo_pos3_t cur_pos;	/* current position */
	vect3_t	cur_pos_3d;	/* current position relative to our aircraft */
	bool_t	alt_rptg;	/* target altitude is available */
	double	agl;		/* altitude above ground level in meters */
	double	gs;		/* true groundspeed (horizontal) in m/s */
	double	trk;		/* true track in degrees */
	vect2_t	trk_v;		/* true track as unit vector */
	double	vvel;		/* computed vertical velocity in m/s */
	bool_t	trend_data_ready; /* indicates if gs/trk/vvel is available */
	bool_t	up_to_date;	/* used for efficient position updates */
	bool_t	on_ground;	/* on-ground condition */
	cpa_t	*cpa;		/* CPA this aircraft participates in */
	threat_t	threat;	/* type of TCAS threat */

	avl_node_t	node;		/* used by other_acf_glob tree */
} tcas_acf_t;

typedef enum {
	ADV_STATE_NONE,
	ADV_STATE_TA,
	ADV_STATE_RA
} tcas_adv_t;

typedef enum {
	RA_SENSE_UPWARD,
	RA_SENSE_LEVEL_OFF,
	RA_SENSE_DOWNWARD
} tcas_RA_sense_t;

typedef enum {
	RA_TYPE_CORRECTIVE,
	RA_TYPE_PREVENTIVE
} tcas_RA_type_t;

typedef enum {
	RA_CROSS_REQ,
	RA_CROSS_REJ,
	RA_CROSS_ANY
} tcas_RA_cross_t;

typedef struct {
	tcas_RA_msg_t	msg, rev_msg;
	bool_t		initial;
	bool_t		subseq;
	tcas_RA_sense_t	sense;
	tcas_RA_type_t	type;
	tcas_RA_cross_t	cross;
	struct {
		struct {
			double min, max;
		} in;
		struct {
			double min, max;
		} out;
	} vs;
} tcas_RA_info_t;

typedef struct {
	const tcas_RA_info_t	*info;
	avl_tree_t		*cpas;
	bool_t			reversal;	/* sense reversal */
	bool_t			crossing;	/* crossing intruder's alt */
	bool_t			alim_achieved;	/* seps are at least ALIM */
	double			min_sep;	/* Minimum among seps */
	double			vs_corr_reqd;	/* required VS correction */
	avl_node_t		node;
} tcas_RA_t;

typedef enum {
	RA_STRENGTH_1,
	RA_STRENGTH_2
} tcas_RA_strength_t;

typedef enum {
	TCAS_MODE_TARA,
	TCAS_MODE_TAONLY
} tcas_mode_t;

typedef enum {
	TCAS_FILTER_NONE,
	TCAS_FILTER_ABV,
	TCAS_FILTER_BLW
} tcas_filter_t;

typedef struct {
	tcas_adv_t	adv_state;
	tcas_RA_t	*ra;
	double		initial_ra_vs;	/* VS when first RA was issued */
	uint64_t	change_t;	/* microclock() timestamp */
	tcas_mode_t	mode;
	tcas_filter_t	filter;
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
	.type = RA_TYPE_PREVENTIVE, .cross = RA_CROSS_REJ,
	.vs.in.min = 0, .vs.in.max = INFINITY,
	.vs.out.min = 0, .vs.out.max = INFINITY,
    },
    {	/* MONITOR VERTICAL SPEED */
	.msg = RA_MSG_MONITOR_VS, .rev_msg = RA_MSG_MONITOR_VS,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_UPWARD,
	.type = RA_TYPE_PREVENTIVE, .cross = RA_CROSS_REJ,
	.vs.in.min = FPM2MPS(-500), .vs.in.max = INFINITY,
	.vs.out.min = FPM2MPS(-500), .vs.out.max = INFINITY
    },
    {	/* MONITOR VERTICAL SPEED */
	.msg = RA_MSG_MONITOR_VS, .rev_msg = RA_MSG_MONITOR_VS,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_UPWARD,
	.type = RA_TYPE_PREVENTIVE, .cross = RA_CROSS_REJ,
	.vs.in.min = FPM2MPS(-1000), .vs.in.max = INFINITY,
	.vs.out.min = FPM2MPS(-1000), .vs.out.max = INFINITY
    },
    {	/* MONITOR VERTICAL SPEED */
	.msg = RA_MSG_MONITOR_VS, .rev_msg = RA_MSG_MONITOR_VS,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_UPWARD,
	.type = RA_TYPE_PREVENTIVE, .cross = RA_CROSS_REJ,
	.vs.in.min = FPM2MPS(-2000), .vs.in.max = INFINITY,
	.vs.out.min = FPM2MPS(-2000), .vs.out.max = INFINITY
    },

/* Preventive descending RAs */
    {	/* MONITOR VERTICAL SPEED */
	.msg = RA_MSG_MONITOR_VS, .rev_msg = RA_MSG_MONITOR_VS,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_DOWNWARD,
	.type = RA_TYPE_PREVENTIVE, .cross = RA_CROSS_REJ,
	.vs.in.min = -INFINITY, .vs.in.max = 0,
	.vs.out.min = -INFINITY, .vs.out.max = 0
    },
    {	/* MONITOR VERTICAL SPEED */
	.msg = RA_MSG_MONITOR_VS, .rev_msg = RA_MSG_MONITOR_VS,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_DOWNWARD,
	.type = RA_TYPE_PREVENTIVE, .cross = RA_CROSS_REJ,
	.vs.in.min = -INFINITY, .vs.in.max = FPM2MPS(500),
	.vs.out.min = -INFINITY, .vs.out.max = FPM2MPS(500)
    },
    {	/* MONITOR VERTICAL SPEED */
	.msg = RA_MSG_MONITOR_VS, .rev_msg = RA_MSG_MONITOR_VS,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_DOWNWARD,
	.type = RA_TYPE_PREVENTIVE, .cross = RA_CROSS_REJ,
	.vs.in.min = -INFINITY, .vs.in.max = FPM2MPS(1000),
	.vs.out.min = -INFINITY, .vs.out.max = FPM2MPS(1000)
    },
    {	/* MONITOR VERTICAL SPEED */
	.msg = RA_MSG_MONITOR_VS, .rev_msg = RA_MSG_MONITOR_VS,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_DOWNWARD,
	.type = RA_TYPE_PREVENTIVE, .cross = RA_CROSS_REJ,
	.vs.in.min = -INFINITY, .vs.in.max = FPM2MPS(2000),
	.vs.out.min = -INFINITY, .vs.out.max = FPM2MPS(2000)
    },

/* Corrective level-off RAs */
    {	/* LEVEL OFF */
	.msg = RA_MSG_LEVEL_OFF, .rev_msg = RA_MSG_LEVEL_OFF,
	.initial = B_TRUE, .subseq = B_TRUE, .sense = RA_SENSE_LEVEL_OFF,
	.type = RA_TYPE_CORRECTIVE, .cross = RA_CROSS_REJ,
	.vs.in.min = 0, .vs.in.max = INFINITY,
	.vs.out.min = -LEVEL_VVEL_THRESH, .vs.out.max = 0
    },
    {	/* LEVEL OFF */
	.msg = RA_MSG_LEVEL_OFF, .rev_msg = RA_MSG_LEVEL_OFF,
	.initial = B_TRUE, .subseq = B_TRUE, .sense = RA_SENSE_LEVEL_OFF,
	.type = RA_TYPE_CORRECTIVE, .cross = RA_CROSS_REJ,
	.vs.in.min = -INFINITY, .vs.in.max = 0,
	.vs.out.min = 0, .vs.out.max = LEVEL_VVEL_THRESH
    },

/* Corrective climbing RAs */
    {	/* MAINTAIN VERTICAL SPEED, CROSSING MAINTAIN - large VS range */
	.msg = RA_MSG_MAINT_VS_CROSS, .rev_msg = -1,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_UPWARD,
	.type = RA_TYPE_CORRECTIVE, .cross = RA_CROSS_REQ,
	.vs.in.min = FPM2MPS(1500), .vs.in.max = FPM2MPS(4400),
	.vs.out.min = FPM2MPS(1500), .vs.out.max = FPM2MPS(4400)
    },
    {	/* MAINTAIN VERTICAL SPEED, MAINTAIN - large VS range */
	.msg = RA_MSG_MAINT_VS, .rev_msg = -1,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_UPWARD,
	.type = RA_TYPE_CORRECTIVE, .cross = RA_CROSS_REJ,
	.vs.in.min = FPM2MPS(1500), .vs.in.max = FPM2MPS(4400),
	.vs.out.min = FPM2MPS(1500), .vs.out.max = FPM2MPS(4400)
    },
    {	/* MAINTAIN VERTICAL SPEED, CROSSING MAINTAIN */
	.msg = RA_MSG_MAINT_VS_CROSS, .rev_msg = -1,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_UPWARD,
	.type = RA_TYPE_CORRECTIVE, .cross = RA_CROSS_REQ,
	.vs.in.min = FPM2MPS(1500), .vs.in.max = FPM2MPS(2000),
	.vs.out.min = FPM2MPS(1500), .vs.out.max = FPM2MPS(2000)
    },
    {	/* MAINTAIN VERTICAL SPEED, MAINTAIN */
	.msg = RA_MSG_MAINT_VS, .rev_msg = -1,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_UPWARD,
	.type = RA_TYPE_CORRECTIVE, .cross = RA_CROSS_REJ,
	.vs.in.min = FPM2MPS(1500), .vs.in.max = FPM2MPS(2000),
	.vs.out.min = FPM2MPS(1500), .vs.out.max = FPM2MPS(2000)
    },
    {	/* CLIMB, CLIMB */
	.msg = RA_MSG_CLB, .rev_msg = RA_MSG_CLB_NOW,
	.initial = B_TRUE, .subseq = B_TRUE, .sense = RA_SENSE_UPWARD,
	.type = RA_TYPE_CORRECTIVE, .cross = RA_CROSS_REJ,
	.vs.in.min = -INFINITY, .vs.in.max = FPM2MPS(1500),
	.vs.out.min = FPM2MPS(1500), .vs.out.max = FPM2MPS(2000)
    },
    {	/* CLIMB, CROSSING CLIMB */
	.msg = RA_MSG_CLB_CROSS, .rev_msg = RA_MSG_CLB_NOW,
	.initial = B_TRUE, .subseq = B_TRUE, .sense = RA_SENSE_UPWARD,
	.type = RA_TYPE_CORRECTIVE, .cross = RA_CROSS_REQ,
	.vs.in.min = -INFINITY, .vs.in.max = FPM2MPS(1500),
	.vs.out.min = FPM2MPS(1500), .vs.out.max = FPM2MPS(2000)
    },
    {	/* INCREASE CLIMB */
	.msg = RA_MSG_CLB_MORE, .rev_msg = -1,
	.initial = B_FALSE, .subseq = B_TRUE, .sense = RA_SENSE_UPWARD,
	.type = RA_TYPE_CORRECTIVE, .cross = RA_CROSS_ANY,
	.vs.in.min = FPM2MPS(1500), .vs.in.max = FPM2MPS(2500),
	.vs.out.min = FPM2MPS(2500), .vs.out.max = FPM2MPS(4400)
    },


/* Corrective descending RAs */
    {	/* MAINTAIN VERTICAL SPEED, CROSSING MAINTAIN - large VS range */
	.msg = RA_MSG_MAINT_VS_CROSS, .rev_msg = -1,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_DOWNWARD,
	.type = RA_TYPE_PREVENTIVE, .cross = RA_CROSS_REQ,
	.vs.in.min = FPM2MPS(-4400), .vs.in.max = FPM2MPS(-1500),
	.vs.out.min = FPM2MPS(-4400), .vs.out.max = FPM2MPS(-1500)
    },
    {	/* MAINTAIN VERTICAL SPEED, MAINTAIN - large VS range */
	.msg = RA_MSG_MAINT_VS, .rev_msg = -1,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_DOWNWARD,
	.type = RA_TYPE_PREVENTIVE, .cross = RA_CROSS_REJ,
	.vs.in.min = FPM2MPS(-4400), .vs.in.max = FPM2MPS(-1500),
	.vs.out.min = FPM2MPS(-4400), .vs.out.max = FPM2MPS(-1500)
    },
    {	/* MAINTAIN VERTICAL SPEED, CROSSING MAINTAIN - high VS range */
	.msg = RA_MSG_MAINT_VS_CROSS, .rev_msg = -1,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_DOWNWARD,
	.type = RA_TYPE_PREVENTIVE, .cross = RA_CROSS_REQ,
	.vs.in.min = FPM2MPS(-2000), .vs.in.max = FPM2MPS(-1500),
	.vs.out.min = FPM2MPS(-2000), .vs.out.max = FPM2MPS(-1500)
    },
    {	/* MAINTAIN VERTICAL SPEED, MAINTAIN */
	.msg = RA_MSG_MAINT_VS, .rev_msg = -1,
	.initial = B_TRUE, .subseq = B_FALSE, .sense = RA_SENSE_DOWNWARD,
	.type = RA_TYPE_PREVENTIVE, .cross = RA_CROSS_REJ,
	.vs.in.min = FPM2MPS(-2000), .vs.in.max = FPM2MPS(-1500),
	.vs.out.min = FPM2MPS(-2000), .vs.out.max = FPM2MPS(-1500)
    },
    {	/* DESCEND, DESCEND */
	.msg = RA_MSG_DES, .rev_msg = RA_MSG_DES_NOW,
	.initial = B_TRUE, .subseq = B_TRUE, .sense = RA_SENSE_DOWNWARD,
	.type = RA_TYPE_CORRECTIVE, .cross = RA_CROSS_REJ,
	.vs.in.min = INFINITY, .vs.in.max = FPM2MPS(-1500),
	.vs.out.min = FPM2MPS(-2000), .vs.out.max = FPM2MPS(-1500)
    },
    {	/* DESCEND, CROSSING DESCEND */
	.msg = RA_MSG_DES_CROSS, .rev_msg = RA_MSG_DES_NOW,
	.initial = B_TRUE, .subseq = B_TRUE, .sense = RA_SENSE_DOWNWARD,
	.type = RA_TYPE_CORRECTIVE, .cross = RA_CROSS_REQ,
	.vs.in.min = INFINITY, .vs.in.max = FPM2MPS(-1500),
	.vs.out.min = FPM2MPS(-2000), .vs.out.max = FPM2MPS(-1500)
    },
    {	/* INCREASE DESCENT */
	.msg = RA_MSG_DES_MORE, .rev_msg = -1,
	.initial = B_FALSE, .subseq = B_TRUE, .sense = RA_SENSE_DOWNWARD,
	.type = RA_TYPE_CORRECTIVE, .cross = RA_CROSS_ANY,
	.vs.in.min = FPM2MPS(-2500), .vs.in.max = FPM2MPS(-1500),
	.vs.out.min = FPM2MPS(-4400), .vs.out.max = FPM2MPS(-2500)
    }
};

static mutex_t acf_lock;
static tcas_acf_t my_acf_glob;
static avl_tree_t other_acf_glob;
static double last_t = 0;
static tcas_state_t tcas_state;
static bool_t inited = B_FALSE;

static condvar_t worker_cv;
static thread_t worker_thr;
static mutex_t worker_lock;
static bool_t worker_shutdown = B_FALSE;

static const sim_intf_ops_t *ops = NULL;

static const char *
RA_msg2str(tcas_RA_msg_t msg)
{
	switch (msg) {
	case RA_MSG_CLB:
		return "CLB";
	case RA_MSG_CLB_CROSS:
		return "CLB CROSS";
	case RA_MSG_CLB_MORE:
		return "INC CLIMB";
	case RA_MSG_CLB_NOW:
		return "CLB NOW";
	case RA_MSG_CLEAR:
		return "CLR";
	case RA_MSG_DES:
		return "DES";
	case RA_MSG_DES_CROSS:
		return "DES CROSS";
	case RA_MSG_DES_MORE:
		return "INC DESCEND";
	case RA_MSG_DES_NOW:
		return "DES NOW";
	case RA_MSG_MONITOR_VS:
		return "MON VS";
	case RA_MSG_MAINT_VS:
		return "MAINT VS";
	case RA_MSG_MAINT_VS_CROSS:
		return "MAINT VS CROSS";
	case RA_MSG_LEVEL_OFF:
		return "LEVEL OFF";
	default:
		return "<invalid>";
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

static void
update_my_position(double t)
{
	ops->get_my_acf_pos(&my_acf_glob.cur_pos, &my_acf_glob.agl);
	my_acf_glob.cur_pos_3d = VECT3(0, 0, my_acf_glob.cur_pos.elev);
	xtcas_obj_pos_update(&my_acf_glob.pos_upd, t, my_acf_glob.cur_pos,
	    my_acf_glob.agl);
	my_acf_glob.trend_data_ready = (
	    xtcas_obj_pos_get_gs(&my_acf_glob.pos_upd, &my_acf_glob.gs) &&
	    xtcas_obj_pos_get_trk(&my_acf_glob.pos_upd, &my_acf_glob.trk) &&
	    xtcas_obj_pos_get_vvel(&my_acf_glob.pos_upd, &my_acf_glob.vvel));
	if (my_acf_glob.trend_data_ready)
		my_acf_glob.trk_v = hdg2dir(my_acf_glob.trk);

	dbg_log(tcas, 1, "my_pos: " PRINTF_ACF_FMT,
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
	fpp_t fpp = stereo_fpp_init(GEO3_TO_GEO2(my_pos), 0, &wgs84, B_FALSE);
	double gnd_level = (my_alt_agl <= ON_GROUND_AGL_CHK_THRESH) ?
	    (my_pos.elev - my_alt_agl) : MIN_ELEV;

	ops->get_oth_acf_pos(&pos, &count);

	/* walk the tree and mark all acf as out-of-date */
	for (tcas_acf_t *acf = avl_first(&other_acf_glob); acf != NULL;
	    acf = AVL_NEXT(&other_acf_glob, acf))
		acf->up_to_date = B_FALSE;

	for (size_t i = 0; i < count; i++) {
		avl_index_t where;
		tcas_acf_t srch = { .acf_id = pos[i].acf_id };
		tcas_acf_t *acf = avl_find(&other_acf_glob, &srch, &where);
		vect2_t proj;

		if (acf == NULL) {
			acf = calloc(1, sizeof (*acf));
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
		    xtcas_obj_pos_get_vvel(&acf->pos_upd, &acf->vvel));
		my_acf_glob.trk_v = (acf->trend_data_ready) ?
		    hdg2dir(my_acf_glob.trk) : NULL_VECT2;
		acf->on_ground = (acf->alt_rptg &&
		    acf->cur_pos.elev < gnd_level);
		/* mark acf as up-to-date */
		acf->up_to_date = B_TRUE;

		dbg_log(tcas, 2, "bogie %p " PRINTF_ACF_FMT, acf->acf_id,
		    PRINTF_ACF_ARGS(acf));
	}

	/* walk the tree again and remove out-of-date aircraft */
	for (tcas_acf_t *acf = avl_first(&other_acf_glob), *acf_next = NULL;
	    acf != NULL; acf = acf_next) {
		acf_next = AVL_NEXT(&other_acf_glob, acf);
		if (!acf->up_to_date) {
			dbg_log(tcas, 2, "bogie %p contact lost", acf->acf_id);
			avl_remove(&other_acf_glob, acf);
			free(acf);
		}
	}

	dbg_log(tcas, 1, "total bogies: %lu", avl_numnodes(&other_acf_glob));

	free(pos);
}

/*
 * Copies all of the global aircraft position info (our aircraft and other
 * aircraft) to create a thread-local copy. `my_acf_copy' will be populated
 * with our aircraft's position info, while `other_acf_copy' will be
 * populated with the state of all other aircraft. Neither may be NULL.
 */
static void
copy_acf_state(tcas_acf_t *my_acf_copy, avl_tree_t *other_acf_copy)
{
	ASSERT(my_acf_copy != NULL);
	ASSERT(other_acf_copy != NULL);

	avl_create(other_acf_copy, acf_compar, sizeof (tcas_acf_t),
	    offsetof(tcas_acf_t, node));

	mutex_enter(&acf_lock);

	memcpy(my_acf_copy, &my_acf_glob, sizeof (*my_acf_copy));

	for (tcas_acf_t *acf = avl_first(&other_acf_glob); acf != NULL;
	    acf = AVL_NEXT(&other_acf_glob, acf)) {
		tcas_acf_t *acf_copy = calloc(1, sizeof (*acf));
		memcpy(acf_copy, acf, sizeof (*acf));
		avl_add(other_acf_copy, acf_copy);
	}

	mutex_exit(&acf_lock);
}

static void
destroy_acf_state(tcas_acf_t *my_acf_copy, avl_tree_t *other_acf_copy)
{
	void *cookie = NULL;
	tcas_acf_t *acf;

	if (my_acf_copy != NULL)
		free(my_acf_copy);

	while ((acf = avl_destroy_nodes(other_acf_copy, &cookie)) != NULL) {
		ASSERT(acf->cpa == NULL);
		free(acf);
	}
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
make_cpa(unsigned d_t, tcas_acf_t *acf_a, tcas_acf_t *acf_b,
    vect3_t pos_a, vect3_t pos_b)
{
	cpa_t *cpa = calloc(1, sizeof (*cpa));

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
	vect2_t my_dir = vect2_set_abs(hdg2dir(my_acf->trk), my_acf->gs);
	vect3_t my_vel = VECT3(my_dir.x, my_dir.y, my_acf->vvel);

	avl_create(cpas, cpa_compar, sizeof (cpa_t), offsetof(cpa_t, node));

	for (tcas_acf_t *acf = avl_first(other_acf); acf != NULL;
	    acf = AVL_NEXT(other_acf, acf)) {
		vect2_t dir;
		vect3_t pos_3d, vel, rel_vel, cpa_pos, my_cpa_pos;
		double t_cpa, dist;
		cpa_t *cpa;

		/*
		 * Don't compute CPAs for contacts that either:
		 * 1) Don't have any trend data available yet.
		 * 2) Fall outside of our maximum vertical filter boundaries.
		 */
		if (!acf->trend_data_ready || ABS(acf->cur_pos.elev -
		    my_acf->cur_pos.elev) > NORM_VERT_FILTER)
			continue;

		pos_3d = acf->cur_pos_3d;
		dir = vect2_set_abs(hdg2dir(acf->trk), acf->gs);
		vel = VECT3(dir.x, dir.y, acf->vvel);
		rel_vel = vect3_sub(vel, my_vel);
		t_cpa = (-(pos_3d.x * rel_vel.x) - (pos_3d.y * rel_vel.y) -
		    (pos_3d.z - rel_vel.z)) /
		    (POW2(rel_vel.x) + POW2(rel_vel.y) + POW2(rel_vel.z));

		/*
		 * If the CPA is in the past or too far into the future, don't
		 * even bother.
		 */
		if (t_cpa <= 0 || t_cpa > CPA_MAX_T) {
			dbg_log(tcas, 2, "bogie %p t_cpa: %.1f, culling",
			    acf->acf_id, t_cpa);
			continue;
		}

		cpa_pos = vect3_add(pos_3d, vect3_scmul(vel, t_cpa));
		my_cpa_pos = vect3_add(my_pos_3d, vect3_scmul(my_vel, t_cpa));
		/* If we are too far apart at CPA, don't even bother. */
		dist = vect3_abs(vect3_sub(cpa_pos, my_cpa_pos));
		if (dist > CPA_MAX_DIST) {
			dbg_log(tcas, 2, "bogie %p dist: %.0f, culling",
			    acf->acf_id, dist);
			continue;
		}

		cpa = make_cpa(t_cpa, my_acf, acf, my_cpa_pos, cpa_pos);
		dbg_log(tcas, 1, "bogie %p cpa  d_t:%.1f  pos_a:%.0fx%.0fx%.0f"
		    "  pos_b:%.0fx%.0fx%.0f  d_h:%.0f  d_v:%.0f",
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

static void
assign_threat_level(vect3_t my_pos_3d, tcas_acf_t *oacf, const SL_t *sl,
    tcas_filter_t filter)
{
	double d_h = vect2_abs(vect2_sub(VECT3_TO_VECT2(oacf->cur_pos_3d),
	    VECT3_TO_VECT2(my_pos_3d)));
	double d_v = ABS(my_pos_3d.z - oacf->cur_pos_3d.z);
	cpa_t *cpa = oacf->cpa;
	double filter_min = my_pos_3d.z, filter_max = my_pos_3d.z;
	bool_t vert_filter = B_TRUE;

	filter_min -= (filter == TCAS_FILTER_ABV ? SHORT_VERT_FILTER :
	    NORM_VERT_FILTER);
	filter_max += (filter == TCAS_FILTER_BLW ? SHORT_VERT_FILTER :
	    NORM_VERT_FILTER);
	if (oacf->alt_rptg) {
		vert_filter = (oacf->cur_pos.elev >= filter_min &&
		    oacf->cur_pos.elev <= filter_max);
	}

	if (cpa != NULL) {
		/*
		 * RA threat iff:
		 * 1) reporting an altitude AND either:
		 * 2a) its CPA point violates the protected volume AND
		 *	time to CPA is <= tau_RA
		 * 2b) its current position violates our protected volume
		 * The vertical filter is NOT applied to RAs.
		 */
		if (oacf->alt_rptg &&
		    ((cpa->d_h <= sl->dmod_RA && cpa->d_v <= sl->zthr_RA &&
		    cpa->d_t <= sl->tau_RA) ||
		    (d_h <= sl->dmod_RA && d_v <= sl->zthr_RA))) {
			dbg_log(tcas, 1, "bogie %p RA_THREAT", oacf->acf_id);
			oacf->threat = RA_THREAT;
			return;
		}
		/*
		 * TA threat iff (subject to vertical filter being satisfied):
		 * 1) ALL of the following conditions are true:
		 *	1a) horiz separation at CPA violates protected volume
		 *	1b) the aircraft DOESN'T report altitude, OR vert
		 *	    separation at CPA violates protected volume
		 *	1c) time to CPA <= tau_TA
		 * 2) OR, ALL of the following conditions are true:
		 *	2a) horiz separation NOW violates protected volume
		 *	2b) the aircraft DOESN'T report altitude, OR vert
		 *	    separation NOW violates protected volume
		 *	2c) time to CPA <= tau_TA
		 */
		if (vert_filter && ((cpa->d_h <= sl->dmod_TA &&
		    (!oacf->alt_rptg || cpa->d_v <= sl->zthr_TA) &&
		    cpa->d_t <= sl->tau_TA) || (d_h <= sl->dmod_TA &&
		    (!oacf->alt_rptg || d_v <= sl->zthr_TA)))) {
			dbg_log(tcas, 1, "bogie %p TA_THREAT", oacf->acf_id);
			oacf->threat = TA_THREAT;
			return;
		}
	}
	/*
	 * This condition prevents downgrade of RA threats during evasive
	 * maneuvers. RAs are downgraded only once we have passed CPA.
	 */
	if (oacf->threat <= TA_THREAT || oacf->cpa == NULL) {
		/*
		 * Proximate traffic iff:
		 * 1) horiz separation within prox traffic dist thresh, AND
		 * 2a) target doesn't report altitude, OR
		 * 2b) vert separation within prox traffic altitude threshold
		 */
		if (d_h <= PROX_DIST_THRESH &&
		    (!oacf->alt_rptg || d_v <= PROX_ALT_THRESH)) {
			dbg_log(tcas, 1, "bogie %p PROX_THREAT", oacf->acf_id);
			oacf->threat = PROX_THREAT;
		} else {
			dbg_log(tcas, 1, "bogie %p OTH_THREAT", oacf->acf_id);
			oacf->threat = OTH_THREAT;
		}
	}
}

static int
ra_compar(const void *ra_a, const void *ra_b)
{
	const tcas_RA_t *a = ra_a, *b = ra_b;

	/* RAs can only be sorted if they refer to the same encounter */
	ASSERT3P(a->cpas, ==, b->cpas);

	/* if the RAs point to the same info, they must be the same RA */
	if (a->info == b->info) {
		ASSERT(memcmp(a, b, offsetof(tcas_RA_t, node)) == 0);
		return (0);
	}

	/* if one RA provides ALIM and the other doesn't, that's sufficient */
	if (a->alim_achieved && !b->alim_achieved)
		return (-1);
	if (!a->alim_achieved && b->alim_achieved)
		return (1);

	/*
	 * If neither achieves ALIM, we pick the one with the greatest
	 * minimum separation from any contact.
	 */
	if (!a->alim_achieved && !b->alim_achieved) {
		if (a->min_sep > b->min_sep)
			return (-1);
		else
			return (1);
	}

	/* prefer non-crossing RAs */
	if (!a->crossing && b->crossing)
		return (-1);
	if (a->crossing && !b->crossing)
		return (1);

	/* prefer non-sense-reversing RAs */
	if (!a->reversal && b->reversal)
		return (-1);
	if (a->reversal && !b->reversal)
		return (1);

	/* pick the RA departing least from our original VS */
	if (!isnan(tcas_state.initial_ra_vs)) {
		tcas_acf_t *my_acf = ((cpa_t *)avl_first(a->cpas))->acf_a;
		double init_vs = roundmul(tcas_state.initial_ra_vs,
		    ALT_ROUND_MUL);
		double cur_vs = roundmul(my_acf->vvel, ALT_ROUND_MUL);
		double d_vs_a = fabs(init_vs - (cur_vs + ABS(a->vs_corr_reqd)));
		double d_vs_b = fabs(init_vs - (cur_vs + ABS(a->vs_corr_reqd)));
		if (d_vs_a < d_vs_b)
			return (-1);
		if (d_vs_a > d_vs_b)
			return (1);
	}

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
	double man_t, rmng_t;

	delay_t = MIN(cpa_t, delay_t);
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
	return (my_acf->cur_pos.elev +
	    my_acf->vvel * (delay_t + man_t) + 0.5 * accel * POW2(man_t) +
	    vsr * rmng_t);
}

static double
compute_separation(const tcas_acf_t *my_acf, cpa_t *cpa,
    const tcas_RA_info_t *ri, double delay_t, double accel)
{
	double elev_min = predict_elev_at_CPA(my_acf, cpa->d_t,
	    ri->vs.out.min > my_acf->vvel ? delay_t : 0, accel, ri->vs.out.min);
	double elev_max = predict_elev_at_CPA(my_acf, cpa->d_t,
	    ri->vs.out.max < my_acf->vvel ? delay_t : 0, accel, ri->vs.out.max);

	if (cpa->pos_b.z < elev_min)
		return (elev_min - cpa->pos_b.z);
	if (cpa->pos_b.z > elev_max)
		return (cpa->pos_b.z - elev_max);

	return (0);
}

/*
 * Check for incompatible sequences of RA message annunciations. Sometimes
 * we want to prevent calling out an RA even if it has changed. For example,
 * when weakening a preventive RA with the same messages, don't call out
 * the weakening action - it's already fine.
 * Returns the actual RA message which should be played, or RA_MSG_NONE.
 */
static tcas_RA_msg_t
RA_msg_sequence_check(tcas_RA_msg_t prev_msg, tcas_RA_msg_t next_msg)
{
	/* Prevent repeating the same RA */
	if (prev_msg == next_msg)
		return (-1);
	return (next_msg);
}

static tcas_RA_t *
CAS_logic(const tcas_acf_t *my_acf, const tcas_RA_t *prev_ra, avl_tree_t *cpas,
    const SL_t *sl)
{
	bool_t initial = (prev_ra != NULL);
	double delay_t = (initial ? INITIAL_RA_DELAY : SUBSEQ_RA_DELAY),
	    accel = (initial ? INITIAL_RA_D_VVEL : SUBSEQ_RA_D_VVEL);
	avl_tree_t prio;
	tcas_RA_t *ra, *rra;
	void *cookie = NULL;

	avl_create(&prio, ra_compar, sizeof (tcas_RA_t),
	    offsetof(tcas_RA_t, node));

	for (int i = 0; i < NUM_RA_INFOS; i++) {
		const tcas_RA_info_t *ri = &RA_info[i];
		tcas_RA_sense_t prev_sense = (prev_ra != NULL ?
		    prev_ra->info->sense : RA_SENSE_LEVEL_OFF);
		bool_t reversal = (prev_sense != RA_SENSE_LEVEL_OFF &&
		    prev_sense != ri->sense);

		ASSERT3U(ri->msg, >=, 0);
		ASSERT3U(ri->msg, <, RA_NUM_MSGS);

		if (/* ri must allow initial/subseq ann as necessary */
		    (initial && !ri->initial) ||
		    (!initial && !ri->subseq) ||
		    /* ri must satisfy input VS condition */
		    my_acf->vvel < ri->vs.in.min ||
		    my_acf->vvel > ri->vs.in.max ||
		    /* if performing a sense reversal, ri must allow it */
		    (reversal && (int)ri->rev_msg == -1))
			continue;

		ra = calloc(1, sizeof (*ra));
		ra->info = ri;
		ra->cpas = cpas;
		ra->reversal = reversal;
		ra->min_sep = INFINITY;
		for (cpa_t *cpa = avl_first(cpas); cpa != NULL;
		    cpa = AVL_NEXT(cpas, cpa), i++) {
			double sep = compute_separation(my_acf, cpa, ri,
			    delay_t, accel);
			ra->min_sep = MIN(sep, ra->min_sep);
			/*
			 * An encounter is crossing if either:
			 * 1) the RA sense is upward and at CPA our elev would
			 *    have been below the intruder (+ some margin), OR
			 * 2) the RA sense is downward and at CPA our elev
			 *    would have been above the intruder
			 */
			ra->crossing |= ((ri->sense == RA_SENSE_UPWARD &&
			    cpa->pos_b.z - cpa->pos_a.z > EQ_ALT_THRESH) ||
			    (ri->sense == RA_SENSE_DOWNWARD &&
			    cpa->pos_a.z - cpa->pos_b.z > EQ_ALT_THRESH));
		}
		ra->min_sep = floormul(ra->min_sep, ALT_ROUND_MUL);
		ra->alim_achieved = (ra->min_sep >= sl->alim_RA);
		if (my_acf->vvel < ri->vs.out.min) {
			ra->vs_corr_reqd = roundmul(ri->vs.out.min -
			    my_acf->vvel, ALT_ROUND_MUL);
		} else if (my_acf->vvel > ri->vs.out.max) {
			ra->vs_corr_reqd = roundmul(ri->vs.out.max -
			    my_acf->vvel, ALT_ROUND_MUL);
		}
		/*
		 * RA filtering conditions:
		 */
		if (/* 1) Above FL480 we want to inhibit climb RAs. */
		    (my_acf->cur_pos.elev > INHIBIT_CLB_RA &&
		    (ra->info->msg == RA_MSG_CLB ||
		    ra->info->msg == RA_MSG_CLB_CROSS ||
		    ra->info->msg == RA_MSG_CLB_MORE ||
		    ra->info->msg == RA_MSG_CLB_NOW)) ||
		    /* 2) Inhibit INCREASE DESCENT RAs below 1550ft AGL. */
		    (my_acf->agl < INHIBIT_INC_DESC_RA &&
		    ra->info->msg == RA_MSG_DES_MORE) ||
		    /* 3) Inhibit all DESCEND RAs below 1100ft AGL. */
		    (my_acf->agl < INHIBIT_INC_DESC_RA &&
		    (ra->info->msg == RA_MSG_DES ||
		    ra->info->msg == RA_MSG_DES_CROSS ||
		    ra->info->msg == RA_MSG_DES_MORE ||
		    ra->info->msg == RA_MSG_DES_NOW)) ||
		    /*
		     * 4) Pick an RA that makes sense from a sequence POV. We
		     *    want to prevent a CLB/DES MORE RA from reducing to a
		     *    regular CLB or CROSS CLB RA. In those cases, we want
		     *    to either LEVEL OFF or perform a sense reversal.
		     */
		    (prev_ra->info->msg == RA_MSG_CLB_MORE &&
		    (ra->info->msg == RA_MSG_CLB ||
		    ra->info->msg == RA_MSG_CLB_CROSS)) ||
		    (prev_ra->info->msg == RA_MSG_DES_MORE &&
		    (ra->info->msg == RA_MSG_DES ||
		    ra->info->msg == RA_MSG_DES_CROSS))) {
			free(ra);
			continue;
		}
		avl_add(&prio, ra);
	}
	ASSERT(avl_numnodes(&prio) != 0);

	avl_remove(&prio, ra);
	while ((rra = avl_destroy_nodes(&prio, &cookie)) == NULL)
		free(rra);
	avl_destroy(&prio);

	/*
	 * Now that we have an RA, we need to determine if it's sensible
	 * given the previously issued RA.
	 */
	if (prev_ra != NULL) {
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
resolve_CPAs(tcas_acf_t *my_acf, avl_tree_t *other_acf, avl_tree_t *cpas,
    const SL_t *sl, uint64_t now)
{
	bool_t TA_found = B_FALSE, RA_found = B_FALSE;

	/* No CPAs or in standby mode */
	if (avl_numnodes(cpas) == 0 || sl->tau_TA == 0)
		return;

	/* Re-assign threat level as necessary. */
	for (tcas_acf_t *acf = avl_first(other_acf); acf != NULL;
	    acf = AVL_NEXT(other_acf, acf)) {
		assign_threat_level(my_acf->cur_pos_3d, acf, sl,
		    tcas_state.filter);
		TA_found |= (acf->threat == TA_THREAT);
		RA_found |= (acf->threat == RA_THREAT);
	}

	if (RA_found) {
		tcas_RA_t *ra;
		avl_tree_t ra_cpas;
		void *cookie = NULL;
		double d_t = NAN;

		avl_create(&ra_cpas, cpa_compar, sizeof (cpa_t),
		    offsetof(cpa_t, ra_node));
		for (cpa_t *cpa = avl_first(cpas); cpa != NULL;
		    cpa = AVL_NEXT(cpas, cpa)) {
			if (cpa->acf_b->threat != RA_THREAT)
				continue;
			if (isnan(d_t))
				d_t = cpa->d_t;
			ASSERT3F(d_t, <=, cpa->d_t);
			if (d_t <= cpa->d_t + INITIAL_RA_DELAY)
				avl_add(&ra_cpas, cpa);
		}
		ASSERT(!isnan(d_t));
		ASSERT(avl_numnodes(&ra_cpas) != 0);

		dbg_log(tcas, 1, "resolve_CPAs: RA  count:%lu  adv_state:%d  "
		    "elapsed:%.0f", avl_numnodes(&ra_cpas),
		    tcas_state.adv_state,
		    (now - tcas_state.change_t) / 1000000.0);

		ra = CAS_logic(my_acf, tcas_state.ra, &ra_cpas, sl);
		/* On initial annunciation, we must ALWAYS issue an RA */
		ASSERT(ra != NULL || tcas_state.ra != NULL);

		dbg_log(tcas, 1, "RA msg:%s  type:%s  sense:%s  cross:%s  "
		    "in.min:%.1f  in.max:%.1f  out.min:%.1f  out.max:%.1f  "
		    "rev:%d  cross:%d  alim:%d  min_sep:%.1f  vscorr:%.1f",
		    RA_msg2str(ra->info->msg), RA_type2str(ra->info->type),
		    RA_sense2str(ra->info->sense), RA_cross2str(
		    ra->info->cross), ra->info->vs.in.min, ra->info->vs.in.max,
		    ra->info->vs.out.min, ra->info->vs.out.max, ra->reversal,
		    ra->crossing, ra->alim_achieved, ra->min_sep,
		    ra->vs_corr_reqd);

		if (ra != NULL) {
			if (now - tcas_state.change_t >= STATE_CHG_DELAY) {
				tcas_RA_msg_t prev_msg = -1;
				tcas_RA_msg_t msg;
				if (tcas_state.ra != NULL) {
					prev_msg = tcas_state.ra->info->msg;
					free(tcas_state.ra);
					/* memorize what VS we started at */
					tcas_state.initial_ra_vs =
					    my_acf->vvel;
				}
				tcas_state.ra = ra;
				tcas_state.change_t = now;
				tcas_state.adv_state = ADV_STATE_RA;
				/* Filter out pointless annunciations */
				msg = RA_msg_sequence_check(prev_msg,
				    ra->info->msg);
				if ((int)msg != -1 &&
				    my_acf->agl > INHIBIT_AUDIO) {
					xtcas_play_msg(msg);
				}
			} else {
				free(ra);
			}
		}

		while ((avl_destroy_nodes(&ra_cpas, &cookie)) != NULL)
			;
		avl_destroy(&ra_cpas);
	} else if (TA_found) {
		/*
		 * TRAFFIC is annunciated only from the NONE state and after
		 * the minimum state change delay has passed. This prevents
		 * annunciating TRAFFIC after an RA has been resolved, but
		 * while the traffic still falls into the TA range.
		 */
		dbg_log(tcas, 1, "resolve_CPAs: TA  adv_state:%d  elapsed:%.0f",
		    tcas_state.adv_state,
		    (now - tcas_state.change_t) / 1000000.0);
		if (tcas_state.adv_state < ADV_STATE_TA &&
		    now - tcas_state.change_t >= STATE_CHG_DELAY) {
			if (my_acf->agl > INHIBIT_AUDIO)
				xtcas_play_msg(RA_MSG_TFC);
			free(tcas_state.ra);
			tcas_state.ra = NULL;
			tcas_state.initial_ra_vs = NAN;
			tcas_state.adv_state = ADV_STATE_TA;
		}
	} else if (tcas_state.adv_state != ADV_STATE_NONE &&
	    now - tcas_state.change_t >= STATE_CHG_DELAY) {
		dbg_log(tcas, 1, "resolve_CPAs: NONE  adv_state:%d  "
		    "elapsed:%.0f", tcas_state.adv_state,
		    (now - tcas_state.change_t) / 1000000.0);
		if (tcas_state.adv_state == ADV_STATE_RA &&
		    my_acf->agl > INHIBIT_AUDIO) {
			xtcas_play_msg(RA_MSG_CLEAR);
		}
		free(tcas_state.ra);
		tcas_state.ra = NULL;
		tcas_state.initial_ra_vs = NAN;
		tcas_state.adv_state = ADV_STATE_NONE;
		tcas_state.change_t = now;
	}
}

static void
main_loop(void *ignored)
{
	const SL_t *sl = NULL;
	double last_t = ops->get_time();

	dbg_log(tcas, 1, "main_loop: entry (%.1f)", last_t);

	UNUSED(ignored);
	ASSERT(inited);

	mutex_enter(&worker_lock);
	for (double now = microclock(); !worker_shutdown; now = microclock()) {
		tcas_acf_t my_acf;
		avl_tree_t other_acf, cpas;
		double now_t = ops->get_time();

		dbg_log(tcas, 1, "main_loop: start (%.1f)", now_t);

		/* If sim time hasn't advanced, we're paused, so wait */
		if (!(last_t < now_t)) {
			dbg_log(tcas, 3, "main_loop: time hasn't progressed");
			cv_timedwait(&worker_cv, &worker_lock,
			    now + WORKER_LOOP_INTVAL);
			continue;
		}
		last_t = now_t;

		/*
		 * We'll create a local copy of all aircraft positions so
		 * we don't have to hold acf_lock throughout.
		 */
		copy_acf_state(&my_acf, &other_acf);

		/*
		 * Based on our altitudes, determine the sensitivity level.
		 * SL change is prevented while in an RA to avoid excessive
		 * RA switching. TA-only mode always selects SL2.
		 */
		if (sl == NULL || tcas_state.adv_state != ADV_STATE_RA) {
			sl = xtcas_SL_select(sl != NULL ? sl->SL_id : 1,
			    my_acf.cur_pos.elev, my_acf.agl,
			    tcas_state.mode == TCAS_MODE_TAONLY ? 2 : 0);
			dbg_log(tcas, 1, "SL: %d", sl->SL_id);
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
		 */
		resolve_CPAs(&my_acf, &other_acf, &cpas, sl, now);

		destroy_CPAs(&cpas);

		/*
		 * Dispose of the local position copy.
		 */
		destroy_acf_state(&my_acf, &other_acf);

		dbg_log(tcas, 2, "main_loop: end");

		/*
		 * Jump forward at fixed intervals to guarantee our
		 * execution schedule.
		 */
		do {
			cv_timedwait(&worker_cv, &worker_lock,
			    now + WORKER_LOOP_INTVAL);
		} while (microclock() < now + WORKER_LOOP_INTVAL);
	}
	mutex_exit(&worker_lock);
}

void
xtcas_run(void)
{
	double t = ops->get_time();

	dbg_log(tcas, 1, "run: %.1f", t);

	ASSERT(inited);

	/* protection in case the sim is paused */
	if (t <= last_t)
		return;

	mutex_enter(&acf_lock);
	update_my_position(t);
	update_bogie_positions(t, my_acf_glob.cur_pos, my_acf_glob.agl);

	if (!my_acf_glob.trend_data_ready) {
		mutex_exit(&acf_lock);
		return;
	}
	mutex_exit(&acf_lock);
	last_t = t;
}

void
xtcas_init(const sim_intf_ops_t *intf_ops)
{
	dbg_log(tcas, 1, "init");

	memset(&my_acf_glob, 0, sizeof (my_acf_glob));
	avl_create(&other_acf_glob, acf_compar,
	    sizeof (tcas_acf_t), offsetof(tcas_acf_t, node));
	mutex_init(&acf_lock);

	mutex_init(&worker_lock);
	cv_init(&worker_cv);
	VERIFY(thread_create(&worker_thr, main_loop, NULL));

	memset(&tcas_state, 0, sizeof (tcas_state));
	tcas_state.initial_ra_vs = NAN;

	ops = intf_ops;

	inited = B_TRUE;
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

	destroy_acf_state(NULL, &other_acf_glob);

	mutex_destroy(&acf_lock);

	cv_destroy(&worker_cv);
	mutex_destroy(&worker_lock);

	free(tcas_state.ra);

	inited = B_FALSE;
}

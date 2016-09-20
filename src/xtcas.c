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

#define	PREDICTION_DEBUG	1

#include <stdlib.h>
#include <string.h>
#include <stddef.h>

#if	PREDICTION_DEBUG
#include "XPLMDisplay.h"
#include "XPLMGraphics.h"
#include "XPLMScenery.h"
#endif	/* PREDICTION_DEBUG */

#include "avl.h"
#include "helpers.h"
#include "geom.h"
#include "log.h"
#include "pos.h"
#include "SL.h"
#include "thread.h"
#include "time.h"
#include "xplane.h"

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

typedef struct tcas_acf {
	void	*acf_id;	/* identifier - used for locating in tree */
	obj_pos_t pos;		/* position derivatives */
	double	gs, d_gs;	/* true groundspeed and rate of GS change */
	double	trk, d_trk;	/* true track and rate of track change */
	double	vvel, d_vvel;	/* computed vertical velocity & vvel change */
	bool_t	acf_ready;	/* indicates if derivatives are available */
	bool_t	up_to_date;	/* used for efficient position updates */

	vect3_t	*extr_pos;	/* extrapolated positions in stereographic */
	size_t	num_extr_pos;	/* projection from current my_acf position */

	avl_node_t node;	/* used by other_acf tree */
} tcas_acf_t;

typedef enum {
	TCAS_STATE_NORM,
	TCAS_STATE_TA,
	TCAS_STATE_RA
} tcas_state_t;

#define	WORKER_LOOP_INTVAL	1000000		/* us */

#define	CLB1_VVEL		FEET2MET(2000)
#define	CLB2_VVEL		FEET2MET(4500)
#define	LEVEL_VVEL_THRESH	FEET2MET(200)
#define	LEVEL_D_VVEL_THRESH	FEET2MET(50)

/*
 * This is the number of aircraft position extrapolation steps we perform.
 * This needs to be greater than the highest TAU value in SL.c, plus some
 * buffer to make sure that sudden changes in orientation still have a little
 * bit of extra time to provide for the TA and RA tau values to be hit near
 * their maximum.
 */
#define	EXTRAPOLATION_STEPS	60

static mutex_t acf_lock;
static tcas_acf_t my_acf;
static avl_tree_t other_acf;
static double last_t = 0;

static condvar_t worker_cv;
static thread_t worker_thr;
static mutex_t worker_lock;
static bool_t worker_shutdown = B_FALSE;

#if	PREDICTION_DEBUG
static XPLMObjectRef prediction_object = NULL;
#endif	/* PREDICTION_DEBUG */

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
	geo_pos3_t new_pos;
	double new_agl;

	xtcas_get_my_acf_pos(&new_pos, &new_agl);
	xtcas_obj_pos_update(&my_acf.pos, t, new_pos, new_agl);
	my_acf.acf_ready = (
	    xtcas_obj_pos_get_gs(&my_acf.pos, &my_acf.gs, &my_acf.d_gs) &&
	    xtcas_obj_pos_get_trk(&my_acf.pos, &my_acf.trk, &my_acf.d_trk) &&
	    xtcas_obj_pos_get_vvel(&my_acf.pos, &my_acf.vvel, &my_acf.d_vvel));
}

static void
update_bogie_positions(double t)
{
	acf_pos_t *pos;
	size_t count;

	xtcas_get_acf_pos(&pos, &count);

	/* walk the tree and mark all acf as out-of-date */
	for (tcas_acf_t *acf = avl_first(&other_acf); acf != NULL;
	    acf = AVL_NEXT(&other_acf, acf))
		acf->up_to_date = B_FALSE;

	for (size_t i = 0; i < count; i++) {
		avl_index_t where;
		tcas_acf_t srch = { .acf_id = pos[i].acf_id };
		tcas_acf_t *acf = avl_find(&other_acf, &srch, &where);

		if (acf == NULL) {
			acf = calloc(1, sizeof (*acf));
			acf->acf_id = pos[i].acf_id;
			avl_insert(&other_acf, acf, where);
		}
		xtcas_obj_pos_update(&acf->pos, t, pos[i].pos, -1);
		acf->acf_ready = (
		    xtcas_obj_pos_get_gs(&acf->pos, &acf->gs, &acf->d_gs) &&
		    xtcas_obj_pos_get_trk(&acf->pos, &acf->trk, &acf->d_trk) &&
		    xtcas_obj_pos_get_vvel(&acf->pos, &acf->vvel,
		    &acf->d_vvel));
		/* mark acf as up-to-date */
		acf->up_to_date = B_TRUE;
	}

	/* walk the tree again and remove out-of-date aircraft */
	for (tcas_acf_t *acf = avl_first(&other_acf), *acf_next = NULL;
	    acf != NULL; acf = acf_next) {
		acf_next = AVL_NEXT(&other_acf, acf);
		if (!acf->up_to_date) {
			avl_remove(&other_acf, acf);
			free(acf);
		}
	}

	free(pos);
}

/*
 * Based on the vertical velocity and rate of vvel change of `acf', this
 * routine predicts how the aircraft's vertical velocity is going to change.
 * The function returns the predicted target vertical velocity the aircraft
 * will attain at its current rate of vertical velocity change (d_vvel).
 */
static double
predict_vvel_trend(const tcas_acf_t *acf)
{
	if (acf->vvel > -LEVEL_VVEL_THRESH && acf->vvel < LEVEL_VVEL_THRESH) {
		/* aircraft is in level flight */
		return (0.0);
	} else {
		/* aircraft is either climbing or descending */
		int vvel_sign = (acf->vvel > 0) ? 1 : -1;
		ASSERT(acf->vvel != 0.0);

		/*
		 * If the rate of change in vvel is significantly opposite
		 * to the immediate vvel, the aircraft is most likely
		 * leveling off.
		 */
		if ((vvel_sign < 0 && acf->d_vvel > LEVEL_D_VVEL_THRESH) ||
		    (vvel_sign > 0 && acf->d_vvel < -LEVEL_D_VVEL_THRESH)) {
			return (0.0);
		/*
		 * If the rate of change in vvel is significantly in the
		 * direction of the immediate vvel, the aircraft's rate of
		 * climb/descent is most likely increasing.
		 */
		} else if ((vvel_sign < 0 && acf->d_vvel <
		    -LEVEL_D_VVEL_THRESH) ||
		    (vvel_sign > 0 && acf->d_vvel > LEVEL_D_VVEL_THRESH)) {
			double abs_vvel = fabs(acf->vvel);
			/*
			 * Try to estimate what it's going to increase to,
			 * given some typical average rates of climb. Above
			 * CLB2_VVEL we'll assume that no more increase is
			 * going to occur.
			 */
			if (abs_vvel < CLB1_VVEL)
				return (vvel_sign * CLB1_VVEL);
			else if (abs_vvel < CLB2_VVEL)
				return (vvel_sign * CLB2_VVEL);
			else
				return (acf->vvel);
		/*
		 * The rate of change is not significant anymore, so it's
		 * likely that the aircraft has reached its target vvel.
		 */
		} else {
			return (acf->vvel);
		}
	}
}

static void
extrapolate_acf_pos(tcas_acf_t *acf, unsigned secs, const fpp_t *fpp)
{
	double trk = acf->trk, gs = acf->gs, vvel = acf->vvel;
	double targ_vvel;
	vect2_t cur_pos;
	double alt = CUR_OBJ_ALT_MSL(&acf->pos);

	free(acf->extr_pos);
	acf->extr_pos = NULL;
	acf->num_extr_pos = 0;

	targ_vvel = predict_vvel_trend(acf);
	cur_pos = geo2fpp(GEO3_TO_GEO2(CUR_OBJ_POS3(&acf->pos)), fpp);
	if (IS_NULL_VECT(cur_pos))
		return;

	acf->num_extr_pos = secs;
	acf->extr_pos = calloc(secs, sizeof (*acf->extr_pos));

	for (unsigned i = 0; i < secs; i++) {
		vect2_t dir;

		if (fabs(vvel - targ_vvel) >
		    (fabs(vvel + acf->d_vvel - targ_vvel))) {
			/*
			 * If d_vvel causes us to converge on the target
			 * vvel, use it.
			 */
			vvel += acf->d_vvel;
		} else {
			/* Otherwise just jump to the target vvel. */
			vvel = targ_vvel;
		}

		trk += acf->d_trk;
		gs += acf->d_gs;
		alt += vvel;
		dir = hdg2dir(trk);

		cur_pos = vect2_add(cur_pos, vect2_set_abs(dir, KT2MPS(gs)));

		acf->extr_pos[i].x = cur_pos.x;
		acf->extr_pos[i].y = cur_pos.y;
		acf->extr_pos[i].z = alt;
	}
}

static void
main_loop(void *ignored)
{
	UNUSED(ignored);

	mutex_enter(&worker_lock);
	for (uint64_t now = microclock(); !worker_shutdown;
	    now += WORKER_LOOP_INTVAL) {
		const SL_t *sl;
		fpp_t fpp;

		mutex_enter(&acf_lock);
		fpp = stereo_fpp_init(GEO3_TO_GEO2(CUR_OBJ_POS3(&my_acf.pos)),
		    0, &wgs84, B_FALSE);
		sl = xtcas_SL_select(CUR_OBJ_ALT_MSL(&my_acf.pos),
		    CUR_OBJ_ALT_AGL(&my_acf.pos));
		extrapolate_acf_pos(&my_acf, EXTRAPOLATION_STEPS, &fpp);
		for (tcas_acf_t *acf = avl_first(&other_acf);
		    acf != NULL; AVL_NEXT(&other_acf, acf))
			extrapolate_acf_pos(acf, EXTRAPOLATION_STEPS, &fpp);
		mutex_exit(&acf_lock);

		cv_timedwait(&worker_cv, &worker_lock,
		    now + WORKER_LOOP_INTVAL);
	}
	mutex_exit(&worker_lock);

}

#if	PREDICTION_DEBUG

#define	SET_DRAW_INFO(i, v) \
	do { \
		geo_pos2_t p = fpp2geo(VECT2((v)->x, (v)->y), &fpp); \
		vect3_t loc; \
		XPLMWorldToLocal(p.lat, p.lon, (v)->z, &loc.x, &loc.y, &loc.z);\
		*(i) = (XPLMDrawInfo_t){ \
		    .structSize = sizeof (XPLMDrawInfo_t), \
		    .x = loc.x, .y = loc.y, .z = loc.z, \
		    .pitch = 0, .heading = 0, .roll = 0 \
		}; \
	} while (0)

static int
draw_prediction_objects(XPLMDrawingPhase phase, int before, void *ref)
{
	UNUSED(phase);
	UNUSED(before);
	UNUSED(ref);

	XPLMDrawInfo_t *draw_info;
	fpp_t fpp;

	ASSERT(prediction_object != NULL);

	mutex_enter(&acf_lock);
	fpp = stereo_fpp_init(GEO3_TO_GEO2(CUR_OBJ_POS3(&my_acf.pos)),
	    0, &wgs84, B_TRUE);
	draw_info = calloc(my_acf.num_extr_pos, sizeof (*draw_info));
	for (size_t i = 0; i < my_acf.num_extr_pos; i++)
		SET_DRAW_INFO(&draw_info[i], &my_acf.extr_pos[i]);
	if (my_acf.num_extr_pos != 0)
		XPLMDrawObjects(prediction_object, my_acf.num_extr_pos,
		    draw_info, 1, 1);
	free(draw_info);
	mutex_exit(&acf_lock);

	return (1);
}

#endif	/* PREDICTION_DEBUG */

void
xtcas_run(void)
{
	double t = xtcas_get_time();

	if (t <= last_t)
		return;

	mutex_enter(&acf_lock);
	update_my_position(t);
	update_bogie_positions(t);

	if (!my_acf.acf_ready) {
		mutex_exit(&acf_lock);
		return;
	}
	mutex_exit(&acf_lock);
	last_t = t;
}

void
xtcas_init(void)
{
	memset(&my_acf, 0, sizeof (my_acf));
	avl_create(&other_acf, acf_compar,
	    sizeof (tcas_acf_t), offsetof(tcas_acf_t, node));
	mutex_init(&acf_lock);

	mutex_init(&worker_lock);
	cv_init(&worker_cv);
	VERIFY(thread_create(&worker_thr, main_loop, NULL));

#if	PREDICTION_DEBUG
	prediction_object = XPLMLoadObject("Resources/default scenery/"
	    "airport scenery/Aircraft/General_Aviation/Cessna_172.obj");
	VERIFY(prediction_object != NULL);
	XPLMRegisterDrawCallback(draw_prediction_objects,
	    xplm_Phase_Objects, 0, NULL);
#endif	/* PREDICTION_DEBUG */
}

void
xtcas_fini(void)
{
	void *cookie = NULL;
	tcas_acf_t *p;

#if	PREDICTION_DEBUG
	XPLMUnloadObject(prediction_object);
	prediction_object = NULL;
	XPLMUnregisterDrawCallback(draw_prediction_objects,
	    xplm_Phase_Objects, 0, NULL);
#endif	/* PREDICTION_DEBUG */

	mutex_enter(&worker_lock);
	worker_shutdown = B_TRUE;
	cv_broadcast(&worker_cv);
	mutex_exit(&worker_lock);
	thread_join(&worker_thr);

	while ((p = avl_destroy_nodes(&other_acf, &cookie)) != NULL)
		free(p);
	avl_destroy(&other_acf);
	mutex_destroy(&acf_lock);

	cv_destroy(&worker_cv);
	mutex_destroy(&worker_lock);
}

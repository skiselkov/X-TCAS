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
#include <string.h>
#include <stddef.h>

#include "helpers.h"
#include "SL.h"
#include "geom.h"
#include "pos.h"
#include "xplane.h"
#include "avl.h"

#include "xtcas.h"

#define	BOGIE_TBL_SZ	128

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
	void *acf_id;		/* identifier - used for locating in tree */
	obj_pos_t pos;		/* position derivatives */
	double gs, d_gs;	/* true groundspeed and rate of GS change */
	double trk, d_trk;	/* true track and rate of track change */
	double vvel;		/* computed immediate vertical velocity */
	bool_t acf_ready;	/* indicates if derivatives are available */
	bool_t up_to_date;	/* used for efficient position updates */

	avl_node_t node;	/* used by other_acf tree */
} tcas_acf_t;

static tcas_acf_t my_acf;
static avl_tree_t other_acf;
static double last_t = 0;

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

	xtcas_get_acf_pos(MY_ACF_ID, &new_pos, &new_agl);
	xtcas_obj_pos_update(&my_acf.pos, t, new_pos, new_agl);
	my_acf.acf_ready = (
	    xtcas_obj_pos_get_gs(&my_acf.pos, &my_acf.gs, &my_acf.d_gs) &&
	    xtcas_obj_pos_get_trk(&my_acf.pos, &my_acf.trk, &my_acf.d_trk) &&
	    xtcas_obj_pos_get_vvel(&my_acf.pos, &my_acf.vvel, NULL));
}

static void
update_bogie_positions(double t)
{
	void **id_list;
	size_t count;

	xtcas_get_acf_ids(&id_list, &count);

	/* walk the tree and mark all acf as out-of-date */
	for (tcas_acf_t *acf = avl_first(&other_acf); acf != NULL;
	    acf = AVL_NEXT(&other_acf, acf))
		acf->up_to_date = B_FALSE;

	for (size_t i = 0; i < count; i++) {
		avl_index_t where;
		tcas_acf_t srch = { .acf_id = id_list[i] };
		tcas_acf_t *acf = avl_find(&other_acf, &srch, &where);
		geo_pos3_t new_pos;
		double alt_agl;

		if (acf == NULL) {
			acf = calloc(1, sizeof (*acf));
			acf->acf_id = id_list[i];
			avl_insert(&other_acf, acf, where);
		}
		xtcas_get_acf_pos(acf->acf_id, &new_pos, &alt_agl);
		xtcas_obj_pos_update(&acf->pos, t, new_pos, alt_agl);
		acf->acf_ready = (
		    xtcas_obj_pos_get_gs(&acf->pos, &acf->gs, &acf->d_gs) &&
		    xtcas_obj_pos_get_trk(&acf->pos, &acf->trk, &acf->d_trk) &&
		    xtcas_obj_pos_get_vvel(&acf->pos, &acf->vvel, NULL));
		/* mark acf as up-to-date */
		acf->up_to_date = B_TRUE;
	}

	/* walk the tree again and more out-of-date aircraft */
	for (tcas_acf_t *acf = avl_first(&other_acf), *acf_next = NULL;
	    acf != NULL; acf = acf_next) {
		acf_next = AVL_NEXT(&other_acf, acf);
		if (!acf->up_to_date) {
			avl_remove(&other_acf, acf);
			free(acf);
		}
	}

	free(id_list);
}

void
xtcas_run(void)
{
	const SL_t *sl;
	double t = xtcas_get_time();

	if (t <= last_t)
		return;

	update_my_position(t);
	update_bogie_positions(t);

	if (!my_acf.acf_ready)
		return;

	sl = xtcas_SL_select(CUR_OBJ_ALT_MSL(&my_acf.pos),
	    CUR_OBJ_ALT_AGL(&my_acf.pos));

	last_t = t;
}

void
xtcas_init(void)
{
	memset(&my_acf, 0, sizeof (my_acf));
	avl_create(&other_acf, acf_compar,
	    sizeof (tcas_acf_t), offsetof(tcas_acf_t, node));
}

void
xtcas_fini(void)
{
	void *cookie = NULL;
	tcas_acf_t *p;

	while ((p = avl_destroy_nodes(&other_acf, &cookie)) != NULL)
		free(p);
	avl_destroy(&other_acf);
}

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
#include <stddef.h>

#include <XPLMDisplay.h>
#include <XPLMGraphics.h>

#include <acfutils/assert.h>
#include <acfutils/avl.h>
#include <acfutils/dr.h>
#include <acfutils/geom.h>
#include <acfutils/glew.h>
#include <acfutils/helpers.h>
#include <acfutils/perf.h>
#include <acfutils/safe_alloc.h>
#include <acfutils/thread.h>
#include <acfutils/types.h>

#include "xplane_test.h"

#define	DEBUG_INTF_SZ		500
#define	DEBUG_INTF_MARGIN	20
#define	DEBUG_INTF_SCALE	(40000 / DEBUG_INTF_SZ)
#define	SYM_SZ			5

typedef struct {
	void		*acf_id;
	double		rbrg;
	double		rdist;
	double		ralt;
	double		vs;
	tcas_threat_t	level;

	avl_node_t	node;
} contact_t;

static bool_t inited = B_FALSE;
static XPLMWindowID win = NULL;

static mutex_t contacts_lock;
static avl_tree_t contacts;

static int
dummy_func(void)
{
	return (1);
}

static void
draw(XPLMWindowID window, void *refcon)
{
	UNUSED(window);
	UNUSED(refcon);

	XPLMSetGraphicsState(0, 0, 0, 0, 1, 0, 0);

	glColor3f(0, 0, 0);
	glBegin(GL_QUADS);
	glVertex2f(0, 0);
	glVertex2f(0, DEBUG_INTF_SZ);
	glVertex2f(DEBUG_INTF_SZ, DEBUG_INTF_SZ);
	glVertex2f(DEBUG_INTF_SZ, 0);
	glEnd();

	/*
	 * Standby if mode is STBY and we have no contacts (TCAS test
	 * not in progress).
	 */
	if (xtcas_get_mode() == TCAS_MODE_STBY &&
	    avl_numnodes(&contacts) == 0) {
		glColor3f(1, 1, 1);
		glBegin(GL_LINES);
		glVertex2f(0, 0);
		glVertex2f(DEBUG_INTF_SZ, DEBUG_INTF_SZ);
		glVertex2f(0, DEBUG_INTF_SZ);
		glVertex2f(DEBUG_INTF_SZ, 0);
		glEnd();
		return;
	}

	glColor3f(1, 1, 1);
	glBegin(GL_LINES);
	glVertex2f(DEBUG_INTF_SZ / 2 - SYM_SZ, DEBUG_INTF_SZ / 2);
	glVertex2f(DEBUG_INTF_SZ / 2 + SYM_SZ, DEBUG_INTF_SZ / 2);
	glVertex2f(DEBUG_INTF_SZ / 2, DEBUG_INTF_SZ / 2 - SYM_SZ);
	glVertex2f(DEBUG_INTF_SZ / 2, DEBUG_INTF_SZ / 2 + SYM_SZ);
	glEnd();

	mutex_enter(&contacts_lock);
	for (contact_t *ctc = avl_first(&contacts); ctc != NULL;
	    ctc = AVL_NEXT(&contacts, ctc)) {
		vect2_t v = vect2_scmul(hdg2dir(ctc->rbrg), ctc->rdist);
		v.x = (v.x / DEBUG_INTF_SCALE) + (DEBUG_INTF_SZ / 2);
		v.y = (v.y / DEBUG_INTF_SCALE) + (DEBUG_INTF_SZ / 2);
		if (v.x < DEBUG_INTF_MARGIN || v.y < DEBUG_INTF_MARGIN ||
		    v.x > DEBUG_INTF_SZ - DEBUG_INTF_MARGIN ||
		    v.y > DEBUG_INTF_SZ - DEBUG_INTF_MARGIN)
			continue;
		switch (ctc->level) {
		case OTH_THREAT:
			glColor3f(1, 1, 1);
			glBegin(GL_LINE_LOOP);
			glVertex2f(v.x - SYM_SZ, v.y);
			glVertex2f(v.x, v.y - SYM_SZ);
			glVertex2f(v.x + SYM_SZ, v.y);
			glVertex2f(v.x, v.y + SYM_SZ);
			glEnd();
			break;
		case PROX_THREAT:
			glColor3f(1, 1, 1);
			glBegin(GL_QUADS);
			glVertex2f(v.x - SYM_SZ, v.y);
			glVertex2f(v.x, v.y + SYM_SZ);
			glVertex2f(v.x + SYM_SZ, v.y);
			glVertex2f(v.x, v.y - SYM_SZ);
			glEnd();
			break;
		case TA_THREAT:
			glColor3f(1, 1, 0);
			glBegin(GL_TRIANGLE_FAN);
			glVertex2f(v.x, v.y);
			glVertex2f(v.x, v.y + SYM_SZ);
			glVertex2f(v.x + SYM_SZ * 0.67, v.y + SYM_SZ * .67);
			glVertex2f(v.x + SYM_SZ, v.y);
			glVertex2f(v.x + SYM_SZ * 0.67, v.y - SYM_SZ * .67);
			glVertex2f(v.x, v.y - SYM_SZ);
			glVertex2f(v.x - SYM_SZ * 0.67, v.y - SYM_SZ * .67);
			glVertex2f(v.x - SYM_SZ, v.y);
			glVertex2f(v.x - SYM_SZ * 0.67, v.y + SYM_SZ * .67);
			glVertex2f(v.x, v.y + SYM_SZ);
			glEnd();
			break;
		default:
			glColor3f(1, 0, 0);
			glBegin(GL_QUADS);
			glVertex2f(v.x - SYM_SZ, v.y - SYM_SZ);
			glVertex2f(v.x - SYM_SZ, v.y + SYM_SZ);
			glVertex2f(v.x + SYM_SZ, v.y + SYM_SZ);
			glVertex2f(v.x + SYM_SZ, v.y - SYM_SZ);
			glEnd();
			break;
		}

		glBegin(GL_LINES);
		if (ctc->vs > FPM2MPS(250)) {
			glVertex2f(v.x - 2 * SYM_SZ, v.y - SYM_SZ);
			glVertex2f(v.x - 2 * SYM_SZ, v.y + SYM_SZ);
			glVertex2f(v.x - 2.5 * SYM_SZ, v.y + 0.5 * SYM_SZ);
			glVertex2f(v.x - 2 * SYM_SZ, v.y + SYM_SZ);
			glVertex2f(v.x - 2 * SYM_SZ, v.y + SYM_SZ);
			glVertex2f(v.x - 1.5 * SYM_SZ, v.y + 0.5 * SYM_SZ);
		} else if (ctc->vs < FPM2MPS(-250)) {
			glVertex2f(v.x - 2 * SYM_SZ, v.y - SYM_SZ);
			glVertex2f(v.x - 2 * SYM_SZ, v.y + SYM_SZ);
			glVertex2f(v.x - 2.5 * SYM_SZ, v.y - 0.5 * SYM_SZ);
			glVertex2f(v.x - 2 * SYM_SZ, v.y - SYM_SZ);
			glVertex2f(v.x - 2 * SYM_SZ, v.y - SYM_SZ);
			glVertex2f(v.x - 1.5 * SYM_SZ, v.y - 0.5 * SYM_SZ);
		}
		glEnd();
	}
	mutex_exit(&contacts_lock);
}

static int
contact_compar(const void *a, const void *b)
{
	const contact_t *ctc_a = a, *ctc_b = b;

	if (ctc_a->acf_id < ctc_b->acf_id)
		return (-1);
	else if (ctc_a->acf_id > ctc_b->acf_id)
		return (1);
	return (0);
}

void
xplane_test_init(void)
{
	int w, h;

	if (inited)
		return;

	XPLMGetScreenSize(&w, &h);
	win = XPLMCreateWindow(0, h - DEBUG_INTF_SZ, w - DEBUG_INTF_SZ, 0, 1,
	    draw, (XPLMHandleKey_f)(void *)dummy_func,
	    (XPLMHandleMouseClick_f)(void *)dummy_func, NULL);

	mutex_init(&contacts_lock);
	avl_create(&contacts, contact_compar, sizeof (contact_t),
	    offsetof(contact_t, node));

	inited = B_TRUE;
}

void
xplane_test_fini(void)
{
	void *cookie = NULL;
	contact_t *ctc;

	if (!inited)
		return;

	XPLMDestroyWindow(win);
	win = NULL;

	mutex_destroy(&contacts_lock);
	while ((ctc = avl_destroy_nodes(&contacts, &cookie)) != NULL)
		free(ctc);
	avl_destroy(&contacts);

	inited = B_FALSE;
}

void
xplane_test_update_contact(void *handle, void *acf_id, double rbrg,
    double rdist, double ralt, double vs, double trk, double gs,
    tcas_threat_t level)
{
	contact_t srch, *ctc;
	avl_index_t where;

	UNUSED(handle);
	UNUSED(trk);
	UNUSED(gs);

	if (!inited)
		return;

	srch.acf_id = acf_id;

	mutex_enter(&contacts_lock);
	ctc = avl_find(&contacts, &srch, &where);
	if (ctc == NULL) {
		ctc = safe_calloc(1, sizeof (*ctc));
		ctc->acf_id = acf_id;
		avl_insert(&contacts, ctc, where);
	}

	ctc->rbrg = rbrg;
	ctc->rdist = rdist;
	ctc->ralt = ralt;
	ctc->vs = vs;
	ctc->level = level;

	mutex_exit(&contacts_lock);
}

void
xplane_test_delete_contact(void *handle, void *acf_id)
{
	contact_t srch, *ctc;

	UNUSED(handle);

	if (!inited)
		return;

	srch.acf_id = acf_id;

	mutex_enter(&contacts_lock);
	ctc = avl_find(&contacts, &srch, NULL);
	if (ctc != NULL)
		avl_remove(&contacts, ctc);
	mutex_exit(&contacts_lock);

	free(ctc);
}

void
xplane_test_update_RA(void *handle, tcas_adv_t adv, tcas_msg_t msg,
    tcas_RA_type_t type, tcas_RA_sense_t sense, bool_t crossing,
    bool_t reversal, double min_sep_cpa, double min_green, double max_green,
    double min_red_lo, double max_red_lo, double min_red_hi, double max_red_hi)
{
	const char *adv_str, *sense_str;

	UNUSED(handle);

	if (!inited)
		return;

	if (adv == ADV_STATE_NONE) {
		logMsg("TCAS: CLEAR OF CONFLICT");
		return;
	} else if (adv == ADV_STATE_TA) {
		adv_str = "TA";
	} else {
		adv_str = "RA";
	}

	if  (sense == RA_SENSE_UPWARD)
		sense_str = "UP";
	else if (sense == RA_SENSE_LEVEL_OFF)
		sense_str = "LVL";
	else
		sense_str = "DN";

	logMsg("TCAS: adv:%s msg:%s type:%s sense:%s cross:%d rev:%d "
	    "sep:%.0f mingreen:%.2f maxgreen:%.2f minredlo:%.2f maxredlo:%.2f "
	    "minredhi:%.2f maxredhi:%.2f", adv_str, xtcas_RA_msg2text(msg),
	    type == RA_TYPE_CORRECTIVE ? "CORR" : "PREV", sense_str,
	    crossing, reversal, min_sep_cpa, min_green, max_green, min_red_lo,
	    max_red_lo, min_red_hi, max_red_hi);
}

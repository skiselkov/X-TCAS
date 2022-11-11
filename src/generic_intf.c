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
 * Copyright 2018 Saso Kiselkov. All rights reserved.
 */

#include <acfutils/thread.h>

#include "../xtcas/generic_intf.h"
#include "xplane.h"

static void generic_update_contact(void *handle, void *acf_id, double rbrg,
    double rdist, double ralt, double vs, double trk, double gs,
    tcas_threat_t level);
static void generic_delete_contact(void *handle, void *acf_id);
static void generic_update_RA(void *handle, tcas_adv_t adv, tcas_msg_t msg,
    tcas_RA_type_t type, tcas_RA_sense_t sense, bool_t crossing,
    bool_t reversal, double min_sep_cpa, double min_green,
    double max_green, double min_red_lo, double max_red_lo,
    double min_red_hi, double max_red_hi);
static void generic_update_RA_prediction(void *handle, tcas_msg_t msg,
    tcas_RA_type_t type, tcas_RA_sense_t sense, bool_t crossing,
    bool_t reversal, double min_sep_cpa);
static void generic_play_audio_msg(void *handle, tcas_msg_t msg);

static tcas_mode_t generic_get_mode(void);
static tcas_mode_t generic_get_mode_act(void);
static tcas_filter_t generic_get_filter(void);
static void generic_test(bool_t force_fail);
static bool_t generic_test_is_in_prog(void);
static void generic_set_output_ops(const sim_intf_output_ops_t *ops);

static sim_intf_output_ops_t my_ops = {
    .handle = NULL,
    .update_contact = generic_update_contact,
    .delete_contact = generic_delete_contact,
    .update_RA = generic_update_RA,
    .update_RA_prediction = generic_update_RA_prediction,
    .play_audio_msg = generic_play_audio_msg
};
/*
 * We need to track init state, because we might be called by external
 * plugins after we were de-inited. So in those cases we gracefully
 * ignore their requests.
 */
static bool_t inited = B_FALSE;
static const sim_intf_output_ops_t *out_ops = NULL;
static mutex_t out_ops_lock;

static xtcas_generic_intf_t generic_ops = {
    .set_mode = generic_set_mode,
    .get_mode = generic_get_mode,
    .get_mode_act = generic_get_mode_act,
    .set_filter = generic_set_filter,
    .get_filter = generic_get_filter,
    .test = generic_test,
    .test_is_in_prog = generic_test_is_in_prog,
    .set_output_ops = generic_set_output_ops,
    .set_has_RA = xtcas_set_has_RA,
    .set_has_WOW = xtcas_set_has_WOW,
    .set_RA = xtcas_set_RA,
    .set_WOW = xtcas_set_WOW,
    .set_gear_ext = xtcas_set_gear_ext
};

static void
generic_update_contact(void *handle, void *acf_id, double rbrg,
    double rdist, double ralt, double vs, double trk, double gs,
    tcas_threat_t level)
{
	UNUSED(handle);

	if (!inited)
		return;
	mutex_enter(&out_ops_lock);
	if (out_ops != NULL && out_ops->update_contact != NULL) {
		out_ops->update_contact(out_ops->handle, acf_id,
		    rbrg, rdist, ralt, vs, trk, gs, level);
	}
	mutex_exit(&out_ops_lock);
}

static void
generic_delete_contact(void *handle, void *acf_id)
{
	UNUSED(handle);

	if (!inited)
		return;
	mutex_enter(&out_ops_lock);
	if (out_ops != NULL && out_ops->delete_contact != NULL)
		out_ops->delete_contact(out_ops->handle, acf_id);
	mutex_exit(&out_ops_lock);
}

static void
generic_update_RA(void *handle, tcas_adv_t adv, tcas_msg_t msg,
    tcas_RA_type_t type, tcas_RA_sense_t sense, bool_t crossing,
    bool_t reversal, double min_sep_cpa, double min_green,
    double max_green, double min_red_lo, double max_red_lo,
    double min_red_hi, double max_red_hi)
{
	UNUSED(handle);

	if (!inited)
		return;
	mutex_enter(&out_ops_lock);
	if (out_ops != NULL && out_ops->update_RA != NULL) {
		out_ops->update_RA(out_ops->handle, adv, msg, type, sense,
		    crossing, reversal, min_sep_cpa, min_green, max_green,
		    min_red_lo, max_red_lo, min_red_hi, max_red_hi);
	}
	mutex_exit(&out_ops_lock);
}

static void
generic_play_audio_msg(void *handle, tcas_msg_t msg)
{
	UNUSED(handle);

	if (!inited)
		return;
	mutex_enter(&out_ops_lock);
	if (out_ops != NULL && out_ops->play_audio_msg != NULL)
		out_ops->play_audio_msg(out_ops->handle, msg);
	mutex_exit(&out_ops_lock);
}

static void
generic_update_RA_prediction(void *handle, tcas_msg_t msg,
    tcas_RA_type_t type, tcas_RA_sense_t sense, bool_t crossing,
    bool_t reversal, double min_sep_cpa)
{
	UNUSED(handle);

	if (!inited)
		return;
	mutex_enter(&out_ops_lock);
	if (out_ops != NULL && out_ops->update_RA_prediction != NULL) {
		out_ops->update_RA_prediction(out_ops->handle, msg, type,
		    sense, crossing, reversal, min_sep_cpa);
	}
	mutex_exit(&out_ops_lock);
}

static tcas_mode_t
generic_get_mode(void)
{
	if (!inited)
		return (0);
	return (xtcas_get_mode());
}

static tcas_mode_t
generic_get_mode_act(void)
{
	if (!inited)
		return (0);
	return (xtcas_get_mode_act());
}

static tcas_filter_t
generic_get_filter(void)
{
	if (!inited)
		return (0);
	return (xtcas_get_filter());
}

static void
generic_test(bool_t force_fail)
{
	if (!inited)
		return;
	xtcas_test(force_fail);
}

static bool_t
generic_test_is_in_prog(void)
{
	if (!inited)
		return (B_FALSE);
	return (xtcas_test_is_in_prog());
}

static void
generic_set_output_ops(const sim_intf_output_ops_t *ops)
{
	if (!inited)
		return;
	mutex_enter(&out_ops_lock);
	out_ops = ops;
	mutex_exit(&out_ops_lock);
}

void
generic_intf_init(void)
{
	ASSERT(!inited);
	inited = B_TRUE;
	mutex_init(&out_ops_lock);
}

void
generic_intf_fini(void)
{
	if (!inited)
		return;
	inited = B_FALSE;

	mutex_enter(&out_ops_lock);
	out_ops = NULL;
	mutex_exit(&out_ops_lock);

	mutex_destroy(&out_ops_lock);
}

sim_intf_output_ops_t *
generic_intf_get_xtcas_ops(void)
{
	if (out_ops != NULL)
		return (&my_ops);
	else
		return (NULL);
}

xtcas_generic_intf_t *
generic_intf_get_intf_ops(void)
{
	return (&generic_ops);
}

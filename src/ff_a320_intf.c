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

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include <XPLMDataAccess.h>
#include <XPLMDisplay.h>
#include <XPLMGraphics.h>
#include <XPLMPlanes.h>
#include <XPLMPlugin.h>
#include <XPLMProcessing.h>
#include <XPLMUtilities.h>

#include <FF_A320/SharedValue.h>

#include <acfutils/assert.h>
#include <acfutils/avl.h>
#include <acfutils/dr.h>
#include <acfutils/geom.h>
#include <acfutils/log.h>
#include <acfutils/helpers.h>
#include <acfutils/perf.h>
#include <acfutils/types.h>
#include <acfutils/thread.h>

#include "dbg_log.h"
#include "snd_sys.h"
#include "xtcas.h"

#define	MAX_CONTACTS		100

typedef enum {
	UPDATE_DELETE,
	UPDATE_OTHER_THREAT,
	UPDATE_PROX_THREAT,
	UPDATE_TA_THREAT,
	UPDATE_RA_THREAT
} update_type_t;

typedef struct {
	bool_t		in_use;
	bool_t		deleted;

	const void	*acf_id;
	double		rbrg;
	double		rdist;
	double		ralt;
	double		vs;
	double		trk;
	tcas_threat_t	level;
	avl_node_t	node;
} contact_t;

SharedValuesInterface	svi;
mutex_t			lock;
static avl_tree_t	contacts_tree;
contact_t		contacts_array[MAX_CONTACTS];
static struct {
	bool_t		inited;

	/* inputs */
	int		valid;
	int		cancel;
	int		suppress;
	int		on_ground;
	int		gear_down;
	int		filter;
	int		mode;

	/* system-wide outputs */
	int		sys_state;
	int		adv_type;
	int		green_band_top;
	int		green_band_bottom;
	int		vs_area_mask;
	int		alert;

	/* per-intruder outputs */
	int		intr_index;
	int		intr_upd_type;
	int		intr_rbrg;
	int		intr_rdist;
	int		intr_ralt;
	int		intr_trend;
	int		intr_trk;
} ids;

static dr_t magvar_dr;
static double magvar;

static void __stdcall ff_a320_update(double step, void *tag);
static void update_contact(void *handle, void *acf_id, double rbrg,
    double rdist, double ralt, double vs, double trk, double gs,
    tcas_threat_t level);
static void delete_contact(void *handle, void *acf_id);
static void update_RA(void *handle, tcas_adv_t adv, tcas_msg_t msg,
    tcas_RA_type_t type, tcas_RA_sense_t sense, bool_t crossing,
    bool_t reversal, double min_sep_cpa, double min_green,
    double max_green, double min_red_lo, double max_red_lo,
    double min_red_hi, double max_red_hi);

static const char *type2str(unsigned int t);

static const sim_intf_output_ops_t ops = {
	.handle = NULL,
	.update_contact = update_contact,
	.delete_contact = delete_contact,
	.update_RA = update_RA,
};

static struct {
	tcas_adv_t adv;
	double min_green;
	double max_green;
	bool_t lower_red;
	bool_t upper_red;
} tcas;

static int
ctc_compar(const void *a, const void *b)
{
	const contact_t *ca = a, *cb = b;

	if (ca->acf_id < cb->acf_id)
		return (-1);
	else if (ca->acf_id > cb->acf_id)
		return (1);
	return (0);
}

const sim_intf_output_ops_t *
ff_a320_intf_init(void)
{
	XPLMPluginID plugin;
	char author[64];
	dr_t author_dr;

	fdr_find(&author_dr, "sim/aircraft/view/acf_author");
	dr_gets(&author_dr, author, sizeof (author));
	if (strcmp(author, "FlightFactor") != 0) {
		dbg_log(ff_a320, 1, "init fail: not FF");
		return (NULL);
	}

	plugin = XPLMFindPluginBySignature(XPLM_FF_SIGNATURE);
	if (plugin == XPLM_NO_PLUGIN_ID) {
		dbg_log(ff_a320, 1, "init fail: plugin not found");
		return (NULL);
	}
	XPLMSendMessageToPlugin(plugin, XPLM_FF_MSG_GET_SHARED_INTERFACE, &svi);
	if (svi.DataAddUpdate == NULL) {
		dbg_log(ff_a320, 1, "init fail: func vector empty");
		return (NULL);
	}

	svi.DataAddUpdate((SharedDataUpdateProc)ff_a320_update, NULL);

	memset(&ids, 0, sizeof (ids));
	memset(&contacts_array, 0, sizeof (contacts_array));
	mutex_init(&lock);
	avl_create(&contacts_tree, ctc_compar, sizeof (contact_t),
	    offsetof(contact_t, node));

	fdr_find(&magvar_dr, "sim/flightmodel/position/magnetic_variation");

	return (&ops);
}

void
ff_a320_intf_fini(void)
{
	void *cookie = NULL;

	if (svi.DataDelUpdate != NULL)
		svi.DataDelUpdate((SharedDataUpdateProc)ff_a320_update, NULL);
	mutex_destroy(&lock);

	while (avl_destroy_nodes(&contacts_tree, &cookie) != NULL)
		;
	avl_destroy(&contacts_tree);

	dbg_log(ff_a320, 1, "fini");
}

void
ff_a320_intf_update(void)
{
	magvar = dr_getf(&magvar_dr);
}

static inline int32_t
gets32(int id)
{
	int val;
	unsigned int type = svi.ValueType(id);
	ASSERT_MSG(type >= Value_Type_sint8 && type <= Value_Type_uint32,
	    "%s isn't an integer type, instead it is %s",
	    svi.ValueName(id), type2str(type));
	svi.ValueGet(id, &val);
	return (val);
}

static inline void
sets32(int id, int32_t val)
{
	unsigned int type = svi.ValueType(id);
	ASSERT_MSG(type >= Value_Type_sint8 && type <= Value_Type_uint32,
	    "%s isn't an integer type, instead it is %s",
	    svi.ValueName(id), type2str(type));
	svi.ValueSet(id, &val);
}

static inline float getf32(int id) __attribute__((unused));
static inline float
getf32(int id)
{
	float val;
	unsigned int type = svi.ValueType(id);
	ASSERT_MSG(type == Value_Type_float32,
	    "%s isn't a float32 type, instead it is %s",
	    svi.ValueName(id), type2str(type));
	svi.ValueGet(id, &val);
	return (val);
}

static inline void
setf32(int id, float val)
{
	unsigned int type = svi.ValueType(id);
	ASSERT_MSG(type == Value_Type_float32,
	    "%s isn't a float32 type, instead it is %s",
	    svi.ValueName(id), type2str(type));
	svi.ValueSet(id, &val);
}

static const char *
type2str(unsigned int t)
{
	switch (t) {
	case Value_Type_Deleted:
		return "deleted";
	case Value_Type_Object:
		return "object";
	case Value_Type_sint8:
		return "sint8";
	case Value_Type_uint8:
		return "uint8";
	case Value_Type_sint16:
		return "sint16";
	case Value_Type_uint16:
		return "uint16";
	case Value_Type_sint32:
		return "sint32";
	case Value_Type_uint32:
		return "uint32";
	case Value_Type_float32:
		return "float32";
	case Value_Type_float64:
		return "float64";
	case Value_Type_String:
		return "string";
	case Value_Type_Time:
		return "time";
	default:
		return "unknown";
	}
}
static void
units2str(unsigned int units, char buf[32])
{
	snprintf(buf, 32, "%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s%s"
	    "%s%s%s%s%s%s%s%s%s%s%s%s",
	    (units & Value_Unit_Object) ? "O" : "",
	    (units & Value_Unit_Failure) ? "F" : "",
	    (units & Value_Unit_Button) ? "B" : "",
	    (units & Value_Unit_Ratio) ? "R" : "",
	    (units & Value_Unit_State) ? "s" : "",
	    (units & Value_Unit_Flags) ? "f" : "",
	    (units & Value_Unit_Ident) ? "I" : "",
	    (units & Value_Unit_Length) ? "M" : "",
	    (units & Value_Unit_Speed) ? "S" : "",
	    (units & Value_Unit_Accel) ? "^" : "",
	    (units & Value_Unit_Force) ? "N" : "",
	    (units & Value_Unit_Weight) ? "K" : "",
	    (units & Value_Unit_Angle) ? "D" : "",
	    (units & Value_Unit_AngularSpeed) ? "@" : "",
	    (units & Value_Unit_AngularAccel) ? "c" : "",
	    (units & Value_Unit_Temperature) ? "t" : "",
	    (units & Value_Unit_Pressure) ? "P" : "",
	    (units & Value_Unit_Flow) ? "L" : "",
	    (units & Value_Unit_Voltage) ? "V" : "",
	    (units & Value_Unit_Frequency) ? "H" : "",
	    (units & Value_Unit_Current) ? "A" : "",
	    (units & Value_Unit_Power) ? "W": "",
	    (units & Value_Unit_Density) ? "d" : "",
	    (units & Value_Unit_Volume) ? "v" : "",
	    (units & Value_Unit_Conduction) ? "S" : "",
	    (units & Value_Unit_Capacity) ? "C" : "",
	    (units & Value_Unit_Heat) ? "T" : "",
	    (units & Value_Unit_Position) ? "r" : "",
	    (units & Value_Unit_TimeDelta) ? "'" : "",
	    (units & Value_Unit_TimeStart) ? "`" : "",
	    (units & Value_Unit_Label) ? "9" : "");
}

static int
val_id(const char *name)
{
	int id = svi.ValueIdByName(name);
	char units[32];

	units2str(id, units);
	dbg_log(ff_a320, 3, "%-44s  %-8s  %02x  %-10s  %s", name,
	    type2str(svi.ValueType(id)),
	    svi.ValueFlags(id),
	    svi.ValueDesc(id),
	    units);

	return (id);
}

static void
ff_a320_ids_init(void)
{
	if (ids.inited)
		return;

	ids.valid = val_id("Aircraft.Navigation.TCAS.Valid");
	ids.cancel = val_id("Aircraft.Navigation.TCAS.Cancel");
	ids.suppress = val_id("Aircraft.Navigation.TCAS.Suppress");
	ids.on_ground = val_id("Aircraft.Navigation.TCAS.OnGround");
	ids.gear_down = val_id("Aircraft.Navigation.TCAS.GearsDown");
	ids.filter = val_id("Aircraft.Navigation.TCAS.Show");
	ids.mode = val_id("Aircraft.Navigation.TCAS.Mode");

	ids.intr_index = val_id("Aircraft.Navigation.TCAS.Index");
	ids.intr_upd_type = val_id("Aircraft.Navigation.TCAS.Type");
	ids.intr_rbrg = val_id("Aircraft.Navigation.TCAS.RelBearing");
	ids.intr_rdist = val_id("Aircraft.Navigation.TCAS.RelDistance");
	ids.intr_ralt = val_id("Aircraft.Navigation.TCAS.RelAltitude");
	ids.intr_trend = val_id("Aircraft.Navigation.TCAS.Trend");
	ids.intr_trk = val_id("Aircraft.Navigation.TCAS.Track");

	ids.sys_state = val_id("Aircraft.Navigation.TCAS.State");
	ids.adv_type = val_id("Aircraft.Navigation.TCAS.Advisory");
	ids.green_band_top = val_id("Aircraft.Navigation.TCAS.AdvisoryTrendUP");
	ids.green_band_bottom =
	    val_id("Aircraft.Navigation.TCAS.AdvisoryTrendDN");
	ids.vs_area_mask = val_id("Aircraft.Navigation.TCAS.AdvisoryAreas");

	ids.alert = val_id("Aircraft.Navigation.TCAS.Alert");

	ids.inited = B_TRUE;
}

static void __stdcall
ff_a320_update(double step, void *tag)
{
	static int last_slot = 0;
	int i;
	int vs_band_mask = 0;
#ifndef	XTCAS_NO_AUDIO
	bool_t suppress;
#endif
	tcas_mode_t mode;
	tcas_filter_t filter = TCAS_FILTER_ALL;

	UNUSED(step);
	UNUSED(tag);

	VERIFY(svi.ValueIdByName != NULL);
	VERIFY(svi.ValueGet != NULL);
	VERIFY(svi.ValueType != NULL);
	VERIFY(svi.ValueName != NULL);

	ff_a320_ids_init();

	/* State input */
	mode = gets32(ids.mode);
	if (mode != xtcas_get_mode()) {
		dr_t dr;
		fdr_find(&dr, "xtcas/mode_req");
		dr_seti(&dr, mode);
		dbg_log(ff_a320, 1, "set_mode:%d", mode);
	}
	switch (gets32(ids.filter)) {
	case 0:
		filter = TCAS_FILTER_THRT;
		break;
	case 1:
		filter = TCAS_FILTER_ALL;
		break;
	case 2:
		filter = TCAS_FILTER_ABV;
		break;
	case 3:
		filter = TCAS_FILTER_BLW;
		break;
	}
	if (filter != xtcas_get_filter()) {
		dr_t dr;
		fdr_find(&dr, "xtcas/filter_req");
		dr_seti(&dr, filter);
		dbg_log(ff_a320, 1, "set_filter:%d", filter);
	}

#ifndef	XTCAS_NO_AUDIO
	if (gets32(ids.cancel) != 0 && xtcas_msg_is_playing()) {
		dbg_log(ff_a320, 1, "Cancelling playing message");
		xtcas_stop_msg(B_TRUE);
	}
	suppress = (gets32(ids.suppress) != 0);
	if (suppress != xtcas_is_suppressed()) {
		dbg_log(ff_a320, 1, "set suppressed %d", suppress);
		xtcas_set_suppressed(suppress);
	}
#endif	/* !defined(XTCAS_NO_AUDIO) */

	if (mode == TCAS_MODE_TARA && xtcas_get_SL() <= 2)
		mode = TCAS_MODE_TAONLY;

	/* System-wide state output */
	sets32(ids.sys_state, mode);
	sets32(ids.adv_type, tcas.adv);
	setf32(ids.green_band_top, tcas.max_green);
	setf32(ids.green_band_bottom, tcas.min_green);
	if (tcas.upper_red)
		vs_band_mask |= (1 << 0);
	if (tcas.min_green != tcas.max_green)
		vs_band_mask |= (1 << 1);
	if (tcas.lower_red)
		vs_band_mask |= (1 << 2);
	sets32(ids.vs_area_mask, vs_band_mask);
#ifndef	XTCAS_NO_AUDIO
	sets32(ids.alert, xtcas_msg_is_playing());
#endif
	/*
	 * Per-aircraft updates.
	 * We perform one update per call and keep track of the slot we last
	 * serviced in `last_slot'.
	 */

	mutex_enter(&lock);
	for (i = 1; i <= MAX_CONTACTS; i++) {
		int slot = (last_slot + i) % MAX_CONTACTS;
		contact_t *ctc = &contacts_array[slot];

		if (ctc->in_use) {
			/* Update contact info */
			double rbrg = ctc->rbrg;

			/* convert to the -180..+180 format for the A320 */
			if (rbrg > 180)
				rbrg -= 360;

			dbg_log(ff_a320, 2, "ff_a320_update slot:%d acf_id:%p "
			    "rbrg:%.1f rdist:%.0f ralt:%.0f vs:%.2f lvl:%d",
			    slot, ctc->acf_id, rbrg, ctc->rdist, ctc->ralt,
			    ctc->vs, ctc->level);

			sets32(ids.intr_index, slot);
			switch (ctc->level) {
			case OTH_THREAT:
				sets32(ids.intr_upd_type, 1);
				break;
			case PROX_THREAT:
				sets32(ids.intr_upd_type, 2);
				break;
			case TA_THREAT:
				sets32(ids.intr_upd_type, 3);
				break;
			case RA_THREAT_PREV:
			case RA_THREAT_CORR:
				sets32(ids.intr_upd_type, 4);
				break;
			}
			setf32(ids.intr_rbrg, rbrg);
			setf32(ids.intr_rdist, ctc->rdist);
			setf32(ids.intr_ralt, ctc->ralt);
			if (ctc->vs >= LEVEL_VVEL_THRESH)
				sets32(ids.intr_trend, 1);
			else if (ctc->vs <= -LEVEL_VVEL_THRESH)
				sets32(ids.intr_trend, -1);
			else
				sets32(ids.intr_trend, 0);
			setf32(ids.intr_trk, ctc->trk);

			break;
		} else if (ctc->deleted) {
			/* Reset contact slot to a clean status */
			dbg_log(ff_a320, 2, "ff_a320_update delete slot:%d",
			    slot);
			sets32(ids.intr_index, slot);
			sets32(ids.intr_upd_type, 0);
			memset(ctc, 0, sizeof (*ctc));
			break;
		}
	}

	last_slot += i;

	mutex_exit(&lock);
}

static void
update_contact(void *handle, void *acf_id, double rbrg, double rdist,
    double ralt, double vs, double trk, double gs, tcas_threat_t level)
{
	contact_t srch, *ctc;
	avl_index_t where;

	UNUSED(handle);
	UNUSED(gs);

	dbg_log(ff_a320, 2, "update_contact acf_id:%p rpos:%.0fx%.0fx%.0f "
	    "vs:%.2f lvl:%d", acf_id, rbrg, rdist, ralt, vs, level);

	srch.acf_id = acf_id;

	mutex_enter(&lock);
	ctc = avl_find(&contacts_tree, &srch, &where);
	if (ctc == NULL) {
		for (int i = 0; i < MAX_CONTACTS; i++) {
			if (!contacts_array[i].in_use &&
			    !contacts_array[i].deleted) {
				ctc = &contacts_array[i];
				dbg_log(ff_a320, 2, "new contact, slot:%d", i);
				break;
			}
		}
		if (ctc == NULL) {
			logMsg("CAUTION: X-TCAS is out of contact slots! "
			    "Dropping contact %p.", acf_id);
			mutex_exit(&lock);
			return;
		}
		ctc->in_use = B_TRUE;
		ctc->acf_id = acf_id;
		avl_insert(&contacts_tree, ctc, where);
	}
	ctc->rbrg = rbrg;
	ctc->rdist = rdist;
	ctc->ralt = ralt;
	ctc->vs = vs;
	ctc->level = level;
	ctc->trk = trk;
	mutex_exit(&lock);
}

static void
delete_contact(void *handle, void *acf_id)
{
	contact_t srch, *ctc;

	UNUSED(handle);

	srch.acf_id = acf_id;
	mutex_enter(&lock);
	ctc = avl_find(&contacts_tree, &srch, NULL);
	if (ctc != NULL) {
		dbg_log(ff_a320, 2, "delete_contact acf_id:%p", acf_id);
		avl_remove(&contacts_tree, ctc);
		ctc->in_use = B_FALSE;
		ctc->deleted = B_TRUE;
	}
	mutex_exit(&lock);
}

static void
update_RA(void *handle, tcas_adv_t adv, tcas_msg_t msg,
    tcas_RA_type_t type, tcas_RA_sense_t sense, bool_t crossing,
    bool_t reversal, double min_sep_cpa, double min_green,
    double max_green, double min_red_lo, double max_red_lo,
    double min_red_hi, double max_red_hi)
{
	UNUSED(handle);
	UNUSED(msg);
	UNUSED(type);
	UNUSED(sense);
	UNUSED(crossing);
	UNUSED(reversal);
	UNUSED(min_sep_cpa);

	mutex_enter(&lock);
	tcas.adv = adv;
	if (min_green != max_green) {
		tcas.min_green = min_green;
		tcas.max_green = max_green;
		tcas.lower_red = (min_red_lo != max_red_lo);
		tcas.upper_red = (min_red_hi != max_red_hi);
	} else if (min_red_lo != max_red_lo) {
		tcas.lower_red = B_TRUE;
		tcas.upper_red = B_FALSE;
		tcas.min_green = tcas.max_green = max_red_lo;
	} else {
		tcas.lower_red = B_FALSE;
		tcas.upper_red = B_TRUE;
		tcas.min_green = tcas.max_green = min_red_hi;
	}
	dbg_log(ff_a320, 1, "update_RA adv:%d ming:%.2f maxg:%.2f lored:%d "
	    "hired:%d", tcas.adv, tcas.min_green, tcas.max_green,
	    tcas.lower_red, tcas.upper_red);
	mutex_exit(&lock);
}

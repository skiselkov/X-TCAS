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
#include "xtcas.h"

typedef struct {
	const void	*acf_id;
	geo_pos3_t	pos;
	double		trk;
	double		vs;
	tcas_threat_t	level;
	avl_node_t	node;
} contact_t;

SharedValuesInterface	svi;
mutex_t			lock;
static avl_tree_t	contacts;
static struct {
	bool_t		inited;

	/* inputs */
	int		valid;
	int		cancel;
	int		suppress;
	int		on_ground;
	int		gear_down;
	int		hdg;
	int		rad_alt;
	int		baro_alt;
	int		filter;
	int		mode;

	/* system-wide outputs */
	int		sys_state;
	int		adv_type;
	int		green_band_top;
	int		green_band_bottom;
	int		adv_area_mask;
	int		alert;

	/* per-intruder outputs */
	int		intr_index;
	int		intr_upd_type;
	int		intr_rbrg;
	int		intr_rdist;
	int		intr_ralt;
	int		intr_trend;
} ids;

static void ff_a320_update(double step, void *tag);
static void update_contact(void *handle, void *acf_id, geo_pos3_t pos,
    double trk, double vs, tcas_threat_t level);
static void delete_contact(void *handle, void *acf_id);
static void update_RA(void *handle, tcas_adv_t adv, tcas_msg_t msg,
    tcas_RA_type_t type, tcas_RA_sense_t sense, bool_t crossing,
    bool_t reversal, double min_sep_cpa, double min_green,
    double max_green, double min_red_lo, double max_red_lo,
    double min_red_hi, double max_red_hi);

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

	svi.DataAddUpdate(ff_a320_update, NULL);

	memset(&ids, 0, sizeof (ids));
	mutex_init(&lock);
	avl_create(&contacts, ctc_compar, sizeof (contact_t),
	    offsetof(contact_t, node));

	return (&ops);
}

void
ff_a320_intf_fini(void)
{
	void *cookie = NULL;
	contact_t *ctc;

	if (svi.DataDelUpdate != NULL)
		svi.DataDelUpdate(ff_a320_update, NULL);
	mutex_destroy(&lock);

	while ((ctc = avl_destroy_nodes(&contacts, &cookie)) != NULL)
		free(ctc);
	avl_destroy(&contacts);

	dbg_log(ff_a320, 1, "fini");
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

static inline double
getf64(int id)
{
	double val;
	unsigned int type = svi.ValueType(id);
	ASSERT_MSG(type == Value_Type_float64,
	    "%s isn't a float64 type, instead it is %s",
	    svi.ValueName(id), type2str(type));
	svi.ValueGet(id, &val);
	return (val);
}

static inline void
setf64(int id, double val)
{
	unsigned int type = svi.ValueType(id);
	ASSERT_MSG(type == Value_Type_float64,
	    "%s isn't a float64 type, instead it is %s",
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
	ids.hdg = val_id("Aircraft.Navigation.TCAS.Heading");
	ids.rad_alt = val_id("Aircraft.Navigation.TCAS.Height");
	ids.baro_alt= val_id("Aircraft.Navigation.TCAS.Altitude");
	ids.filter = val_id("Aircraft.Navigation.TCAS.Show");
	ids.mode = val_id("Aircraft.Navigation.TCAS.Mode");

	ids.intr_index = val_id("Aircraft.Navigation.TCAS.Index");
	ids.intr_upd_type = val_id("Aircraft.Navigation.TCAS.Type");
	ids.intr_rbrg = val_id("Aircraft.Navigation.TCAS.RelBearing");
	ids.intr_rdist = val_id("Aircraft.Navigation.TCAS.RelDistance");
	ids.intr_ralt = val_id("Aircraft.Navigation.TCAS.RelAltitude");
	ids.intr_trend = val_id("Aircraft.Navigation.TCAS.Trend");

	ids.sys_state = val_id("Aircraft.Navigation.TCAS.State");
	ids.adv_type = val_id("Aircraft.Navigation.TCAS.Advisory");
	ids.green_band_top = val_id("Aircraft.Navigation.TCAS.AdvisoryTrendUP");
	ids.green_band_bottom =
	    val_id("Aircraft.Navigation.TCAS.AdvisoryTrendDN");
	ids.adv_area_mask = val_id("Aircraft.Navigation.TCAS.AdvisoryAreas");

	ids.alert = val_id("Aircraft.Navigation.TCAS.Alert");

	ids.inited = B_TRUE;
}

static void
ff_a320_update(double step, void *tag)
{
	UNUSED(step);
	UNUSED(tag);

	VERIFY(svi.ValueIdByName != NULL);
	VERIFY(svi.ValueGet != NULL);
	VERIFY(svi.ValueType != NULL);
	VERIFY(svi.ValueName != NULL);

	ff_a320_ids_init();
}

static void
update_contact(void *handle, void *acf_id, geo_pos3_t pos, double trk,
    double vs, tcas_threat_t level)
{
	contact_t srch, *ctc;
	avl_index_t where;

	UNUSED(handle);

	srch.acf_id = acf_id;
	ctc = avl_find(&contacts, &srch, &where);
	if (ctc == NULL) {
		ctc = calloc(1, sizeof (*ctc));
		ctc->acf_id = acf_id;
		avl_insert(&contacts, ctc, where);
	}
	ctc->pos = pos;
	ctc->trk = trk;
	ctc->vs = vs;
	ctc->level = level;
}

static void
delete_contact(void *handle, void *acf_id)
{
	contact_t srch, *ctc;

	UNUSED(handle);

	srch.acf_id = acf_id;
	ctc = avl_find(&contacts, &srch, NULL);
	if (ctc != NULL) {
		avl_remove(&contacts, ctc);
		free(ctc);
	}
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
}

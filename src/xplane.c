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

#include <ctype.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include <XPLMDataAccess.h>
#include <XPLMDisplay.h>
#include <XPLMGraphics.h>
#include <XPLMPlanes.h>
#include <XPLMPlugin.h>
#include <XPLMProcessing.h>
#include <XPLMUtilities.h>

#include <acfutils/assert.h>
#include <acfutils/avl.h>
#include <acfutils/dr.h>
#include <acfutils/except.h>
#include <acfutils/geom.h>
#include <acfutils/glew.h>
#include <acfutils/log.h>
#include <acfutils/helpers.h>
#include <acfutils/perf.h>
#include <acfutils/safe_alloc.h>
#include <acfutils/types.h>
#include <acfutils/thread.h>

#include "../xtcas/generic_intf.h"
#include "dbg_log.h"
#include "ff_a320_intf.h"
#ifndef	XTCAS_NO_AUDIO
#include "snd_sys.h"
#endif
#include "xtcas.h"
#include "xplane.h"

#include "vsi.h"
#if	!VSI_DRAW_MODE
#include "xplane_test.h"
#endif

#define	FLOOP_INTVAL			0.1
#define	POS_UPDATE_INTVAL		0.1
#define	STARTUP_DELAY			5.0	/* seconds */
#define	XTCAS_PLUGIN_NAME		"X-TCAS (%x)"
#define	XTCAS_PLUGIN_DESCRIPTION \
	"Generic TCAS II v7.1 implementation for X-Plane"

#define	MAX_MP_PLANES		19
#define	MAX_TCAS_TARGETS	64
#define	MAX_DR_NAME_LEN		256
#define	CTC_INACT_DELAY		10	/* seconds */

#define	BUSNR_DFL	0
#define	BUSNR_MAX	6
#define	MIN_VOLTS_DFL	22

static bool_t intf_inited = B_FALSE;
static bool_t xtcas_inited = B_FALSE;
static bool_t standalone_mode UNUSED_ATTR;
static bool_t standalone_mode = B_FALSE;
static struct {
	dr_t	time;
	dr_t	elev;
	dr_t	rad_alt_ft;
	dr_t	hdg;
	dr_t	lat;
	dr_t	lon;
	dr_t	view_is_ext;
	dr_t	warn_volume;
	dr_t	sound_on;
	dr_t	bus_volts;
	dr_t	xpdr_mode;
	dr_t	xpdr_fail;
	dr_t	altm_fail;
	dr_t	adc_fail;
	dr_t	gear_deploy;
	dr_t	on_ground;

	bool_t	have_tcas_targets;	/* X-Plane 11.53 TCAS DRs valid */
	dr_t	tcas_target_lat;	/* deg[64] */
	dr_t	tcas_target_lon;	/* deg[64] */
	dr_t	tcas_target_elev;	/* m[64] */
	dr_t	tcas_target_on_gnd;	/* bool[64] */
	dr_t	tcas_target_number;	/* int */

	/* our datarefs */
	dr_t	busnr;
	dr_t	min_volts;
	dr_t	mode_req;
	dr_t	mode_act;
	dr_t	filter_req;
	dr_t	filter_act;
	dr_t	fail_dr_name_dr;

	/* provided by 3rd party */
	dr_t	custom_bus_dr;
} drs;

static struct {
	dr_t	x;
	dr_t	y;
	dr_t	z;
} mp_planes[MAX_MP_PLANES];

static mutex_t acf_pos_lock;
static avl_tree_t acf_pos_tree;
static geo_pos3_t my_acf_pos;
static double my_acf_agl = 0;
static double my_acf_hdg = 0;
static bool_t my_acf_gear_ext = B_FALSE;
static bool_t my_acf_on_ground = B_FALSE;
static double cur_sim_time = 0;
static double first_sim_time = 0;

static char plugindir[512] = { 0 };

static int busnr = BUSNR_DFL;
static float min_volts = MIN_VOLTS_DFL;
static char custom_bus_name[128] = { 0 };
static bool_t custom_bus = B_FALSE;
static int mode_req = -1;
static int mode_act = -1;
static int filter_req = -1;
static int filter_act = -1;
static char fail_dr_name[128] = { 0 };

const conf_t *xtcas_conf = NULL;
conf_t *conf = NULL;

static double xp_get_time(void *handle);
static void xp_get_my_acf_pos(void *handle, geo_pos3_t *pos, double *alt_agl,
    double *hdg, bool_t *gear_ext, bool_t *on_ground);
static void xp_get_oth_acf_pos(void *handle, acf_pos_t **pos_p, size_t *num);

static int tcas_config_handler(XPLMCommandRef, XPLMCommandPhase, void *);

#if	!VSI_DRAW_MODE
static int test_gui_handler(XPLMCommandRef, XPLMCommandPhase, void *);
static XPLMCommandRef show_test_gui_cmd;
static XPLMCommandRef hide_test_gui_cmd;
#endif	/* !VSI_DRAW_MODE */

static XPLMCommandRef filter_all_cmd, filter_thrt_cmd, filter_abv_cmd;
static XPLMCommandRef filter_blw_cmd;

static XPLMCommandRef mode_stby_cmd, mode_taonly_cmd, mode_tara_cmd;
static XPLMCommandRef tcas_test_cmd;

static const sim_intf_input_ops_t xp_intf_in_ops = {
	.handle = NULL,
	.get_time = xp_get_time,
	.get_my_acf_pos = xp_get_my_acf_pos,
	.get_oth_acf_pos = xp_get_oth_acf_pos,
};

#if	VSI_DRAW_MODE
static const sim_intf_output_ops_t vsi_out_ops = {
	.handle = NULL,
	.update_contact = vsi_update_contact,
	.delete_contact = vsi_delete_contact,
	.update_RA = vsi_update_RA,
	.update_RA_prediction = NULL
};
#else	/* !VSI_DRAW_MODE */
static const sim_intf_output_ops_t xplane_test_out_ops = {
	.handle = NULL,
	.update_contact = xplane_test_update_contact,
	.delete_contact = xplane_test_delete_contact,
	.update_RA = xplane_test_update_RA,
	.update_RA_prediction = NULL
};
#endif	/* !VSI_DRAW_MODE */

static bool_t ff_a320_intf_inited = B_FALSE;

static int
acf_pos_compar(const void *a, const void *b)
{
	const acf_pos_t *pa = a, *pb = b;

	if (pa->acf_id < pb->acf_id)
		return (-1);
	else if (pa->acf_id == pb->acf_id)
		return (0);
	else
		return (1);
}

static void
sim_intf_init(void)
{
	fdr_find(&drs.time, "sim/time/total_running_time_sec");
	fdr_find(&drs.elev, "sim/flightmodel/position/elevation");
	fdr_find(&drs.rad_alt_ft,
	    "sim/cockpit2/gauges/indicators/radio_altimeter_height_ft_pilot");
	fdr_find(&drs.lat, "sim/flightmodel/position/latitude");
	fdr_find(&drs.lon, "sim/flightmodel/position/longitude");
	fdr_find(&drs.hdg, "sim/flightmodel/position/true_psi");

	fdr_find(&drs.view_is_ext, "sim/graphics/view/view_is_external");
	fdr_find(&drs.warn_volume, "sim/operation/sound/warning_volume_ratio");
	fdr_find(&drs.sound_on, "sim/operation/sound/sound_on");

	for (int i = 0; i < MAX_MP_PLANES; i++) {
		fdr_find(&mp_planes[i].x,
		    "sim/multiplayer/position/plane%d_x", i + 1);
		fdr_find(&mp_planes[i].y,
		    "sim/multiplayer/position/plane%d_y", i + 1);
		fdr_find(&mp_planes[i].z,
		    "sim/multiplayer/position/plane%d_z", i + 1);
	}
	/*
	 * Since X-Plane 11.53, we can grab up to 64 contacts, including
	 * their WoW status.
	 */
	drs.have_tcas_targets =
	    (dr_find(&drs.tcas_target_lat,
	    "sim/cockpit2/tcas/targets/position/lat") &&
	    dr_find(&drs.tcas_target_lon,
	    "sim/cockpit2/tcas/targets/position/lon") &&
	    dr_find(&drs.tcas_target_elev,
	    "sim/cockpit2/tcas/targets/position/ele") &&
	    dr_find(&drs.tcas_target_on_gnd,
	    "sim/cockpit2/tcas/targets/position/weight_on_wheels") &&
	    dr_find(&drs.tcas_target_number,
	    "sim/cockpit2/tcas/indicators/tcas_num_acf"));

	avl_create(&acf_pos_tree, acf_pos_compar, sizeof (acf_pos_t),
	    offsetof(acf_pos_t, tree_node));
	mutex_init(&acf_pos_lock);
	custom_bus = B_FALSE;
	intf_inited = B_TRUE;
}

static void
sim_intf_fini(void)
{
	void *cookie = NULL;
	acf_pos_t *p;

	memset(&drs, 0, sizeof (drs));
	memset(&mp_planes, 0, sizeof (mp_planes));

	while ((p = avl_destroy_nodes(&acf_pos_tree, &cookie)) != NULL)
		free(p);
	avl_destroy(&acf_pos_tree);
	mutex_destroy(&acf_pos_lock);

	intf_inited = B_FALSE;
}

static float
acf_pos_collector(float elapsed1, float elapsed2, int counter, void *refcon)
{
	double gear_deploy[2];
	int on_ground[3];
	int num_planes;
	bool_t use_tcas_targets;

	UNUSED(elapsed1);
	UNUSED(elapsed2);
	UNUSED(counter);
	UNUSED(refcon);

	/* grab our aircraft position */
	my_acf_pos.lat = dr_getf(&drs.lat);
	my_acf_pos.lon = dr_getf(&drs.lon);
	my_acf_pos.elev = dr_getf(&drs.elev);
	my_acf_agl = FEET2MET(dr_getf(&drs.rad_alt_ft));
	my_acf_hdg = dr_getf(&drs.hdg);
	/* Two gear check should suffice - you're not landing on just one! */
	VERIFY3S(dr_getvf(&drs.gear_deploy, gear_deploy, 0, 2), ==, 2);
	my_acf_gear_ext = (gear_deploy[0] != 0.0 || gear_deploy[1] != 0.0);
	VERIFY3S(dr_getvi(&drs.on_ground, on_ground, 0, 3), ==, 3);
	my_acf_on_ground = (on_ground[0] != 0 || on_ground[1] != 0 ||
	    on_ground[2] != 0);

	/* grab all other aircraft positions */
	use_tcas_targets = (drs.have_tcas_targets &&
	    dr_geti(&drs.tcas_target_number) > 1);
	if (use_tcas_targets) {
		num_planes = MAX_TCAS_TARGETS - 1;
	} else {
		num_planes = MAX_MP_PLANES;
	}
	if (!use_tcas_targets) {
		/* Expunge any targets with an ID > MAX_MP_PLANES */
		for (int i = MAX_MP_PLANES; i < MAX_TCAS_TARGETS; i++) {
			acf_pos_t srch = {
			    .acf_id = (void *)(uintptr_t)(i + 1)
			};
			acf_pos_t *pos = avl_find(&acf_pos_tree, &srch, NULL);
			if (pos != NULL) {
				avl_remove(&acf_pos_tree, pos);
				free(pos);
			}
		}
	}
	for (int i = 0; i < num_planes; i++) {
		geo_pos3_t world = NULL_GEO_POS3;
		avl_index_t where;
		acf_pos_t srch;
		acf_pos_t *pos;
		bool_t on_ground = B_FALSE;

		if (use_tcas_targets) {
			dr_getvf(&drs.tcas_target_lat, &world.lat, i + 1, 1);
			dr_getvf(&drs.tcas_target_lon, &world.lon, i + 1, 1);
			dr_getvf(&drs.tcas_target_elev, &world.elev, i + 1, 1);
			if (world.lat == 0 && world.lon == 0)
				world = NULL_GEO_POS3;
		} else {
			vect3_t local = VECT3(dr_getf(&mp_planes[i].x),
			    dr_getf(&mp_planes[i].y),
			    dr_getf(&mp_planes[i].z));
			if (!IS_ZERO_VECT3(local)) {
				XPLMLocalToWorld(local.x, local.y, local.z,
				    &world.lat, &world.lon, &world.elev);
			}
		}

		mutex_enter(&acf_pos_lock);

		srch.acf_id = (void *)(uintptr_t)(i + 1);
		pos = avl_find(&acf_pos_tree, &srch, &where);

		if (IS_NULL_GEO_POS3(world)) {
			if (pos != NULL) {
				avl_remove(&acf_pos_tree, pos);
				free(pos);
			}
		} else {
			if (pos == NULL) {
				pos = safe_calloc(1, sizeof (*pos));
				pos->acf_id = (void *)(uintptr_t)(i + 1);
				ASSERT(pos->acf_id != NULL);
				pos->pos = NULL_GEO_POS3;
				pos->stale = true;
				avl_insert(&acf_pos_tree, pos, where);
			}
			/*
			 * Since when the slot becomes disused, it simply stops
			 * being updated. To detect that, we compare the new
			 * position to the previous position. If the contact
			 * hasn't moved in a while, we mark it as stale and
			 * stop forwarding it to the TCAS core. We can't simply
			 * remove it from the list, as that would result in us
			 * falsely re-adding it in the very new loop.
			 */
			if (!GEO3_EQ(pos->pos, world)) {
				pos->pos = world;
				pos->on_ground = on_ground;
				pos->last_seen = cur_sim_time;
				pos->stale = B_FALSE;
			} else if (cur_sim_time - pos->last_seen >
			    CTC_INACT_DELAY) {
				pos->stale = B_TRUE;
			}
		}

		mutex_exit(&acf_pos_lock);
	}

	dbg_log(xplane, 1, "Collector run complete, %lu contacts",
	    avl_numnodes(&acf_pos_tree));

	return (POS_UPDATE_INTVAL);
}

/*
 * Called from X-TCAS to get the current simulator time.
 */
static double
xp_get_time(void *handle)
{
	UNUSED(handle);
	ASSERT(intf_inited);
	return (cur_sim_time);
}

/*
 * Called from X-TCAS to get our aircraft position.
 */
static void
xp_get_my_acf_pos(void *handle, geo_pos3_t *pos, double *alt_agl, double *hdg,
    bool_t *gear_ext, bool_t *on_ground)
{
	UNUSED(handle);
	ASSERT(intf_inited);
	*pos = my_acf_pos;
	*alt_agl = my_acf_agl;
	*hdg = my_acf_hdg;
	*gear_ext = my_acf_gear_ext;
	*on_ground = my_acf_on_ground;
}

/*
 * Called from X-TCAS to gather intruder aircraft position.
 */
static void
xp_get_oth_acf_pos(void *handle, acf_pos_t **pos_p, size_t *num)
{
	size_t i;
	acf_pos_t *pos;

	UNUSED(handle);

	mutex_enter(&acf_pos_lock);
	*num = 0;
	for (pos = avl_first(&acf_pos_tree), i = 0; pos != NULL;
	    pos = AVL_NEXT(&acf_pos_tree, pos), i++) {
		if (!pos->stale)
			(*num)++;
	}
	*pos_p = safe_calloc(*num, sizeof (*pos));
	for (pos = avl_first(&acf_pos_tree), i = 0; pos != NULL;
	    pos = AVL_NEXT(&acf_pos_tree, pos)) {
		if (!pos->stale) {
			ASSERT3U(i, <, *num);
			memcpy(&(*pos_p)[i], pos, sizeof (*pos));
			i++;
		}
	}
	mutex_exit(&acf_pos_lock);
}

/*
 * Called by the plugin flight loop every simulator frame.
 */
static float
floop_cb(float elapsed_since_last_call, float elapsed_since_last_floop,
    int counter, void *refcon)
{
#ifndef	XTCAS_NO_AUDIO
	double volume = (dr_geti(&drs.view_is_ext) != 1 &&
	    dr_geti(&drs.sound_on) != 0) ? dr_getf(&drs.warn_volume) : 0;
#endif	/* !defined(XTCAS_NO_AUDIO) */

	UNUSED(elapsed_since_last_call);
	UNUSED(elapsed_since_last_floop);
	UNUSED(counter);
	UNUSED(refcon);

	cur_sim_time = dr_getf(&drs.time);

	if (isnan(first_sim_time))
		first_sim_time = cur_sim_time;

	if (cur_sim_time - first_sim_time < STARTUP_DELAY)
		return (-1.0);

	if (!xtcas_inited) {
		const sim_intf_output_ops_t *out_ops = ff_a320_intf_init();

		if (out_ops != NULL) {
			/* FF A320 integration mode */
			ff_a320_intf_inited = B_TRUE;
		} else {
#if	VSI_DRAW_MODE
			out_ops = &vsi_out_ops;
#else	/* !VSI_DRAW_MODE */
			out_ops = generic_intf_get_xtcas_ops();
			if (out_ops == NULL) {
				standalone_mode = B_TRUE;
				out_ops = &xplane_test_out_ops;

				XPLMRegisterCommandHandler(show_test_gui_cmd,
				    test_gui_handler, 1, NULL);
				XPLMRegisterCommandHandler(hide_test_gui_cmd,
				    test_gui_handler, 1, NULL);
			}
#endif	/* !VSI_DRAW_MODE */

			XPLMRegisterCommandHandler(filter_all_cmd,
			    tcas_config_handler, 1, NULL);
			XPLMRegisterCommandHandler(filter_thrt_cmd,
			    tcas_config_handler, 1, NULL);
			XPLMRegisterCommandHandler(filter_abv_cmd,
			    tcas_config_handler, 1, NULL);
			XPLMRegisterCommandHandler(filter_blw_cmd,
			    tcas_config_handler, 1, NULL);

			XPLMRegisterCommandHandler(mode_stby_cmd,
			    tcas_config_handler, 1, NULL);
			XPLMRegisterCommandHandler(mode_taonly_cmd,
			    tcas_config_handler, 1, NULL);
			XPLMRegisterCommandHandler(mode_tara_cmd,
			    tcas_config_handler, 1, NULL);
		}

		XPLMRegisterCommandHandler(tcas_test_cmd,
		    tcas_config_handler, 1, NULL);

		xtcas_init(&xp_intf_in_ops, out_ops);
		xtcas_inited = B_TRUE;
	} else if (xtcas_is_powered() && !xtcas_is_failed() &&
	    mode_req >= TCAS_MODE_STBY && mode_req <= TCAS_MODE_TARA &&
	    filter_req >= TCAS_FILTER_ALL && filter_req <= TCAS_FILTER_EXP) {
#if	VSI_DRAW_MODE
		if (dr_geti(&drs.xpdr_mode) != 1) {
			xtcas_set_mode(mode_req);
			mode_act = mode_req;
		} else {
			xtcas_set_mode(TCAS_MODE_STBY);
			mode_act = TCAS_MODE_STBY;
		}
#else	/* !VSI_DRAW_MODE */
		xtcas_set_mode(mode_req);
		mode_act = mode_req;
#endif	/* !VSI_DRAW_MODE */
		xtcas_set_filter(filter_req);
		filter_act = filter_req;
		xtcas_run();
#ifndef	XTCAS_NO_AUDIO
		xtcas_snd_sys_run(volume);
#endif
		if (ff_a320_intf_inited)
			ff_a320_intf_update();
	} else {
		xtcas_set_mode(TCAS_MODE_STBY);
		mode_act = TCAS_MODE_STBY;
#ifndef	XTCAS_NO_AUDIO
		xtcas_snd_sys_run(volume);
#endif
	}

	return (-1.0);
}

bool_t
xtcas_is_powered(void)
{
#if	VSI_DRAW_MODE
	bool_t powered = B_TRUE;

	if (dr_geti(&drs.xpdr_mode) == 0) {
		powered = B_FALSE;
	} else if (custom_bus_name[0] != '\0') {
		/* allow late resolution of custom bus name */
		if (!custom_bus) {
			custom_bus = dr_find(&drs.custom_bus_dr, "%s",
			    custom_bus_name);
		}
		if (custom_bus)
			powered = (dr_getf(&drs.custom_bus_dr) >= min_volts);
	} else if (min_volts > 0 && busnr >= 0 && busnr < BUSNR_MAX) {
		double volts;
		VERIFY3S(dr_getvf(&drs.bus_volts, &volts, busnr, 1), ==, 1);
		powered = (volts >= min_volts);
	}

	return (powered);
#else	/* !VSI_DRAW_MODE */
	UNUSED(custom_bus);
	return (B_TRUE);
#endif
}

bool_t
xtcas_is_failed(void)
{
	dr_t dr;

	if (strcmp(fail_dr_name, "") != 0 && dr_find(&dr, "%s", fail_dr_name)) {
		return (dr_geti(&dr) == 6);
	} else {
		return (dr_geti(&drs.xpdr_fail) == 6 ||
		    dr_geti(&drs.altm_fail) == 6 ||
		    dr_geti(&drs.adc_fail) == 6);
	}
}

double
xtcas_min_volts(void)
{
	return (min_volts);
}

static int
tcas_config_handler(XPLMCommandRef ref, XPLMCommandPhase phase, void *refcon)
{
	UNUSED(refcon);
	if (phase != xplm_CommandEnd)
		return (1);

	if (ref == tcas_test_cmd) {
#if	VSI_DRAW_MODE
		if (!xtcas_is_powered()) {
			logMsg("Cannot perform TCAS test: transponder is OFF");
			return (1);
		}
		if (xtcas_is_failed()) {
			logMsg("Cannot perform TCAS test: "
			    "transponder has failed");
			xtcas_test(B_TRUE);
			return (1);
		}
#endif	/* VSI_DRAW_MODE */
		if (xtcas_get_mode() == TCAS_MODE_STBY) {
			logMsg("TCAS TEST STARTED");
			xtcas_test(B_FALSE);
		} else {
			logMsg("Cannot perform TCAS TEST: mode not STBY");
		}
	} else if (ref == mode_stby_cmd) {
		logMsg("TCAS MODE: STBY");
		mode_req = TCAS_MODE_STBY;
	} else if (ref == mode_taonly_cmd) {
		logMsg("TCAS MODE: TA-ONLY");
		mode_req = TCAS_MODE_TAONLY;
	} else if (ref == mode_tara_cmd) {
		logMsg("TCAS MODE: TA/RA");
		mode_req = TCAS_MODE_TARA;
	} else if (ref == filter_all_cmd) {
		logMsg("TCAS FILTER: ALL");
		filter_req = TCAS_FILTER_ALL;
	} else if (ref == filter_thrt_cmd) {
		logMsg("TCAS FILTER: THRT");
		filter_req = TCAS_FILTER_THRT;
	} else if (ref == filter_abv_cmd) {
		logMsg("TCAS FILTER: ABV");
		filter_req = TCAS_FILTER_ABV;
	} else if (ref == filter_blw_cmd) {
		logMsg("TCAS FILTER: BLW");
		filter_req = TCAS_FILTER_BLW;
	} else {
		VERIFY_MSG(0, "Unknown command %p received", ref);
	}
	return (1);
}

#if	!VSI_DRAW_MODE
static int
test_gui_handler(XPLMCommandRef ref, XPLMCommandPhase phase, void *refcon)
{
	UNUSED(refcon);
	if (phase != xplm_CommandEnd)
		return (1);
	if (ref == show_test_gui_cmd) {
		xplane_test_init();
	} else {
		ASSERT3P(ref, ==, hide_test_gui_cmd);
		xplane_test_fini();
	}
	return (1);
}
#endif	/* !VSI_DRAW_MODE */

static void
config_load(void)
{
	char *path;
	int errline;
	FILE *fp;

	conf = conf_create_empty();
	xtcas_conf = conf;

	memset(&xtcas_dbg, 0, sizeof (xtcas_dbg));

	path = mkpathname(plugindir, "X-TCAS.cfg", NULL);
	if (!file_exists(path, NULL)) {
		free(path);
		goto errout;
	}

	fp = fopen(path, "rb");
	if (fp == NULL) {
		logMsg("Error opening configuration file %s: %s",
		    path, strerror(errno));
		free(path);
		goto errout;
	}
	conf = conf_read(fp, &errline);
	if (conf == NULL) {
		logMsg("Error parsing configuration file %s: "
		    "error on line %d\n", path, errline);
		free(path);
		fclose(fp);
		goto errout;
	}

	free(path);
	fclose(fp);

#define	READ_DBG_CONF(var) \
	conf_get_i(conf, "debug_" #var, &xtcas_dbg.var)
	READ_DBG_CONF(all);
	READ_DBG_CONF(snd);
	READ_DBG_CONF(wav);
	READ_DBG_CONF(tcas);
	READ_DBG_CONF(xplane);
	READ_DBG_CONF(test);
	READ_DBG_CONF(ra);
	READ_DBG_CONF(cpa);
	READ_DBG_CONF(sl);
	READ_DBG_CONF(contact);
	READ_DBG_CONF(threat);
	READ_DBG_CONF(ff_a320);
#undef	READ_DBG_CONF

	xtcas_conf = conf;
	return;
errout:
	conf = conf_create_empty();
	xtcas_conf = conf;
}

PLUGIN_API int
XPluginStart(char *name, char *sig, char *desc)
{
	char *snd_dir, *p;

	log_init(XPLMDebugString, "xtcas");
#ifdef	EXCEPT_DEBUG
	except_init();
#endif

	/* Always use Unix-native paths on the Mac! */
	XPLMEnableFeature("XPLM_USE_NATIVE_PATHS", 1);
	XPLMGetPluginInfo(XPLMGetMyID(), NULL, plugindir, NULL, NULL);
#if	IBM
	fix_pathsep(plugindir);
#endif
	/* cut off the trailing path component (our filename) */
	if ((p = strrchr(plugindir, DIRSEP)) != NULL)
		*p = '\0';
	/* cut off an optional '32' or '64' trailing component */
	if ((p = strrchr(plugindir, DIRSEP)) != NULL) {
		if (strcmp(p + 1, "64") == 0 || strcmp(p + 1, "32") == 0 ||
		    strcmp(p + 1, "win_x64") == 0 ||
		    strcmp(p + 1, "mac_x64") == 0 ||
		    strcmp(p + 1, "lin_x64") == 0)
			*p = '\0';
	}

	snprintf(name, 64, XTCAS_PLUGIN_NAME, XTCAS_VER);
	strcpy(sig, XTCAS_PLUGIN_SIG);
	strcpy(desc, XTCAS_PLUGIN_DESCRIPTION);
	logMsg("This is X-TCAS version %x (confopts: VSI:%d STYLE:%d "
	    "GTS820:%d)", XTCAS_VER, VSI_DRAW_MODE, VSI_STYLE, GTS820_MODE);
	sim_intf_init();
	snd_dir = mkpathname(plugindir, "data", "msgs", NULL);
#ifndef	XTCAS_NO_AUDIO
	if (!xtcas_snd_sys_init(snd_dir)) {
		free(snd_dir);
		return (0);
	}
#endif	/* !defined(XTCAS_NO_AUDIO) */
	free(snd_dir);

#if	!VSI_DRAW_MODE
	show_test_gui_cmd = XPLMCreateCommand("X-TCAS/show_debug_gui",
	    "Show debugging interface");
	hide_test_gui_cmd = XPLMCreateCommand("X-TCAS/hide_debug_gui",
	    "Hide debugging interface");
#endif	/* !VSI_DRAW_MODE */

	filter_all_cmd = XPLMCreateCommand("X-TCAS/filter_all",
	    "Set TCAS display filter to ALL");
	filter_thrt_cmd = XPLMCreateCommand("X-TCAS/filter_thrt",
	    "Set TCAS display filter to THREAT");
	filter_abv_cmd = XPLMCreateCommand("X-TCAS/filter_abv",
	    "Set TCAS display filter to ABOVE");
	filter_blw_cmd = XPLMCreateCommand("X-TCAS/filter_blw",
	    "Set TCAS display filter to BELOW");

	mode_stby_cmd = XPLMCreateCommand("X-TCAS/mode_stby",
	    "Set TCAS mode to STANDBY");
	mode_taonly_cmd = XPLMCreateCommand("X-TCAS/mode_taonly",
	    "Set TCAS mode to TA-ONLY");
	mode_tara_cmd = XPLMCreateCommand("X-TCAS/mode_tara",
	    "Set TCAS mode to TA/RA");

	tcas_test_cmd = XPLMCreateCommand("X-TCAS/test", "Perform TCAS test");
	ASSERT(tcas_test_cmd != NULL);

	return (1);
}

PLUGIN_API void
XPluginStop(void)
{
#ifndef	XTCAS_NO_AUDIO
	xtcas_snd_sys_fini();
#endif
	sim_intf_fini();
#ifdef	EXCEPT_DEBUG
	except_fini();
#endif
}

PLUGIN_API int
XPluginEnable(void)
{
	const char *s;

	config_load();

	XPLMRegisterFlightLoopCallback(floop_cb, FLOOP_INTVAL, NULL);
	XPLMRegisterFlightLoopCallback(acf_pos_collector, POS_UPDATE_INTVAL,
	    NULL);
	first_sim_time = NAN;

	dr_create_i(&drs.busnr, &busnr, B_TRUE, "xtcas/busnr");
	dr_create_f(&drs.min_volts, &min_volts, B_TRUE, "xtcas/min_volts");
	dr_create_i(&drs.mode_req, &mode_req, B_TRUE, "xtcas/mode_req");
	dr_create_i(&drs.mode_act, &mode_act, B_FALSE, "xtcas/mode_act");
	dr_create_i(&drs.filter_req, &filter_req, B_TRUE, "xtcas/filter_req");
	dr_create_i(&drs.filter_act, &filter_act, B_FALSE, "xtcas/filter_act");
	dr_create_b(&drs.fail_dr_name_dr, fail_dr_name, sizeof (fail_dr_name),
	    B_TRUE, "xtcas/fail_dr");

	if (conf_get_str(xtcas_conf, "busnr", &s) && strlen(s) > 3 &&
	    !isdigit(s[0])) {
		strlcpy(custom_bus_name, s, sizeof (custom_bus_name));
	} else {
		conf_get_i(xtcas_conf, "busnr", &busnr);
	}
	conf_get_f(xtcas_conf, "min_volts", &min_volts);
	if (conf_get_str(xtcas_conf, "fail_dr", &s))
		strlcpy(fail_dr_name, s, sizeof (fail_dr_name));

	fdr_find(&drs.xpdr_mode, "sim/cockpit/radios/transponder_mode");
	fdr_find(&drs.bus_volts, "sim/cockpit2/electrical/bus_volts");
	fdr_find(&drs.xpdr_fail, "sim/operation/failures/rel_xpndr");
	fdr_find(&drs.altm_fail, "sim/operation/failures/rel_g_alt");
	fdr_find(&drs.adc_fail, "sim/operation/failures/rel_adc_comp");
	fdr_find(&drs.gear_deploy, "sim/aircraft/parts/acf_gear_deploy");
	fdr_find(&drs.on_ground, "sim/flightmodel2/gear/on_ground");

#if	VSI_DRAW_MODE
	if (!vsi_init(plugindir)) {
		XPluginDisable();
		return (0);
	}
#endif	/* VSI_DRAW_MODE */

	generic_intf_init();

	return (1);
}

PLUGIN_API void
XPluginDisable(void)
{
	generic_intf_fini();

#if	VSI_DRAW_MODE
	vsi_fini();
#endif

	dr_delete(&drs.busnr);
	dr_delete(&drs.min_volts);
	dr_delete(&drs.mode_req);
	dr_delete(&drs.mode_act);
	dr_delete(&drs.filter_req);
	dr_delete(&drs.filter_act);
	dr_delete(&drs.fail_dr_name_dr);

	if (xtcas_inited) {
		xtcas_fini();

		if (ff_a320_intf_inited) {
			/* FF A320 integration mode */
			ff_a320_intf_fini();
			ff_a320_intf_inited = B_FALSE;
		} else {
#if !VSI_DRAW_MODE
			if (standalone_mode) {
				XPLMUnregisterCommandHandler(show_test_gui_cmd,
				    test_gui_handler, 1, NULL);
				XPLMUnregisterCommandHandler(hide_test_gui_cmd,
				    test_gui_handler, 1, NULL);
			}
#endif	/* !VSI_DRAW_MODE */

			XPLMUnregisterCommandHandler(filter_all_cmd,
			    tcas_config_handler, 1, NULL);
			XPLMUnregisterCommandHandler(filter_thrt_cmd,
			    tcas_config_handler, 1, NULL);
			XPLMUnregisterCommandHandler(filter_abv_cmd,
			    tcas_config_handler, 1, NULL);
			XPLMUnregisterCommandHandler(filter_blw_cmd,
			    tcas_config_handler, 1, NULL);

			XPLMUnregisterCommandHandler(mode_stby_cmd,
			    tcas_config_handler, 1, NULL);
			XPLMUnregisterCommandHandler(mode_taonly_cmd,
			    tcas_config_handler, 1, NULL);
			XPLMUnregisterCommandHandler(mode_tara_cmd,
			    tcas_config_handler, 1, NULL);

			XPLMUnregisterCommandHandler(tcas_test_cmd,
			    tcas_config_handler, 1, NULL);
		}

		xtcas_inited = B_FALSE;
	}

	XPLMUnregisterFlightLoopCallback(acf_pos_collector, NULL);
	XPLMUnregisterFlightLoopCallback(floop_cb, NULL);

	if (conf != NULL) {
		conf_free(conf);
		conf = NULL;
		xtcas_conf = NULL;
	}
}

PLUGIN_API void
XPluginReceiveMessage(XPLMPluginID from, int msg, void *param)
{
	UNUSED(from);

	if (msg == XTCAS_GENERIC_INTF_GET && param != NULL) {
		*(xtcas_generic_intf_t **)param = generic_intf_get_intf_ops();
	}
}

void
generic_set_mode(tcas_mode_t mode)
{
	mode_req = mode;
}

void
generic_set_filter(tcas_filter_t filter)
{
	filter_req = filter;
}

#if	IBM
BOOL WINAPI
DllMain(HINSTANCE hinst, DWORD reason, LPVOID resvd)
{
	UNUSED(hinst);
	UNUSED(resvd);
	lacf_glew_dllmain_hook(reason);
	return (TRUE);
}
#endif	/* IBM */

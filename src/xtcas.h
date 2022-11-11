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

#ifndef	_XTCAS_H_
#define	_XTCAS_H_

#include <acfutils/avl.h>
#include <acfutils/geom.h>

#ifdef __cplusplus
extern "C" {
#endif

#define	LEVEL_VVEL_THRESH	FPM2MPS(500)

/*
 * Controls the master TCAS operation mode:
 *
 * TCAS_MODE_STBY: X-TCAS is disabled and no contacts, threats, TAs or RAs
 *	will be displayed. Please note that you must still call tcas_run
 *	and will be called back via the sim_intf_ops_t INPUT functions
 *	(get_time, get_my_acf_pos, get_oth_acf_pos). The only OUTPUT
 *	function call should get is delete_contact to remove any old
 *	contacts that X-TCAS still had cached.
 * TCAS_MODE_TAONLY: X-TCAS will only issue Traffic Advisories (TAs),
 *	but no Resolution Advisories (RAs). This also implies selection of
 *	TCAS sensitivity-level 2 (SL2).
 * TCAS_MODE_TARA: X-TCAS will issue TAs and RAs as necessary and display
 *	all the standard vertical guidance on the vertical speed tape,
 *	primary flight display or vertical speed indicator (as appropriate
 *	for installed equipment).
 */
typedef enum {
	TCAS_MODE_STBY,		/* default */
	TCAS_MODE_TAONLY,
	TCAS_MODE_TARA
} tcas_mode_t;

/*
 * Controls the setting of the contact display and threat filter.
 * The display filter is applied by either calling update_contact or
 * delete_contact via the sim_intf_ops_t callbacks. The threat filter
 * is applied by either issuing or not issuing TAs or RAs against the
 * appropriate threats.
 *
 * TCAS_FILTER_ALL: enables display of all contacts without any conditions
 *	(TCAS intruders are displayed when detected).
 * TCAS_FILTER_THRT: PROX and OTH contacts (see tcas_threat_t) are only
 *	displayed when a TA or RA is active.
 * TCAS_FILTER_ABV: irrespective of threat level, contacts are only
 *	displayed and TAs/RAs issued if they are within -2700ft..+9900ft
 *	of our aircraft.
 * TCAS_FILTER_ABV: irrespective of threat level, contacts are only
 *	displayed and TAs/RAs issued if they are within -9900ft..+2700ft
 *	of our aircraft.
 * TCAS_FILTER_EXP: expanded surveillance envelope from -9900ft..+9900ft.
 */
typedef enum {
	TCAS_FILTER_ALL,	/* default */
	TCAS_FILTER_THRT,
	TCAS_FILTER_ABV,
	TCAS_FILTER_BLW,
	TCAS_FILTER_EXP
} tcas_filter_t;

/*
 * This is the position structure passed read by X-TCAS from the simulator
 * interface. The simulator should populate only the following fields:
 * 1) acf_id: a unique opaque aircraft identifier. X-TCAS uses this to refer
 *	subsequent position updates to the same aircraft. X-TCAS does NOT
 *	perform any sort of aircraft path matching to identify contacts.
 * 2) pos: The current 3-dimensional position of the aircraft. The lat & lon
 *	fields should contain the aircraft's latitude & longitude in degrees
 *	(east/north increasing) and the elev field should contain the
 *	aircraft's current barometric altitude in meters.
 * 3) on_gnd: Boolean field filled from the Mode S data (if available).
 */
typedef struct {
	void		*acf_id;
	geo_pos3_t	pos;
	bool_t		on_ground;
	double		last_seen;
	bool_t		stale;
	avl_node_t	tree_node;
} acf_pos_t;

/*
 * Threat level of traffic. This determines how the contact is drawn on the
 * TCAS display.
 */
typedef enum {
	OTH_THREAT,	/* other traffic, draw as empty diamond */
	PROX_THREAT,	/* proximate traffic, draw as filled diamond */
	TA_THREAT,	/* traffic advisory, filled yellow circle */
	RA_THREAT_PREV,	/* preventive or corrective resolution... */
	RA_THREAT_CORR	/* ...advisory, draw both as filled red square */
} tcas_threat_t;

/*
 * An aural advisory message issued by X-TCAS. Played over the aircraft's
 * loudspeaker system.
 */
typedef enum {
	RA_MSG_CLB,		/* 2x CLIMB */
	RA_MSG_CLB_CROSS,	/* 2x CLIMB CROSSING CLIMB */
	RA_MSG_CLB_MORE,	/* 2x INCREASE CLIMB */
	RA_MSG_CLB_NOW,		/* 2x CLIMB, CLIMB NOW */
	RA_MSG_CLEAR,		/* 1x CLEAR OF CONFLICT */
	RA_MSG_DES,		/* 2x DESCEND */
	RA_MSG_DES_CROSS,	/* 2x DESCEND, CROSSING DESCEND */
	RA_MSG_DES_MORE,	/* 2x INCREASE DESCENT */
	RA_MSG_DES_NOW,		/* 2x DESCEND, DESCEND NOW */
	RA_MSG_MONITOR_VS,	/* 1x MONITOR VERTICAL SPEED */
	RA_MSG_MAINT_VS,	/* 1x MAINTAIN VERTICAL SPEED */
	RA_MSG_MAINT_VS_CROSS,	/* 1x MAINTAIN VERT. SPEED, CROSSING MAINTAIN */
	RA_MSG_LEVEL_OFF,	/* 2x LEVEL OFF */
	RA_MSG_TFC,		/* 2x TRAFFIC  (only 1x on GTS 820) */
	TCAS_TEST_PASS,
	TCAS_TEST_FAIL,
#if	GTS820_MODE
	GTS820_MSG_1CLK,	/* "One o'clock" */
	GTS820_MSG_2CLK,	/* "Two o'clock" */
	GTS820_MSG_3CLK,	/* "Three o'clock" */
	GTS820_MSG_4CLK,	/* "Four o'clock" */
	GTS820_MSG_5CLK,	/* "Five o'clock" */
	GTS820_MSG_6CLK,	/* "Six o'clock" */
	GTS820_MSG_7CLK,	/* "Seven o'clock" */
	GTS820_MSG_8CLK,	/* "Eight o'clock" */
	GTS820_MSG_9CLK,	/* "Nine o'clock" */
	GTS820_MSG_10CLK,	/* "Ten o'clock" */
	GTS820_MSG_11CLK,	/* "Eleven o'clock" */
	GTS820_MSG_12CLK,	/* "Twelve o'clock" */
	GTS820_MSG_HIGH,	/* "High" */
	GTS820_MSG_LOW,		/* "Low" */
	GTS820_MSG_SAME_ALT,	/* "Same altitude" */
	GTS820_MSG_M1NM,	/* "Less than 1 mile */
	GTS820_MSG_1NM,		/* "One mile" */
	GTS820_MSG_2NM,		/* "Two miles" */
	GTS820_MSG_3NM,		/* "Three miles" */
	GTS820_MSG_4NM,		/* "Four miles" */
	GTS820_MSG_5NM,		/* "Five miles" */
	GTS820_MSG_6NM,		/* "Six miles" */
	GTS820_MSG_7NM,		/* "Seven miles" */
	GTS820_MSG_8NM,		/* "Eight miles" */
	GTS820_MSG_9NM,		/* "Nine miles" */
	GTS820_MSG_10NM,	/* "Ten miles" */
	GTS820_MSG_P10NM,	/* "More than ten miles" */
#endif	/* GTS820_MODE */
	RA_NUM_MSGS		/* invalid */
} tcas_msg_t;

/*
 * The type of RA that is being issued. Preventive RAs do not contain a
 * green "fly-to" vertical speed band, only a red "avoid" VS band. The
 * purpose of a preventive RA is to prevent the aircraft from maneuvering
 * in a sense that would reduce vertical separation to an intruder,
 * wheras a corrective RA requires that the aircraft depart its current
 * flight path in order to obtain vertical separation.
 */
typedef enum {
	RA_TYPE_CORRECTIVE,
	RA_TYPE_PREVENTIVE
} tcas_RA_type_t;

/*
 * The sense discriminator of an RA. An initial RA is always either UPWARD
 * or DOWNWARD. Subsequent RAs can be UPWARD, DOWNWARD or a special case
 * of a level-off weakening RA (LEVEL_OFF).
 */
typedef enum {
	RA_SENSE_UPWARD,
	RA_SENSE_LEVEL_OFF,
	RA_SENSE_DOWNWARD
} tcas_RA_sense_t;

typedef enum {
	ADV_STATE_NONE,
	ADV_STATE_TA,
	ADV_STATE_RA
} tcas_adv_t;
const char *xtcas_RA_msg2text(tcas_msg_t msg);

/*
 * This is the X-TCAS simulator interface. Everything that the core of
 * X-TCAS needs for operation, it gets via these functions.
 */
typedef struct {
	/* Interface handle - for use by the interface provider */
	void	*handle;

	/*
	 * INPUT:
	 * These are the X-TCAS input functions. They represent how X-TCAS
	 * learns about the simulator environment.
	 */

	/*
	 * This function tells X-TCAS about the flow of time in the simulator.
	 * It should return a monotonically increasing second counter from
	 * some arbitrary point in the past. It needs to be at least
	 * millisecond-accurate.
	 */
	double	(*get_time)(void *handle);
	/*
	 * This function returns the current position of our own aircraft.
	 * The arguments must be filled with the current LAT/LON/ELEV, altitude
	 * AGL and heading of our own aircraft. LAT & LON should be in degrees
	 * from the 0'th parallel and 0'th meridian (north/east increasing).
	 * ELEV should be our current barometric altitude. All altitudes
	 * are in meters. Heading is degrees from true north.
	 * `gear_ext' should be filled with the gear extension status (B_TRUE,
	 * for gear extended, B_FALSE otherwise). This is only used for the
	 * GTS 820 mode.
	 * `on_ground' should be filled with the gear-on-ground status.
	 */
	void	(*get_my_acf_pos)(void *handle, geo_pos3_t *pos,
		    double *alt_agl, double *hdg, bool_t *gear_ext,
		    bool_t *on_ground);
	/*
	 * This function serves to feed TCAS aircraft contacts into X-TCAS.
	 * The callee must malloc() an array of acf_pos_t structures and
	 * fill them with data. The number of allocated structures is
	 * returned in `num'. X-TCAS then free()s the structure on its own.
	 * See the description of acf_pos_t for details on what to put in it.
	 */
	void	(*get_oth_acf_pos)(void *handle, acf_pos_t **pos_p,
		    size_t *num);
} sim_intf_input_ops_t;

typedef struct {
	/*
	 * OUTPUT:
	 * These are the X-TCAS output functions. They represent how X-TCAS
	 * tells the aircraft's avionics about traffic threats and possible
	 * resolution advisories.
	 */

	/* Interface handle - for use by the interface provider */
	void	*handle;

	/*
	 * This tells the avionics about TCAS aircraft contacts. It is called
	 * every time X-TCAS does a position update (once every second). When
	 * a new contact is detected, this function also represents a "new
	 * contact" call. Contacts that have been lost will be explicitly
	 * removed via delete_contact (see below).
	 * Please note that X-TCAS can call this function from threads other
	 * than your main thread, so take care to lock any private structures
	 * as necessary.
	 * Arguments:
	 * 1) handle The handle passed to xtcas_init in the ops structure.
	 * 2) acf_id A unique opaque aircraft ID as was returned by
	 *	get_oth_acf_pos. This is the identifier by which X-TCAS
	 *	discriminates between contacts.
	 * 3) rbrg Relative bearing in degrees from the nose (increasing
	 *	clockwise looking down on our aircraft).
	 * 4) rdist Relative distance in meters.
	 * 5) ralt Relative altitude in meters (positive up).
	 * 6) vs The current vertical speed (in m/s) of this contact.
	 * 7) trk True track of the contact.
	 * 8) gs True groundspeed of the contact.
	 * 9) level The current TCAS threat level of this aircraft.
	 *	This can be used to determine with which symbol the contact
	 *	should be displayed (see tcas_threat_t).
	 */
	void	(*update_contact)(void *handle, void *acf_id, double rbrg,
	    double rdist, double ralt, double vs, double trk, double gs,
	    tcas_threat_t level);
	/*
	 * Deletes a contact that was previously acquired via get_oth_acf_pos.
	 * Please note that X-TCAS may send a delete_contact before ever
	 * sending an update_contact in case the contact was lost before
	 * X-TCAS could acquire enough information about it to make a threat
	 * assessment.
	 * Please also note that X-TCAS can call this function from threads
	 * other than your main thread, so take care to lock any private
	 * structures as necessary.
	 * Arguments:
	 * 1) handle The handle passed to xtcas_init in the ops structure.
	 * 2) acf_id An unique opaque aircraft ID as was returned by
	 *	get_oth_acf_pos. This is the identifier by which X-TCAS
	 *	discriminates between contacts.
	 */
	void	(*delete_contact)(void *handle, void *acf_id);
	/*
	 * Notifies the avionics of an active resolution advisory (RA). This
	 * is in effect what X-TCAS has decided is the most proper course of
	 * action to take. Arguments:
	 * 1) handle The handle passed to xtcas_init in the ops structure.
	 * 2) adv The current TCAS advisory state. The values here mean:
	 *	ADV_STATE_NONE: X-TCAS is not currently declaring any
	 *		TAs or RAs. The remaining arguments will be undefined.
	 *	ADV_STATE_TA: X-TCAS has declared a Traffic Advisory (TA),
	 *		but not yet a Resolution Advisory (RA). The remaining
	 *		arguments will be undefined.
	 *	ADV_STATE_RA: X-TCAS has declared a Resolution Advisory (RA)
	 *		and the remaining arguments will take on the values
	 *		of the respective RA. Subsequent RAs will call this
	 *		function again with new values for the other arguments.
	 * 3) msg The current TCAS RA message. See tcas_msg_t.
	 * 4) type The current TCAS RA type. See tcas_RA_type_t.
	 * 5) sense The current TCAS RA sense. See tcas_RA_sense_t.
	 * 6) crossing A flag indicating whether the current RA is going to
	 *	cross the intruder's altitude or not.
	 * 7) reversal A flag indicating whether the current RA is a
	 *	sense-reversal from a previously issued RA.
	 * 8) min_sep_cpa The predicted minimum vertical separation (in meters)
	 *	that will be achieved from all intruders when following the
	 *	current RA.
	 * 9) min_green The vertical speed green "fly-to" zone's minimum
	 *	(in m/s).
	 * 10) min_green The vertical speed green "fly-to" zone's maximum.
	 *	If min_green == max_green, the current advisory doesn't
	 *	declare a "fly-to" zone.
	 * 11) min_red_lo The lower vertical speed red "avoid" zone's minimum.
	 * 12) max_red_lo The lower vertical speed red "avoid" zone's maximum.
	 * 13) min_red_hi The upper vertical speed red "avoid" zone's minimum.
	 * 14) max_red_hi The upper vertical speed red "avoid" zone's maximum.
	 */
	void	(*update_RA)(void *handle, tcas_adv_t adv, tcas_msg_t msg,
		    tcas_RA_type_t type, tcas_RA_sense_t sense, bool_t crossing,
		    bool_t reversal, double min_sep_cpa, double min_green,
		    double max_green, double min_red_lo, double max_red_lo,
		    double min_red_hi, double max_red_hi);
	/*
	 * This is similar to update_RA, but provides 1-second updates to
	 * X-TCAS's internal logic as it updates its estimates. It's use
	 * is mainly in debugging X-TCAS. Providing this callback is optional.
	 */
	void	(*update_RA_prediction)(void *handle, tcas_msg_t msg,
		    tcas_RA_type_t type, tcas_RA_sense_t sense,
		    bool_t crossing, bool_t reversal, double min_sep_cpa);
	/*
	 * Optional callback that specifically informs the avionics that
	 * X-TCAS wishes to generate an audio message. This is useful for
	 * X-TCAS builds with XTCAS_NO_AUDIO defined.
	 */
	void	(*play_audio_msg)(void *handle, tcas_msg_t msg);
} sim_intf_output_ops_t;

void xtcas_run(void);
void xtcas_init(const sim_intf_input_ops_t *const intf_input_ops,
    const sim_intf_output_ops_t *const intf_output_ops);
void xtcas_fini(void);

/*
 * External configuration functions.
 */
void xtcas_set_mode(tcas_mode_t mode);
tcas_mode_t xtcas_get_mode(void);
tcas_mode_t xtcas_get_mode_act(void);
void xtcas_set_filter(tcas_filter_t filter);
tcas_filter_t xtcas_get_filter(void);
int xtcas_get_SL(void);
void xtcas_set_has_RA(bool_t flag);
void xtcas_set_has_WOW(bool_t flag);
void xtcas_set_RA(double agl_hgt_m);
void xtcas_set_WOW(bool_t on_ground);
void xtcas_set_gear_ext(bool_t gear_ext);

void xtcas_test(bool_t force_fail);
bool_t xtcas_test_is_in_prog(void);

#ifdef __cplusplus
}
#endif

#endif	/* _XTCAS_H_ */

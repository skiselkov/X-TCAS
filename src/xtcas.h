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

#include "sim_intf.h"

#ifdef __cplusplus
extern "C" {
#endif

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
 */
typedef enum {
	TCAS_FILTER_ALL,	/* default */
	TCAS_FILTER_THRT,
	TCAS_FILTER_ABV,
	TCAS_FILTER_BLW
} tcas_filter_t;

void xtcas_run(void);
void xtcas_init(const sim_intf_ops_t *intf_ops);
void xtcas_fini(void);

/*
 * External configuration functions.
 */
void xtcas_set_mode(tcas_mode_t mode);
tcas_mode_t xtcas_get_mode(void);
void xtcas_set_filter(tcas_filter_t filter);
tcas_filter_t xtcas_get_filter(void);

#ifdef __cplusplus
}
#endif

#endif	/* _XTCAS_H_ */

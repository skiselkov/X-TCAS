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

#ifndef	_VSI_H_
#define	_VSI_H_

#include "xtcas.h"

#ifdef __cplusplus
extern "C" {
#endif

#define	VSI_STYLE_ATR		1
#define	VSI_STYLE_HONEYWELL	2

#ifndef	VSI_STYLE
#define	VSI_STYLE	VSI_STYLE_ATR
#endif

#ifndef	VSI_DRAW_MODE
#define	VSI_DRAW_MODE	0
#endif

bool_t vsi_init(const char *plugindir);
void vsi_fini(void);

void vsi_update_contact(void *handle, void *acf_id, double rbrg,
    double rdist, double ralt, double vs, double trk, double gs,
    tcas_threat_t level);
void vsi_delete_contact(void *handle, void *acf_id);
void vsi_update_RA(void *handle, tcas_adv_t adv, tcas_msg_t msg,
    tcas_RA_type_t type, tcas_RA_sense_t sense, bool_t crossing,
    bool_t reversal, double min_sep_cpa, double min_green, double max_green,
    double min_red_lo, double max_red_lo, double min_red_hi, double max_red_hi);

#ifdef __cplusplus
}
#endif

#endif	/* _VSI_H_ */

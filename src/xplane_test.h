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

#ifndef	_XTCAS_XPLANE_TEST_H_
#define	_XTCAS_XPLANE_TEST_H_

#include "xtcas.h"

#ifdef __cplusplus
extern "C" {
#endif

void xplane_test_init(void);
void xplane_test_fini(void);

void xplane_test_update_contact(void *handle, void *acf_id, double rbrg,
    double rdist, double ralt, double vs, double trk, double gs,
    tcas_threat_t level);
void xplane_test_delete_contact(void *handle, void *acf_id);
void xplane_test_update_RA(void *handle, tcas_adv_t adv, tcas_msg_t msg,
    tcas_RA_type_t type, tcas_RA_sense_t sense, bool_t crossing,
    bool_t reversal, double min_sep_cpa, double min_green, double max_green,
    double min_red_lo, double max_red_lo, double min_red_hi, double max_red_hi);

#ifdef __cplusplus
}
#endif

#endif	/* _XTCAS_XPLANE_TEST_H_ */

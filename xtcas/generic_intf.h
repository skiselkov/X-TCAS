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

#ifndef	_XTCAS_GENERIC_INTF_H_
#define	_XTCAS_GENERIC_INTF_H_

#include "../src/xtcas.h"

#ifdef __cplusplus
extern "C" {
#endif

#define	XTCAS_PLUGIN_SIG	"skiselkov.xtcas"
#define	XTCAS_GENERIC_INTF_GET	0x100000

typedef struct {
	void		(*set_mode)(tcas_mode_t mode);
	tcas_mode_t	(*get_mode)(void);
	tcas_mode_t	(*get_mode_act)(void);
	void		(*set_filter)(tcas_filter_t filter);
	tcas_filter_t	(*get_filter)(void);
	void		(*test)(bool_t force_fail);
	bool_t		(*test_is_in_prog)(void);
	void		(*set_output_ops)(const sim_intf_output_ops_t *cbs);
	void		(*set_has_RA)(bool_t flag);
	void		(*set_has_WOW)(bool_t flag);
	void		(*set_RA)(double agl_hgt_m);
	void		(*set_WOW)(bool_t on_ground);
	void		(*set_gear_ext)(bool_t gear_ext);
} xtcas_generic_intf_t;

/* X-TCAS internal */
void generic_intf_init(void);
void generic_intf_fini(void);
sim_intf_output_ops_t *generic_intf_get_xtcas_ops(void);
xtcas_generic_intf_t *generic_intf_get_intf_ops(void);

#ifdef __cplusplus
}
#endif

#endif	/* _XTCAS_GENERIC_INTF_H_ */

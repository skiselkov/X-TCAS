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
 *
 * Copyright 2017 Saso Kiselkov. All rights reserved.
 */

#ifndef	_SIM_INTF_H_
#define	_SIM_INTF_H_

#include "avl.h"
#include "geom.h"

#ifdef	__cplusplus
extern "C" {
#endif

typedef struct {
	void		*acf_id;
	geo_pos3_t	pos;
	avl_node_t	tree_node;
} acf_pos_t;

typedef double (*get_time_t)(void);
typedef void (*get_my_acf_pos_t)(geo_pos3_t *pos, double *alt_agl);
typedef void (*get_oth_acf_pos_t)(acf_pos_t **pos_p, size_t *num);
typedef bool_t (*sound_on_t)(void);

typedef struct {
	get_time_t		get_time;
	get_my_acf_pos_t	get_my_acf_pos;
	get_oth_acf_pos_t	get_oth_acf_pos;
} sim_intf_ops_t;

#ifdef	__cplusplus
}
#endif

#endif	/* _SIM_INTF_H_ */

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
 * Copyright 2016 Saso Kiselkov. All rights reserved.
 */

#ifndef	_XTCAS_XPLANE_H_
#define	_XTCAS_XPLANE_H_

#include <stdlib.h>

#include "XPLMDefs.h"

#include "avl.h"
#include "geom.h"
#include "list.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	void		*acf_id;
	geo_pos3_t	pos;
	avl_node_t	tree_node;
} acf_pos_t;

double xtcas_get_time(void);
void xtcas_get_my_acf_pos(geo_pos3_t *pos, double *alt_agl);
void xtcas_get_acf_pos(acf_pos_t **pos_p, size_t *num);

/*
 * X-Plane-specific plugin hooks.
 */
PLUGIN_API int XPluginStart(char *name, char *sig, char *desc);
PLUGIN_API void XPluginStop(void);
PLUGIN_API void XPluginEnable(void);
PLUGIN_API void XPluginDisable(void);
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID from, int msg, void *param);

#ifdef __cplusplus
}
#endif

#endif	/* _XTCAS_XPLANE_H_ */

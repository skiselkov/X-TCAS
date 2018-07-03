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

#ifndef	_XTCAS_XPLANE_H_
#define	_XTCAS_XPLANE_H_

#include <stdlib.h>

#include <XPLMDefs.h>

#include <acfutils/avl.h>
#include <acfutils/conf.h>
#include <acfutils/geom.h>
#include <acfutils/list.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifndef	GTS820_MODE
#define	GTS820_MODE	0
#endif

/*
 * X-Plane-specific plugin hooks.
 */
PLUGIN_API int XPluginStart(char *name, char *sig, char *desc);
PLUGIN_API void XPluginStop(void);
PLUGIN_API int XPluginEnable(void);
PLUGIN_API void XPluginDisable(void);
PLUGIN_API void XPluginReceiveMessage(XPLMPluginID from, int msg, void *param);

extern const conf_t *xtcas_conf;

bool_t xtcas_is_powered(void);
bool_t xtcas_is_failed(void);
double xtcas_min_volts(void);

void generic_set_mode(tcas_mode_t mode);
void generic_set_filter(tcas_filter_t filter);

#ifdef __cplusplus
}
#endif

#endif	/* _XTCAS_XPLANE_H_ */

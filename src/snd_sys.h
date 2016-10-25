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

#ifndef	_SND_SYS_H_
#define	_SND_SYS_H_

#include <stdlib.h>

#include "types.h"

#ifdef	__cplusplus
extern "C" {
#endif

typedef enum {
	RA_MSG_CLB,
	RA_MSG_CLB_CROSS,
	RA_MSG_CLB_MORE,
	RA_MSG_CLB_NOW,
	RA_MSG_CLEAR,
	RA_MSG_DES,
	RA_MSG_DES_CROSS,
	RA_MSG_DES_MORE,
	RA_MSG_DES_NOW,
	RA_MSG_MONITOR_VS,
	RA_MSG_MAINT_VS,
	RA_MSG_MAINT_VS_CROSS,
	RA_MSG_LEVEL_OFF,
	RA_MSG_TFC,
	RA_NUM_MSGS
} tcas_RA_msg_t;

void play_msg(tcas_RA_msg_t msg);

bool_t snd_sys_init(const char *plugindir);
void snd_sys_fini(void);

#ifdef	__cplusplus
}
#endif

#endif	/* _SND_SYS_H_ */

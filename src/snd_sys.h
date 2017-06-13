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

#include "sim_intf.h"
#include "types.h"
#include "xtcas.h"

#ifdef	__cplusplus
extern "C" {
#endif

void xtcas_play_msg(tcas_msg_t msg);

bool_t xtcas_snd_sys_init(const char *snd_dir, sound_on_t snd_op);
void xtcas_snd_sys_fini(void);

#ifdef	__cplusplus
}
#endif

#endif	/* _SND_SYS_H_ */

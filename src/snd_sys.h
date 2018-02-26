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

#include "xtcas.h"

#ifdef	__cplusplus
extern "C" {
#endif

bool_t xtcas_snd_sys_init(const char *snd_dir);
void xtcas_snd_sys_fini(void);

void xtcas_play_msg(tcas_msg_t msg);
void xtcas_play_msgs(tcas_msg_t *msgs);
void xtcas_set_suppressed(bool_t flag);
bool_t xtcas_is_suppressed(void);
void xtcas_stop_msg(bool_t empty_queue);
bool_t xtcas_msg_is_playing(void);
void xtcas_snd_sys_run(double volume);

#ifdef	__cplusplus
}
#endif

#endif	/* _SND_SYS_H_ */

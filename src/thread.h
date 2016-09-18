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

#ifndef	_XTCAS_THREAD_H_
#define	_XTCAS_THREAD_H_

#if APL || LIN
#include <pthread.h>
#define	mutex_t	pthread_mutex_t
#else
#error	"Unsupported platform"
#endif

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

void xtcas_mutex_init(mutex_t *mtx);
void xtcas_mutex_destroy(mutex_t *mtx);
void xtcas_mutex_enter(mutex_t *mtx);
bool_t xtcas_mutex_tryenter(mutex_t *mtx);
void xtcas_mutex_exit(mutex_t *mtx);

#ifdef __cplusplus
}
#endif

#endif	/* _XTCAS_THREAD_H_ */

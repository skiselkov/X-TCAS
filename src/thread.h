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

#ifndef	_XTCAS_THREAD_H_
#define	_XTCAS_THREAD_H_

#include <stdlib.h>

#if APL || LIN
#include <pthread.h>
#include <stdint.h>
#else	/* !APL && !LIN */
#error	"Unsupported platform"
#endif	/* !APL && !LIN */

#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

#if	APL || LIN

#define	thread_t		pthread_t
#define	mutex_t			pthread_mutex_t
#define	condvar_t		pthread_cond_t

#define	thread_create(thrp, proc, arg) \
	(pthread_create(thrp, NULL, (void *(*)(void *))proc, \
	    arg) == 0)
#define	thread_join(thrp)	pthread_join(*(thrp), NULL)

#define	mutex_init(x)		pthread_mutex_init((x), NULL)
#define	mutex_destroy(x)	pthread_mutex_destroy((x))
#define	mutex_enter(x)		pthread_mutex_lock((x))
#define	mutex_tryenter(x)	(pthread_mutex_trylock((x)) == 0)
#define	mutex_exit(x)		pthread_mutex_unlock((x))

#define	cv_wait(cond, mtx)	pthread_cond_wait((cond), (mtx))
#define	cv_timedwait		xtcas_cv_timedwait
#define	cv_init(x)		pthread_cond_init((x), NULL)
#define	cv_destroy(x)		pthread_cond_destroy((x))
#define	cv_signal(x)		pthread_cond_signal((x))
#define	cv_broadcast(x)		pthread_cond_broadcast((x))

bool_t xtcas_cv_timedwait(condvar_t *cond, mutex_t *mtx, uint64_t microtime);

#else	/* !APL && !LIN */
#error "Missing threading implementation for this platform"
#endif	/* !APL && !LIN */

#ifdef __cplusplus
}
#endif

#endif	/* _XTCAS_THREAD_H_ */

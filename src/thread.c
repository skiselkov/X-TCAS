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

#include "thread.h"

#if	APL || LIN

void
xtcas_mutex_init(mutex_t *mtx)
{
	pthread_mutex_init(mtx, NULL);
}

void
xtcas_mutex_destroy(mutex_t *mtx)
{
	pthread_mutex_destroy(mtx);
}

void
xtcas_mutex_enter(mutex_t *mtx)
{
	pthread_mutex_lock(mtx);
}

bool_t
xtcas_mutex_tryenter(mutex_t *mtx)
{
	return (pthread_mutex_trylock(mtx) == 0);
}

void
xtcas_mutex_exit(mutex_t *mtx)
{
	pthread_mutex_unlock(mtx);
}

#else	/* !APL && !LIN */
#error	"Unsupported platform"
#endif	/* !APL && !LIN */

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

#include "thread.h"

#if	APL || LIN

bool_t
xtcas_cv_timedwait(condvar_t *cond, mutex_t *mtx, uint64_t microtime)
{
	struct timespec ts = { .tv_sec = microtime / 1000000,
	    .tv_nsec = (microtime % 1000000) * 1000 };
	return (pthread_cond_timedwait(cond, mtx, &ts) == 0);
}

#endif	/* APL || LIN */

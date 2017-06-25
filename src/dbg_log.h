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

#ifndef	_XTCAS_DBG_LOG_H_
#define	_XTCAS_DBG_LOG_H_

#ifdef	__cplusplus
extern "C" {
#endif

typedef struct {
	int all;
	int snd;
	int wav;
	int tcas;
	int xplane;
	int test;
	int ra;
	int cpa;
	int sl;
	int contact;
	int threat;
} debug_config_t;

extern debug_config_t xtcas_dbg;

#define dbg_log(class, level, ...) \
	do { \
		if (xtcas_dbg.class >= level || xtcas_dbg.all >= level) { \
			log_impl(log_basename(__FILE__), __LINE__, \
			    "[" #class "/" #level "] " __VA_ARGS__); \
		} \
	} while (0)

#ifdef	__cplusplus
}
#endif

#endif	/* _XTCAS_DBG_LOG_H_ */

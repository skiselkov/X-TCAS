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

#include "dbg_log.h"

debug_config_t xtcas_dbg = {
	.all = 0, .snd = 0, .wav = 0, .tcas = 0, .xplane = 0, .test = 0,
	.ra = 0, .cpa = 0, .sl = 0, .contact = 0, .threat = 0, .ff_a320 = 0
};

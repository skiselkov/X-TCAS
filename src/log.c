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

#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "XPLMUtilities.h"
#include "helpers.h"
#include "log.h"

#define	XTCAS_PREFIX_LEN	8
#define	XTCAS_PREFIX		"X-TCAS: "

void xtcas_log(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    xtcas_log_v(fmt, ap);
    va_end(ap);
}

void xtcas_log_v(const char *fmt, va_list ap)
{
    va_list ap_copy;
    char *buf;
    int len;

    va_copy(ap_copy, ap);
    len = vsnprintf(NULL, 0, fmt, ap_copy);
    va_end(ap_copy);

    buf = (char *)malloc(len + XTCAS_PREFIX_LEN + 1);
    VERIFY(vsnprintf(buf + XTCAS_PREFIX_LEN, len + 1, fmt, ap) == len);
    memcpy(buf, XTCAS_PREFIX, XTCAS_PREFIX_LEN);
    XPLMDebugString(buf);
    free(buf);
}

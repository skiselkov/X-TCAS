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

#ifndef _XTCAS_LOG_H_
#define _XTCAS_LOG_H_

#include <stdarg.h>
#include "helpers.h"

#ifdef __cplusplus
extern "C" {
#endif

void xtcas_log(const char *fmt, ...) PRINTF_ATTR(1);
void xtcas_log_v(const char *fmt, va_list ap);

#ifdef __cplusplus
}
#endif

#endif /* _XTCAS_LOG_H_ */

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

#ifndef	_XTCAS_HTBL_H_
#define	_XTCAS_HTBL_H_

#include <stdint.h>

#include "list.h"

#ifdef	__cplusplus
extern "C" {
#endif

struct htbl_bucket_item_s;

typedef struct {
	void				*value;
	list_node_t			node;
	struct htbl_bucket_item_s	*item;
} htbl_multi_value_t;

typedef struct htbl_bucket_item_s {
	list_node_t	bucket_node;
	union {
		void		*value;
		struct {
			list_t	list;
			size_t	num;
		} multi;
	};
	uint8_t		key[1];	/* variable length, depends on htbl->key_sz */
} htbl_bucket_item_t;

typedef struct {
	size_t		tbl_sz;
	size_t		key_sz;
	list_t		*buckets;
	size_t		num_values;
	int		multi_value;
} htbl_t;

void htbl_create(htbl_t *htbl, size_t tbl_sz, size_t key_sz, int multi_value);
void htbl_destroy(htbl_t *htbl);
void htbl_empty(htbl_t *htbl, void (*func)(void *, void *), void *arg);
size_t htbl_count(const htbl_t *htbl);

void htbl_set(htbl_t *htbl, void *key, void *value);
void htbl_remove(htbl_t *htbl, void *key, int nil_ok);
void htbl_remove_multi(htbl_t *htbl, void *key, void *list_item);

void *htbl_lookup(const htbl_t *htbl, const void *key);
#define	HTBL_VALUE_MULTI(x)	(((htbl_multi_value_t *)(x))->value)
const list_t *htbl_lookup_multi(const htbl_t *htbl, const void *key);

void htbl_foreach(const htbl_t *htbl,
    void (*func)(const void *, void *, void *), void *arg);

char *htbl_dump(const htbl_t *htbl, bool_t printable_keys);

#ifdef	__cplusplus
}
#endif

#endif	/* _XTCAS_HTBL_H_ */

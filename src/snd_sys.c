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

#include <stddef.h>
#include <string.h>

#ifndef	TEST_STANDALONE_BUILD
#include <XPLMProcessing.h>
#else	/* TEST_STANDALONE_BUILD */
#include <acfutils/thread.h>
#include <acfutils/time.h>
#endif	/* TEST_STANDALONE_BUILD */

#include <acfutils/assert.h>
#include <acfutils/list.h>
#include <acfutils/wav.h>

#include "dbg_log.h"
#include "snd_sys.h"

typedef struct msg {
	const char *file;
	wav_t *wav;
} msg_info_t;

static msg_info_t voice_msgs[RA_NUM_MSGS] = {
	{ .file = "clb.wav",		.wav = NULL },
	{ .file = "clb_cross.wav",	.wav = NULL },
	{ .file = "clb_more.wav",	.wav = NULL },
	{ .file = "clb_now.wav",	.wav = NULL },
	{ .file = "clear.wav",		.wav = NULL },
	{ .file = "des.wav",		.wav = NULL },
	{ .file = "des_cross.wav",	.wav = NULL },
	{ .file = "des_more.wav",	.wav = NULL },
	{ .file = "des_now.wav",	.wav = NULL },
	{ .file = "monitor_vs.wav",	.wav = NULL },
	{ .file = "maint_vs.wav",	.wav = NULL },
	{ .file = "maint_vs_cross.wav",	.wav = NULL },
	{ .file = "level_off.wav",	.wav = NULL },
	{ .file = "tfc.wav",		.wav = NULL }
};

static bool_t		inited = B_FALSE;
static tcas_msg_t	cur_msg = -1;
static double		cur_volume = 1.0;

bool_t
xtcas_snd_sys_init(const char *snd_dir)
{
	dbg_log(snd, 1, "snd_sys_init");

	ASSERT(!inited);

	/* no WAV/OpenAL calls before this */
	if (!openal_init())
		return (B_FALSE);

	for (tcas_msg_t msg = 0; msg < RA_NUM_MSGS; msg++) {
		char *pathname;

		ASSERT3P(voice_msgs[msg].wav, ==, NULL);
		pathname = mkpathname(snd_dir, voice_msgs[msg].file, NULL);
		voice_msgs[msg].wav = wav_load(pathname, voice_msgs[msg].file);
		if (voice_msgs[msg].wav == NULL) {
			free(pathname);
			goto errout;
		}
		wav_set_gain(voice_msgs[msg].wav, 1.0);
		free(pathname);
	}

	inited = B_TRUE;

	return (B_TRUE);

errout:
	for (tcas_msg_t msg = 0; msg < RA_NUM_MSGS; msg++) {
		if (voice_msgs[msg].wav != NULL) {
			wav_free(voice_msgs[msg].wav);
			voice_msgs[msg].wav = NULL;
		}
	}
	openal_fini();

	return (B_FALSE);
}

void
xtcas_snd_sys_fini(void)
{
	dbg_log(snd, 1, "snd_sys_fini");

	if (!inited)
		return;

	for (tcas_msg_t msg = 0; msg < RA_NUM_MSGS; msg++) {
		if (voice_msgs[msg].wav != NULL) {
			wav_free(voice_msgs[msg].wav);
			voice_msgs[msg].wav = NULL;
		}
	}

	/* no more OpenAL/WAV calls after this */
	openal_fini();

	inited = B_FALSE;
}

void
xtcas_play_msg(tcas_msg_t msg)
{
	ASSERT(inited);
	ASSERT3U(msg, <, RA_NUM_MSGS);
	cur_msg = msg;
}

void
xtcas_snd_sys_run(double volume)
{
	tcas_msg_t msg;

	ASSERT(inited);

	if (cur_volume != volume) {
		cur_volume = volume;
		for (int i = 0; i < RA_NUM_MSGS; i++)
			wav_set_gain(voice_msgs[i].wav, volume);
	}

	msg = cur_msg;
	if ((int)msg != -1) {
		cur_msg = -1u;
		ASSERT3U(msg, <, RA_NUM_MSGS);
		(void) wav_play(voice_msgs[msg].wav);
	}
}

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

#include <XPLMProcessing.h>
#include <XPLMUtilities.h>

#include "assert.h"
#include "list.h"
#include "wav.h"
#include "xplane.h"

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
	{ .file = "adjust_vs.wav",	.wav = NULL },
	{ .file = "tfc.wav",		.wav = NULL }
};

static bool_t inited = B_FALSE;
static bool_t view_is_ext = B_TRUE;
static tcas_RA_msg_t cur_msg = -1;

static void
set_sound_on(bool_t flag)
{
	for (int i = 0; i < RA_NUM_MSGS; i++)
		wav_set_gain(voice_msgs[i].wav, flag ? 1.0 : 0);
}

void
play_msg(tcas_RA_msg_t msg)
{
	cur_msg = msg;
}

static float
snd_sched_cb(float elapsed_since_last_call, float elapsed_since_last_floop,
    int counter, void *refcon)
{
	tcas_RA_msg_t msg = cur_msg;

	ASSERT(inited);
	UNUSED(elapsed_since_last_call);
	UNUSED(elapsed_since_last_floop);
	UNUSED(counter);
	UNUSED(refcon);

	if ((int)msg != -1)
		return (-1.0);
	cur_msg = -1;

	/*
	 * Make sure our messages are only audible when we're inside
	 * the cockpit and AC power is on.
	 */
	if (view_is_ext && !xtcas_view_is_external()) {
		set_sound_on(B_TRUE);
		view_is_ext = B_FALSE;
	} else if (!view_is_ext && xtcas_view_is_external()) {
		set_sound_on(B_FALSE);
		view_is_ext = B_TRUE;
	}

	(void) wav_play(voice_msgs[msg].wav);

	return (-1.0);
}

bool_t
snd_sys_init(const char *plugindir)
{
	dbg_log(snd, 1, "snd_sys_init");

	ASSERT(!inited);

	/* no WAV/OpenAL calls before this */
	if (!openal_init())
		return (B_FALSE);

	for (tcas_RA_msg_t msg = 0; msg < RA_NUM_MSGS; msg++) {
		char *pathname;

		ASSERT3P(voice_msgs[msg].wav, ==, NULL);
		pathname = mkpathname(plugindir, "data", "msgs",
		    voice_msgs[msg].file, NULL);
		voice_msgs[msg].wav = wav_load(pathname, voice_msgs[msg].file);
		if (voice_msgs[msg].wav == NULL) {
			free(pathname);
			goto errout;
		}
		wav_set_gain(voice_msgs[msg].wav, 1.0);
		free(pathname);
	}

	XPLMRegisterFlightLoopCallback(snd_sched_cb, -1.0, NULL);

	inited = B_TRUE;

	return (B_TRUE);

errout:
	for (tcas_RA_msg_t msg = 0; msg < RA_NUM_MSGS; msg++) {
		if (voice_msgs[msg].wav != NULL) {
			wav_free(voice_msgs[msg].wav);
			voice_msgs[msg].wav = NULL;
		}
	}
	openal_fini();

	return (B_FALSE);
}

void
snd_sys_fini(void)
{
	dbg_log(snd, 1, "snd_sys_fini");

	if (!inited)
		return;

	XPLMUnregisterFlightLoopCallback(snd_sched_cb, NULL);

	for (tcas_RA_msg_t msg = 0; msg < RA_NUM_MSGS; msg++) {
		if (voice_msgs[msg].wav != NULL) {
			wav_free(voice_msgs[msg].wav);
			voice_msgs[msg].wav = NULL;
		}
	}

	/* no more OpenAL/WAV calls after this */
	openal_fini();

	inited = B_FALSE;
}

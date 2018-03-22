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
#include <acfutils/safe_alloc.h>
#include <acfutils/time.h>
#include <acfutils/thread.h>

#include "dbg_log.h"
#include "snd_sys.h"

typedef struct msg {
	const char	*file;
	wav_t		*wav;
	const double	gap;			/* seconds */
} msg_info_t;

typedef struct {
	msg_info_t	*mi;
	list_node_t	node;
} msg_play_t;

static msg_info_t voice_msgs[RA_NUM_MSGS] = {
#if	GTS820_MODE
	{ .file = NULL },			/* RA_MSG_CLB */
	{ .file = NULL },			/* RA_MSG_CLB_CROSS */
	{ .file = NULL },			/* RA_MSG_CLB_MORE */
	{ .file = NULL },			/* RA_MSG_CLB_NOW */
	{ .file = NULL },			/* RA_MSG_CLEAR */
	{ .file = NULL },			/* RA_MSG_DES */
	{ .file = NULL },			/* RA_MSG_DES_CROSS */
	{ .file = NULL },			/* RA_MSG_DES_MORE */
	{ .file = NULL },			/* RA_MSG_DES_NOW */
	{ .file = NULL },			/* RA_MSG_MONITOR_VS */
	{ .file = NULL },			/* RA_MSG_MAINT_VS */
	{ .file = NULL },			/* RA_MSG_MAINT_VS_CROSS */
	{ .file = NULL },			/* RA_MSG_LEVEL_OFF */
	{ .file = "tfc.wav", .gap = 0.25 },	/* RA_MSG_TFC */
	{ .file = "tas_test_pass.wav" },	/* TCAS_TEST_PASS */
	{ .file = "tas_test_fail.wav" },	/* TCAS_TEST_FAIL */
	{ .file = "1clk.wav", .gap = 0.25 },	/* GTS820_MSG_1CLK */
	{ .file = "2clk.wav", .gap = 0.25 },	/* GTS820_MSG_2CLK */
	{ .file = "3clk.wav", .gap = 0.25 },	/* GTS820_MSG_3CLK */
	{ .file = "4clk.wav", .gap = 0.25 },	/* GTS820_MSG_4CLK */
	{ .file = "5clk.wav", .gap = 0.25 },	/* GTS820_MSG_5CLK */
	{ .file = "6clk.wav", .gap = 0.25 },	/* GTS820_MSG_6CLK */
	{ .file = "7clk.wav", .gap = 0.25 },	/* GTS820_MSG_7CLK */
	{ .file = "8clk.wav", .gap = 0.25 },	/* GTS820_MSG_8CLK */
	{ .file = "9clk.wav", .gap = 0.25 },	/* GTS820_MSG_9CLK */
	{ .file = "10clk.wav", .gap = 0.25 },	/* GTS820_MSG_10CLK */
	{ .file = "11clk.wav", .gap = 0.25 },	/* GTS820_MSG_11CLK */
	{ .file = "12clk.wav", .gap = 0.25 },	/* GTS820_MSG_12CLK */
	{ .file = "high.wav", .gap = 0.25 },	/* GTS820_MSG_HIGH */
	{ .file = "low.wav", .gap = 0.25 },	/* GTS820_MSG_LOW */
	{ .file = "same_alt.wav", .gap = 0.25 },/* GTS820_MSG_SAME_ALT */
	{ .file = "m1nm.wav", .gap = 0.6 },	/* GTS820_MSG_M1NM */
	{ .file = "1nm.wav", .gap = 0.6 },	/* GTS820_MSG_1NM */
	{ .file = "2nm.wav", .gap = 0.6 },	/* GTS820_MSG_2NM */
	{ .file = "3nm.wav", .gap = 0.6 },	/* GTS820_MSG_3NM */
	{ .file = "4nm.wav", .gap = 0.6 },	/* GTS820_MSG_4NM */
	{ .file = "5nm.wav", .gap = 0.6 },	/* GTS820_MSG_5NM */
	{ .file = "6nm.wav", .gap = 0.6 },	/* GTS820_MSG_6NM */
	{ .file = "7nm.wav", .gap = 0.6 },	/* GTS820_MSG_7NM */
	{ .file = "8nm.wav", .gap = 0.6 },	/* GTS820_MSG_8NM */
	{ .file = "9nm.wav", .gap = 0.6 },	/* GTS820_MSG_9NM */
	{ .file = "10nm.wav", .gap = 0.6 },	/* GTS820_MSG_10NM */
	{ .file = "p10nm.wav", .gap = 0.6 }	/* GTS820_MSG_P10NM */
#else	/* !GTS820_MODE */
	{ .file = "clb.wav" },
	{ .file = "clb_cross.wav" },
	{ .file = "clb_more.wav" },
	{ .file = "clb_now.wav" },
	{ .file = "clear.wav" },
	{ .file = "des.wav" },
	{ .file = "des_cross.wav" },
	{ .file = "des_more.wav" },
	{ .file = "des_now.wav" },
	{ .file = "monitor_vs.wav" },
	{ .file = "maint_vs.wav" },
	{ .file = "maint_vs_cross.wav" },
	{ .file = "level_off.wav" },
	{ .file = "tfc.wav" },
	{ .file = "tcas_test_pass.wav" },
	{ .file = "tcas_test_fail.wav" }
#endif	/* !GTS820_MODE */
};

static bool_t		inited = B_FALSE;
static mutex_t		lock;
static list_t		cur_msgs;
static msg_info_t	*playing_msg = NULL;
static uint64_t		msg_started_t = 0;	/* microclock units */
static uint64_t		msg_dur = 0;		/* microseconds */
static bool_t		suppressed = B_FALSE;
static double		cur_volume = 1.0;
static alc_t		*alc = NULL;

bool_t
xtcas_snd_sys_init(const char *snd_dir)
{
	dbg_log(snd, 1, "snd_sys_init");

	ASSERT(!inited);

	mutex_init(&lock);
	list_create(&cur_msgs, sizeof (msg_play_t),
	    offsetof(msg_play_t, node));

	/* no WAV/OpenAL calls before this */
	alc = openal_init(NULL, B_FALSE);
	if (alc == NULL)
		return (B_FALSE);

	for (tcas_msg_t msg = 0; msg < RA_NUM_MSGS; msg++) {
		char *pathname;

		if (voice_msgs[msg].file == NULL)
			continue;

		ASSERT3P(voice_msgs[msg].wav, ==, NULL);
		pathname = mkpathname(snd_dir, voice_msgs[msg].file, NULL);
		voice_msgs[msg].wav = wav_load(pathname, voice_msgs[msg].file,
		    alc);
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
	if (alc != NULL) {
		openal_fini(alc);
		alc = NULL;
	}

	return (B_FALSE);
}

void
xtcas_snd_sys_fini(void)
{
	msg_play_t *play;

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
	openal_fini(alc);
	alc = NULL;

	while ((play = list_remove_head(&cur_msgs)) != NULL)
		free(play);
	list_destroy(&cur_msgs);
	mutex_destroy(&lock);

	inited = B_FALSE;
}

/*
 * Schedules a message for playback. This will be picked up by
 * xtcas_snd_sys_run and played (if not suppressed).
 */
void
xtcas_play_msg(tcas_msg_t msg)
{
	tcas_msg_t msgs[] = { msg, (tcas_msg_t)-1 };

	ASSERT(inited);
	ASSERT3U(msg, <, RA_NUM_MSGS);

	xtcas_play_msgs(msgs);
}

void xtcas_play_msgs(tcas_msg_t *msgs)
{
	msg_play_t *play;

	ASSERT(inited);

	mutex_enter(&lock);
	while ((play = list_remove_head(&cur_msgs)) != NULL)
		free(play);
	for (int i = 0; msgs[i] != (tcas_msg_t)-1u; i++) {
		tcas_msg_t msg = msgs[i];

		ASSERT3U(msg, <, RA_NUM_MSGS);
		ASSERT(voice_msgs[msg].wav != NULL);
		play = safe_calloc(1, sizeof (*play));
		play->mi = &voice_msgs[msg];
		list_insert_tail(&cur_msgs, play);
	}
	mutex_exit(&lock);
}

static void
xtcas_stop_msg_impl(bool_t empty_queue)
{
	playing_msg = NULL;
	if (empty_queue) {
		msg_play_t *play;

		while ((play = list_remove_head(&cur_msgs)) != NULL)
			free(play);
	}
}

/*
 * Stops playback of the currently playing message (if any).
 */
void
xtcas_stop_msg(bool_t empty_queue)
{
	mutex_enter(&lock);
	xtcas_stop_msg_impl(empty_queue);
	mutex_exit(&lock);
}

/*
 * Sets the suppressing flag. Any new messages will be queued, but won't
 * be played until suppression is lifted.
 */
void
xtcas_set_suppressed(bool_t flag)
{
	mutex_enter(&lock);
	if (flag)
		xtcas_stop_msg_impl(B_FALSE);
	suppressed = flag;
	mutex_exit(&lock);
}

bool_t
xtcas_is_suppressed(void)
{
	return (suppressed);
}

/*
 * Returns true if a message is currently playing, false otherwise.
 */
bool_t
xtcas_msg_is_playing(void)
{
	return (msg_started_t + msg_dur > microclock());
}

/*
 * Main sound scheduling loop. You must call this periodically to let X-TCAS's
 * sound system operate.
 */
void
xtcas_snd_sys_run(double volume)
{
	ASSERT(inited);

	if (cur_volume != volume) {
		cur_volume = volume;
		for (int i = 0; i < RA_NUM_MSGS; i++)
			wav_set_gain(voice_msgs[i].wav, volume);
	}

	mutex_enter(&lock);

	if (xtcas_msg_is_playing() && playing_msg == NULL) {
		/*
		 * Hard playback stop requested. Stop everything.
		 */
		for (int i = 0; i < RA_NUM_MSGS; i++)
			wav_stop(voice_msgs[i].wav);
		msg_started_t = 0;
	}

	/* While suppressed, don't play any messages, but keep them queued. */
	if (suppressed) {
		mutex_exit(&lock);
		return;
	}

	if (!xtcas_msg_is_playing()) {
		msg_play_t *play;
		uint64_t now = microclock();

		playing_msg = NULL;
		play = list_remove_head(&cur_msgs);
		if (play != NULL) {
			msg_info_t *mi = play->mi;

			(void) wav_play(mi->wav);
			playing_msg = mi;
			msg_started_t = now;
			msg_dur = SEC2USEC(mi->wav->duration + mi->gap);
			free(play);
		}
	}

	mutex_exit(&lock);
}

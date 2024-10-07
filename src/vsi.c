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
 * Copyright 2024 Saso Kiselkov. All rights reserved.
 */

#include <ctype.h>
#include <stddef.h>

#include <cairo.h>
#include <cairo-ft.h>
#include <ft2build.h>
#include FT_FREETYPE_H

#include <XPLMGraphics.h>
#include <XPLMDisplay.h>

#include <acfutils/assert.h>
#include <acfutils/avl.h>
#include <acfutils/dr.h>
#include <acfutils/glew.h>
#include <acfutils/math.h>
#include <acfutils/mt_cairo_render.h>
#include <acfutils/perf.h>
#include <acfutils/safe_alloc.h>
#include <acfutils/thread.h>
#include <acfutils/time.h>

#include "vsi.h"
#include "xplane.h"

#define	VSI_SCREEN_DELAY	2
#define	VSI_SCREEN_WHITE_DELAY	2.1
#define	VSI_RING_DELAY		4
#define	VSI_IND_DELAY		6
#define	VSI_TCAS_DELAY		8
#define	MAX_BUSNR		6
#define	BRT_DFL			50

#define	X(x)	((x) * tex->sz)

#define	VSI_NUM_SCALES	5
#define	VSI_SCALE_DFL	1

#if	VSI_STYLE == VSI_STYLE_ATR
static int vsi_scales[VSI_NUM_SCALES] = { 3, 6, 12, 24, 48 };
#else	/* VSI_STYLE == VSI_STYLE_HONEYWELL */
static int vsi_scales[VSI_NUM_SCALES] = { 3, 5, 10, 20, 40 };
#endif	/* VSI_STYLE == VSI_STYLE_HONEYWELL */

#define	DRAW_INTVAL		50000	/* microseconds = 20 fps */
#define	MAX_SZ			2048	/* pixels */
#define	DR_NAME_MAX		128	/* bytes */
#define	BACKLIGHT_BLEED		0.05
#define	DEFAULT_BRT		50.0
#define	FONT_FILE		"RobotoCondensed-Regular.ttf"
#define	MAX_VS			6000
#define	MIN_VS			-6000
#define	VSI_RING_RADIUS		0.4
#define	VSI_CTC_RADIUS		0.39

#define	CYAN_RGB		0.2588, 1, 0.9648
#define	RED_RGB			1, 0, 0
#define	YELLOW_RGB		1, 1, 0

typedef struct {
	tcas_adv_t	adv;
	double		min_green;
	double		max_green;
	double		min_red_lo;
	double		max_red_lo;
	double		min_red_hi;
	double		max_red_hi;
} vsi_state_t;

typedef struct {
	cairo_t		*cr;
	cairo_surface_t	*surf;
	unsigned	sz;
	bool_t		chg;
	GLuint		gl_tex;
} vsi_tex_t;

typedef enum {
	VS_FMT_FPM,		/* feet per minute */
	VS_FMT_MPS,		/* meters per second */
	VS_FMT_FPS,		/* feet per second */
	VS_FMT_MPM		/* meters per minute */
} vs_dr_fmt_t;

typedef struct {
	unsigned	x, y;
	dr_t		x_dr, y_dr;
	unsigned	sz;
	dr_t		sz_dr;
	int		scale_enum;	/* display scale (diameter) in NM */
	dr_t		scale_enum_dr;
	unsigned	brt;	/* 0..25 */
	dr_t		brt_dr;

	double		vs_value;	/* always in feet per minute */

	dr_t		vs_dr;
	char		vs_dr_name[DR_NAME_MAX];
	dr_t		vs_dr_name_dr;

	vs_dr_fmt_t	vs_dr_fmt;
	dr_t		vs_dr_fmt_dr;

	dr_t		fail_dr;
	char		fail_dr_name[DR_NAME_MAX];
	dr_t		fail_dr_name_dr;

	int		busnr;
	dr_t		busnr_dr;

	bool_t		custom_bus;
	char		custom_bus_name[128];
	dr_t		custom_bus_dr;

	bool_t		functional;
	bool_t		running;
	bool_t		shutdown;
	thread_t	thread;
	mutex_t		lock;
	condvar_t	cv;

	mutex_t		tex_lock;
	int		cur_tex;
	vsi_tex_t	tex[2];
	cairo_t		*cr;

	mutex_t		state_lock;
	vsi_state_t	state;

	uint64_t	start_time;
} vsi_t;

typedef struct {
	const void	*acf_id;
	double		rbrg;
	double		rdist;
	double		ralt;
	double		vs;
	tcas_threat_t	level;
	avl_node_t	node;
} ctc_t;

typedef struct {
	double		bottom;		/* bottom edge of range in feet */
	double		top;		/* top edge of range in feet */
	double		bottom_angle;	/* degrees from 0 for bottom mark */
	double		top_angle;	/* degrees from 0 for top mark */
	const char	*label;		/* number format */
	double		major_len;	/* major scale line length */
	double		minor_len;	/* minor scale line length */
	double		major_thickness;/* major scale line thickness */
	double		minor_thickness;/* minor scale line thickness */
	double		step;		/* submark step */
	vect2_t		text_off;
	double		font_sz;
} vsi_range_t;

#define	NUM_VSI_RANGES	8

static vsi_range_t vsi_ranges[] = {
    {	/* just the '0' tick mark */
	.bottom = -1, .top = 0, .bottom_angle = -1, .top_angle = 0,
	.label = "0", .major_len = 0.03, .major_thickness = 0.015,
	.step = 1, .text_off = VECT2(-0.07, 0), .font_sz = 0.085
    },
    {	/* 0..500 */
	.bottom = 0, .top = 500, .bottom_angle = 0, .top_angle = 35,
	.label = ".5", .major_len = 0.05, .minor_len = 0.03,
	.major_thickness = 0.01, .minor_thickness = 0.008, .step = 100,
	.text_off = VECT2(-0.07, 0.03), .font_sz = 0.075
    },
    {	/* 500..1000 */
	.bottom = 500, .top = 1000, .bottom_angle = 35, .top_angle = 70,
	.label = "1", .major_len = 0.08, .minor_len = 0.03,
	.major_thickness = 0.015, .minor_thickness = 0.008, .step = 100,
	.text_off = VECT2(-0.06, 0.07), .font_sz = 0.085
    },
    {	/* 1000..2000 */
	.bottom = 1000, .top = 2000, .bottom_angle = 70, .top_angle = 105,
	.label = "2", .major_len = 0.08, .minor_len = 0.05,
	.major_thickness = 0.015, .minor_thickness = 0.01, .step = 500,
	.text_off = VECT2(0.055, 0.07), .font_sz = 0.085
    },
    {	/* 2000..3000 */
	.bottom = 2000, .top = 3000, .bottom_angle = 105, .top_angle = 122.5,
	.label = NULL, .major_len = 0.08, .minor_len = 0.05,
	.major_thickness = 0.015, .minor_thickness = 0.01, .step = 500
    },
    {	/* 3000..4000 */
	.bottom = 3000, .top = 4000, .bottom_angle = 122.5, .top_angle = 140,
	.label = "4", .major_len = 0.08, .minor_len = 0.05,
	.major_thickness = 0.015, .minor_thickness = 0.01, .step = 500,
	.text_off = VECT2(0.09, 0.06), .font_sz = 0.085
    },
    {	/* 4000..5000 */
	.bottom = 4000, .top = 5000, .bottom_angle = 140, .top_angle = 155,
	.label = NULL, .major_len = 0.08, .minor_len = 0.05,
	.major_thickness = 0.015, .minor_thickness = 0.01, .step = 500
    },
    {	/* 5000..6000 */
	.bottom = 5000, .top = 6000, .bottom_angle = 155, .top_angle = 170,
	.label = "6",
#if	VSI_STYLE == VSI_STYLE_ATR
	.major_len = 0.04,
#else	/* VSI_STYLE == VSI_STYLE_HONEYWELL */
	.major_len = 0.08,
#endif	/* VSI_STYLE == VSI_STYLE_HONEYWELL */
	.minor_len = 0.05,
	.major_thickness = 0.015, .minor_thickness = 0.01, .step = 500,
	.text_off = VECT2(0.075, 0.01), .font_sz = 0.085
    }
};


#define	MAX_VSIS	4
static vsi_t vsis[MAX_VSIS];
static bool_t inited = B_FALSE;
static mutex_t ctc_lock;
static avl_tree_t ctcs;
static dr_t bus_volts;
static bool_t xpdr_functional = B_FALSE;

static FT_Library ft = NULL;
static FT_Face font = NULL;
static cairo_font_face_t *cr_font = NULL;

static int
ctc_compar(const void *a, const void *b)
{
	const ctc_t *ca = a, *cb = b;

	if (ca->acf_id < cb->acf_id)
		return (-1);
	if (ca->acf_id > cb->acf_id)
		return (1);
	return (0);
}

static void
shutdown_vsi(unsigned vsi_nr)
{
	vsi_t *vsi = &vsis[vsi_nr];

	ASSERT3U(vsi_nr, <, MAX_VSIS);

	if (!vsi->running)
		return;

	mutex_enter(&vsi->lock);
	vsi->shutdown = B_TRUE;
	cv_broadcast(&vsi->cv);
	mutex_exit(&vsi->lock);

	thread_join(&vsi->thread);
	vsi->running = B_FALSE;
	vsi->shutdown = B_FALSE;
}

static void
set_color(vsi_t *vsi, vsi_tex_t *tex, double r, double g, double b)
{
	/* Rescale the RGB values to include a bit of backlight bleed */
	r = r * (1 - BACKLIGHT_BLEED) + BACKLIGHT_BLEED;
	g = g * (1 - BACKLIGHT_BLEED) + BACKLIGHT_BLEED;
	b = b * (1 - BACKLIGHT_BLEED) + BACKLIGHT_BLEED;
	cairo_set_source_rgb(tex->cr,
	    r * ((pow(4.0, vsi->brt / DEFAULT_BRT)) / 4.0),
	    g * ((pow(4.0, vsi->brt / DEFAULT_BRT)) / 4.0),
	    b * ((pow(4.0, vsi->brt / DEFAULT_BRT)) / 4.0));
}

static void
draw_ranges(vsi_t *vsi, vsi_tex_t *tex)
{
	set_color(vsi, tex, 1, 1, 1);
	cairo_set_line_width(tex->cr, X(0.006));
	cairo_set_font_face(tex->cr, cr_font);

	for (int i = 0; i < NUM_VSI_RANGES; i++) {
		vsi_range_t *r = &vsi_ranges[i];
		double scale = (r->top_angle - r->bottom_angle) /
		    (r->top - r->bottom);

		for (double x = r->bottom + r->step; x <= r->top;
		    x += r->step) {
			double a = ((x - r->bottom) * scale) + r->bottom_angle;
			vect2_t s = VECT2(-VSI_RING_RADIUS, 0);
			vect2_t e, s_pos, e_pos, s_neg, e_neg;
			double t;

			if (x < r->top) {
				e = VECT2((-VSI_RING_RADIUS - r->minor_len), 0);
				t = r->minor_thickness;
			} else {
				e = VECT2((-VSI_RING_RADIUS - r->major_len), 0);
				t = r->major_thickness;
			}
			s_pos = vect2_rot(s, a);
			s_neg = vect2_rot(s, -a);
			e_pos = vect2_rot(e, a);
			e_neg = vect2_rot(e, -a);

			cairo_set_line_width(tex->cr, X(t));
			cairo_move_to(tex->cr, X(s_pos.x), X(s_pos.y));
			cairo_line_to(tex->cr, X(e_pos.x), X(e_pos.y));
			if (i != 0) {
				cairo_move_to(tex->cr, X(s_neg.x), X(s_neg.y));
				cairo_line_to(tex->cr, X(e_neg.x), X(e_neg.y));
			}
			cairo_stroke(tex->cr);
		}
	}

#if	VSI_STYLE == VSI_STYLE_HONEYWELL
	cairo_set_line_width(tex->cr, X(0.005));
	cairo_arc(tex->cr, 0, 0, X(VSI_RING_RADIUS), 0, DEG2RAD(360));
	cairo_stroke(tex->cr);
#endif	/* VSI_STYLE == VSI_STYLE_HONEYWELL */

	for (int i = 0; i < NUM_VSI_RANGES; i++) {
		vsi_range_t *r = &vsi_ranges[i];
		cairo_text_extents_t te;

		if (r->label == NULL)
			continue;

		vect2_t v = VECT2(-VSI_RING_RADIUS, 0);

		cairo_set_font_size(tex->cr, round(X(r->font_sz)));
		cairo_text_extents(tex->cr, r->label, &te);

#if	VSI_STYLE == VSI_STYLE_HONEYWELL
		if (strcmp(r->label, "6") == 0) {
			cairo_move_to(tex->cr,
			    X(VSI_RING_RADIUS + r->major_len) - te.width / 2 -
			    te.x_bearing, -te.height / 2 - te.y_bearing);
			cairo_show_text(tex->cr, r->label);
			continue;
		}
#endif	/* VSI_STYLE == VSI_STYLE_HONEYWELL */

		v = vect2_rot(v, r->top_angle);
		cairo_move_to(tex->cr,
		    X(v.x + r->text_off.x) - te.width / 2 - te.x_bearing,
		    X(v.y + r->text_off.y) - te.height / 2 - te.y_bearing);
		cairo_show_text(tex->cr, r->label);
		cairo_move_to(tex->cr,
		    X(v.x + r->text_off.x) - te.width / 2 - te.x_bearing,
		    X(-v.y - r->text_off.y) - te.height / 2 - te.y_bearing);
		cairo_show_text(tex->cr, r->label);
	}
}

static double
find_vs_angle(double vs)
{
	vs = MIN(MAX(MPS2FPM(vs), MIN_VS), MAX_VS);
	for (int i = 1; i < NUM_VSI_RANGES; i++) {
		vsi_range_t *r = &vsi_ranges[i];
		double scale = (r->top_angle - r->bottom_angle) /
		    (r->top - r->bottom);
		double a = ((fabs(vs) - r->bottom) * scale) + r->bottom_angle;

		if (vs >= vsi_ranges[i].bottom && vs <= vsi_ranges[i].top) {
			return (a);
		} else if (vs <= -vsi_ranges[i].bottom &&
		    vs >= -vsi_ranges[i].top) {
			return (-a);
		}
	}
	VERIFY_MSG(0, "Internal inconsistency with VS %f", vs);
}

static void
draw_needle(vsi_t *vsi, vsi_tex_t *tex)
{
#define	NEEDLE_THICKNESS	0.02
#define	NEEDLE_LENGTH		0.39
#if	VSI_STYLE == VSI_STYLE_ATR
#define	NEEDLE_HEAD_LENGTH	0.05
#else	/* VSI_STYLE == VSI_STYLE_HONEYWELL */
#define	NEEDLE_HEAD_LENGTH	0.09
#endif	/* VSI_STYLE == VSI_STYLE_HONEYWELL */

	double angle = find_vs_angle(FPM2MPS(vsi->vs_value));

	if (USEC2SEC(microclock() - vsi->start_time) < VSI_IND_DELAY)
		return;

	cairo_save(tex->cr);
	cairo_rotate(tex->cr, DEG2RAD(angle));

	set_color(vsi, tex, 1, 1, 1);
	cairo_move_to(tex->cr, 0, X(NEEDLE_THICKNESS / 2));
	cairo_line_to(tex->cr, X(-NEEDLE_LENGTH + NEEDLE_HEAD_LENGTH),
	    X(NEEDLE_THICKNESS / 2));
	cairo_line_to(tex->cr, X(-NEEDLE_LENGTH + NEEDLE_HEAD_LENGTH),
	    X(NEEDLE_THICKNESS));
	cairo_line_to(tex->cr, X(-NEEDLE_LENGTH), 0);
	cairo_line_to(tex->cr, X(-NEEDLE_LENGTH + NEEDLE_HEAD_LENGTH),
	    X(-NEEDLE_THICKNESS));
	cairo_line_to(tex->cr, X(-NEEDLE_LENGTH + NEEDLE_HEAD_LENGTH),
	    X(-NEEDLE_THICKNESS / 2));
	cairo_line_to(tex->cr, 0, X(-NEEDLE_THICKNESS / 2));
	cairo_close_path(tex->cr);
	cairo_fill(tex->cr);

	cairo_restore(tex->cr);
}

static vect2_t
rel2xy(double rbrg, double rdist)
{
	return (vect2_scmul(hdg2dir(rbrg), rdist));
}

static int
get_scale(vsi_t *vsi)
{
	int scale_enum = vsi->scale_enum;

	if (scale_enum >= VSI_NUM_SCALES) {
		scale_enum = VSI_NUM_SCALES - 1;
		vsi->scale_enum = VSI_NUM_SCALES - 1;
	} else if (scale_enum < 0) {
		scale_enum = 0;
		vsi->scale_enum = 0;
	}
	return (vsi_scales[scale_enum]);
}

static vect2_t
scale_ctc(vsi_t *vsi, vect2_t xy, bool_t clamp)
{
	int scale = get_scale(vsi);

#define	CENTER_Y_OFF	(VSI_RING_RADIUS * 0.3333333)
	xy = vect2_scmul(xy, (1.0 / NM2MET(2 * scale)) * 2 * VSI_RING_RADIUS);
	xy.y = -xy.y + CENTER_Y_OFF;
	/*
	 * We clamp contacts a little closer than VSI_CTC_RADIUS, since that
	 * is also used for the clip arc. We want just a little more than
	 * half the aircraft symbol sticking out from that arc.
	 */
	if (clamp && vect2_abs(xy) > fabs(VSI_CTC_RADIUS) * 0.99)
		xy = vect2_set_abs(xy, VSI_CTC_RADIUS * 0.99);

	return (xy);
}

static void
draw_own_acf(vsi_t *vsi, vsi_tex_t *tex)
{
#if	VSI_STYLE == VSI_STYLE_ATR
#define	OWN_ACF_SYM_SIZE	0.05
#define	OWN_ACF_SYM_THICKNESS	0.01
	set_color(vsi, tex, 1, 1, 1);
#else	/* VSI_STYLE == VSI_STYLE_HONEYWELL */
#define	OWN_ACF_SYM_SIZE	0.04
#define	OWN_ACF_SYM_THICKNESS	0.005
	set_color(vsi, tex, CYAN_RGB);
#endif	/* VSI_STYLE == VSI_STYLE_HONEYWELL */
	vect2_t v = scale_ctc(vsi, ZERO_VECT2, B_FALSE);

	if (USEC2SEC(microclock() - vsi->start_time) < VSI_TCAS_DELAY)
		return;

	cairo_set_line_width(tex->cr, X(OWN_ACF_SYM_THICKNESS));

#if	VSI_STYLE == VSI_STYLE_ATR
	cairo_move_to(tex->cr, X(v.x), X(v.y));
	cairo_rel_move_to(tex->cr, 0, X(-OWN_ACF_SYM_SIZE * 0.25));
	cairo_rel_line_to(tex->cr, 0, X(OWN_ACF_SYM_SIZE));
#else	/* VSI_STYLE == VSI_STYLE_HONEYWELL */
	cairo_move_to(tex->cr, X(v.x), X(v.y));
	cairo_rel_move_to(tex->cr, 0, X(-OWN_ACF_SYM_SIZE * 0.35));
	cairo_rel_line_to(tex->cr, 0, X(OWN_ACF_SYM_SIZE));
#endif	/* VSI_STYLE == VSI_STYLE_HONEYWELL */

#if	VSI_STYLE == VSI_STYLE_ATR
	cairo_move_to(tex->cr, X(v.x), X(v.y));
	cairo_rel_move_to(tex->cr, X(-OWN_ACF_SYM_SIZE * 0.5), 0);
	cairo_rel_line_to(tex->cr, X(OWN_ACF_SYM_SIZE), 0);
#else	/* VSI_STYLE == VSI_STYLE_HONEYWELL */
	cairo_move_to(tex->cr, X(v.x), X(v.y));
	cairo_rel_move_to(tex->cr, X(-OWN_ACF_SYM_SIZE * 0.5),
	    X(OWN_ACF_SYM_SIZE * 0.2));
	cairo_rel_line_to(tex->cr, X(OWN_ACF_SYM_SIZE / 2),
	    X(-OWN_ACF_SYM_SIZE * 0.2));
	cairo_rel_line_to(tex->cr, X(OWN_ACF_SYM_SIZE / 2),
	    X(OWN_ACF_SYM_SIZE * 0.2));
#endif	/* VSI_STYLE == VSI_STYLE_HONEYWELL */

	cairo_move_to(tex->cr, X(v.x), X(v.y));
#if	VSI_STYLE == VSI_STYLE_ATR
	cairo_rel_move_to(tex->cr, X(-OWN_ACF_SYM_SIZE * 0.25),
	    X(OWN_ACF_SYM_SIZE * 0.75));
#else	/* VSI_STYLE == VSI_STYLE_HONEYWELL */
	cairo_rel_move_to(tex->cr, X(-OWN_ACF_SYM_SIZE * 0.25),
	    X(OWN_ACF_SYM_SIZE * 0.6));
#endif	/* VSI_STYLE == VSI_STYLE_HONEYWELL */
	cairo_rel_line_to(tex->cr, X(OWN_ACF_SYM_SIZE * 0.5), 0);

	cairo_stroke(tex->cr);
}

static void
draw_own_acf_ring(vsi_t *vsi, vsi_tex_t *tex)
{
#if	VSI_STYLE == VSI_STYLE_ATR
#define	OWN_ACF_RING_THICKNESS	0.005
	set_color(vsi, tex, 1, 1, 1);
#else	/* VSI_STYLE == VSI_STYLE_HONEYWELL */
#define	OWN_ACF_RING_DOT_SZ	0.005
	set_color(vsi, tex, CYAN_RGB);
#endif	/* VSI_STYLE == VSI_STYLE_HONEYWELL */

	cairo_arc(tex->cr, 0, 0, X(VSI_CTC_RADIUS), 0, DEG2RAD(360));
	cairo_clip(tex->cr);
#if	VSI_STYLE == VSI_STYLE_ATR
	cairo_set_line_width(tex->cr, X(OWN_ACF_RING_THICKNESS));
#endif
	for (double x = 0; x < 360; x += 30) {
		vect2_t p1 = scale_ctc(vsi, rel2xy(x, NM2MET(2)), B_FALSE);

#if	VSI_STYLE == VSI_STYLE_ATR
		vect2_t p2 = scale_ctc(vsi, rel2xy(x, NM2MET(2.3)), B_FALSE);

		cairo_move_to(tex->cr, X(p1.x), X(p1.y));
		cairo_line_to(tex->cr, X(p2.x), X(p2.y));
#else	/* VSI_STYLE == VSI_STYLE_HONEYWELL */
		cairo_arc(tex->cr, X(p1.x), X(p1.y), X(OWN_ACF_RING_DOT_SZ),
		    0, DEG2RAD(360));
		cairo_fill(tex->cr);
#endif	/* VSI_STYLE == VSI_STYLE_HONEYWELL */
	}
#if	VSI_STYLE == VSI_STYLE_ATR
	cairo_stroke(tex->cr);
#endif
	cairo_reset_clip(tex->cr);

	UNUSED(rel2xy);
}

static void
draw_contacts(vsi_t *vsi, vsi_tex_t *tex)
{
#define	CTC_SZ		0.06

	if (USEC2SEC(microclock() - vsi->start_time) < VSI_TCAS_DELAY)
		return;

	cairo_set_line_width(tex->cr, X(0.006));
	cairo_set_font_face(tex->cr, cr_font);
	cairo_set_font_size(tex->cr, round(X(CTC_SZ * 1.2)));

	cairo_arc(tex->cr, 0, 0, X(VSI_CTC_RADIUS), 0, DEG2RAD(360));
	cairo_clip(tex->cr);

	mutex_enter(&ctc_lock);
	for (const ctc_t *ctc = avl_first(&ctcs); ctc != NULL;
	    ctc = AVL_NEXT(&ctcs, ctc)) {
		vect2_t p = scale_ctc(vsi, rel2xy(ctc->rbrg, ctc->rdist),
		    ctc->level >= TA_THREAT);
		cairo_text_extents_t te;
		char alt_str[8];

		switch (ctc->level) {
		case OTH_THREAT:
		case PROX_THREAT:
#if	VSI_STYLE == VSI_STYLE_ATR
			set_color(vsi, tex, CYAN_RGB);
#else	/* VSI_STYLE == VSI_STYLE_HONEYWELL */
			set_color(vsi, tex, 1, 1, 1);
#endif
			cairo_move_to(tex->cr, X(p.x), X(p.y));
			cairo_rel_move_to(tex->cr, X(-CTC_SZ / 2), 0);
			cairo_rel_line_to(tex->cr, X(CTC_SZ / 2),
			    X(-CTC_SZ) / 2);
			cairo_rel_line_to(tex->cr, X(CTC_SZ / 2),
			    X(CTC_SZ) / 2);
			cairo_rel_line_to(tex->cr, X(-CTC_SZ / 2),
			    X(CTC_SZ) / 2);
			cairo_close_path(tex->cr);
			if (ctc->level == OTH_THREAT)
				cairo_stroke(tex->cr);
			else
				cairo_fill(tex->cr);
			break;
		case TA_THREAT:
			set_color(vsi, tex, YELLOW_RGB);
			cairo_arc(tex->cr, X(p.x), X(p.y), X(CTC_SZ / 2 * 0.9),
			    0, DEG2RAD(360));
			cairo_fill(tex->cr);
			break;
		default:
			set_color(vsi, tex, RED_RGB);
			cairo_rectangle(tex->cr, X(p.x - CTC_SZ / 2 * 0.8),
			    X(p.y - CTC_SZ / 2 * 0.8), X(CTC_SZ * 0.8),
			    X(CTC_SZ * 0.8));
			cairo_fill(tex->cr);
			break;
		}

		if (ctc->vs <= -LEVEL_VVEL_THRESH) {
			cairo_move_to(tex->cr, X(p.x + CTC_SZ * 0.8),
			    X(p.y - CTC_SZ / 2));
			cairo_rel_line_to(tex->cr, 0, X(CTC_SZ));
			cairo_move_to(tex->cr, X(p.x + CTC_SZ * 0.6),
			    X(p.y + CTC_SZ * 0.2));
			cairo_rel_line_to(tex->cr, X(CTC_SZ * 0.2),
			    X(CTC_SZ * 0.3));
			cairo_rel_line_to(tex->cr, X(CTC_SZ * 0.2),
			    X(-CTC_SZ * 0.3));
			cairo_stroke(tex->cr);
		} else if (ctc->vs >= LEVEL_VVEL_THRESH) {
			cairo_move_to(tex->cr, X(p.x + CTC_SZ * 0.8),
			    X(p.y - CTC_SZ / 2));
			cairo_rel_line_to(tex->cr, 0, X(CTC_SZ));
			cairo_move_to(tex->cr, X(p.x + CTC_SZ * 0.6),
			    X(p.y - CTC_SZ * 0.2));
			cairo_rel_line_to(tex->cr, X(CTC_SZ * 0.2),
			    X(-CTC_SZ * 0.3));
			cairo_rel_line_to(tex->cr, X(CTC_SZ * 0.2),
			    X(CTC_SZ * 0.3));
			cairo_stroke(tex->cr);
		}

		snprintf(alt_str, sizeof (alt_str), "%+03i",
		    (int)round((MET2FEET(ctc->ralt) / 100)));
		cairo_text_extents(tex->cr, alt_str, &te);
		if (MET2FEET(ctc->ralt) > -100) {
			cairo_move_to(tex->cr,
			    X(p.x) - te.width / 2 - te.x_bearing,
			    X(p.y - CTC_SZ) - te.height / 2 - te.y_bearing);
		} else {
			cairo_move_to(tex->cr,
			    X(p.x) - te.width / 2 - te.x_bearing,
			    X(p.y + CTC_SZ) - te.height / 2 - te.y_bearing);
		}
		cairo_show_text(tex->cr, alt_str);
	}
	mutex_exit(&ctc_lock);

	cairo_reset_clip(tex->cr);
}

static void
draw_band(vsi_t *vsi, vsi_tex_t *tex, double vs_lo, double vs_hi,
    double thickness)
{
	double a1 = find_vs_angle(vs_lo), a2 = find_vs_angle(vs_hi);

	UNUSED(vsi);

	cairo_arc(tex->cr, 0, 0, X(VSI_RING_RADIUS), DEG2RAD(a1),
	    DEG2RAD(a2));
	cairo_arc_negative(tex->cr, 0, 0, X(VSI_RING_RADIUS + thickness),
	    DEG2RAD(a2), DEG2RAD(a1));
	cairo_fill(tex->cr);

#if	VSI_STYLE == VSI_STYLE_HONEYWELL
	cairo_set_line_width(tex->cr, X(0.005));
	set_color(vsi, tex, 1, 1, 1);
	cairo_arc_negative(tex->cr, 0, 0, X(VSI_RING_RADIUS + thickness),
	    DEG2RAD(a2), DEG2RAD(a1));
	cairo_stroke(tex->cr);
#endif	/* VSI_STYLE == VSI_STYLE_HONEYWELL */
}

static void
draw_color_bands(vsi_t *vsi, vsi_tex_t *tex)
{
#if	VSI_STYLE == VSI_STYLE_ATR
#define	RED_BAND_THICKNESS 0.08
#define	GREEN_BAND_THICKNESS 0.04
#else	/* VSI_STYLE == VSI_STYLE_HONEYWELL */
#define	RED_BAND_THICKNESS 0.04
#define	GREEN_BAND_THICKNESS 0.05
#endif	/* VSI_STYLE == VSI_STYLE_HONEYWELL */

	vsi_state_t st;

	if (USEC2SEC(microclock() - vsi->start_time) < VSI_TCAS_DELAY)
		return;

	mutex_enter(&vsi->state_lock);
	st = vsi->state;
	mutex_exit(&vsi->state_lock);

	if (st.adv != ADV_STATE_RA)
		return;

	cairo_save(tex->cr);
	cairo_scale(tex->cr, -1, -1);

	if (st.min_green != st.max_green) {
		set_color(vsi, tex, 0, 1, 0);
		draw_band(vsi, tex, st.min_green, st.max_green,
		    GREEN_BAND_THICKNESS);
	}
	if (st.min_red_lo != st.max_red_lo) {
		set_color(vsi, tex, 1, 0, 0);
		draw_band(vsi, tex, st.min_red_lo, st.max_red_lo,
		    RED_BAND_THICKNESS);
	}
	if (st.min_red_hi != st.max_red_hi) {
		set_color(vsi, tex, 1, 0, 0);
		draw_band(vsi, tex, st.min_red_hi, st.max_red_hi,
		    RED_BAND_THICKNESS);
	}

	cairo_restore(tex->cr);
}

static void
draw_mode(vsi_t *vsi, vsi_tex_t *tex)
{
#if	VSI_STYLE == VSI_STYLE_ATR
#define	MODE_MSG_SZ	0.08
#define	MSG_X_OFF	0.02
	const char *msg;
	tcas_mode_t mode = xtcas_get_mode();
	cairo_text_extents_t te;

	if (USEC2SEC(microclock() - vsi->start_time) < VSI_TCAS_DELAY ||
	    !xpdr_functional)
		msg = "TCAS FAIL";
	else if (xtcas_test_is_in_prog())
		msg = "TEST";
	else if (mode == TCAS_MODE_STBY)
		msg = "TCAS OFF";
	else if (mode == TCAS_MODE_TAONLY)
		msg = "TA ONLY";
	else
		return;

	cairo_set_font_face(tex->cr, cr_font);
	cairo_set_font_size(tex->cr, round(X(MODE_MSG_SZ)));
	cairo_text_extents(tex->cr, msg, &te);

	set_color(vsi, tex, 1, 1, 1);
	cairo_rectangle(tex->cr, X(0.5 - MODE_MSG_SZ * 0.1 - MSG_X_OFF) -
	    te.width, -te.height / 2 - X(MODE_MSG_SZ * 0.1),
	    te.width + X(MODE_MSG_SZ * 0.2), te.height + X(MODE_MSG_SZ * 0.2));
	cairo_fill(tex->cr);

	set_color(vsi, tex, 0, 0, 0);
	cairo_move_to(tex->cr, X(0.5 - MSG_X_OFF) - te.width - te.x_bearing,
	    -te.height / 2 - te.y_bearing);
	cairo_show_text(tex->cr, msg);
#else	/* VSI_STYLE == VSI_STYLE_HONEYWELL */
#define	MODE_MSG_SZ	0.06
#define	MSG_X_OFF	-0.48
#define	MSG_Y_OFF	0.38
	const char *msgs[2] = { NULL, NULL };
	tcas_mode_t mode = xtcas_get_mode();

	if (USEC2SEC(microclock() - vsi->start_time) < VSI_TCAS_DELAY ||
	    !xpdr_functional) {
		set_color(vsi, tex, 1, 1, 0);
		msgs[0] = "NO TCAS";
	} else if (xtcas_test_is_in_prog()) {
		set_color(vsi, tex, 1, 1, 0);
		msgs[0] = "     TEST";
	} else if (mode == TCAS_MODE_STBY) {
		set_color(vsi, tex, CYAN_RGB);
		msgs[0] = "     TCAS";
		msgs[1] = "     STBY";
	} else if (mode == TCAS_MODE_TAONLY) {
		set_color(vsi, tex, CYAN_RGB);
		msgs[0] = "TA ONLY";
	} else {
		set_color(vsi, tex, CYAN_RGB);
		msgs[0] = "    TA/RA";
	}

	cairo_set_font_face(tex->cr, cr_font);
	cairo_set_font_size(tex->cr, round(X(MODE_MSG_SZ)));

	for (int i = 0; i < 2; i++) {
		cairo_text_extents_t te;

		if (msgs[i] == NULL)
			continue;
		cairo_text_extents(tex->cr, msgs[i], &te);
		cairo_move_to(tex->cr, X(MSG_X_OFF),
		    X(MSG_Y_OFF + i * MODE_MSG_SZ) -
		    te.height / 2 - te.y_bearing);
		cairo_show_text(tex->cr, msgs[i]);
	}

#endif	/* VSI_STYLE == VSI_STYLE_HONEYWELL */
}

static void
draw_scale(vsi_t *vsi, vsi_tex_t *tex)
{
#if	VSI_STYLE == VSI_STYLE_ATR
#define	BOX_X_OFF		0.315
#define	BOX_Y_OFF		-0.46
#define	BOX_X_SZ		.07
#define	BOX_Y_SZ		.11
#define	BOX_LINE_WIDTH		0.005
#define	SCALE_MSG_SZ		0.065
#define	SCALE_MSG_X_OFF		(BOX_X_OFF + BOX_X_SZ / 2)
#define	SCALE_MSG_Y_OFF		(BOX_Y_OFF + SCALE_MSG_SZ / 2)
#define	SCALE_UNIT_SZ		0.04
#define	SCALE_UNIT_X_OFF	(BOX_X_OFF + BOX_X_SZ / 2)
#define	SCALE_UNIT_Y_OFF	(BOX_Y_OFF + SCALE_MSG_SZ + SCALE_UNIT_SZ / 2)
	char msg[8];
	cairo_text_extents_t te;

	if (USEC2SEC(microclock() - vsi->start_time) < VSI_TCAS_DELAY)
		return;

	set_color(vsi, tex, 1, 1, 1);

	cairo_set_line_width(tex->cr, X(BOX_LINE_WIDTH));
	cairo_rectangle(tex->cr, X(BOX_X_OFF), X(BOX_Y_OFF), X(BOX_X_SZ),
	    X(BOX_Y_SZ));
	cairo_stroke(tex->cr);

	cairo_set_font_face(tex->cr, cr_font);

	cairo_set_font_size(tex->cr, round(X(SCALE_MSG_SZ)));
	snprintf(msg, sizeof (msg), "%d", get_scale(vsi));
	cairo_text_extents(tex->cr, msg, &te);
	cairo_move_to(tex->cr,
	    X(SCALE_MSG_X_OFF) - te.width / 2 - te.x_bearing,
	    X(SCALE_MSG_Y_OFF) - te.height / 2 - te.y_bearing);
	cairo_show_text(tex->cr, msg);

	cairo_set_font_size(tex->cr, round(X(SCALE_UNIT_SZ)));
	cairo_text_extents(tex->cr, "NM", &te);
	cairo_move_to(tex->cr,
	    X(SCALE_UNIT_X_OFF) - te.width / 2 - te.x_bearing,
	    X(SCALE_UNIT_Y_OFF) - te.height / 2 - te.y_bearing);
	cairo_show_text(tex->cr, "NM");
#else	/* VSI_STYLE == VSI_STYLE_HONEYWELL */
#define	SCALE_X_OFF		0.22
#define	SCALE_Y_OFF		-0.45
#define	SCALE_MSG_SZ		0.07
	char msg[8];
	cairo_text_extents_t te;

	set_color(vsi, tex, CYAN_RGB);
	cairo_set_font_face(tex->cr, cr_font);
	cairo_set_font_size(tex->cr, round(X(SCALE_MSG_SZ)));

	snprintf(msg, sizeof (msg), "RNG %d", get_scale(vsi));
	cairo_text_extents(tex->cr, msg, &te);

	cairo_move_to(tex->cr, X(SCALE_X_OFF),
	    X(SCALE_Y_OFF) - te.height / 2 - te.y_bearing);
	cairo_show_text(tex->cr, msg);
#endif	/* VSI_STYLE == VSI_STYLE_HONEYWELL */
}

static void
vsi_draw(vsi_t *vsi, vsi_tex_t *tex)
{
	uint64_t now = microclock();

	cairo_translate(tex->cr, tex->sz / 2, tex->sz / 2);

	if (!vsi->functional ||
	    USEC2SEC(now - vsi->start_time) < VSI_SCREEN_DELAY) {
		cairo_set_source_rgb(tex->cr, 0, 0, 0);
		cairo_rectangle(tex->cr, X(-0.5), X(-0.5), X(1), X(1));
		cairo_fill(tex->cr);
		goto out;
	}

	if (USEC2SEC(now - vsi->start_time) < VSI_SCREEN_WHITE_DELAY) {
		cairo_set_source_rgb(tex->cr, 1, 1, 1);
		cairo_rectangle(tex->cr, X(-0.5), X(-0.5), X(1), X(1));
		cairo_fill(tex->cr);
		goto out;
	}
	set_color(vsi, tex, 0, 0, 0);
	cairo_rectangle(tex->cr, X(-0.5), X(-0.5), X(1), X(1));
	cairo_fill(tex->cr);

	if (USEC2SEC(now - vsi->start_time) < VSI_RING_DELAY) {
		draw_ranges(vsi, tex);
		goto out;
	}

	draw_color_bands(vsi, tex);
	draw_ranges(vsi, tex);
	draw_own_acf_ring(vsi, tex);
	draw_needle(vsi, tex);
	draw_contacts(vsi, tex);
	draw_own_acf(vsi, tex);
	draw_mode(vsi, tex);
	draw_scale(vsi, tex);

out:
	cairo_identity_matrix(tex->cr);
}

static void
vsi_worker(void *userinfo)
{
	vsi_t *vsi = userinfo;

	mutex_enter(&vsi->lock);

	while (!vsi->shutdown) {
		unsigned sz = vsi->sz;
		vsi_tex_t *tex;

		if (sz == 0 || sz > MAX_SZ)
			goto end;

		/* always draw into the non-current texture */
		tex = &vsi->tex[!vsi->cur_tex];

		if (tex->sz != sz || tex->cr == NULL) {
			if (tex->cr != NULL) {
				cairo_destroy(tex->cr);
				cairo_surface_destroy(tex->surf);
			}
			tex->sz = sz;
			tex->surf = cairo_image_surface_create(
			    CAIRO_FORMAT_ARGB32, sz, sz);
			tex->cr = cairo_create(tex->surf);
		}

		if (vsi->functional) {
			if (vsi->start_time == 0)
				vsi->start_time = microclock();
		} else {
			vsi->start_time = 0;
		}

		vsi_draw(vsi, tex);
		tex->chg = B_TRUE;

		mutex_enter(&vsi->tex_lock);
		vsi->cur_tex = !vsi->cur_tex;
		mutex_exit(&vsi->tex_lock);
end:
		cv_timedwait(&vsi->cv, &vsi->lock, microclock() + DRAW_INTVAL);
	}

	mutex_enter(&vsi->tex_lock);
	for (int i = 0; i < 2; i++) {
		if (vsi->tex[i].cr != NULL) {
			cairo_destroy(vsi->tex[i].cr);
			cairo_surface_destroy(vsi->tex[i].surf);
			vsi->tex[i].cr = NULL;
			vsi->tex[i].surf = 0;
		}
	}
	mutex_exit(&vsi->tex_lock);

	mutex_exit(&vsi->lock);
}

static void
composite_vsi(vsi_t *vsi)
{
	vsi_tex_t *tex;

	mutex_enter(&vsi->tex_lock);

	if (vsi->cur_tex == -1) {
		mutex_exit(&vsi->tex_lock);
		return;
	}
	ASSERT3S(vsi->cur_tex, >=, 0);
	ASSERT3S(vsi->cur_tex, <, 2);

	XPLMSetGraphicsState(0, 1, 0, 0, 1, 0, 0);

	tex = &vsi->tex[vsi->cur_tex];
	ASSERT(tex->gl_tex != 0);
	glBindTexture(GL_TEXTURE_2D, tex->gl_tex);
	if (tex->chg) {
		cairo_surface_flush(tex->surf);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tex->sz, tex->sz, 0,
		    GL_BGRA, GL_UNSIGNED_BYTE,
		    cairo_image_surface_get_data(tex->surf));
		tex->chg = B_FALSE;
	}

	mutex_exit(&vsi->tex_lock);

	glBegin(GL_QUADS);
	glTexCoord2f(0, 1);
	glVertex2f(vsi->x, vsi->y);
	glTexCoord2f(0, 0);
	glVertex2f(vsi->x, vsi->y + vsi->sz);
	glTexCoord2f(1, 0);
	glVertex2f(vsi->x + vsi->sz, vsi->y + vsi->sz);
	glTexCoord2f(1, 1);
	glVertex2f(vsi->x + vsi->sz, vsi->y);
	glEnd();
}

static void
vsi_drs_update(vsi_t *vsi)
{
	bool_t failed = B_FALSE, powered = B_TRUE;

	if (strcmp(vsi->vs_dr.name, vsi->vs_dr_name) != 0 &&
	    !dr_find(&vsi->vs_dr, "%s", vsi->vs_dr_name)) {
		memset(&vsi->vs_dr, 0, sizeof (vsi->vs_dr));
		return;
	}
	if (strcmp(vsi->fail_dr.name, vsi->fail_dr_name) != 0 &&
	    !dr_find(&vsi->fail_dr, "%s", vsi->fail_dr_name)) {
		memset(&vsi->fail_dr, 0, sizeof (vsi->fail_dr));
	}
	if (strcmp(vsi->fail_dr.name, "") != 0)
		failed = (dr_geti(&vsi->fail_dr) == 6);

	if (vsi->custom_bus_name[0] != '\0') {
		/*
		 * If the configuration supplied a dataref name, try to
		 * look it up (delayed dataref lookup to avoid plugin
		 * loading issues).
		 */
		if (!vsi->custom_bus) {
			vsi->custom_bus = dr_find(&vsi->custom_bus_dr, "%s",
			    vsi->custom_bus_name);
		}
		if (vsi->custom_bus) {
			/* Custom bus lookup ok */
			powered = (dr_getf(&vsi->custom_bus_dr) >=
			    xtcas_min_volts());
		}
	} else if (vsi->busnr >= 0 && vsi->busnr < MAX_BUSNR) {
		double volts;

		VERIFY3S(dr_getvf(&bus_volts, &volts, vsi->busnr, 1), ==, 1);
		powered = (volts >= xtcas_min_volts());
	}
	vsi->functional = (!failed && powered);

	switch (vsi->vs_dr_fmt) {
	case VS_FMT_FPM:
		vsi->vs_value = dr_getf(&vsi->vs_dr);
		break;
	case VS_FMT_MPS:
		vsi->vs_value = MPS2FPM(dr_getf(&vsi->vs_dr));
		break;
	case VS_FMT_FPS:
		vsi->vs_value = dr_getf(&vsi->vs_dr) * 60;
		break;
	case VS_FMT_MPM:
		vsi->vs_value = MPS2FPM(dr_getf(&vsi->vs_dr) / 60);
		break;
	default:
		vsi->vs_dr_fmt = VS_FMT_FPM;
	}
}

static int
draw_vsis(XPLMDrawingPhase phase, int before, void *refcon)
{
	UNUSED(phase);
	UNUSED(before);
	UNUSED(refcon);

	xpdr_functional = (xtcas_is_powered() && !xtcas_is_failed());

	for (int i = 0; i < MAX_VSIS; i++) {
		vsi_t *vsi = &vsis[i];

		if (vsi->sz <= 0 || vsi->sz > MAX_SZ) {
			if (vsi->running) {
				char key[64];

				shutdown_vsi(i);
				vsi->start_time = 0;
				vsi->brt = BRT_DFL;
				vsi->scale_enum = VSI_SCALE_DFL;

				snprintf(key, sizeof (key),
				    "vsis/%d/brt", i);
				conf_get_i(xtcas_conf, key, (int *)&vsi->brt);
				snprintf(key, sizeof (key),
				    "vsis/%d/scale", i);
				conf_get_i(xtcas_conf, key, &vsi->scale_enum);
				vsi->scale_enum = MIN(vsi->scale_enum,
				    VSI_NUM_SCALES - 1);
			}
			continue;
		} else if (!vsi->running) {
			vsi->shutdown = B_FALSE;
			vsi->running = B_TRUE;
			VERIFY(thread_create(&vsi->thread, vsi_worker, vsi));
			continue;
		}
		vsi_drs_update(vsi);
		composite_vsi(&vsis[i]);
	}

	return (1);
}

bool_t
vsi_init(const char *plugindir)
{
	char *fontdir;
	FT_Error err;

	ASSERT(!inited);
	inited = B_TRUE;

	XPLMRegisterDrawCallback(draw_vsis, xplm_Phase_Gauges, 0, NULL);

	mutex_init(&ctc_lock);
	avl_create(&ctcs, ctc_compar, sizeof (ctc_t), offsetof (ctc_t, node));

	memset(vsis, 0, sizeof (vsis));
	for (int i = 0; i < MAX_VSIS; i++) {
		vsi_t *vsi = &vsis[i];
		const char *s;

		dr_create_i(&vsi->x_dr, (int *)&vsi->x, B_TRUE,
		    "xtcas/vsi/%d/x", i);
		dr_create_i(&vsi->y_dr, (int *)&vsi->y, B_TRUE,
		    "xtcas/vsi/%d/y", i);
		dr_create_i(&vsi->sz_dr, (int *)&vsi->sz, B_TRUE,
		    "xtcas/vsi/%d/sz", i);
		dr_create_i(&vsi->scale_enum_dr, (int *)&vsi->scale_enum,
		    B_TRUE, "xtcas/vsi/%d/scale", i);
		dr_create_i(&vsi->brt_dr, (int *)&vsi->brt, B_TRUE,
		    "xtcas/vsi/%d/brt", i);
		dr_create_b(&vsi->vs_dr_name_dr, vsi->vs_dr_name,
		    sizeof (vsi->vs_dr_name), B_TRUE,
		    "xtcas/vsi/%d/vs_src", i);
		dr_create_i(&vsi->vs_dr_fmt_dr, (int *)&vsi->vs_dr_fmt, B_TRUE,
		    "xtcas/vsi/%d/vs_src_fmt", i);
		dr_create_b(&vsi->fail_dr_name_dr, vsi->fail_dr_name,
		    sizeof (vsi->fail_dr_name), B_TRUE,
		    "xtcas/vsi/%d/fail_dr", i);
		dr_create_i(&vsi->busnr_dr, (int *)&vsi->busnr, B_TRUE,
		    "xtcas/vsi/%d/busnr", i);

		mutex_init(&vsi->tex_lock);
		vsi->cur_tex = -1;
		vsi->brt = BRT_DFL;
		vsi->scale_enum = VSI_SCALE_DFL;

		switch (i) {
		case 0:
		case 2:
			strlcpy(vsi->vs_dr_name,
			    "sim/cockpit2/gauges/indicators/vvi_fpm_pilot",
			    sizeof (vsi->vs_dr_name));
			strlcpy(vsi->fail_dr_name,
			    "sim/operation/failures/rel_ss_vvi",
			    sizeof (vsi->fail_dr_name));
			break;
		case 1:
		case 3:
			strlcpy(vsi->vs_dr_name,
			    "sim/cockpit2/gauges/indicators/vvi_fpm_copilot",
			    sizeof (vsi->vs_dr_name));
			strlcpy(vsi->fail_dr_name,
			    "sim/operation/failures/rel_cop_vvi",
			    sizeof (vsi->fail_dr_name));
			break;
		}

		conf_get_i_v(xtcas_conf, "vsi/%d/x", (int *)&vsi->x, i);
		conf_get_i_v(xtcas_conf, "vsi/%d/y", (int *)&vsi->y, i);
		conf_get_i_v(xtcas_conf, "vsi/%d/sz", (int *)&vsi->sz, i);
		vsi->sz = MIN(vsi->sz, MAX_SZ);
		conf_get_i_v(xtcas_conf, "vsi/%d/brt", (int *)&vsi->brt, i);
		conf_get_i_v(xtcas_conf, "vsi/%d/scale", &vsi->scale_enum, i);
		vsi->scale_enum = MIN(vsi->scale_enum, VSI_NUM_SCALES - 1);
		if (conf_get_str_v(xtcas_conf, "vsi/%d/vs_src", &s, i))
			strlcpy(vsi->vs_dr_name, s, sizeof (vsi->vs_dr_name));
		conf_get_i_v(xtcas_conf, "vsi/%d/vs_src_fmt",
		    (int *)&vsi->vs_dr_fmt, i);
		if (conf_get_str_v(xtcas_conf, "vsi/%d/fail_dr", &s, i)) {
			strlcpy(vsi->fail_dr_name, s,
			    sizeof (vsi->fail_dr_name));
		}

		/*
		 * Allows overriding the electrical bus number.
		 * If the user provided a dataref name instead of a bus
		 * 
		 */
		if (conf_get_str_v(xtcas_conf, "vsi/%d/busnr", &s, i) &&
		    strlen(s) > 3 && !isdigit(s[0])) {
			strlcpy(vsi->custom_bus_name, s,
			    sizeof (vsi->custom_bus_name));
		} else if (!conf_get_i_v(xtcas_conf, "vsi/%d/busnr",
		    (int *)&vsi->vs_dr_fmt, i) ||
		    vsi->busnr < 0 || vsi->busnr > MAX_BUSNR) {
			vsi->busnr = i;
		}

		for (int j = 0; j < 2; j++) {
			glGenTextures(1, &vsi->tex[j].gl_tex);
			glBindTexture(GL_TEXTURE_2D, vsi->tex[j].gl_tex);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
			    GL_LINEAR);
			glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
			    GL_LINEAR);
		}

		mutex_init(&vsi->lock);
		cv_init(&vsi->cv);
		mutex_init(&vsi->state_lock);
	}

	fdr_find(&bus_volts, "sim/cockpit2/electrical/bus_volts");

	if ((err = FT_Init_FreeType(&ft)) != 0) {
		logMsg("Error initializing FreeType library: %s",
		    ft_err2str(err));
		goto errout;
	}
	fontdir = mkpathname(plugindir, "data", "fonts", NULL);
	if (!try_load_font(fontdir, FONT_FILE, ft, &font, &cr_font)) {
		free(fontdir);
		goto errout;
	}
	free(fontdir);

	return (B_TRUE);
errout:
	vsi_fini();
	return (B_FALSE);
}

void
vsi_fini(void)
{
	ctc_t *ctc;
	void *cookie = NULL;

	if (!inited)
		return;

	XPLMUnregisterDrawCallback(draw_vsis, xplm_Phase_Gauges, 0, NULL);

	for (int i = 0; i < MAX_VSIS; i++) {
		dr_delete(&vsis[i].x_dr);
		dr_delete(&vsis[i].y_dr);
		dr_delete(&vsis[i].sz_dr);
		dr_delete(&vsis[i].scale_enum_dr);
		dr_delete(&vsis[i].vs_dr_name_dr);
		dr_delete(&vsis[i].vs_dr_fmt_dr);
		dr_delete(&vsis[i].fail_dr_name_dr);

		for (int j = 0; j < 2; j++) {
			if (vsis[i].tex[j].gl_tex != 0)
				glDeleteTextures(1, &vsis[i].tex[j].gl_tex);
		}

		shutdown_vsi(i);

		mutex_destroy(&vsis[i].tex_lock);

		mutex_destroy(&vsis[i].lock);
		cv_destroy(&vsis[i].cv);
		mutex_destroy(&vsis[i].state_lock);
	}

	while ((ctc = avl_destroy_nodes(&ctcs, &cookie)) != NULL)
		free(ctc);
	avl_destroy(&ctcs);
	mutex_destroy(&ctc_lock);

#define	FREE_FONT(f) \
	do { \
		if (cr_ ## f != NULL) { \
			cairo_font_face_destroy(cr_ ## f); \
			cr_ ## f = NULL; \
		} \
		if (f != NULL) { \
			FT_Done_Face(f); \
			(f) = NULL; \
		} \
	} while (0)
	FREE_FONT(font);
	if (ft != NULL) {
		FT_Done_FreeType(ft);
		ft = NULL;
	}
#undef	FREE_FONT

	inited = B_FALSE;
}

void
vsi_update_contact(void *handle, void *acf_id, double rbrg, double rdist,
    double ralt, double vs, double trk, double gs, tcas_threat_t level)
{
	ctc_t *ctc;
	const ctc_t srch = { .acf_id = acf_id };
	avl_index_t where;

	ASSERT(inited);
	UNUSED(handle);
	UNUSED(trk);
	UNUSED(gs);

	mutex_enter(&ctc_lock);

	ctc = avl_find(&ctcs, &srch, &where);
	if (ctc == NULL) {
		ctc = safe_calloc(1, sizeof (*ctc));
		ctc->acf_id = acf_id;
		avl_insert(&ctcs, ctc, where);
	}
	ctc->rbrg = rbrg;
	ctc->rdist = rdist;
	ctc->ralt = ralt;
	ctc->vs = vs;
	ctc->level = level;

	mutex_exit(&ctc_lock);
}

void vsi_delete_contact(void *handle, void *acf_id)
{
	ctc_t *ctc;
	const ctc_t srch = { .acf_id = acf_id };

	ASSERT(inited);
	UNUSED(handle);

	mutex_enter(&ctc_lock);
	ctc = avl_find(&ctcs, &srch, NULL);
	if (ctc != NULL) {
		avl_remove(&ctcs, ctc);
		free(ctc);
	}
	mutex_exit(&ctc_lock);
}

void
vsi_update_RA(void *handle, tcas_adv_t adv, tcas_msg_t msg,
    tcas_RA_type_t type, tcas_RA_sense_t sense, bool_t crossing,
    bool_t reversal, double min_sep_cpa, double min_green, double max_green,
    double min_red_lo, double max_red_lo, double min_red_hi, double max_red_hi)
{
	UNUSED(handle);
	UNUSED(msg);
	UNUSED(type);
	UNUSED(sense);
	UNUSED(crossing);
	UNUSED(reversal);
	UNUSED(min_sep_cpa);

	ASSERT(inited);

	for (int i = 0; i < MAX_VSIS; i++) {
		if (vsis[i].sz <= 0)
			continue;

		mutex_enter(&vsis[i].state_lock);
		vsis[i].state.adv = adv;
		vsis[i].state.min_green = min_green;
		vsis[i].state.max_green = max_green;
		vsis[i].state.min_red_lo = min_red_lo;
		vsis[i].state.max_red_lo = max_red_lo;
		vsis[i].state.min_red_hi = min_red_hi;
		vsis[i].state.max_red_hi = max_red_hi;
		mutex_exit(&vsis[i].state_lock);
	}
}

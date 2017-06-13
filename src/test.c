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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <fcntl.h>
#include <unistd.h>


#include <ncurses.h>

#include "assert.h"
#include "geom.h"
#include "list.h"
#include "log.h"
#include "snd_sys.h"
#include "time.h"
#include "thread.h"

#include "test.h"

#define	SIMSTEP		100000	/* microseconds */

typedef struct {
	uint64_t	s_t;	/* absolute start time in microclock() ticks */
	double		d_t;	/* delta-t (duration) */
	double		d_h;	/* rate-of-change in heading (deg/s) */
	double		d_vs;	/* rate-of-change in vertical speed (m/s^2) */
	double		d_gs;	/* rate-of-change in ground speed (m/s^2) */
	bool_t		automan;/* automatically maneuver in response to RAs */
	list_node_t	node;
} maneuver_t;

typedef struct {
	int		id;
	bool_t		inited;
	list_t		maneuvers;
	vect3_t		pos;
	double		trk;
	double		vs;
	double		gs;
	list_t		node;
	tcas_threat_t	threat_level;
} acf_t;

static fpp_t		fpp;
static acf_t		my_acf = { .inited = B_FALSE };
static list_t		other_acf;
static geo_pos2_t	refpt = {0, 0};
static double		refrot = 0;
static double		gnd_elev = 0;
static double		scaleh = 100;
static double		scalev = 200;
static double		RA_min_green = 0, RA_max_green = 0;
static double		RA_min_red_lo = 0, RA_max_red_lo = 0;
static double		RA_min_red_hi = 0, RA_max_red_hi = 0;
static double		RA_start = 0;
static int		RA_counter = 0;
static double		RA_tgt = 0;
static double		reaction_fact = 1.0;
static tcas_msg_t	RA_msg = -1u;
static tcas_adv_t	RA_adv = ADV_STATE_NONE;
static double		RA_min_sep_cpa = 0;
static double		d_h_min = INFINITY;
static double		d_v_min = INFINITY;

static double get_time(void *handle);
static void get_my_acf_pos(void *handle, geo_pos3_t *pos, double *alt_agl);
static void get_oth_acf_pos(void *handle, acf_pos_t **pos_p, size_t *num);
static void update_contact(void *handle, void *acf_id, geo_pos3_t pos,
    double trk, double vs, tcas_threat_t level);
static void delete_contact(void *handle, void *acf_id);
static void update_RA(void *handle, tcas_adv_t adv, tcas_msg_t msg,
    tcas_RA_type_t type, tcas_RA_sense_t sense, bool_t crossing,
    bool_t reversal, double min_sep_cpa, double min_green, double max_green,
    double min_red_lo, double max_red_lo, double min_red_hi, double max_red_hi);
static void update_RA_prediction(void *handle, tcas_msg_t msg,
    tcas_RA_type_t type, tcas_RA_sense_t sense, bool_t crossing,
    bool_t reversal, double min_sep_cpa);

static sim_intf_ops_t test_ops = {
	.get_time = get_time,
	.get_my_acf_pos = get_my_acf_pos,
	.get_oth_acf_pos = get_oth_acf_pos,
	.update_contact = update_contact,
	.delete_contact = delete_contact,
	.update_RA = update_RA,
	.update_RA_prediction = update_RA_prediction
};

static bool_t
sound_on(void)
{
	return (B_TRUE);
}

static bool_t
read_command_file(FILE *fp)
{
	char cmd[64];
	int acf_id = 0;
	acf_t *acf = NULL;

	while (!feof(fp)) {
		if (fscanf(fp, "%63s", cmd) != 1)
			break;

		if (cmd[0] == '#') {
			int c;
			do {
				c = fgetc(fp);
			} while (c != '\n' && c != EOF);
			continue;
		}

		if (strcmp(cmd, "reflat") == 0) {
			if (fscanf(fp, "%lf", &refpt.lat) != 1) {
				fprintf(stderr, "Command file syntax error: "
				    "expected a number following \"reflat\"\n");
				return (B_FALSE);
			}
		} else if (strcmp(cmd, "reflon") == 0) {
			if (fscanf(fp, "%lf", &refpt.lon) != 1) {
				fprintf(stderr, "Command file syntax error: "
				    "expected a number following \"reflon\"\n");
				return (B_FALSE);
			}
		} else if (strcmp(cmd, "refrot") == 0) {
			if (fscanf(fp, "%lf", &refrot) != 1) {
				fprintf(stderr, "Command file syntax error: "
				    "expected a number following \"refrot\"\n");
				return (B_FALSE);
			}
		} else if (strcmp(cmd, "gnd") == 0) {
			if (fscanf(fp, "%lf", &gnd_elev) != 1) {
				fprintf(stderr, "Command file syntax error: "
				    "expected a number following \"gnd\"\n");
				return (B_FALSE);
			}
		} else if (strcmp(cmd, "scaleh") == 0) {
			if (fscanf(fp, "%lf", &scaleh) != 1) {
				fprintf(stderr, "Command file syntax error: "
				    "expected a number following \"scaleh\"\n");
				return (B_FALSE);
			}
		} else if (strcmp(cmd, "scalev") == 0) {
			if (fscanf(fp, "%lf", &scalev) != 1) {
				fprintf(stderr, "Command file syntax error: "
				    "expected a number following \"scalev\"\n");
				return (B_FALSE);
			}
		} else if (strcmp(cmd, "acf") == 0) {
			if (acf == NULL) {
				acf = &my_acf;
			} else {
				acf = calloc(1, sizeof (*acf));
				list_insert_tail(&other_acf, acf);
			}
			acf->id = acf_id++;
			list_create(&acf->maneuvers, sizeof (maneuver_t),
			    offsetof(maneuver_t, node));
		} else if (strcmp(cmd, "pos") == 0) {
			if (acf == NULL) {
				fprintf(stderr, "Command file syntax error: "
				    "\"pos\" must be preceded by \"acf\"\n");
				return (B_FALSE);
			}
			if (fscanf(fp, "%lf %lf %lf", &acf->pos.x,
			    &acf->pos.y, &acf->pos.z) != 3) {
				fprintf(stderr, "Command file syntax error: "
				    "expected three numbers following "
				    "\"pos\"\n");
				return (B_FALSE);
			}
		} else if (strcmp(cmd, "trk") == 0) {
			if (acf == NULL) {
				fprintf(stderr, "Command file syntax error: "
				    "\"trk\" must be preceded by \"acf\"\n");
				return (B_FALSE);
			}
			if (fscanf(fp, "%lf", &acf->trk) != 1) {
				fprintf(stderr, "Command file syntax error: "
				    "expected a number following \"trk\"\n");
				return (B_FALSE);
			}
		} else if (strcmp(cmd, "vs") == 0) {
			if (acf == NULL) {
				fprintf(stderr, "Command file syntax error: "
				    "\"vs\" must be preceded by \"acf\"\n");
				return (B_FALSE);
			}
			if (fscanf(fp, "%lf", &acf->vs) != 1) {
				fprintf(stderr, "Command file syntax error: "
				    "expected a number following \"vs\"\n");
				return (B_FALSE);
			}
		} else if (strcmp(cmd, "gs") == 0) {
			if (acf == NULL) {
				fprintf(stderr, "Command file syntax error: "
				    "\"gs\" must be preceded by \"acf\"\n");
				return (B_FALSE);
			}
			if (fscanf(fp, "%lf", &acf->gs) != 1) {
				fprintf(stderr, "Command file syntax error: "
				    "expected a number following \"gs\"\n");
				return (B_FALSE);
			}
		} else if (strcmp(cmd, "man") == 0) {
			maneuver_t *man;
			if (acf == NULL) {
				fprintf(stderr, "Command file syntax error: "
				    "\"man\" must be preceded by \"acf\"\n");
				return (B_FALSE);
			}
			man = calloc(1, sizeof (*man));
			list_insert_tail(&acf->maneuvers, man);
			if (fscanf(fp, "%lf", &man->d_t) != 1) {
				fprintf(stderr, "Command file syntax error: "
				    "expected a number following \"man\"\n");
				return (B_FALSE);
			}
		} else if (strcmp(cmd, "turn") == 0) {
			maneuver_t *man;
			if (acf == NULL ||
			    (man = list_tail(&acf->maneuvers)) == NULL) {
				fprintf(stderr, "Command file syntax error: "
				    "\"turn\" must be preceded by \"man\"\n");
				return (B_FALSE);
			}
			if (fscanf(fp, "%lf", &man->d_h) != 1) {
				fprintf(stderr, "Command file syntax error: "
				    "expected a number following \"turn\"\n");
				return (B_FALSE);
			}
		} else if (strcmp(cmd, "clb") == 0) {
			maneuver_t *man;
			if (acf == NULL ||
			    (man = list_tail(&acf->maneuvers)) == NULL) {
				fprintf(stderr, "Command file syntax error: "
				    "\"clb\" must be preceded by \"man\"\n");
				return (B_FALSE);
			}
			if (fscanf(fp, "%lf", &man->d_vs) != 1) {
				fprintf(stderr, "Command file syntax error: "
				    "expected a number following \"clb\"\n");
				return (B_FALSE);
			}
		} else if (strcmp(cmd, "spd") == 0) {
			maneuver_t *man;
			if (acf == NULL ||
			    (man = list_tail(&acf->maneuvers)) == NULL) {
				fprintf(stderr, "Command file syntax error: "
				    "\"spd\" must be preceded by \"man\"\n");
				return (B_FALSE);
			}
			if (fscanf(fp, "%lf", &man->d_gs) != 1) {
				fprintf(stderr, "Command file syntax error: "
				    "expected a number following \"spd\"\n");
				return (B_FALSE);
			}
		} else if (strcmp(cmd, "auto") == 0) {
			maneuver_t *man = calloc(1, sizeof (*man));
			if (acf == NULL || acf->id != 0) {
				fprintf(stderr, "Command file syntax error: "
				    "\"auto\" must be preceded by \"acf\" "
				    "and may only be used for own "
				    "(non-intruder) aircraft.\n");
				return (B_FALSE);
			}
			man->automan = B_TRUE;
			list_insert_tail(&acf->maneuvers, man);
		} else {
			fprintf(stderr, "Command file syntax error: "
			    "unknown keyword \"%s\"\n", cmd);
			return (B_FALSE);
		}
	}

	return (B_TRUE);
}

static inline double
vs2tgt(double cur_vs, double targ_vs, double vs)
{
	return (cur_vs < targ_vs ? vs : -vs);
}

static void
step_acf(acf_t *acf, uint64_t now, uint64_t step)
{
	double step_s = USEC2SEC(step);
	double d_h = 0, d_vs = 0, d_gs = 0;
	vect2_t dir;

	for (;;) {
		maneuver_t *man = list_head(&acf->maneuvers);
		if (man == NULL)
			break;
		if (man->s_t == 0) {
			/* just started this maneuver */
			man->s_t = now;
		}
		if (man->s_t + SEC2USEC(man->d_t) <= now && !man->automan) {
			/* maneuver has ended */
			list_remove(&acf->maneuvers, man);
			free(man);
		} else if (man->automan) {
			double delay = (RA_counter <= 1 ? 5.0 : 2.5) *
			    reaction_fact;
			double d_vs_man = (RA_counter <= 1 ? 2.5 : 3.3);

			/* auto-maneuver our own aircraft */
			if (RA_counter == 0) {
				if (acf->vs != 0) {
					d_vs = vs2tgt(acf->vs, 0, d_vs_man);
					if (fabs(d_vs) > acf->vs)
						d_vs /= 2;
				}
				break;
			}
			if (USEC2SEC(now) < RA_start + delay)
				break;
			d_vs = vs2tgt(acf->vs, RA_tgt, d_vs_man);
			break;
		} else {
			/* maneuver is active */
			d_h = man->d_h;
			d_vs = man->d_vs;
			d_gs = man->d_gs;
			break;
		}
	}

	acf->trk += d_h * step_s;
	if (acf->trk < 0.0)
		acf->trk += 360.0;
	else if (acf->trk >= 360.0)
		acf->trk -= 360.0;
	acf->vs += d_vs * step_s;
	acf->gs += d_gs * step_s;
	dir = vect2_set_abs(hdg2dir(acf->trk), acf->gs * step_s);
	acf->pos.x += dir.x;
	acf->pos.y += dir.y;
	acf->pos.z += acf->vs * step_s;

	if (acf->threat_level >= RA_THREAT_PREV) {
		double d_h = vect2_abs(vect2_sub(VECT3_TO_VECT2(acf->pos),
		    VECT3_TO_VECT2(my_acf.pos)));
		d_h_min = MIN(d_h_min, d_h);
		if (d_h < 30)
			d_v_min = MIN(d_v_min, ABS(my_acf.pos.z - acf->pos.z));
	}
}

static void
draw_acf(vect3_t my_pos, double my_trk, acf_t *acf, int maxy, int maxx)
{
	int cy = maxy / 2, cx = maxx / 2;
	int x, y;
	vect2_t xy = VECT2(my_pos.x - acf->pos.x, my_pos.y - acf->pos.y);
	int z_rel = acf->pos.z - my_pos.z;
	char clb_symbol = ' ';
	char *acf_symbol;
	int color;
	double trk = normalize_hdg(acf->trk - my_trk);

	switch (acf->threat_level) {
	case OTH_THREAT:
		acf_symbol = "o";
		color = 0;
		break;
	case PROX_THREAT:
		acf_symbol = "o";
		color = 1;
		break;
	case TA_THREAT:
		acf_symbol = "O";
		color = 2;
		break;
	case RA_THREAT_PREV:
	case RA_THREAT_CORR:
		acf_symbol = "X";
		color = 3;
		break;
	default:
		acf_symbol = ".";
		color = 0;
		break;
	}

	if (acf->vs >= FPM2MPS(500))
		clb_symbol = '^';
	else if (acf->vs <= FPM2MPS(-500))
		clb_symbol = 'v';

	xy = vect2_rot(xy, -my_trk);
	x = cx - xy.x / scaleh;
	y = cy + xy.y / scalev;
	if (x - 1 < 0 || x + 6 >= maxx || y - 1 < 0 || y + 2 >= maxy)
		return;

	if (color > 0)
		attron(COLOR_PAIR(color));
	move(y, x);
	printw("%s %+04d%c", acf_symbol, z_rel / 10, clb_symbol);
	move(y + 1, x + 3);
	printw("%+03.0f", acf->vs);
	if (trk >= 337.5 || trk < 22.5) {
		move(y - 1, x);
		printw("|");
	} else if (trk >= 22.5 && trk < 67.5) {
		move(y - 1, x + 1);
		printw("/");
	} else if (trk >= 67.5 && trk < 112.5) {
		move(y, x + 1);
		printw("-");
	} else if (trk >= 112.5 && trk < 157.5) {
		move(y + 1, x + 1);
		printw("\\");
	} else if (trk >= 157.5 && trk < 202.5) {
		move(y + 1, x);
		printw("|");
	} else if (trk >= 202.5 && trk < 247.5) {
		move(y + 1, x - 1);
		printw("/");
	} else if (trk >= 247.5 && trk < 292.5) {
		move(y, x - 1);
		printw("-");
	} else {
		move(y - 1, x - 1);
		printw("\\");
	}
	if (color > 0)
		attroff(COLOR_PAIR(color));
}

static void
gui_display(WINDOW *win)
{
	int maxx = 80, maxy = 25;
	char *buf;
	tcas_mode_t mode;
	tcas_filter_t filter;

	getmaxyx(win, maxy, maxx);
	buf = malloc(maxx);
	memset(buf, ' ', maxx);
	for (int i = 0; i < maxy; i++) {
		move(i, 0);
		printw(buf);
	}
	free(buf);

	int kx = maxx / (1000 / scaleh), ky = maxy / (1000 / scalev),
	    scalestep = kx / 3;

	move(0, 0);

	/* draw the position dots 10km around the origin point */
	for (int i = (int)my_acf.pos.x / 1000 - kx / 2;
	    i <= (int)my_acf.pos.x / 1000 + kx / 2; i++) {
		for (int j = (int)my_acf.pos.y / 1000 - ky / 2;
		    j <= (int)my_acf.pos.y / 1000 + ky / 2; j++) {
			vect2_t v = VECT2(i * 1000 - my_acf.pos.x,
			    j * 1000 - my_acf.pos.y);
			int x, y;
			v = vect2_rot(v, -my_acf.trk);
			x = maxx / 2 + v.x / scaleh;
			y = maxy / 2 - v.y / scalev;
			if (x < 0 || x >= maxx || y < 0 || y >= maxy)
				continue;
			move(y, x);
			printw(".");
			if (i % scalestep == 0 && j % scalestep == 0) {
				int n = snprintf(NULL, 0, "[%d,%d]", i, j);
				if (x - n / 2 > 1 && x + n / 2 < maxx - 2 &&
				    y + 1 <= maxy - 1) {
					move(y + 1, x - n / 2);
					printw("[%d,%d]", i, j);
				}
			}
		}
	}

	move(maxy / 2, maxx / 2);
	printw("^");
	for (acf_t *acf = list_head(&other_acf); acf != NULL;
	    acf = list_next(&other_acf, acf))
		draw_acf(my_acf.pos, my_acf.trk, acf, maxy, maxx);

	/* draw the heading tape */
	move(0, maxx / 2 - 1);
	printw("%03.0f", my_acf.trk);
	move(1, maxx / 2);
	printw("V");
	for (int i = -2; i <= 2; i++) {
		int trk = ((int)my_acf.trk + i * 10) / 10 * 10,
		    trk_disp = trk;
		if (trk_disp < 0)
			trk_disp += 360;
		else if (trk_disp > 359)
			trk_disp -= 360;
		move(2, maxx / 2 - ((int)my_acf.trk - trk) - 1);
		printw("%03d", trk_disp);
	}

	/* draw the speed window */
	move(maxy / 2 - 1, 0);
	printw("+---+");
	move(maxy / 2, 0);
	printw("|%03.0f|", my_acf.gs);
	move(maxy / 2 + 1, 0);
	printw("+---+");

	/* draw the altitude window */
	move(maxy / 2 - 1, maxx - 11);
	printw("+-----+");
	move(maxy / 2, maxx - 11);
	printw("|%05.0f|", my_acf.pos.z);
	move(maxy / 2 + 1, maxx - 11);
	printw("+-----+");

	/* draw the VS tape */
	move(maxy / 2, maxx - 4);
	printw("===");
	if (my_acf.vs >= 1 || my_acf.vs <= -1) {
		if (my_acf.vs < 0) {
			for (int i = 1; i < -(int)my_acf.vs; i++) {
				move(maxy / 2 + i, maxx - 3);
				printw("|");
			}
		} else {
			for (int i = -1; i > -(int)my_acf.vs; i--) {
				move(maxy / 2 + i, maxx - 3);
				printw("|");
			}
		}
		move(maxy / 2 - (int)my_acf.vs, maxx - 4);
		printw("%+03.0f", my_acf.vs);
	}

	if (RA_min_red_lo != RA_max_red_lo) {
		attron(COLOR_PAIR(3));
		for (int i = -20; i < 20; i++) {
			if (RA_min_red_lo <= i && i <= RA_max_red_lo) {
				move(maxy / 2 - i, maxx - 1);
				printw("X");
			}
		}
		attroff(COLOR_PAIR(3));
	}
	if (RA_min_red_hi != RA_max_red_hi) {
		attron(COLOR_PAIR(3));
		for (int i = -20; i < 20; i++) {
			if (RA_min_red_hi <= i && i <= RA_max_red_hi) {
				move(maxy / 2 - i, maxx - 1);
				printw("X");
			}
		}
		attroff(COLOR_PAIR(3));
	}
	if (RA_min_green != RA_max_green) {
		attron(COLOR_PAIR(4));
		for (int i = -20; i < 20; i++) {
			if (RA_min_green <= i && i <= RA_max_green) {
				move(maxy / 2 - i, maxx - 1);
				printw("O");
			}
		}
		attroff(COLOR_PAIR(4));
	}

	move(0, 0);
	switch (RA_adv) {
	case ADV_STATE_TA:
		attron(COLOR_PAIR(5));
		break;
	case ADV_STATE_RA:
		attron(COLOR_PAIR(6));
		break;
	default:break;
	}
	printw("%s", xtcas_RA_msg2text(RA_msg));
	switch (RA_adv) {
	case ADV_STATE_TA:
		attroff(COLOR_PAIR(5));
		break;
	case ADV_STATE_RA:
		attroff(COLOR_PAIR(6));
		break;
	default:break;
	}

	move(1, 0);
	printw("d_h_min: %.0fm (%.0fft)", isfinite(d_h_min) ? d_h_min : -1.0,
	    isfinite(d_h_min) ? MET2FEET(d_h_min) : -1.0);
	move(2, 0);
	printw("d_v_min: %.0fm (%.0fft)", isfinite(d_v_min) ? d_v_min : -1.0,
	    isfinite(d_v_min) ? MET2FEET(d_v_min) : -1.0);

	move(3, 0);
	printw("CPA:     %.0fm (%.0fft)", RA_min_sep_cpa,
	    MET2FEET(RA_min_sep_cpa));

	mode = xtcas_get_mode();
	filter = xtcas_get_filter();
#define	PAINT_MODE(y, x, str, condition) \
	do { \
		if (condition) \
			attron(COLOR_PAIR(7)); \
		move(y, x); \
		printw(str); \
		if (condition) \
			attroff(COLOR_PAIR(7)); \
	} while (0)
	PAINT_MODE(0, maxx - 17, "ALL", filter == TCAS_FILTER_ALL);
	PAINT_MODE(0, maxx - 13, "THRT", filter == TCAS_FILTER_THRT);
	PAINT_MODE(0, maxx - 8, "ABV", filter == TCAS_FILTER_ABV);
	PAINT_MODE(0, maxx - 4, "BLW", filter == TCAS_FILTER_BLW);

	PAINT_MODE(1, maxx - 18, "STBY", mode == TCAS_MODE_STBY);
	PAINT_MODE(1, maxx - 13, "TAONLY", mode == TCAS_MODE_TAONLY);
	PAINT_MODE(1, maxx - 6, "TA/RA", mode == TCAS_MODE_TARA);

	refresh();
}

int
main(int argc, char **argv)
{
	FILE *cmdfile;
	int opt;
	int debug_strength = 0;
	char *snd_dir = "male1";
	uint64_t now;
	mutex_t mtx;
	condvar_t cv;
	int flags;
	WINDOW *win;
	bool_t gfx = B_TRUE;

	list_create(&other_acf, sizeof (acf_t), offsetof(acf_t, node));

	while ((opt = getopt(argc, argv, "r:gdD:s:")) != -1) {
		switch (opt) {
		case 'r':
			reaction_fact = atof(optarg);
			break;
		case 'g':
			gfx = B_FALSE;
			break;
		case 'd':
			debug_strength++;
			break;
		case 'D':
			debug_strength++;
			if (strcmp(optarg, "all") == 0) {
				xtcas_dbg.all += debug_strength;
			} else if (strcmp(optarg, "snd") == 0) {
				xtcas_dbg.snd += debug_strength;
			} else if (strcmp(optarg, "wav") == 0) {
				xtcas_dbg.wav += debug_strength;
			} else if (strcmp(optarg, "tcas") == 0) {
				xtcas_dbg.tcas += debug_strength;
			} else if (strcmp(optarg, "xplane") == 0) {
				xtcas_dbg.xplane += debug_strength;
			} else if (strcmp(optarg, "test") == 0) {
				xtcas_dbg.test += debug_strength;
			} else if (strcmp(optarg, "ra") == 0) {
				xtcas_dbg.ra += debug_strength;
			} else if (strcmp(optarg, "cpa") == 0) {
				xtcas_dbg.cpa += debug_strength;
			} else {
				fprintf(stderr, "Invalid argument to -D\n");
				return (1);
			}
			debug_strength = 0;
			break;
		case 's':
			snd_dir = optarg;
			break;
		default:
			return (1);
		}
	}

	argc -= optind;
	argv += optind;

	if (argc != 1) {
		fprintf(stderr, "Invalid options, expected exactly one "
		    "non-option argument (command file to read).\n");
		return (1);
	}

	cmdfile = fopen(argv[0], "r");
	if (cmdfile == NULL) {
		perror("Cannot open command file");
		return (1);
	}
	if (!read_command_file(cmdfile))
		return (1);

	fclose(cmdfile);

	if (!xtcas_snd_sys_init(snd_dir, sound_on))
		return (1);

	fpp = ortho_fpp_init(refpt, refrot, &wgs84, B_TRUE);

	now = microclock();
	mutex_init(&mtx);
	cv_init(&cv);

	flags = fcntl(STDIN_FILENO, F_GETFL);
	flags |= O_NONBLOCK;
	fcntl(STDIN_FILENO, F_SETFL, flags);

	if (gfx) {
		win = initscr();
		ASSERT(win != NULL);
		start_color();
		init_pair(1, COLOR_BLACK, COLOR_WHITE);
		init_pair(2, COLOR_YELLOW, COLOR_BLACK);
		init_pair(3, COLOR_RED, COLOR_BLACK);
		init_pair(4, COLOR_GREEN, COLOR_BLACK);
		init_pair(5, COLOR_BLACK, COLOR_YELLOW);
		init_pair(6, COLOR_BLACK, COLOR_RED);
		init_pair(7, COLOR_BLACK, COLOR_WHITE);
	}

	xtcas_init(&test_ops);
	xtcas_set_mode(TCAS_MODE_TARA);

	mutex_enter(&mtx);
	for (;;) {
		if (gfx) {
			int c = getch();

			switch (c) {
			case 'q':
			case 'Q':
				goto out;
			case 'u':
			case 'U':
				xtcas_set_filter(TCAS_FILTER_ALL);
				break;
			case 'i':
			case 'I':
				xtcas_set_filter(TCAS_FILTER_THRT);
				break;
			case 'o':
			case 'O':
				xtcas_set_filter(TCAS_FILTER_ABV);
				break;
			case 'p':
			case 'P':
				xtcas_set_filter(TCAS_FILTER_BLW);
				break;
			case 'j':
			case 'J':
				xtcas_set_mode(TCAS_MODE_STBY);
				break;
			case 'k':
			case 'K':
				xtcas_set_mode(TCAS_MODE_TAONLY);
				break;
			case 'l':
			case 'L':
				xtcas_set_mode(TCAS_MODE_TARA);
				break;
			}
		}

		step_acf(&my_acf, now, SIMSTEP);
		for (acf_t *acf = list_head(&other_acf); acf != NULL;
		    acf = list_next(&other_acf, acf))
			step_acf(acf, now, SIMSTEP);

		xtcas_run();

		if (gfx)
			gui_display(win);

		cv_timedwait(&cv, &mtx, now + SIMSTEP);
		now += SIMSTEP;
	}
out:
	mutex_exit(&mtx);

	endwin();

	mutex_destroy(&mtx);
	cv_destroy(&cv);

	xtcas_fini();

	for (maneuver_t *man = list_head(&my_acf.maneuvers); man != NULL;
	    man = list_head(&my_acf.maneuvers)) {
		list_remove(&my_acf.maneuvers, man);
		free(man);
	}
	for (acf_t *acf = list_head(&other_acf); acf != NULL;
	    acf = list_head(&other_acf)) {
		for (maneuver_t *man = list_head(&acf->maneuvers); man != NULL;
		    man = list_head(&acf->maneuvers)) {
			list_remove(&acf->maneuvers, man);
			free(man);
		}
		list_destroy(&acf->maneuvers);
		list_remove(&other_acf, acf);
		free(acf);
	}
	list_destroy(&other_acf);

	xtcas_snd_sys_fini();
	return (0);
}

static double
get_time(void *handle)
{
	UNUSED(handle);
	return (USEC2SEC(microclock()));
}

static void
get_my_acf_pos(void *handle, geo_pos3_t *pos, double *alt_agl)
{
	geo_pos2_t pos2 = fpp2geo(VECT3_TO_VECT2(my_acf.pos), &fpp);
	UNUSED(handle);
	*pos = GEO_POS3(pos2.lat, pos2.lon, my_acf.pos.z);
	*alt_agl = my_acf.pos.z - gnd_elev;
}

static void
get_oth_acf_pos(void *handle, acf_pos_t **pos_p, size_t *num)
{
	size_t i = 0, n = 0;
	UNUSED(handle);
	for (acf_t *acf = list_head(&other_acf); acf != NULL;
	    acf = list_next(&other_acf, acf))
		n++;
	*num = n;
	*pos_p = calloc(n, sizeof (acf_pos_t));
	for (acf_t *acf = list_head(&other_acf); acf != NULL;
	    acf = list_next(&other_acf, acf)) {
		acf_pos_t *pos = &(*pos_p)[i++];
		geo_pos2_t pos2 = fpp2geo(VECT3_TO_VECT2(acf->pos), &fpp);
		pos->acf_id = (void *)(uintptr_t)acf->id;
		pos->pos = GEO_POS3(pos2.lat, pos2.lon, acf->pos.z);
	}
}

static void
update_contact(void *handle, void *acf_id, geo_pos3_t pos, double trk,
    double vs, tcas_threat_t level)
{
	UNUSED(handle);
	UNUSED(pos);
	UNUSED(trk);
	UNUSED(vs);
	for (acf_t *acf = list_head(&other_acf); acf != NULL;
	    acf = list_next(&other_acf, acf)) {
		if (acf_id == (void *)(uintptr_t)acf->id) {
			acf->threat_level = level;
			return;
		}
	}
	VERIFY(0);
}

static void
delete_contact(void *handle, void *acf_id)
{
	UNUSED(handle);
	for (acf_t *acf = list_head(&other_acf); acf != NULL;
	    acf = list_next(&other_acf, acf)) {
		if (acf_id == (void *)(uintptr_t)acf->id) {
			acf->threat_level = -1u;
			return;
		}
	}
	VERIFY(0);
}

static void
update_RA(void *handle, tcas_adv_t adv, tcas_msg_t msg, tcas_RA_type_t type,
    tcas_RA_sense_t sense, bool_t crossing, bool_t reversal, double min_sep_cpa,
    double min_green, double max_green, double min_red_lo, double max_red_lo,
    double min_red_hi, double max_red_hi)
{
	UNUSED(handle);
	UNUSED(type);
	UNUSED(sense);
	UNUSED(crossing);
	UNUSED(reversal);
	UNUSED(min_sep_cpa);

	RA_min_green = min_green;
	RA_max_green = max_green;
	RA_min_red_lo = min_red_lo;
	RA_max_red_lo = max_red_lo;
	RA_min_red_hi = min_red_hi;
	RA_max_red_hi = max_red_hi;

	RA_msg = msg;
	RA_adv = adv;
	if (adv == ADV_STATE_NONE) {
		/* clear of conflict */
		RA_counter = 0;
		RA_tgt = 0;
	} else if (adv == ADV_STATE_RA) {
		if (fabs(my_acf.vs - RA_tgt) < 0.5 || RA_counter == 0 ||
		    reversal)
			RA_start = get_time(NULL);
		RA_counter++;
		if (min_green != max_green) {
			RA_tgt = (fabs(my_acf.vs - min_green) < fabs(
			    my_acf.vs - max_green)) ? min_green : max_green;
		} else if (min_red_lo != max_red_lo) {
			RA_tgt = (fabs(my_acf.vs - min_red_hi) < fabs(
			    my_acf.vs - max_red_lo)) ? min_red_lo : max_red_lo;
		} else {
			RA_tgt = (fabs(my_acf.vs - min_red_hi) < fabs(
			    my_acf.vs - max_red_hi)) ? min_red_hi : max_red_hi;
		}
	}
}

static void
update_RA_prediction(void *handle, tcas_msg_t msg, tcas_RA_type_t type,
    tcas_RA_sense_t sense, bool_t crossing, bool_t reversal, double min_sep_cpa)
{
	UNUSED(handle);
	UNUSED(msg);
	UNUSED(type);
	UNUSED(sense);
	UNUSED(crossing);
	UNUSED(reversal);
	RA_min_sep_cpa = min_sep_cpa;
}

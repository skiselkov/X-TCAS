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
static double		min_green = 0, max_green = 0, min_red = 0, max_red = 0;
static double		RA_start = 0;
static int		RA_counter = 0;
static double		reaction_fact = 1.0;

static double get_time(void *handle);
static void get_my_acf_pos(void *handle, geo_pos3_t *pos, double *alt_agl);
static void get_oth_acf_pos(void *handle, acf_pos_t **pos_p, size_t *num);
static void update_threat(void *handle, void *acf_id, tcas_threat_t level);
static void update_RA(void *, double, double, double, double);

static sim_intf_ops_t test_ops = {
	.get_time = get_time,
	.get_my_acf_pos = get_my_acf_pos,
	.get_oth_acf_pos = get_oth_acf_pos,
	.update_threat = update_threat,
	.update_RA = update_RA
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
			maneuver_t *man;
			if (acf == NULL || acf->id != 0 ||
			    (man = list_tail(&acf->maneuvers)) == NULL) {
				fprintf(stderr, "Command file syntax error: "
				    "\"auto\" must be preceded by \"man\" "
				    "and may only be used for own "
				    "(non-intruder) aircraft.\n");
				return (B_FALSE);
			}
			man->automan = B_TRUE;
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
						d_vs /= 4;
				}
				break;
			}
			if (USEC2SEC(now) < RA_start + delay)
				break;
			if (min_green != max_green) {
				if (fabs(acf->vs - min_green) <
				    fabs(acf->vs - max_green)) {
					d_vs = vs2tgt(acf->vs, min_green,
					    d_vs_man);
				} else {
					d_vs = vs2tgt(acf->vs, max_green,
					    d_vs_man);
				}
			} else {
				if (fabs(acf->vs - min_red) <
				    fabs(acf->vs - max_red)) {
					d_vs = vs2tgt(acf->vs, min_red,
					    d_vs_man);
				} else {
					d_vs = vs2tgt(acf->vs, max_red,
					    d_vs_man);
				}
			}
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
	default:
		acf_symbol = "X";
		color = 3;
		break;
	}

	if (acf->vs >= FPM2MPS(500))
		clb_symbol = '^';
	else if (acf->vs <= FPM2MPS(-500))
		clb_symbol = 'v';

	xy = vect2_rot(xy, -my_trk);
	x = cx - xy.x / scaleh;
	y = cy + xy.y / scalev;
	if (x < 0 || x + 5 >= maxx || y < 0 || y >= maxy)
		return;

	if (color > 0)
		attron(COLOR_PAIR(color));
	move(y, x);
	printw("%s %+04d%c", acf_symbol, z_rel / 10, clb_symbol);
	move(y + 1, x + 3);
	printw("%+03.0f", acf->vs);
	if (color > 0)
		attroff(COLOR_PAIR(color));
}

static void
gui_display(WINDOW *win)
{
	int maxx = 80, maxy = 25;
	char *buf;

	getmaxyx(win, maxy, maxx);
	buf = malloc(maxx);
	memset(buf, ' ', maxx);
	for (int i = 0; i < maxy; i++) {
		move(i, 0);
		printw(buf);
	}
	free(buf);

	/* draw the position dots 10km around the origin point */
	for (int i = (int)my_acf.pos.x / 1000 - 5;
	    i <= (int)my_acf.pos.x / 1000 + 5; i++) {
		for (int j = (int)my_acf.pos.y / 1000 - 5;
		    j <= (int)my_acf.pos.y / 1000 + 5; j++) {
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

	if (min_green != max_green) {
		attron(COLOR_PAIR(4));
		for (int i = -20; i < 20; i++) {
			if (min_green <= i && i <= max_green) {
				move(maxy / 2 - i, maxx - 1);
				printw("O");
			}
		}
		attroff(COLOR_PAIR(4));
	}

	if (min_red != max_red) {
		attron(COLOR_PAIR(3));
		for (int i = -20; i < 20; i++) {
			if (min_red <= i && i <= max_red) {
				move(maxy / 2 - i, maxx - 1);
				printw("X");
			}
		}
		attroff(COLOR_PAIR(3));
	}

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
	}

	xtcas_init(&test_ops);

	mutex_enter(&mtx);
	while (getch() != 'q') {
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
update_threat(void *handle, void *acf_id, tcas_threat_t level)
{
	UNUSED(handle);
	for (acf_t *acf = list_head(&other_acf); acf != NULL;
	    acf = list_next(&other_acf, acf)) {
		if (acf_id == (void *)(uintptr_t)acf->id) {
			acf->threat_level = level;
			return;
		}
	}
	abort();
}

static void
update_RA(void *handle, double min_g, double max_g, double min_r, double max_r)
{
	UNUSED(handle);
	min_green = min_g;
	max_green = max_g;
	min_red = min_r;
	max_red = max_r;

	if (min_green == max_green && min_red == max_red) {
		/* clear of conflict */
		RA_counter = 0;
	} else {
		RA_start = get_time(NULL);
		RA_counter++;
	}
}

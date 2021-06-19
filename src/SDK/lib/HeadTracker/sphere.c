// Copyright 2020, Felix Pfreundtner, All rights reserved.

#include "sphere.h"


void distance(float* pc1, float* pc2, float* dist){
	float dist_x;
	float dist_y;
	float dist_z;

	dist_x = pc2[A_x] - pc1[A_x];
	dist_y = pc2[A_y] - pc1[A_y];
	dist_z = pc2[A_z] - pc1[A_z];
	*dist = fabsf(sqrtf(dist_x * dist_x + dist_y * dist_y + dist_z * dist_z));
}

void distance_in_ms(float* pc1, float* pc2, float sos, float* dist_ms) {
	float dist;
	distance(pc1, pc2, &dist);
	m_to_sec(dist, sos, dist_ms);
	*dist_ms = *dist_ms * 1000.0f;
}

void get_sph_max_n(float** sph_x, int* az_n_max, int* el_n_max, float* max_amp, int Naz, int Nel) {
	long max_n_tmp = 0;
	float max_amp_tmp = 0.0f;
	int az_n;

	*az_n_max = 0;
	*el_n_max = 90;
	for (az_n = 0; az_n < Naz; az_n++) {
		ar_max(sph_x[az_n], (long)Nel, &max_n_tmp, &max_amp_tmp);
		if (max_amp_tmp > *max_amp) {
			*max_amp = max_amp_tmp;
			*az_n_max = az_n;
			*el_n_max = (int)max_n_tmp;
		}
	}
}


void set_sph_max_n(float** sph_x_read, float** sph_x_set, int Naz, int Nel, float value) {
	int az_n_max;
	int el_n_max;
	float max_amp;
	get_sph_max_n(sph_x_read, &az_n_max, &el_n_max, &max_amp, Naz, Nel);
	sph_x_set[az_n_max][el_n_max] = value;
}


void get_sph_max_az_el(float** sph_x, float* az_max, float* el_max, float* az_var, float* el_var, int ang_stepsize, int Naz, int Nel, int Nmax) {

	if (Nmax == 1) {
		// declare maximum variables
		int az_n_max;
		int el_n_max;
		float max_amp;

		// get maximum
		get_sph_max_n(sph_x, &az_n_max, &el_n_max, &max_amp, Naz, Nel);

		// process maxima
		az_n_max *= ang_stepsize;
		el_n_max *= ang_stepsize;
		*az_max = (float)(az_n_max * ang_stepsize);
		*el_max = (float)(el_n_max * ang_stepsize);
		*az_var = 0.0f;
		*el_var = 0.0f;

	}
	else {
		// declare maxima array with Nmax entries
		int* az_n_max = ar_declarei(Nmax);
		int* el_n_max = ar_declarei(Nmax);
		float* max_amp = ar_declare(Nmax);
		int n;

		// get maxima
		ar_Nmax2di(sph_x, Naz, Nel, az_n_max, el_n_max, max_amp, Nmax);

		// process maxima
		for (n = 0; n < Nmax; n++) {
			az_n_max[n] *= ang_stepsize;
			el_n_max[n] *= ang_stepsize;
		}
		sample_mean_var_circulari(az_n_max, Nmax, 360.0f, az_max, az_var);
		sample_meani(el_n_max, Nmax, false, el_max);
		sample_vari(el_n_max, Nmax, el_max, el_var);

		// free memory
		free(az_n_max);
		free(el_n_max);
		free(max_amp);
	}

}


void reset_sph(float** sph_x, long az_N, long el_N, float value) {
	for (int az = 0; az < az_N; az++) {
		for (int el = 0; el < el_N; el++) {
			sph_x[az][el] = value;
		}
	}
}

void get_map_tdn_ang(long cor_N, long cor_fs, long cor_Nlim, long Nang, long* map_tdn_ang, long* w_Ntdn_ang, int Ntd_ang, bool reset) {
	// initialize
	float td;
	float ang_td_float;
	long tdn_ang;
	long tdn_ang_p;

	if (reset) {
		ar_setl(w_Ntdn_ang, Nang, 0l);
	}

	long N_0delay = ((long)floorf(((float)cor_N) / 2.0f));
	long N_Startdelay = N_0delay - cor_Nlim;
	tdn_ang_p = -1000; // init with unreachable value
	if (N_Startdelay >= 0) {
		long N = 2 * cor_Nlim + 1;
		// for each time delay point
		for (long tdn = 0; tdn < N; tdn++) {
			// find spherical angle that maps time delay to sphere
			td = (float)(tdn - cor_Nlim) / (float)cor_fs;
			td_to_ang(td, (float)cor_Nlim / (float)cor_fs, &ang_td_float);
			tdn_ang = (long)roundf((ang_td_float + 90.0f));
			// if new angle: map start cross-correlation bin to angle
			if (tdn_ang != tdn_ang_p) {
				map_tdn_ang[tdn_ang] = tdn + N_Startdelay;
				tdn_ang_p = tdn_ang;
			}
			// map length cross-correlation bins to angle
			w_Ntdn_ang[tdn_ang] += 1;
		}
	}else {
        console_log(PRIO_LITTLEWARNING, "Frame length to small for set cross-correlation delay limit");
	}

	// show warning when cross-correlation is smaller than one bin per incidence angle (distribution normalization will not work properly, as w_Nsphn is to high as some azimuth bins are not ever used but counted in compensation, not problem for z-split, could be solved for x,y split if all points on sphere can be derived (than no normalization we be needed anymore))
	if (cor_Nlim * 2 + 1 < Ntd_ang) {
		console_log(PRIO_LITTLEWARNING, "cross-correlation to small for full spherical mapping, energy distribution compensation calculation will be wrong\n");
	}
	//write_txt_debug_long(w_Ntdn_ang, Nang, 0);
}

void add_td_energy_to_sph_select(float* cor, long cor_N, long cor_fs, long cor_Nlim, float** sph_x, float** sph_xtmp, long*** map_tdn_sph, long* map_tdn_ang, long* w_Ntdn_ang, long* w_Ntd_ang_sol, long** w_Nsphn, int Naz, int Nel, int tdn_ang_limit, int tdn_ang_max, bool orthogonal, bool cor_positive) {
	if (orthogonal) {
		add_td_energy_to_sph_orthogonal(cor, cor_N, cor_fs,cor_Nlim, sph_x, sph_xtmp, map_tdn_sph, map_tdn_ang, w_Ntdn_ang, w_Ntd_ang_sol, w_Nsphn, Naz, Nel, tdn_ang_limit, tdn_ang_max, cor_positive);
	}
	else {
		add_td_energy_to_sph_orthogonal(cor, cor_N, cor_fs,cor_Nlim, sph_x, sph_xtmp, map_tdn_sph, map_tdn_ang, w_Ntdn_ang, w_Ntd_ang_sol, w_Nsphn, Naz, Nel, tdn_ang_limit, tdn_ang_max, cor_positive);
		//add_td_energy_to_sph(cor, cor_N, cor_fs,cor_Nlim, sph_x, sph_xtmp, map_tdn_sph, map_tdn_ang, w_Ntdn_ang, w_Ntd_ang_sol, w_Nsphn, Naz, Nel, tdn_ang_limit, tdn_ang_max, orthogonal, cor_positive);
	}
}

void add_td_energy_to_sph(float *cor, long cor_N, long cor_fs, long cor_Nlim, float** sph_x, float** sph_xtmp, long*** map_tdn_sph, long* map_tdn_ang, long* w_Ntdn_ang, long* w_Ntd_ang_sol, long** w_Nsphn, int Naz, int Nel, int tdn_ang_limit, int tdn_ang_max, bool cor_positive) {
	UNUSED(sph_xtmp);

	// initialize
	long* ang_az;
	long* ang_el;
	int tdn_ang;
	int tdn_ang_start;
	int tdn_ang_end;
	bool tdn_ang_addition;
	float energy_tdn;
	long soln;
	long Npositive;
	long* Nsol;
	long* Nang;
	long* idx;
	float offset;
	long n;
	double timer_diff_sum = 0.0f;
	long timer_start;
	double timer_diff;
	energy_tdn = 0.0f;
	Npositive = 0;
	offset = 0.0001f; // set sphere to offset value where no valid data is pointing to (can not be set 0.0f and is needed to not iterate again over sphere)
	UNUSED(cor);
	UNUSED(cor_N);
	UNUSED(cor_fs);
	UNUSED(cor_Nlim);
	UNUSED(Nel);
	UNUSED(Naz);
	UNUSED(w_Nsphn);

	// calculate energy added to point of sphere for selected angular positions corresponding to time delay of point
	//tdn_ang_limit = 90;
	if (tdn_ang_limit != 90) {
		tdn_ang_max = tdn_ang_max + 90;
		tdn_ang_start = tdn_ang_max - tdn_ang_limit;
		tdn_ang_end = tdn_ang_max + tdn_ang_limit;
		tdn_ang_start = tdn_ang_start < 0 ? 0 : tdn_ang_start;
		tdn_ang_end = tdn_ang_end > 180 ? 180 : tdn_ang_end;
		tdn_ang_addition = true;
	}
	else {
		tdn_ang_start = 0;
		tdn_ang_end = 180;
		tdn_ang_addition = false;
	}
	// for each selected angle bin of cross-correlation (forms a separate ring of ambiguous solution)
	for (tdn_ang = tdn_ang_start; tdn_ang <= tdn_ang_end; tdn_ang++) {
		Nang = &w_Ntdn_ang[tdn_ang]; // number of correlation bins mapped to one integer angle
		idx = &map_tdn_ang[tdn_ang]; // index of correlation bin starting the mapping to one integer angle
		// calculate energy metric and normalize because of multiple correlation bins mapped to one angle bin
		if (cor_positive) {
			// only positive values are averaged with number of positive values (in orthogonal version with positive and negative number)
			if (*Nang == 0) {
				continue;
			}
			if (*Nang == 1) {
				energy_tdn = cor[*idx] >= 0.0f ? cor[*idx] : offset;
			}
			if (*Nang > 1) {
				// inline: sample_mean_positive()
				energy_tdn = 0.0f;
				for (n = 0; n < *Nang; n++) {
					if (cor[*idx + n] > 0.0f) {
						energy_tdn += cor[*idx + n];
						Npositive += 1;
					}
				}
				if (Npositive > 0) {
					energy_tdn /= Npositive;
				}
				energy_tdn = energy_tdn == 0.0f ? offset : energy_tdn;
			}
		}
		else {
			//  positive and negative values are averaged with negative numbers sign flipped
			if (*Nang == 0) {
				continue;
			}
			if (*Nang == 1) {
				energy_tdn = cor[*idx] >= 0.0f ? cor[*idx] : -cor[*idx];
			}
			if (*Nang > 1) {
				// inline: sample_mean_flipped()
				energy_tdn = 0.0f;
				for (n = 0; n < *Nang; n++) {
					if (cor[*idx + n] > 0.0f) {
						energy_tdn += cor[*idx + n];
					}
					else {
						energy_tdn -= cor[*idx + n];
					}
				}
				energy_tdn /= (float)*Nang;
				energy_tdn = energy_tdn == 0.0f ? offset : energy_tdn;

			}
		}

		// add energy of time delay to a ring of all ambiguous integer solution bins attached to correlation angle
		Nsol = &w_Ntd_ang_sol[tdn_ang];
		for (soln = 0; soln < *Nsol; soln++) {
			ang_az = &map_tdn_sph[tdn_ang][soln][A_sph_az];
			ang_el = &map_tdn_sph[tdn_ang][soln][A_sph_el];
			//timer_start = clock_tick();
			if (sph_x[*ang_az][*ang_el] == 0.0f || tdn_ang_addition) {
				sph_x[*ang_az][*ang_el] += energy_tdn; // /w_Ntd_ang_sol[tdn_ang]
			}
			else {
				sph_x[*ang_az][*ang_el] *= energy_tdn; // /w_Ntd_ang_sol[tdn_ang]
			}
			//timer_diff = clock_stop(timer_start);
			//timer_diff_sum += timer_diff;

		}
	}
	// do not normalize over w_Nsphn as each time delay angle has unique mapping and same mapping between time delay angles does not overlap or is wanted as double energy then there (different to orthogonal version)
}

void add_td_energy_to_sph_orthogonal(float *cor, long cor_N, long cor_fs, long cor_Nlim, float** sph_x, float** sph_xtmp, long*** map_tdn_sph, long* map_tdn_ang, long* w_Ntdn_ang, long* w_Ntd_ang_sol, long** w_Nsphn, int Naz, int Nel, int tdn_ang_limit, int tdn_ang_max, bool cor_positive) {
	UNUSED(tdn_ang_max);
	UNUSED(tdn_ang_limit);
	UNUSED(map_tdn_ang);

	// initialize
	long ang_az;
	long ang_el;
	float td;
	float ang_td_float;
	long tdn_ang;
	float energy_tdn;
	float energy_sphn;

	bool normalize_distribution = true;

	// reset energy in temporary array
	reset_sph(sph_xtmp, Naz, Nel, 0.0f);

	// calculate energy added to point of sphere at certain position corresponding to time delay of point
	long N_0delay = ((long)floorf(((float)cor_N) / 2.0f));
	long N_Startdelay = N_0delay - cor_Nlim;
	if (N_Startdelay >= 0) {
		long N = 2 * cor_Nlim + 1;
		// for each time delay point
		for (long tdn = 0; tdn < N; tdn++) {
			// find spherical angle that maps time delay to sphere
			td = (float)(tdn - cor_Nlim) / (float)cor_fs;
			td_to_ang(td, (float)cor_Nlim / (float)cor_fs, &ang_td_float);
			tdn_ang = (long)roundf((ang_td_float + 90.0f));

			//if (tdn == cor_Nlim) {
				//int a = 0;
			//}

			// calculate energy metric
			energy_tdn = cor[tdn + N_Startdelay]; // cor[tdn + N_Startdelay];
			//energy = 0.0f;
			//if (tdn_ang == 90) {
				//energy = 500.0f; // cor[tdn + N_Startdelay];
			//}	
			//if (tdn_ang == 170) {
				//energy = 1.0f; // cor[tdn + N_Startdelay];
			//}	
			if (cor_positive) {
				if (energy_tdn > 0.0f) {
					energy_tdn = fabsf(energy_tdn);
				}
				else {
					energy_tdn = 0.0f;
				}
			}
			else {
				energy_tdn = fabsf(energy_tdn);
			}

			// add energy metric to all possible solutions (bins) on sphere
			if (energy_tdn != 0.0f) {
				energy_sphn = energy_tdn;

				if (normalize_distribution) {
					// remove rate of writing to this spherical angle because of multiple correlation bins mapped to same angle
					if (w_Ntdn_ang[tdn_ang] != 1) {
						energy_sphn = energy_sphn / (float)w_Ntdn_ang[tdn_ang];
					}
				}
				for (int soln = 0; soln < w_Ntd_ang_sol[tdn_ang]; soln++) {
					ang_az = map_tdn_sph[tdn_ang][soln][A_sph_az];
					ang_el = map_tdn_sph[tdn_ang][soln][A_sph_el];
					//if (ang_az == 26 && ang_el == 95) {
						//int ba = 0;
					//}
					sph_xtmp[ang_az][ang_el] += energy_sphn;
				}
			}
		}
		//long timer_start = clock_tick();
		for (ang_az = 0; ang_az < 360; ang_az++) {
			for (ang_el = 0; ang_el <= 180; ang_el++) {
				if (normalize_distribution) {
					// remove rate of writing to this spherical angle because of limited stepsize of spherical coordinate system
					if (w_Nsphn[ang_az][ang_el] != 1 && w_Nsphn[ang_az][ang_el] != 0) { // more efficient would be to only divide if (w_Nsphn[ang_az][ang_el] != 360), but 360 (maximal amount of azimuth mappings) stays not constant over microphone setups
						sph_xtmp[ang_az][ang_el] = sph_xtmp[ang_az][ang_el] / (float)w_Nsphn[ang_az][ang_el];
					}
				}
				sph_x[ang_az][ang_el] = sph_x[ang_az][ang_el] * sph_xtmp[ang_az][ang_el];
			}
		}
		//double timer_diff = clock_stop(timer_start);
	}
	else {
        console_log(PRIO_LITTLEWARNING, "Frame length to small for set cross-correlation delay limit");
	}

}

void map_td_to_sph(float* mic_loc1, float *mic_norm1, float* mic_loc2, float* mic_norm2, float radius, int ang_stepsize, int Naz, int Nel, long*** map_tdn_sph, long* w_Ntd_ang_sol, long** w_Nsphn) {
	// mathemtical derivation of formula for mapping:
	// source: https://math.stackexchange.com/questions/3428731/finding-the-spherical-coordinates-for-the-edge-obtained-by-cutting-a-sphere-with#3428731

	// initialize
	float mic_dist_x;
	float mic_dist_y;
	float mic_dist_z;
	float mic_norm_x;
	float mic_norm_y;
	float mic_norm_z;
	float mic_loc_az;
	float mic_loc_ev;
	float mic_norm_az;
	float mic_norm_ev;
	float mic_mic_az;
	float mic_mic_ev;
	float ang_el;
	float ang_az;
	long bin_az;
	long bin_az_sol;
	long bin_el;
	long bin_el_sol;
	long bin_rd_micsetup;
	long bin_az_shift;
	long bin_el_flip;
	long ang_td;
	int tdn_ang;
	int soln;
	int mic_setup_type;
	float mic_setup_ang_az_unfocused; // in which azimuth direction does microphone most left cross-correlation (-90° angle) bin focuses to in horizontal head coordinate system 
	float mic_setup_ang_el_unfocused; // is microphone focusing nowhere (1.0), on ceiling (1.0) or ground (-1.0)
	float *mic_setup_ang = ar_zeros(sph_Ndim);

	// calculate microphone orientation
	mic_dist_x = mic_loc2[A_x] - mic_loc1[A_x];
	mic_dist_y = mic_loc2[A_y] - mic_loc1[A_y];
	mic_dist_z = mic_loc2[A_z] - mic_loc1[A_z];
	mic_norm_x = mic_norm1[A_x] + mic_norm2[A_x];
	mic_norm_y = mic_norm1[A_y] + mic_norm2[A_y];
	mic_norm_z = mic_norm1[A_z] + mic_norm2[A_z];

	mic_loc_az = atan2f(fabsf(mic_dist_y), fabsf(mic_dist_x)); // either 0 (to the front or back) or pi/2 (to the side)
	mic_norm_az = atan2f(mic_norm_y, mic_norm_x); // either 0 (to the right) or pi (to the left) or 0 (unfocused, both microphone opposite normals)
	mic_mic_az = mic_loc_az + mic_norm_az; // azimuth direction in radians where microphone focuses to

	mic_loc_ev = atan2f(fabsf(mic_dist_z), 0); // either 0 (in horizontal direction) or pi/2 (in elevational direction)
	mic_norm_ev = atan2f(mic_norm_z, 0); // either pi/2 (to the top) or -pi/2 (to the bottom) or 0 (unfocused, both microphone opposite normals)
	mic_mic_ev = mic_norm_ev; // elevational direction in radians where microphone focuses to
	
	// set default microphone setup values
	bin_az_shift = 0;
	mic_setup_ang_az_unfocused = 0.0f;
	mic_setup_ang_el_unfocused = 1.0f;
	mic_setup_ang[A_sph_rd] = 1.0f;

	// determine microphone setup (working now for l-r, l2-r2, ll2, rr2)
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	// microphone splits left-right direction (x)
	if (fabsf(mic_dist_x) > 0.0f) {
		mic_setup_ang_az_unfocused = 90.0f; 
		mic_setup_ang_el_unfocused = 1.0f; // 1.0 means no change, -1.0 means flipped elevational coordinate system
		if ((mic_norm_x == 0.0f && mic_norm_y == 0.0f && mic_norm_z == 0.0f) ) {
			mic_setup_type = A_xno;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused; 
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused;
			mic_setup_ang[A_sph_rd] = 0.0f;
		}
		if (mic_norm_y < 0.0f && mic_norm_x == 0.0f && mic_norm_z == 0.0f) {
			mic_setup_type = A_xb;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused + 180.0f; // focus direction of delay microphone setup in azimuth plane
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused;  // focus direction of delay microphone setup in elevation plane
		}
		if (mic_norm_y > 0.0f && mic_norm_x == 0.0f && mic_norm_z == 0.0f) {
			mic_setup_type = A_xf;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused;
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused;
		}
		if (mic_norm_z < 0.0f && mic_norm_x == 0.0f && mic_norm_y == 0.0f) {
			mic_setup_type = A_xg;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused;
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused * -1.0f;
		}
		if (mic_norm_z > 0.0f && mic_norm_x == 0.0f && mic_norm_y == 0.0f) {
			mic_setup_type = A_xc;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused;
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused;
		}
		if (mic_dist_x > 0.0f) {
			modulusf(mic_setup_ang[A_sph_az] + 180.0f, 360, &mic_setup_ang[A_sph_az]);
		}
	}

	// microphone splits front-back direction (y)
	if (fabsf(mic_dist_y) > 0.0f) {
		mic_setup_ang_az_unfocused = 0.0f;
		mic_setup_ang_el_unfocused = 1.0f;
		if ((mic_norm_x == 0.0f && mic_norm_y == 0.0f && mic_norm_z == 0.0f) ) {
			mic_setup_type = A_yno;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused;
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused;
			mic_setup_ang[A_sph_rd] = 0.0f;
		}
		if (mic_norm_x < 0.0f && mic_norm_y == 0.0f && mic_norm_z == 0.0f) {
			mic_setup_type = A_yl;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused + 180.0f;
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused;
		}
		if (mic_norm_x > 0.0f && mic_norm_y == 0.0f && mic_norm_z == 0.0f) {
			mic_setup_type = A_yr;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused;
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused;
		}
		if (mic_norm_z < 0.0f && mic_norm_x == 0.0f && mic_norm_y == 0.0f) {
			mic_setup_type = A_yg;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused;
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused * -1.0f;
		}
		if (mic_norm_z > 0.0f && mic_norm_x == 0.0f && mic_norm_y == 0.0f) {
			mic_setup_type = A_yc;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused;
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused;
		}
		if (mic_dist_y > 0.0f) {
			modulusf(mic_setup_ang[A_sph_az] + 180.0f, 360, &mic_setup_ang[A_sph_az]);
		}
	}

	// microphone splits ceil-ground direction (z)
	if (fabsf(mic_dist_z) > 0.0f) {
		mic_setup_ang_az_unfocused = 0.0f;
		mic_setup_ang_el_unfocused = 1.0f;
		if ((mic_norm_x == 0.0f && mic_norm_y == 0.0f && mic_norm_z == 0.0f) ) {
			mic_setup_type = A_xno;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused;
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused;
			mic_setup_ang[A_sph_rd] = 0.0f;
		}
		if (mic_norm_x < 0.0f && mic_norm_y == 0.0f && mic_norm_z == 0.0f) {
			mic_setup_type = A_zl;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused + 270.0f;
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused;
		}
		if (mic_norm_x > 0.0f && mic_norm_y == 0.0f && mic_norm_z == 0.0f) {
			mic_setup_type = A_zr;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused + 90.0f;
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused;
		}
		if (mic_norm_y < 0.0f && mic_norm_x == 0.0f && mic_norm_y == 0.0f) {
			mic_setup_type = A_zb;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused + 180.0f;
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused;
		}
		if (mic_norm_y > 0.0f && mic_norm_x == 0.0f && mic_norm_y == 0.0f) {
			mic_setup_type = A_zf;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused;
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused;
		}
		if (mic_dist_z > 0.0f) {
			mic_setup_ang[A_sph_az] = mic_setup_ang[A_sph_el] * -1.0f;
		}
	}

	// calculate standard mapping between time delay and spherical angle
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	tdn_ang = 0;
	for (ang_td = -90; ang_td <= 90; ang_td += ang_stepsize) {
		//wave_loc = cosf((float)ang_td / 180.0f * PI) * radius;
		//wave_loc = - wave_loc;
		bin_el = -ang_td + 90l;
		soln = 0;
		//bin_N = 360 / ang_stepsize;
		//w_Ntd_ang_sol[tdn_ang] = bin_N;
		for (bin_az = 0; bin_az < 360; bin_az += ang_stepsize) {

			// transform relative azimuth and elevation position to absolute azimuth and elevation position for given microphone setup
			bin_az_shift = 270;
			bin_el_flip = 1;
			bin_az_sol = modulus(bin_az + bin_az_shift, 360l);
			bin_el_sol = bin_el * bin_el_flip;
			bin_rd_micsetup = (long)radius;

			// assign spherical mapping between time delay and point on sphere
			map_tdn_sph[tdn_ang][soln][A_sph_az] = bin_az_sol;
			map_tdn_sph[tdn_ang][soln][A_sph_el] = bin_el_sol;
			map_tdn_sph[tdn_ang][soln][A_sph_rd] = (long)radius;
			//if (bin_az_sol == 270 && bin_el_sol == 180) {
				//int agsdg = 0;
			//}
			// add mapping to rate of occurance of those mapping
			//w_Nsphn[bin_az_sol][bin_el_sol] += 1l;

			// increment to next ambigous spherical bin at the same time delay
			soln += 1;
		}
		// increment to next time delay
		tdn_ang += 1;
	}

	// rotate standard mapping between time delay and spherical angle to fit to current microphone setup
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	float* norm_corsetup;
	float* norm_standardsph;
	float corsetup_dist;
	float** R;
	float* xyz;
	float* xyz_new;
	float rd;
	float ang_az_standard;
	float ang_el_standard;
	long soln_standard;

	// get rotation angles from microphone normal
	norm_corsetup = ar_declare(A_Ndimxyz);
	norm_standardsph = ar_declare(A_Ndimxyz);
	R = ar_declare2d(A_Ndimxyz,A_Ndimxyz);
	corsetup_dist = sqrtf(mic_dist_x * mic_dist_x + mic_dist_y * mic_dist_y + mic_dist_z * mic_dist_z);
	norm_corsetup[A_x] = mic_dist_x / corsetup_dist;
	norm_corsetup[A_y] = mic_dist_y / corsetup_dist;
	norm_corsetup[A_z] = mic_dist_z / corsetup_dist;
	norm_standardsph[A_x] = 0.0f;
	norm_standardsph[A_y] = 0.0f;
	norm_standardsph[A_z] = 1.0f;
	rotation_between_vecs_to_mat3(R, norm_standardsph, norm_corsetup);
	//write_txt_debug2d(R, A_Ndimxyz, A_Ndimxyz, 0);


	// set standard rotation matrices

	//float** R_x;
	//float** R_y;
	//float** R_z;
	//float** R;
	//float R_tmp;
	//R_x = ar_zeros2d(A_Ndimxyz, A_Ndimxyz);
	//R_x[0][0] = 1.0f;
	//R_x[1][1] = cosf(rot_x);
	//R_x[1][2] = sinf(rot_x);
	//R_x[2][1] = -sinf(rot_x);
	//R_x[2][2] = cosf(rot_x);

	//R_y = ar_zeros2d(A_Ndimxyz, A_Ndimxyz);
	//R_x[1][1] = 1.0f;
	//R_x[0][0] = cosf(rot_y);
	//R_x[2][0] = sinf(rot_y);
	//R_x[0][2] = -sinf(rot_y);
	//R_x[2][2] = cosf(rot_y);

	//R_z = ar_zeros2d(A_Ndimxyz, A_Ndimxyz);
	//R_x[2][2] = 1.0f;
	//R_x[0][0] = cosf(rot_z);
	//R_x[0][1] = sinf(rot_z);
	//R_x[1][0] = -sinf(rot_z);
	//R_x[1][1] = cosf(rot_z);

	//R = ar_declare2d(A_Ndimxyz, A_Ndimxyz);
	//R_tmp = ar_declare2d(A_Ndimxyz, A_Ndimxyz);
	//ar_m2dM3(R_y, R_z, R_tmp);
	//ar_m2dM3(R_x, R_tmp, R);

	// allocate temporary xyz coordinates
	xyz = ar_declare(A_Ndimxyz);
	xyz_new = ar_declare(A_Ndimxyz);


	// rotate spherical bins assigend to each time delay
	tdn_ang = 0;
	long** w_Nsphn_angtdn = ar_zeros2dl(Naz, Nel);
	for (ang_td = -90; ang_td <= 90; ang_td += ang_stepsize) {
		soln = 0;// bin of transformed standard solution (not always ambigious, without rotation -> w_Ntd_ang_sol = 360, w_Nsphn[bin_az_sol][bin_el_sol] = 1 / ang_stepsize)
		soln_standard = 0; // bin of standard solution (always ambigious -> w_Ntd_ang_sol = 360, w_Nsphn[bin_az_sol][bin_el_sol] = 1 / ang_stepsize)
		//long bin_az_sol_p = -1000; // reset with value cannot be reached
		//long bin_el_sol_p = -1000; // reset with value cannot be reached
		ar_set2dl(w_Nsphn_angtdn, Naz, Nel, 0); // reset as W_Nsphn is only used as temporary variable
		for (bin_az = 0; bin_az < 360; bin_az += ang_stepsize) {
			// get previously calculated spherical angle at bin
			ang_az_standard = (float)map_tdn_sph[tdn_ang][soln_standard][A_sph_az];
			ang_el_standard = (float)map_tdn_sph[tdn_ang][soln_standard][A_sph_el];

			// convert standard mapping spherical coordinate to cartesian coordinate
			sph_to_cart(ang_az_standard, ang_el_standard, 1.0f, &xyz[A_x], &xyz[A_y], &xyz[A_z]);

			// rotate standard mapping  cartesian coordinate to position of microphone normal
			ar_m2d1dM3(R, xyz, xyz_new);

			// convert standard mapping cartesian coordinate to spherical coordinate
			cart_to_sph(xyz_new[A_x], xyz_new[A_y], xyz_new[A_z], &ang_az, &ang_el, &rd);
			
			// convert angle in bin with limited accuracy
			bin_az_sol = (long)roundf(ang_az);
			bin_el_sol = (long)roundf(ang_el);
			if (bin_az_sol == 360) { // at rounding 359.5+ -> 360 can be build, has to be corrected
				bin_az_sol = 0;
			}
			// at e.g. el=0 and el = 180 degree in the standard mapping all values get az = 0 assigned from sph_to_cart() and no inverse mapping cart_to_sph() exists after transformation to the microphone location.
			// so the required ring of ambiguity becomes to a point and to much spherical energy is mapped to one point.
			// even though in the standard coordinate system it might be expressable with integer bins, after the rotation it is not anymore.
			// the same happens for other coordinates where the ring can only be expressed with a certain number of integer bins
			// correct the energy distribution by only saving new bins in each time delay ang_td and store how often they occur
			// w_Nsphn acts here only as temporary variable for each time delay, as stored mappings are kept unique over the time delay, in contrast to orthogonal version where the mapped angles are not unique over the time delay and the amount of global mapped angles is stored	in w_Nsphn
			if (!w_Nsphn_angtdn[bin_az_sol][bin_el_sol]) { // (bin_az_sol != bin_az_sol_p) || (bin_el_sol != bin_el_sol_p)
				// write new spherical angle to bin if bin is not yet in incidence angle set
				map_tdn_sph[tdn_ang][soln][A_sph_az] = bin_az_sol;
				map_tdn_sph[tdn_ang][soln][A_sph_el] = bin_el_sol;
				w_Ntd_ang_sol[tdn_ang] += 1; // how many integer angle bin solutions do we have for each ring at one time delay
				w_Nsphn_angtdn[bin_az_sol][bin_el_sol] += 1;  // how often did the integer bin already occureat time delay
				w_Nsphn[bin_az_sol][bin_el_sol] += 1; // how often did the integer bin already occur in total on sphere
				//if (bin_el_sol == 91 && bin_az_sol == 90) {
					//int aaa = 0;
				//}
				//if (w_Nsphn[bin_az_sol][bin_el_sol] > 1) {
					//int aa = 0;
				//}
				// save current angle to compare it in next iteration for its novelty
				//bin_az_sol_p = bin_az_sol;
				//bin_el_sol_p = bin_el_sol;


				// increment to next ambigous spherical angle with seperate integer bin at the same time delay
				soln += 1;
			}
			// increment to next ambigous spherical angle at the same time delay
			soln_standard += 1;
		}
		// increment to next time delay
		tdn_ang += 1;
	}

	// free temporary coordinates and matrices
	ar_free2dl(w_Nsphn_angtdn, Naz, Nel);
	free(norm_corsetup);
	free(norm_standardsph);
	ar_free2d(R, A_Ndimxyz, A_Ndimxyz);
	free(xyz);
	free(xyz_new);
	free(mic_setup_ang);

}

void map_td_to_sph_orthogonal(float* mic_loc1, float *mic_norm1, float* mic_loc2, float* mic_norm2, float radius, int ang_stepsize, int Naz, int Nel, long*** map_tdn_sph, long* w_Ntd_ang_sol, long** w_Nsphn) {
	// mathemtical derivation of formula for mapping:
	// source: https://math.stackexchange.com/questions/3428731/finding-the-spherical-coordinates-for-the-edge-obtained-by-cutting-a-sphere-with#3428731

	// initialize
	float mic_dist_x;
	float mic_dist_y;
	float mic_dist_z;
	float mic_norm_x;
	float mic_norm_y;
	float mic_norm_z;
	float mic_loc_az;
	float mic_loc_ev;
	float mic_norm_az;
	float mic_norm_ev;
	float mic_mic_az;
	float mic_mic_ev;
	float ang_el_lim;
	float ang_el;
	float ang_az;
	long bin_az;
	long bin_az_sol;
	long bin_az2_micsetup;
	long bin_el;
	long bin_el_sol;
	long bin_rd;
	long bin_rd_micsetup;
	long bin_az_shift;
	long bin_el_flip;
	long bin_el_scale;
	float bin_el_polar;
	long bin_el_lim;
	long bin_el_scale_lim;
	float bin_el_scale_factor;
	float wave_loc;
	float tmp;
	float tmp2;
	float tmp3;
	float tmp4;
	long ang_td;
	int tdn_ang;
	int tdn_ang_symetric;
	int soln;
	int bin_N;
	int mic_setup_type;
	float mic_setup_ang_az_unfocused; // in which azimuth direction does microphone most left cross-correlation (-90° angle) bin focuses to in horizontal head coordinate system 
	float mic_setup_ang_el_unfocused; // is microphone focusing nowhere (1.0), on ceiling (1.0) or ground (-1.0)
	float *mic_setup_ang = ar_zeros(sph_Ndim);
	UNUSED(Naz);
	UNUSED(Nel);

	// calculate microphone orientation
	mic_dist_x = mic_loc2[A_x] - mic_loc1[A_x];
	mic_dist_y = mic_loc2[A_y] - mic_loc1[A_y];
	mic_dist_z = mic_loc2[A_z] - mic_loc1[A_z];
	mic_norm_x = mic_norm1[A_x] + mic_norm2[A_x];
	mic_norm_y = mic_norm1[A_y] + mic_norm2[A_y];
	mic_norm_z = mic_norm1[A_z] + mic_norm2[A_z];

	mic_loc_az = atan2f(fabsf(mic_dist_y), fabsf(mic_dist_x)); // either 0 (to the front or back) or pi/2 (to the side)
	mic_norm_az = atan2f(mic_norm_y, mic_norm_x); // either 0 (to the right) or pi (to the left) or 0 (unfocused, both microphone opposite normals)
	mic_mic_az = mic_loc_az + mic_norm_az; // azimuth direction in radians where microphone focuses to

	mic_loc_ev = atan2f(fabsf(mic_dist_z), 0); // either 0 (in horizontal direction) or pi/2 (in elevational direction)
	mic_norm_ev = atan2f(mic_norm_z, 0); // either pi/2 (to the top) or -pi/2 (to the bottom) or 0 (unfocused, both microphone opposite normals)
	mic_mic_ev = mic_norm_ev; // elevational direction in radians where microphone focuses to
	
	// set default microphone setup values
	bin_az_shift = 0;
	mic_setup_ang_az_unfocused = 0.0f;
	mic_setup_ang_el_unfocused = 1.0f;
	mic_setup_ang[A_sph_rd] = 1.0f;

	// determine microphone setup (working now for l-r, l2-r2, ll2, rr2)
	// microphone splits left-right direction (x)
	if (fabsf(mic_dist_x) > 0.0f) {
		mic_setup_ang_az_unfocused = 90.0f; 
		mic_setup_ang_el_unfocused = 1.0f; // 1.0 means no change, -1.0 means flipped elevational coordinate system
		if ((mic_norm_x == 0.0f && mic_norm_y == 0.0f && mic_norm_z == 0.0f) ) {
			mic_setup_type = A_xno;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused; 
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused;
			mic_setup_ang[A_sph_rd] = 0.0f;
		}
		if (mic_norm_y < 0.0f && mic_norm_x == 0.0f && mic_norm_z == 0.0f) {
			mic_setup_type = A_xb;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused + 180.0f; // focus direction of delay microphone setup in azimuth plane
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused;  // focus direction of delay microphone setup in elevation plane
		}
		if (mic_norm_y > 0.0f && mic_norm_x == 0.0f && mic_norm_z == 0.0f) {
			mic_setup_type = A_xf;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused;
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused;
		}
		if (mic_norm_z < 0.0f && mic_norm_x == 0.0f && mic_norm_y == 0.0f) {
			mic_setup_type = A_xg;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused;
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused * -1.0f;
		}
		if (mic_norm_z > 0.0f && mic_norm_x == 0.0f && mic_norm_y == 0.0f) {
			mic_setup_type = A_xc;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused;
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused;
		}
		if (mic_dist_x > 0.0f) {
			modulusf(mic_setup_ang[A_sph_az] + 180.0f, 360, &mic_setup_ang[A_sph_az]);
		}
	}

	// microphone splits front-back direction (y)
	if (fabsf(mic_dist_y) > 0.0f) {
		mic_setup_ang_az_unfocused = 0.0f;
		mic_setup_ang_el_unfocused = 1.0f;
		if ((mic_norm_x == 0.0f && mic_norm_y == 0.0f && mic_norm_z == 0.0f) ) {
			mic_setup_type = A_yno;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused;
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused;
			mic_setup_ang[A_sph_rd] = 0.0f;
		}
		if (mic_norm_x < 0.0f && mic_norm_y == 0.0f && mic_norm_z == 0.0f) {
			mic_setup_type = A_yl;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused + 180.0f;
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused;
		}
		if (mic_norm_x > 0.0f && mic_norm_y == 0.0f && mic_norm_z == 0.0f) {
			mic_setup_type = A_yr;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused;
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused;
		}
		if (mic_norm_z < 0.0f && mic_norm_x == 0.0f && mic_norm_y == 0.0f) {
			mic_setup_type = A_yg;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused;
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused * -1.0f;
		}
		if (mic_norm_z > 0.0f && mic_norm_x == 0.0f && mic_norm_y == 0.0f) {
			mic_setup_type = A_yc;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused;
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused;
		}
		if (mic_dist_y > 0.0f) {
			modulusf(mic_setup_ang[A_sph_az] + 180.0f, 360, &mic_setup_ang[A_sph_az]);
		}
	}

	// microphone splits ceil-ground direction (z)
	if (fabsf(mic_dist_z) > 0.0f) {
		mic_setup_ang_az_unfocused = 0.0f;
		mic_setup_ang_el_unfocused = 1.0f;
		if ((mic_norm_x == 0.0f && mic_norm_y == 0.0f && mic_norm_z == 0.0f) ) {
			mic_setup_type = A_xno;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused;
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused;
			mic_setup_ang[A_sph_rd] = 0.0f;
		}
		if (mic_norm_x < 0.0f && mic_norm_y == 0.0f && mic_norm_z == 0.0f) {
			mic_setup_type = A_zl;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused + 270.0f;
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused;
		}
		if (mic_norm_x > 0.0f && mic_norm_y == 0.0f && mic_norm_z == 0.0f) {
			mic_setup_type = A_zr;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused + 90.0f;
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused;
		}
		if (mic_norm_y < 0.0f && mic_norm_x == 0.0f && mic_norm_y == 0.0f) {
			mic_setup_type = A_zb;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused + 180.0f;
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused;
		}
		if (mic_norm_y > 0.0f && mic_norm_x == 0.0f && mic_norm_y == 0.0f) {
			mic_setup_type = A_zf;
			mic_setup_ang[A_sph_az] = mic_setup_ang_az_unfocused;
			mic_setup_ang[A_sph_el] = mic_setup_ang_el_unfocused;
		}
		if (mic_dist_z > 0.0f) {
			mic_setup_ang[A_sph_az] = mic_setup_ang[A_sph_el] * -1.0f;
		}
	}

	// calculate mapping between time delay and spherical angle (if plane between both microphones splits z plane)
	if (fabsf(mic_dist_z) > 0.0f && (fabsf(mic_dist_z) > (fabsf(mic_dist_x)))) {
		tdn_ang = 0;
		for (ang_td = -90; ang_td <= 90; ang_td += ang_stepsize) {
			//wave_loc = cosf((float)ang_td / 180.0f * PI) * radius;
			//wave_loc = - wave_loc;
			bin_el = -ang_td + 90l;
			soln = 0;
			bin_N = 360 / ang_stepsize;
			w_Ntd_ang_sol[tdn_ang] = bin_N;
			for (bin_az = 0; bin_az < 360; bin_az += ang_stepsize) {

				// transform relative azimuth and elevation position to absolute azimuth and elevation position for given microphone setup
				bin_az_shift = (long)mic_setup_ang[A_sph_az];
				bin_el_flip = (long)mic_setup_ang[A_sph_el];
				bin_az_sol = modulus(bin_az + bin_az_shift, 360l);
				bin_el_sol = bin_el * bin_el_flip;
				bin_rd_micsetup = (long)radius;

				// assign spherical mapping between time delay and point on sphere
				map_tdn_sph[tdn_ang][soln][A_sph_az] = bin_az_sol;
				map_tdn_sph[tdn_ang][soln][A_sph_el] = bin_el_sol;
				map_tdn_sph[tdn_ang][soln][A_sph_rd] = (long)radius;
				//if (bin_az_sol == 270 && bin_el_sol == 180) {
					//int agsdg = 0;
				//}
				// add mapping to rate of occurance of those mapping
				w_Nsphn[bin_az_sol][bin_el_sol] += 1l;

				soln += 1;
			}
			tdn_ang += 1;
		}
	}

	// calculate mapping between time delay and spherical angle (if plane between both microphones splits x or y plane)
	if (fabsf(mic_dist_x) > 0.0f || fabsf(mic_dist_y) > 0.0f) {
		// init
		tdn_ang = 0;
		// for all negative time delay angles and 0 time delay
		for (ang_td = -90; ang_td <= 0; ang_td += 1) {
			wave_loc = sinf((float)ang_td / 180.0f * PI) * radius;
			tmp = 1.0f - (wave_loc * wave_loc) / (radius * radius);
			ang_el_lim = asinf(sqrtf(tmp)); // sine(ang_el) instead of cosine(ang_el) as spherical angle is defined to  0° for horizontal plane in our head coordinate system, this is in contrast to correct spherical coorindate sytsm
			bin_el_lim = (long)floorf(fabsf(ang_el_lim / PI * 180.0f));
			bin_el_scale_lim = 90; // set to bin_el_lim if no elevation scaling needed (usually necessary outside of sphere where circle goes to point)
			bin_el_scale_factor = (float)bin_el_lim / (float)bin_el_scale_lim;
			soln = 0;
			bin_N = (bin_el_scale_lim == 0) ? 1 : ((bin_el_scale_lim * 2 * 2)/ang_stepsize); //  [elim interval: [0, 90]], el_lim * 2 as upper and lower sphere of elevation angles, 2 * (...) as each elvation angle corresponds to 2 azimuth angles, 1 if sphere collapses to point,
			w_Ntd_ang_sol[tdn_ang] = bin_N; //
			//if (ang_td == -1) {
				//int a = 0;
			//}
			// iterate from highest elevation position to lowest elevation position on sphere
			for (bin_el_scale = bin_el_scale_lim; bin_el_scale >= -bin_el_scale_lim; bin_el_scale -= ang_stepsize) {
				// calculate relative azimuth and elevation position corresponding to time delay
				bin_el_polar = -(float)bin_el_scale * bin_el_scale_factor + 90.0f;
				ang_el = (bin_el_polar * PI / 180.0f);
				tmp2 = cosf(ang_el) * cosf(ang_el);
				tmp3 = tmp - tmp2;
				tmp3 = tmp3 < 0.0f ? 0.0f: tmp3;
				//ang_el = ang_el <= 0.0f ? 0.0f + 0.000000001 : ang_el; // avoid devision through 0 if compiler doesn't support next statement
				tmp4 = sqrtf(tmp3) / sinf(ang_el);
				tmp4 = (ang_el != 0.0f && tmp4 < 1.0f) ? tmp4 : 1.0f;
				//tmp3 = sinf(ang_el);
				//tmp4 = ((tmp - tmp2) < 0.0f || sinf(ang_el) <= 0.0f) ? 0.0f : sqrtf((tmp - tmp2))/sinf(tmp3); // if ang_el to high as result of (purposed) numerical rounding error, reset tmp to 0, such that ang_az becomes = 0.0f
				//tmp4 = tmp4 > 1.0f ? 1.0f : tmp4;
				ang_az = roundf(asinf(tmp4) / PI * 180.0f); // // sine(ang_el) instead of cosine(ang_el), and vice versa
				bin_az = (long)ang_az;
				bin_el = -(long)roundf(bin_el_scale * bin_el_scale_factor) + 90l;
				bin_rd = (long)radius;

				// microphone setup: transform relative azimuth and elevation position to absolute azimuth and elevation position
				bin_az_shift = (long)mic_setup_ang[A_sph_az];
				bin_el_flip = (long)mic_setup_ang[A_sph_el];
				bin_az_sol = modulus(bin_az + bin_az_shift, 360l);
				bin_az2_micsetup = modulus(-bin_az + bin_az_shift, 360l);
				bin_el_sol = bin_el * bin_el_flip;
				bin_rd_micsetup = bin_rd;
				
				// assign spherical mapping between time delay and point on sphere
				map_tdn_sph[tdn_ang][soln][A_sph_az] = bin_az_sol;
				map_tdn_sph[tdn_ang][soln][A_sph_el] = bin_el_sol;
				map_tdn_sph[tdn_ang][soln][A_sph_rd] = bin_rd_micsetup;

				// add mapping to rate of occurance of those mapping
				w_Nsphn[bin_az_sol][bin_el_sol] += 1l;

				soln += 1;


				// if not most outside point on sphere, each elevational angle corresponds to 2 azimuth angles
				if (!(bin_el_scale == bin_el_scale_lim || bin_el_scale == -bin_el_scale_lim)) {

					// assign spherical mapping between time delay and symetric azimuth point on sphere
					map_tdn_sph[tdn_ang][soln][A_sph_az] = bin_az2_micsetup;
					map_tdn_sph[tdn_ang][soln][A_sph_el] = bin_el_sol;
					map_tdn_sph[tdn_ang][soln][A_sph_rd] = bin_rd_micsetup;

					// add mapping to rate of occurance of those mapping
					w_Nsphn[bin_az2_micsetup][bin_el_sol] += 1l;

					soln += 1;
				}
			}
			tdn_ang += 1;
		}
		// for all positive time delay angles (mapping is symetrical)
		for (ang_td = 1; ang_td <= 90; ang_td += 1) {
			tdn_ang_symetric = 90 - ang_td;
			for (soln = 0; soln < w_Ntd_ang_sol[tdn_ang_symetric]; soln++) {
				w_Ntd_ang_sol[tdn_ang] = w_Ntd_ang_sol[tdn_ang_symetric];
				map_tdn_sph[tdn_ang][soln][A_sph_az] = modulus(-(map_tdn_sph[tdn_ang_symetric][soln][A_sph_az] - bin_az_shift) + bin_az_shift + 180l, 360l);
				map_tdn_sph[tdn_ang][soln][A_sph_el] = map_tdn_sph[tdn_ang_symetric][soln][A_sph_el];
				map_tdn_sph[tdn_ang][soln][A_sph_rd] = map_tdn_sph[tdn_ang_symetric][soln][A_sph_rd];
				w_Nsphn[map_tdn_sph[tdn_ang][soln][A_sph_az]][map_tdn_sph[tdn_ang][soln][A_sph_el]] += 1l;

			}
			tdn_ang += 1;
		}
	}

	free(mic_setup_ang);

}




//void sph_ang_to_bin(int ang_az, int ang_el, int ang_stepsize, int *bin_az, int *bin_el){
	//bin_az = (int)round((float)ang_az / (float)ang_stepsize);
	//bin_el = (int)round((-(float)ang_el + 90.0f) / (float)ang_stepsize);
//}
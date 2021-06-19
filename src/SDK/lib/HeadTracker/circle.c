// Copyright 2020, Felix Pfreundtner, All rights reserved.

#include "circle.h"


int** init_cir_az_mapping(int Nxy, int Nxylim, int* prob_az_N, bool flip_x_plane, bool flip_y_plane) {

	// declare
	int n_xsgn;
	int n_x;
	int n_y1;
	int n_y2;
	int n_c1;
	int n_c2;
	float ang_tmp;
	bool map_nonlinear; // map non-linear considering cosine directionality of incidence plane wave

	// parameter
	map_nonlinear = true;

	// init
	int Nxylim2;
	int Nxylim3;
	int Nxylim4;
	int Nxylim5;
	int n_cerror;
	float conv_atan2_to_bin;
	Nxylim2 = 2 * Nxylim;
	Nxylim3 = 3 * Nxylim;
	Nxylim4 = 4 * Nxylim;
	Nxylim5 = 5 * Nxylim;
	conv_atan2_to_bin = Nxylim2 / PI;
	n_y1 = 0;
	n_y2 = 0;
	n_c1 = 0;
	n_c2 = 0;
	n_cerror = Nxylim2 + 1;

	// init probability mapping array
	*prob_az_N = Nxylim4;
	int N_0delay = ((int)floorf(((float)Nxy) / 2.0f));
	int N_start = N_0delay - Nxylim;
	int** prob_map_az = ar_declare2di(2, *prob_az_N);

	// set array to unreachable error value (Nxylim2 + 1) to find missing 2d n_c correlation bins (if non-linear mapping)
	if (map_nonlinear) {
		ar_seti(prob_map_az[A_x], *prob_az_N, n_cerror);
	}

	// assign n_x, n_y correlation bin to 2d n_c correlation bin
	// currently: n_x[-Nxylim, Nxylim], n_y[0...2*Nxylim], n_c[0...2*Nxylim]
	for (n_xsgn = - Nxylim; n_xsgn <= Nxylim; n_xsgn++) {
		if (n_xsgn < 0){
			n_y1 = Nxylim2 + n_xsgn;
			n_y2 = - n_xsgn;
			if (map_nonlinear) {
				ang_tmp = atan2f((float)(n_y1 - Nxylim), (float)(n_xsgn));
				n_c1 = Nxylim5 - (int)roundf(ang_tmp * conv_atan2_to_bin); // ang_tmp: 180:90 -> Nxylim3:0
				// double mapping at left edge (270°)
				if (n_xsgn == - Nxylim) {
					n_c2 = n_c1;
				}
				else {
					ang_tmp = atan2f((float)(n_y2 - Nxylim), (float)(n_xsgn));
					n_c2 = Nxylim - (int)roundf(ang_tmp * conv_atan2_to_bin); // ang_tmp: -180:-90 -> Nxylim3:Nxylim2
				}
			}
			else {
				n_c1 = Nxylim4 + n_xsgn; 
				n_c2 = Nxylim2 - n_xsgn; // Nxylim2 + n_xsgn
			}
		}
		else {
			n_y1 = Nxylim2 - n_xsgn;
			n_y2 = n_xsgn;
			if (map_nonlinear) {
				ang_tmp = atan2f((float)(n_y1 - Nxylim), (float)(n_xsgn));
				n_c1 = Nxylim - (int)roundf(ang_tmp * conv_atan2_to_bin); // ang_tmp: 90:0 -> 0:Nxylim
				ang_tmp = atan2f((float)(n_y2 - Nxylim), (float)(n_xsgn));
				n_c2 = Nxylim - (int)roundf(ang_tmp * conv_atan2_to_bin); // ang_tmp: -90:0 -> Nxylim2:Nxylim
				// double mapping at right edge (90°)
				if (n_xsgn == Nxylim) {
					n_c1 = n_c2;
				}
				else {
					ang_tmp = atan2f((float)(n_y2 - Nxylim), (float)(n_xsgn));
					n_c2 = Nxylim - (int)roundf(ang_tmp * conv_atan2_to_bin); // ang_tmp: -180:-90 -> Nxylim3:Nxylim2
				}
			}
			else {
				n_c1 = n_y2; // 0:Nxylim
				n_c2 = n_y1; // Nxylim2 - n_xsgn
			}
		}

		// assign mapping
		n_x = n_xsgn + Nxylim;
		if (flip_x_plane) {
			n_x = -(n_x - Nxylim) + Nxylim;
		}
		if (flip_y_plane) {
			n_y1 = -(n_y1 - Nxylim) + Nxylim;
			n_y2 = -(n_y2 - Nxylim) + Nxylim;
		}
		prob_map_az[A_x][n_c1] = N_start + n_x;
		prob_map_az[A_y][n_c1] = N_start + n_y1;
		prob_map_az[A_x][n_c2] = N_start + n_x;
		prob_map_az[A_y][n_c2] = N_start + n_y2;
	}

	// find missing 2d n_c correlation bins (if non-linear mapping) and assign neigbors
	if (map_nonlinear) {
		for (n_c1 = 0; n_c1 < *prob_az_N; n_c1++) {
			if (prob_map_az[A_x][n_c1] == n_cerror) {
				prob_map_az[A_x][n_c1] = prob_map_az[A_x][n_c1 - 1];
				prob_map_az[A_y][n_c1] = prob_map_az[A_y][n_c1 - 1];
			}
		}
	}

	// return
	return prob_map_az;
}

int* init_cir_linear_mapping(int N, int Nlim, int* prob_N, bool flip_plane) {
	// init

	// init probability mapping array
	*prob_N = Nlim * 2 + 1;
	int N_0delay = ((int)floorf(((float)N) / 2.0f));
	int N_start = N_0delay - Nlim;
	int* prob_map_linear = ar_declarei(*prob_N);

	// assign probability mapping array
	if (flip_plane) {
		for (int n_c = 0; n_c < *prob_N; n_c++) {
			prob_map_linear[*prob_N - n_c - 1] = N_start + n_c;

		}
	}
	else {
		for (int n_c = 0; n_c < *prob_N; n_c++) {
			prob_map_linear[n_c] = N_start + n_c;
		}
	}

	// return
	return prob_map_linear;
}


void add_cir_ref(struct cirs* c, float* cor_x, float* cor_y, float* cor_z, int frame_m, float ymax_ang, bool binarymax_instead_of_y, bool flip_y_plane) {
	float* ar_tmpx = ar_declare(c->prob_N[A_dim[A_az]]);
	float* ar_tmpy = ar_declare(c->prob_N[A_dim[A_az]]);
	for (int d = 0; d < A_Ndof; d++) {
		switch (A_dim_r[d]) {
			case A_az:
				if (flip_y_plane) {
					ymax_ang = -ymax_ang;
				}
				for (int n = 0; n < c->prob_N[A_dim[A_az]]; n++) {
					ar_tmpx[n] = cor_x[c->prob_map_az[A_x][n]];
					ar_tmpy[n] = cor_y[c->prob_map_az[A_y][n]];
					if (binarymax_instead_of_y) {
						if (ymax_ang <= 0.0f) {
							c->prob[A_dim[A_az]][frame_m][n] = n < c->prob_N[A_dim[A_az]] / 4 ? 0.0f : c->prob[A_dim[A_az]][frame_m][n];
							c->prob[A_dim[A_az]][frame_m][n] = (n >= c->prob_N[A_dim[A_az]] / 4 &&  n < c->prob_N[A_dim[A_az]] / 2) ? cor_x[c->prob_map_az[A_x][n]] : c->prob[A_dim[A_az]][frame_m][n];
							c->prob[A_dim[A_az]][frame_m][n] = (n >= c->prob_N[A_dim[A_az]] / 2 &&  n <= (c->prob_N[A_dim[A_az]] - c->prob_N[A_dim[A_az]] / 4)) ? cor_x[c->prob_map_az[A_x][n]] : c->prob[A_dim[A_az]][frame_m][n];
							c->prob[A_dim[A_az]][frame_m][n] = (n > (c->prob_N[A_dim[A_az]] - c->prob_N[A_dim[A_az]] / 4)) ? 0.0f : c->prob[A_dim[A_az]][frame_m][n];
						}
						else {
							c->prob[A_dim[A_az]][frame_m][n] = n < c->prob_N[A_dim[A_az]] / 4 ? cor_x[c->prob_map_az[A_x][n]] : c->prob[A_dim[A_az]][frame_m][n];
							c->prob[A_dim[A_az]][frame_m][n] = (n >= c->prob_N[A_dim[A_az]] / 4 &&  n < c->prob_N[A_dim[A_az]] / 2) ? 0.0f : c->prob[A_dim[A_az]][frame_m][n];
							c->prob[A_dim[A_az]][frame_m][n] = (n >= c->prob_N[A_dim[A_az]] / 2 &&  n <= (c->prob_N[A_dim[A_az]] - c->prob_N[A_dim[A_az]] / 4)) ? 0.0f : c->prob[A_dim[A_az]][frame_m][n];
							c->prob[A_dim[A_az]][frame_m][n] = (n > (c->prob_N[A_dim[A_az]] - c->prob_N[A_dim[A_az]] / 4))? cor_x[c->prob_map_az[A_x][n]] : c->prob[A_dim[A_az]][frame_m][n];
						}
					}
					else {
						c->prob[A_dim[A_az]][frame_m][n] = cor_x[c->prob_map_az[A_x][n]] * cor_y[c->prob_map_az[A_y][n]];
					}
				}; 
				break;
			case A_el:
				for (int n = 0; n < c->prob_N[A_dim[A_el]]; n++) {
					c->prob[A_dim[A_el]][frame_m][n] = cor_z[c->prob_map_el[n]];
				}
				//memcpy(c->prob[A_dim[A_el]][frame_m], cor_z + c->prob_map_el[0], c->prob_N[A_dim[A_el]] * sizeof(float)); 
				break;
		}
	}
	free(ar_tmpx);
	free(ar_tmpy);
}


void add_cir_diff(struct cirs* c, float* cor_mtd_x, float* cor_mtd_y, float* cor_mtd_z, struct ffts* fft_tmp, char* fft_library, int frame_m, bool delta_instead_of_diff) {
	char cor_type[] = "cor";
	long fft_flim_low = 0;
	long fft_flim_high = 0;
	long sampling_rate = 0;

	// parameters
	bool az_el_diff_on = !cir_delta_instead_of_diff;
	bool az_el_diff_normalize = true;
	// calculate

	for (int d = 0; d < A_Ndof; d++) {
		switch (A_dim_r[d]){
			case A_x:
				memcpy(c->probdiff[d][frame_m], cor_mtd_x + c->prob_map_xyz[0], c->probdiff_N[d] * sizeof(float)); break;
			case A_y:
				memcpy(c->probdiff[d][frame_m], cor_mtd_y + c->prob_map_xyz[0], c->probdiff_N[d] * sizeof(float)); break;
			case A_z:
				memcpy(c->probdiff[d][frame_m], cor_mtd_z + c->prob_map_xyz[0], c->probdiff_N[d] * sizeof(float)); break;
			case A_az: 
				if (az_el_diff_on) {
					crosscorrelate_fft(c->prob[d][frame_m], c->prob_N[d], c->prob_p[d][frame_m], c->prob_N[d], c->probdiff[d][frame_m], c->probdiff_N[d], fft_tmp, fft_library, cor_type, fft_flim_low, fft_flim_high, sampling_rate, az_el_diff_normalize); break;
				}
			case A_el:
				if (az_el_diff_on) {
					crosscorrelate_fft(c->prob[d][frame_m], c->prob_N[d], c->prob_p[d][frame_m], c->prob_N[d], c->probdiff[d][frame_m], c->probdiff_N[d], fft_tmp, fft_library, cor_type, fft_flim_low, fft_flim_high, sampling_rate, az_el_diff_normalize); break;
				}
		}
	}
}

void get_cir_head_pos(struct cirs* c, struct heads* h, int frame_m, bool diffmax_instead_of_diffmean, bool delta_instead_of_diff) {
	float xyz_scale;

	xyz_scale = (float)c->prob_N[A_x] / sos; // x,y,z same scale

	// select dof
	for (int d = 0; d < A_Ndof; d++) {
		if (A_dim_r[d] == A_x || A_dim_r[d] == A_y || A_dim_r[d] == A_z) {

			// differential
			if (!delta_instead_of_diff) {
				get_probability_norm(c->probdiff[d][frame_m], &c->probdiff_norm[d][frame_m], c->probdiff_N[d], true, true);
				//expectation_mean_shift(c->probdiff[d][frame_m], 1.0f, NULL, c->probdiff_N[d], c->prob_N[d], c->scale[d], &h->pos[frame_m][d][A_vel]);
				//expectation_var_shift(c->probdiff[d][frame_m], 1.0f, NULL, c->probdiff_N[d], c->prob_N[d], c->scale[d], &h->pos[frame_m][d][A_vel], &h->pos_var[frame_m][d][A_vel], true, false);
				if (diffmax_instead_of_diffmean) {
					expectation_max_shift(c->probdiff[d][frame_m], 1.0f, NULL, c->probdiff_N[d], c->prob_N[d], c->scale[d], &h->pos[frame_m][d][A_vel]);
				}
				else {
					expectation_mean_shift(c->probdiff[d][frame_m], 1.0f, NULL, c->probdiff_N[d], c->prob_N[d], c->scale[d], &h->pos[frame_m][d][A_vel]);
				}
			}

		}

		if (A_dim_r[d] == A_az) {
			// get expectation value and variance for bin values

			// absolute
			get_probability_norm(c->prob[d][frame_m], &c->prob_norm[d][frame_m], c->prob_N[d], true, true);
			expectation_mean_var_circular(c->prob[d][frame_m], 1.0f, NULL, c->prob_N[d], (float)c->prob_N[d], c->scale[d], &h->pos[frame_m][d][A_ref], &h->pos_var[frame_m][d][A_ref]);
			if (diffmax_instead_of_diffmean) {
				expectation_max_shift(c->prob[d][frame_m], 1.0f, NULL, c->prob_N[d], 0, c->scale[d], &h->pos[frame_m][d][A_ref]);
			}

			// differential
			if (!delta_instead_of_diff) {
				get_probability_norm(c->probdiff[d][frame_m], &c->probdiff_norm[d][frame_m], c->probdiff_N[d], true, true);
				//expectation_mean_shift(c->probdiff[d][frame_m], 1.0f, NULL, c->probdiff_N[d], c->prob_N[d], c->scale[d], &h->pos[frame_m][d][A_vel]);
				//expectation_var_shift(c->probdiff[d][frame_m], 1.0f, NULL, c->probdiff_N[d], c->prob_N[d], c->scale[d], &h->pos[frame_m][d][A_vel], &h->pos_var[frame_m][d][A_vel], true, false);
				if (diffmax_instead_of_diffmean) {
					expectation_max_shift(c->probdiff[d][frame_m], 1.0f, NULL, c->probdiff_N[d], c->prob_N[d], c->scale[d], &h->pos[frame_m][d][A_vel]);
				}
				else {
					expectation_mean_shift(c->probdiff[d][frame_m], 1.0f, NULL, c->probdiff_N[d], c->prob_N[d], c->scale[d], &h->pos[frame_m][d][A_vel]);
				}
			}

		}

		if (A_dim_r[d] == A_el) {
			// get expectation value and variance for bin values

			// absolute
			get_probability_norm(c->prob[d][frame_m], &c->prob_norm[d][frame_m], c->prob_N[d], true, true);
			//expectation_mean_shift(c->prob[d][frame_m], 1.0f, NULL, c->prob_N[d], 0, c->scale[d], &h->pos[frame_m][d][A_ref]);
			//expectation_var_shift(c->prob[d][frame_m], 1.0f, NULL, c->prob_N[d], 0, c->scale[d], &h->pos[frame_m][d][A_ref], &h->pos_var[frame_m][d][A_ref], true, false);
			if (diffmax_instead_of_diffmean) {
				expectation_max_shift(c->prob[d][frame_m], 1.0f, NULL, c->prob_N[d], 0, c->scale[d], &h->pos[frame_m][d][A_ref]);
			}
			else {
				expectation_mean_shift(c->prob[d][frame_m], 1.0f, NULL, c->prob_N[d], 0, c->scale[d], &h->pos[frame_m][d][A_ref]);
			}

			// differential
			if (!delta_instead_of_diff) {
				get_probability_norm(c->probdiff[d][frame_m], &c->probdiff_norm[d][frame_m], c->probdiff_N[d], true, true);
				//expectation_mean_shift(c->probdiff[d][frame_m], 1.0f, NULL, c->probdiff_N[d], c->prob_N[d], c->scale[d], &h->pos[frame_m][d][A_vel]);
				//expectation_var_shift(c->probdiff[d][frame_m], 1.0f, NULL, c->probdiff_N[d], c->prob_N[d], c->scale[d], &h->pos[frame_m][d][A_vel], &h->pos_var[frame_m][d][A_vel], true, false);
				if (diffmax_instead_of_diffmean) {
					expectation_max_shift(c->probdiff[d][frame_m], 1.0f, NULL, c->probdiff_N[d], c->prob_N[d], c->scale[d], &h->pos[frame_m][d][A_vel]);
				}
				else {
					expectation_mean_shift(c->probdiff[d][frame_m], 1.0f, NULL, c->probdiff_N[d], c->prob_N[d], c->scale[d], &h->pos[frame_m][d][A_vel]);
				}
			}
		}
	}

}



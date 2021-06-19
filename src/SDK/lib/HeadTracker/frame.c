// Copyright 2020, Felix Pfreundtner, All rights reserved.

#include "frame.h"

void get_start_sample(float* x, long x_N, float startthreshold, long * Nstart) {
	// initialize
	*Nstart = 0;

	// calculate
	for (long n = 0; n < x_N; n++) {
		if (fabsf(x[n]) > startthreshold) {
			*Nstart = n;
			break;
		}
	}
}

void get_binaural_delay_position(float* x1, float* x2, long x_N, long isinv_N, long rec_N, long center_N, long* Ndelay) {
	// initialize
	long max_N = 0;
	float max_amp = 0;
	long Npeakoffset;
	float Npeakoffsetfactor = 0.0f; // higher -> larger offset of samples included before peak

	// calculate
	// find peak in binaural matched filter
	binaural_maximum(x1, x2, x_N, &max_N, &max_amp);

	// calculate delay position from peak position
	Npeakoffset = (long)((float)(rec_N - isinv_N) * Npeakoffsetfactor);
	if (Npeakoffset < max_N) {
		*Ndelay = max_N - Npeakoffset;
	}
	else {
		*Ndelay = 0;
	}

	// center delay around isinv_N
	*Ndelay = *Ndelay - center_N;

}

void get_binaural_playback_position(float* x1, float* x2, long x_N, long isinv_N, long center_N, long* Ndelay, long* Nplay){
	// see
	// initialize
	long max_N = 0;
	float max_amp = 0;
	long mat_N0 = center_N + *Ndelay;
	long is_N2 = (long)floorf((float)isinv_N / 2.0f);
	long mat_Nlow = (center_N - is_N2) + *Ndelay;
	long mat_Nup = (center_N + is_N2) + *Ndelay;
	long tmp;
	bool binaural_max = false;

	// calculate
	// find peak in binaural matched filter
	if (binaural_max) {
		binaural_maximum(x1, x2, x_N, &max_N, &max_amp); // probably better to use not maximum, but instead calculate for both signals maximum and than take 

		// calculate playback position from peak position (assuming 0 ms delay between playback and recording, edge cases not checked yet)
		// right: playback position 0 up to isinv_N / 2 found
		if (max_N >= mat_N0 && max_N <= mat_Nup) {
			*Nplay = max_N - isinv_N; // 2 * isinv_N - max_N;
		}
		// left: playback position isinv_N / 2 up to isinv_N found
		if (max_N < mat_N0 && max_N > mat_Nlow) {
			*Nplay = max_N - *Ndelay; // isinv_N - max_N;
		}
		// outside of boundary: if playback position has not been found, continue searching for it
		if (max_N <= mat_Nlow) {
			//*Ndelay = is_N2;
			*Nplay = is_N2;
		}
		if (max_N > mat_Nup) {
			*Nplay = is_N2;
		}
	}
	else {
		ar_max(x1, x_N, &max_N, &max_amp);
		if (max_N >= mat_N0 && max_N <= mat_Nup) {
			*Nplay = max_N - isinv_N; // 2 * isinv_N - max_N;
		}
		// left: playback position isinv_N / 2 up to isinv_N found
		if (max_N < mat_N0 && max_N > mat_Nlow) {
			*Nplay = max_N - *Ndelay; // isinv_N - max_N;
		}
		// outside of boundary: if playback position has not been found, continue searching for it
		if (max_N <= mat_Nlow) {
			//*Ndelay = is_N2;
			*Nplay = is_N2;
		}
		if (max_N > mat_Nup) {
			*Nplay = is_N2;
		}

		ar_max(x2, x_N, &max_N, &max_amp);
		if (max_N >= mat_N0 && max_N <= mat_Nup) {
			tmp = max_N - isinv_N; // 2 * isinv_N - max_N;
		}
		// left: playback position isinv_N / 2 up to isinv_N found
		if (max_N < mat_N0 && max_N > mat_Nlow) {
			tmp = max_N - *Ndelay; // isinv_N - max_N;
		}
		// outside of boundary: if playback position has not been found, continue searching for it
		if (max_N <= mat_Nlow) {
			//*Ndelay = is_N2;
			tmp = is_N2;
		}
		if (max_N > mat_Nup) {
			tmp = is_N2;
		}
		if (tmp < *Nplay) {
			*Nplay = tmp;
		}
	}
	//write_txt_debug(x1,  x_N, 1);
	//write_txt_debug(x2,  x_N, 1);

}

void get_binaural_start_sample(float* x1, float * x2, long x_N, long x_N_start, long Nstartdynamicsearch, float start_threshold, long start_offset, long * Nstart) {
	// initialize
	*Nstart = x_N_start;

	// find maximum peak in signal
	float max_1;
	long max_1_N;
	float max_2;
	long max_2_N;
	float maxpeak;
	long maxpeak_N;
	ar_max(x1 + x_N_start, x_N - x_N_start, &max_1_N, &max_1);
	ar_max(x2 + x_N_start, x_N - x_N_start, &max_2_N, &max_2);
	if (max_2 > max_1) {
		maxpeak = max_2;
		maxpeak_N = max_2_N + x_N_start;
	}
	else {
		maxpeak = max_1;
		maxpeak_N = max_1_N + x_N_start;

	}
	// adapt start threshold to maximum peak
	start_threshold *= maxpeak;

	// if not normalized impulse response, search for peak and then search for beginning backwards (faster)
	if (Nstartdynamicsearch != 0) {
		long minsearch_N;
		// find start certain ms backwards from maximum peak
		minsearch_N = (maxpeak_N + Nstartdynamicsearch > x_N_start) ? maxpeak_N + Nstartdynamicsearch : x_N_start;
		for (long n = maxpeak_N; n >= minsearch_N; n--) {
			if (!((fabsf(x1[n]) < start_threshold) && (fabsf(x2[n]) < start_threshold))) {
				*Nstart = n;
			}
		}
	}
	// otherwise search forward for threshold (computational demanding)
	else {
		for (long n = x_N_start; n < x_N; n++) {
			if ((fabsf(x1[n]) > start_threshold) || (fabsf(x2[n]) > start_threshold)) {
				*Nstart = n;
				break;
			}
		}
	}
	if (*Nstart > -start_offset) {
		*Nstart += start_offset;
	}
	//write_txt_debug(x1,  x_N, 1);
	//write_txt_debug(x2,  x_N, 1);

}

void check_start_sample(long N, long frame_M, long frame_N, long frame_Noverlap, long * Nstart) {
	long Nstartlatest = N - (frame_M * (frame_N - frame_Noverlap) + 1);
	if (*Nstart > Nstartlatest) {
		console_log(PRIO_WARNING, "Warning: Impulse response at frame extraction process not long enough for selected amount of frames. Set start to 0 sample\n");
		*Nstart = 0;
	}
}


void extract_frames(struct fras* f, float *x, long x_N, float* window, long frame_N, long frame_M, long frame_Mon, long frame_Mstart, long Nstartframesample, long Nstart, long frame_Noverlap, bool frame_window, bool frame_normalize, int ch, int ch_reference, float **y){
    // initialize
    long s = 0; // sample of full impulse response signal
	long n, m;
	float energymaxamp;
	//bool apply_window = true; // window signal frames at edges with hann window

    // calculate
    // for each frame
    for(m = 0; m < frame_M; m++){

		// reset frame_offset in case of negative sample number
		if (Nstartframesample < 0) {
			Nstartframesample = 0;
			console_log(PRIO_WARNING,  "Warning: Set frame offset at frame extraction process to sample 0 as otherwise frame before signal begin selected\n");
		}

		// calculate bin number of first sample in current frame interval
		s = Nstartframesample + (frame_N - frame_Noverlap) * (m + frame_Mstart); //frame_end = frame_start + frame_N - 1;

		// for each sample in frame interval
		if (s < x_N) {
			if (ch == ch_reference) {
				// get frame weights
				f->m_weight_energy[m] = ar_sumsquared(x + s, frame_N);
				f->m_weight_early[m] = 1.0f - (float)m / (float)frame_M;

				// get sample number of absolute peak
				f->m_Npeak[m] = ar_absmaxn(x + s, frame_N);
				f->m_Npeak[m] = s + f->m_Npeak[m];
				f->m_energyfirst = Nstart >= s ? m : f->m_energyfirst;
			}
			// apply window
			if (frame_window) {
				for (n = 0; n < frame_N; n++) {
					y[m][n] = window[n] * x[s + n];  // 0.5f * (1.0f - (float)cos(2.0f * PI * (float)n / ((float)frame_N - 1.0f)));
				}
			}
			else {
				for (n = 0; n < frame_N; n++) {
					y[m][n] = x[s + n]; 
				}
			}
			// normalize
			if (frame_normalize) {
				mono_normalize(y[m], frame_N);
			};
		}
		else {
			for (n = 0; n < frame_N; n++) {
				y[m][n] = 0.0f;
				console_log(PRIO_WARNING,  "Warning: Impulse response at frame extraction process not long enough for selected amount of frames\n");
			}
		}
    }
	if (ch == ch_reference) {
		// get relative energy amount in each frame
		ar_max(f->m_weight_energy, frame_M, &f->m_energymax, &energymaxamp);
		ar_divide(f->m_weight_energy, frame_M, energymaxamp);

		// select frame_Mon frames with maximal energy
		select_frames(f, frame_N, frame_M, frame_Mon);
	}

}


void select_frames(struct fras* f, long Nframe, long Mframe, long Mframeon) {
    // initialize
	long m, mon;

	// get new active frames
	ar_Nmaxi(f->m_weight_energy, Mframe, f->on_m, f->on_weight_energy, Mframeon);

	// set status for new active frames
	memcpy(f->m, f->tmp_false, Mframe * sizeof(bool));
	memcpy(f->m_on, f->tmp_false, Mframe * sizeof(bool));
	memcpy(f->m_new, f->tmp_false, Mframe * sizeof(bool));
	f->m_Non = 0;
	f->m_first = Mframe;
	for (mon = 0; mon < Mframeon; mon++) {
		m = f->on_m[mon];
		f->m[m] = true;

		// if first active frame
		if (m < f->m_first) {
			f->m_first = m;
		}

		// if frame m was already active before
		if (f->m_p[m]) {
			f->m_on[m] = true;
			f->m_Non += 1;
		}
		else {
			f->m_new[m] = true;
		}
	}
	memcpy(f->m_p, f->m, Mframe * sizeof(bool));

}

void tdamp_to_frame_corpeak(struct fras* f, float** tdamp, int Nch, long m) {
	int ch;
	if (f->m[m]) {
		f->m_weight_corpeak[m] = 1.0f;
		for (ch = 0; ch < Nch; ch++) {
			if (ch == A_ll2 || ch == A_rr2 || ch == A_ff2) {
				f->m_weight_corpeak[m] *= tdamp[ch][m];
				if (tdamp[ch][m] < 0.0f) {
					f->m_weight_corpeak[m] = 0.0f;
				}
			}
		}
	}
	else {
		f->m_weight_corpeak[m] = 0.0f; // inactive when iteration inside frames is only over active frames
	}
}

void el_to_frame_el(struct fras* f, struct heads* h, long m) {
	float el_horizontal;
	float el_weight;
	if (f->m[m]) {
		el_horizontal = h->pos[m][A_dim[A_el]][A_ref] - 90.0f;
		el_weight = 1.0f - fabsf(el_horizontal / 90.0f);
		f->m_weight_elevation[m] = el_weight;
	}
	else {
		f->m_weight_elevation[m] = 0.0f; // inactive when iteration inside frames is only over active frames
	}
}

void tdamp_to_frame_corpeak_normalize(struct fras* f, long Mframe) {
	for (long m = 0; m < Mframe; m++) { // necessary as iteration inside frames is only over active frames
		if (!f->m[m]) {
			f->m_weight_corpeak[m] = 0.0f;
		}
	}
	ar_norm(f->m_weight_corpeak, Mframe, f->m_weight_corpeak);
}

void Nstart_to_frame_xyz(struct fras* f, struct heads* h, struct par6s* p, long Nstartm, long Nstart0, long Nstart, long Nlatency, bool sync, long m, long sample_rate, float speed_of_sound) {
	if (f->m[m]) {
		// Background: Lawrence - A catalog of special plane curves (1972) p. 75
		// http://mathworld.wolfram.com/EccentricAnomaly.html

		// init
		float rd_m;
		float az_m;
		float el_m;
		float az_mfirst;
		float el_mfirst;
		float az_diff_rad;
		float el_diff_rad;
		float tmp;
		float c; // known: radius to direct sound
		float ab; // known: radius from direct sound to reflection (b) plus from reflection to receiver (c)
		float b; // searched: Radius from reflection to receiver
		float ab_2; // elipse: semi major axis of elipse (ab/2, in wiki: a)
		//float g; // elipse: semi minor axis "height" of elipse (sqrtf(ab_2^2 - a_2^2))
		float c_2; // elipse: linear eccentricity "width" of elipse (wiki: c)
		float e; // elipse: eccentricity (c_2 / ab_2 = sqrtf(1-(g/a_2)^2))

		// parameters
		// Get azimuth and elevation angle of reflection
		az_m = h->pos[m][A_dim[A_az]][A_ref]; // p->obs[m].kal[A_dim[A_az]].y[A_ref];
		el_m = h->pos[m][A_dim[A_el]][A_ref]; // p->obs[m].kal[A_dim[A_el]].y[A_ref];
		az_mfirst = h->pos[f->m_energyfirst][A_dim[A_az]][A_ref]; // p->obs[m].kal[A_dim[A_az]].y[A_ref];
		el_mfirst = h->pos[f->m_energyfirst][A_dim[A_el]][A_ref]; // p->obs[m].kal[A_dim[A_el]].y[A_ref];
		az_diff_rad = deg_to_rad(az_mfirst - az_m);
		el_diff_rad = deg_to_rad(el_mfirst - el_m);

		// assign known radi
		ab = N_to_m(Nstartm - (Nstart0), sample_rate, speed_of_sound);
		c = N_to_m(f->m_Npeak[f->m_energyfirst] - (Nstart0), sample_rate, speed_of_sound);

		// calculate elipse parametrization
		if (ab == c) {
			b = c; // true anomaly formula does not handle degeneration of elipse to line
		}
		else {
			c_2 = c / 2.0f;
			ab_2 = ab / 2.0f;
			e = c_2 / ab_2; // e = 0 (circle, c_2 = g) -> r = ab_2 (if loudspeaker on body), maximal e = 1 (no height ellipse g = 0)-> r = 0 (if frame same N as first N)
			b = ab_2 * (1.0f - e * e); // g / ab_2
			tmp = cosf(az_diff_rad) * cosf(el_diff_rad);
			if (tmp > 0.0f) {
				b /= 1.0f - e * tmp; // - version is used with focal point on negative x axis
			}
			else {
				b /= 1.0f + e * tmp;
			}
		}
		rd_m = b;

		// assign to head observation
		bool stationary_at_head_movement = false;
		if (stationary_at_head_movement) {
			if (m != f->m_energyfirst) {
				az_m -= p->pos[A_dim[A_az]][A_ref];
				el_m -= p->pos[A_dim[A_el]][A_ref];
			}
		}
		sph_to_cart(az_m, el_m, rd_m, &h->pos[m][A_x][A_ref], &h->pos[m][A_y][A_ref], &h->pos[m][A_z][A_ref]);
		h->toa[m][A_toaframe] = ab;
		h->toa[m][A_toaspeaker] = c;
		// correction for the case when smartphone was not at x,y,z 0 position at start
		//if (rd < 0) {
			//f->m_pos[m][A_z] = - f->m_pos[m][A_z];
		//}
	}
}
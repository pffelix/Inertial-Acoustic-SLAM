// Copyright 2020, Felix Pfreundtner, All rights reserved.

#include "signals.h"

void mono_normalize(float *x, long x_N) {
	float x_max = x[ar_absmaxn(x, x_N)];
	for (long n = 0; n < x_N; n++) {
		x[n] = x[n] / x_max;
	}
};


void binaural_normalize(float *x1, float *x2, long x_N) {
	float x_max = maximum(fabsf(x1[0]), fabsf(x2[0]));
	float x;
	for (long n = 1; n < x_N; n++) {
		x = maximum(fabsf(x1[n]), fabsf(x2[n]));
		if (x > x_max) {
            x_max = x;
		}
	}
	for (long n = 0; n < x_N; n++) {
		x1[n] = x1[n] / x_max;
		x2[n] = x2[n] / x_max;
	}
};

void binaural_maximum(float* x1, float* x2, long x_N, long* max_N, float* max_amp) {
	// initialize
	long max_N1 = 0;
	long max_N2 = 0;
	float max_amp1 = 0.0f;
	float max_amp2 = 0.0f;

	// calculate
	// find peak in binaural matched filter
	ar_max(x1, x_N, &max_N1, &max_amp1);
	ar_max(x2, x_N, &max_N2, &max_amp2);

	// take peak position that is stronger (closer to source)
	*max_N = (max_amp1 > max_amp2) ? max_N1 : max_N2;
	*max_amp = (max_amp1 > max_amp2) ? max_amp1 : max_amp2;

}

void multichannel_normalize(float ** x, long x_N, int Nchannels) {
	float x_max = absmaximum_ptr(x, x_N, Nchannels);
	for (int m = 0; m < Nchannels; m++) {
		for (long n = 0; n < x_N; n++) {
			x[m][n] = x[m][n]/x_max;
		}
	}
}

float multichannel_normalize_norm(float ** x, long x_N, int Nchannels) {
	float x_max = absmaximum_ptr(x, x_N, Nchannels);
	for (int m = 0; m < Nchannels; m++) {
		for (long n = 0; n < x_N; n++) {
			x[m][n] = x[m][n]/x_max;
		}
	}
	return x_max;
}

void get_average_energy_per_sample(float* x, long x_N, float * energy) {
	*energy = ar_sumabs(x, x_N);
	*energy = *energy / x_N;
}

void get_sample_gain(float x1, float x2, float *gain_db) {
	float gain_abs;
	gain_abs = (x1 * x1) / (x2 * x2);
	*gain_db = indb(gain_abs);
}
void get_array_gain(float* x1, float* x2, long x_N, float *gain_db) {
	float x1_energy;
	float x2_energy;
	x1_energy = ar_sumsquared(x1, x_N);
	x2_energy = ar_sumsquared(x2, x_N);
	get_sample_gain(sqrtf(x1_energy), sqrtf(x2_energy), gain_db);
}


void square(float *x, long x_N) {
	ar_square(x, x_N);
}


void window(float *x, long x_N, char * window_type) {
	float weight = 1.0f;
	if (!strcmp(window_type, "hann")) {
		for (long n = 0; n < x_N; n++) {
			// apply hann window
			weight = 0.5f * (1.0f - (float)cosf(2.0f * PI * (float)n / ((float)x_N - 1.0f)));
			x[n] = weight * x[n]; 
		}
	}
}

void flip(float* x, long x_N) {
	float tmp;
	for (long n = 0; n < x_N / 2; n++) {
		tmp = x[n];
		x[n] = x[x_N - 1 - n];
		x[x_N - 1 - n] = tmp;
	}
}

void shift_linear(float* x, long x_N, long x_Nshift) {
	// right shift
	if (x_Nshift > 0 && x_Nshift < x_N) {
		for (long n = x_N; n >= x_Nshift; n--) {
			x[n] = x[n - x_Nshift];
		}
		for (long n = x_Nshift - 1; n >= 0; n--) {
			x[n] = 0.0f;
		}
	}
	// left shift
	if (x_Nshift < 0 && -x_Nshift < x_N) {
		x_Nshift = -x_Nshift;
		for (long n = 0; n < x_N - x_Nshift; n++) {
			x[n] = x[n + x_Nshift];
		}
		for (long n = x_N - x_Nshift; n < x_N; n++) {
			x[n] = 0.0f;
		}
	}
}

void shift_circular(float* x, long x_N , long x_Nshift) {
	// x_Nshift shift smaller than x_N, assuming periodicity of signal
	x_Nshift = modulus(x_Nshift, x_N); // x_Nshift now always bigger than 0 (if x_N > 0)
	if (x_Nshift > 0 && x_Nshift < x_N) {
		long x_nsmall; // start index of smaller block
		long x_Nsmall; // lenght of smaller block
		long x_nlarge; // start index of larger block
		long x_Nlarge; // length of larger block
		long x_Nremain; // remaining rightsided samples after shift index
		long x_Nlarge_lead; // first part of larger block ( stays at end of new signal)
		long x_Nlarge_trail; // second part of larger block (moves to begging of new signal)

			// check which block, of th signal x splitted at x_Nshift, is larger and save index n of block start
			x_Nremain = x_N - x_Nshift;

			// small shift: right sided signal is larger than left sided signal -> right sided signal will be cutted over edge of signal
			if (x_Nshift <= x_Nremain) {
				x_nlarge = x_Nshift;
				x_Nlarge = x_Nremain;
				x_nsmall = 0;
				x_Nsmall = x_Nshift;
				x_Nlarge_lead = x_Nlarge - x_Nshift; // x_N - x_Nshift - x_Nshift (larger than trail block when x_Nshift < X_N/3, else smaller)
				x_Nlarge_trail = x_Nlarge - x_Nlarge_lead;

			}
			// large shift: left sided signal is larger than right sided signal -> left sided signal will be cutted over edge of signal
			else {
				x_nlarge = 0;
				x_Nlarge = x_Nshift;
				x_nsmall = x_Nshift;
				x_Nsmall = x_Nremain;
				x_Nlarge_lead = x_N - x_Nlarge; // x_N - x_Nshift (always larger than trail block)
				x_Nlarge_trail = x_Nlarge - x_Nlarge_lead;

			}
			float* x_tmp = (float*)malloc(x_Nsmall * sizeof(float)); // temporary array for smaller block
			if (x_tmp) {
				// copy memory of smaller block to temporary array
				memcpy(x_tmp, x + x_nsmall, x_Nsmall * sizeof(float));

				// copy memory of larger block to existing array at new position
				if (x_Nshift <= x_Nremain) {
					// start of new signal (overwritten)
					memcpy(x, x + x_nlarge + x_Nlarge_lead, x_Nlarge_trail * sizeof(float));

					// end of new signal
					memcpy(x + x_Nshift + x_nlarge, x + x_nlarge, x_Nlarge_lead * sizeof(float));
				}
				else{
					// end of new signal (overwritten)
					memcpy(x + x_Nshift + x_nlarge, x + x_nlarge, x_Nlarge_lead * sizeof(float));

					// start of new signal
					memcpy(x, x + x_nlarge + x_Nlarge_lead, x_Nlarge_trail * sizeof(float));
				}

				// copy temporary memory of smaller block to existing array at new position
				// middle of new signal
				memcpy(x + x_Nlarge_trail, x_tmp, x_Nsmall * sizeof(float));

				free(x_tmp);
			}

	}
	//// left sided signal is larger than right sided signal -> left sided signal will be cutted over edge of signal
	//if (x_Nshift > x_Nremain) {

	//	// end of new signal
	//	memcpy(x + x_Nshift + x_nlarge, x + x_nlarge, x_Nlarge_lead * sizeof(float));
	//	// start of new signal
	//	memcpy(x, x + x_Nlarge_lead, x_Nlarge_trail * sizeof(float));
	//	// middle of new signal
	//	memcpy(x + x_Nlarge_trail, x_tmp, x_Nsmall * sizeof(float));
	//}
	//// right sided signal is larger than left sided signal -> right sided signal will be cutted over edge of signal
	//else {

	//	// end of new signal
	//	//memcpy(x + x_Nshift + x_nlarge, x + x_nlarge, x_endN * sizeof(float));
	//	// start of new signal
	//	//memcpy(x, x + x_endN, (x_Nlarge - x_endN) * sizeof(float));
	//	// middle of new signal
	//	//memcpy(x + (x_Nlarge - x_endN), x_tmp, x_Nsmall * sizeof(float));
	//};


	//long y_nEnd = x_nlarge + x_Nlarge + x_Nshift;
	//memcpy(x + x_Nshift, x + x_nsmall, x_Nsmall * sizeof(float));
	//if (x_nEnd > x_N) {

	//}
	//memcpy(x+, x + x_nsmall, x_Nsmall * sizeof(float));

}

void linupsample(float *x, long x_N, float* weight, long Nup, float *y) {
	// setup up linear weigths
	//float *weight;
	//long timer_start = clock_tick(); // 3us for allocating weight array with values
	//weight = ar_declare(Nup);
	//if (weight) {
		//for (long nup = 0; nup < Nup; nup++) {
			//weight[nup] = 1 - ((float) nup) / Nup;
		//};
	//double timer_diff = clock_stop(timer_start);

	// calculate linear interpolation
	long k = 0;
	long n = 0;
	long nup = 0;
	for (n = 0; n < x_N - 1; n++) {
		for (nup = 0; nup < Nup; nup++) {
			y[k] = x[n] * weight[nup] + x[n+1] * (1-weight[nup]);
			k++;
		}
	}
	y[k] = x[x_N - 1];
	//free(weight);
	//}
};

void linupsample2d(float** x,  long x_N, long x_M, float* weight, long Nup, float** y) {
	// setup up linear weigths
	//float* weight;
	weight = ar_declare(Nup);
	//if (weight) {
		//for (long nup = 0; nup < Nup; nup++) {
			//weight[nup] = 1 - ((float)nup) / Nup;
		//};

	// calculate linear interpolation
	for (long m = 0; m < x_M; m++) {
		long k = 0;
		for (long n = 0; n < x_N; n++) {
			for (long nup = 0; nup < Nup; nup++) {
				if (n == x_N - 1) {
					y[m][n] = x[m][n] * weight[nup];
				}
				else {
					y[m][n] = x[m][n] * weight[nup] + x[m][n + 1] * (1 - weight[nup]);
				}
				k++;
			}
		}
	}
		//free(weight);
	//}
};



void fade_in(float* x, long x_N, char* set_fade_window, float fade_percent, bool unfade) {
	long fade_N = (long)(fade_percent * (float)x_N);
	if (strcmp(set_fade_window, "none") && fade_percent > 0.0f && fade_N > 1) {
		// init
		float* w = ar_declare(fade_N);
		long n_start = 0l;
		if (w) {
			// calculate window
			if (strcmp(set_fade_window, "none") == 0) {
				return;
			}

			if (strcmp(set_fade_window, "hann") == 0) {
				for (long n = 0; n < fade_N; n++) {
					w[n] = 0.5f * (1.0f - cosf(2.0f * PI * (float)n / ((float)(fade_N)-1.0f) / 2.0f));
				}
			}

			if (strcmp(set_fade_window, "blackman") == 0) {
				for (long n = 0; n < fade_N; n++) {
					w[n] = 0.42f - 0.5f * cosf(2.0f * PI * (float)n / ((float)(fade_N)-1.0f) / 2.0f) + 0.08f * cosf(4.0f * PI / ((float)(fade_N)-1.0f) / 2.0f);
				}
			}

			if (strcmp(set_fade_window, "linear") == 0) {
				float fade_t_step = 1.0f / ((float)(fade_N)-1.0f);
				w[0] = 0.0f;
				for (long n = 1; n < fade_N; n++)
				{
					w[n] = w[0] + fade_t_step * (float)n;

				};
			}


			// calculate signal
			if (unfade) {
				x[n_start] = 0.0f;
				for (long n = 1; n < fade_N; n++) {
					x[n + n_start] = x[n + n_start] / w[n];
				}
			}
			else {
				for (long n = 0; n < fade_N; n++) {
					x[n + n_start] = x[n + n_start] * w[n];
				}
			}

			// free memory
			free(w);
		}
	}
};


void fade_out(float* x, long x_N, char* set_fade_window, float fade_percent, bool unfade) {
	long fade_N = (long)(fade_percent * (float)x_N);
	if (strcmp(set_fade_window, "none") && fade_percent > 0.0f && fade_N > 1) {
		// init
		float* w = ar_declare(fade_N);
		long n_start = x_N - fade_N;

		if (w) {
			// calculate window
			if (strcmp(set_fade_window, "hann") == 0) {
				for (long n = 0; n < fade_N; n++) {
					w[n] = 0.5f * (1.0f + cosf(2.0f * PI * (float)n / ((float)(fade_N)-1.0f) / 2.0f));
				}
			}

			if (strcmp(set_fade_window, "blackman") == 0) {
				for (long n = 0; n < fade_N; n++) {
					w[n] = 0.42f + 0.5f * cosf(2.0f * PI * (float)n / ((float)(fade_N)-1.0f) / 2.0f) + 0.08f * cosf(4.0f * PI / ((float)(fade_N)-1.0f) / 2.0f);
				}
			}

			if (strcmp(set_fade_window, "linear") == 0) {
				float fade_t_step = 1.0f / ((float)(fade_N)-1.0f);
				w[0] = 1.0f;
				for (long n = 1; n < fade_N; n++)
				{
					w[n] = w[0] - fade_t_step * (float)n;
				}
			}

			// calculate signal
			if (unfade) {
				x[fade_N - 1 + n_start] = 0.0f;
				for (long n = 0; n < fade_N - 1; n++) {
					x[n + n_start] = x[n + n_start] / w[n];
				}
			}
			else {
				for (long n = 0; n < fade_N; n++) {
					x[n + n_start] = x[n + n_start] * w[n];
				}
			}

			// free memory
			free(w);
			//write_txt_debug(x,  x_N, 1);
			//write_txt_debug(x,  x_N, 1);
		}
	}
};

void lowpass(float* x, long x_N, enum TWindowType fir_type, long fir_N, float fir_alpha, long fir_fcut, long fir_fs, struct ffts* fft, char* fft_library) {
	// init
	long fir_Nshift;
	float* filter;

	//calculate filter params
	// make filter symetric
	if (modulus(fir_N, 2) == 1) {
		fir_N = fir_N + 1;
	}
	fir_Nshift = (fir_N - 1) / 2;

	//calculate filter
	filter = ar_declare(fir_N);
	if (filter) {
		basic_fir(filter, fir_N, LPF, fir_fcut / ((float)fir_fs / 2.0f), 0.0f, fir_type, fir_alpha);

		// convolve filter with signal and cut back to original signal length
		convolve_fft(x, x_N, filter, fir_N, x, x_N, fir_Nshift, fft, fft_library, "filter");

		free(filter);
	}
}

void highpass(float* x, long x_N, enum TWindowType fir_type, long fir_N, float fir_alpha, long fir_fcut, long fir_fs, struct ffts* fft, char* fft_library) {
	// init
	long fir_Nshift;
	float* filter;

	//calculate filter params
	// make filter symetric
	if (modulus(fir_N, 2) == 1) {
		fir_N = fir_N + 1;
	}
	fir_Nshift = (fir_N - 1) / 2;

	//calculate filter
	filter = ar_declare(fir_N);
	if (filter) {
		basic_fir(filter, fir_N, HPF, fir_fcut / ((float)fir_fs / 2.0f), 0.0f, fir_type, fir_alpha);

		// convolve filter with signal and cut back to original signal length
		convolve_fft(x, x_N, filter, fir_N, x, x_N, fir_Nshift, fft, fft_library, "filter");

		free(filter);
	}
}


void bandpass(float* x, long x_N, enum TWindowType fir_type, long fir_N, float fir_alpha, long fir_f1, long fir_f2, long fir_fs, struct ffts* fft, char* fft_library) {
	// init
	long fir_Nshift;
	float* filter;
	float fir_fcut;
	float fir_fwidth;

	//calculate filter params
	// make filter symetric
	if (modulus(fir_N, 2) == 1) {
		fir_N = fir_N + 1;
	}
	fir_Nshift = (fir_N - 1) / 2;
	fir_fcut = (((float)fir_f2 + (float)fir_f1) / 2.0f) / ((float)fir_fs / 2.0f);
	fir_fwidth = ((float)fir_f2 - (float)fir_f1) / ((float)fir_fs / 2.0f);

	//calculate filter
	filter = ar_declare(fir_N);
	if (filter) {
		basic_fir(filter, fir_N, BPF, fir_fcut, fir_fwidth, fir_type, fir_alpha);

		// convolve filter with signal and cut back to original signal length
		convolve_fft(x, x_N, filter, fir_N, x, x_N, fir_Nshift, fft, fft_library, "filter");

		free(filter);
	}
}


void inverse_kirkeby(float *x, long x_N, float *x_inv, long xinv_N, const char* xtype, const char* xinvtype, long fs, long flow, long fhigh, struct ffts* fft, char * fft_library){
	// reference papers:
	// Farina - Advancements in impulse response measurements by sine sweeps (2000)
	// Kirkeby, Nelson - Digital Filter Design for Inversion Problems in sound reproeduction (1999)

	// parameters
	
	// "kirkeby-bandlimit
	float eps_min = 0.0f; // maximal regularization
	float eps_max = 10000.0f; // minimal regularization
	long eps_ftransition = 100; // bandwith in Hz of transition band between maximal and minimal regularization (hann window)
	
	// "kirkeby-flat
	long Mleft = 1000;
	long Mright = 10;
	float eps_min_weight = 0.0001f;
	float eps_max_weight = 0.38f;

	// state
	long X_N = xinv_N;
	long X_Ncpx = 2 * X_N;
	float* X = ar_zeros(X_Ncpx); // complex spectrum
	float* eps = ar_declare(X_N); // regularization parameter epsilon

	// temporary variables regularization
	long N_low;
	long N_high;
	long N_transition;
	long N_lowtransition;
	long N_hightransition;
	float fade_scale_factor = (eps_max - eps_min) / eps_max;

	// temporary variables inversion
	float X_imag_conj;
	float tmp_real;
	float tmp_imag;
	float C_real;
	float C_imag;
	float zero = 0.0f;
	long nreal;
	long nimag;
	long n;
	long neps;
	float x_max;

	if (X && eps) 
	{
		// normalize
		x_max = x[ar_absmaxn(x, x_N)];
		if (x_max != 1.0f) {
			for (n = 0; n < x_N; n++) {
				x[n] = x[n] / x_max;
			}
		}

		// copy time signal into frequency domain array
		memcpy(X, x, x_N * sizeof(float));

		// perform FFT of impulse signal
		fast_fft(X, X_N, fft, 1, fft_library);

		// generate regularization

		// Apply Regualrizer only to FFT bins where impulse signal bandwith is located (with smoothing hann window)
		if (!strcmp(xinvtype, "kirkeby-bandlimit")) {
			// init
			N_low = ((long)floorf((float)flow / (float)fs * X_N) + 1); // +1 as DC offset
			N_high = ((long)ceilf((float)fhigh / (float)fs * X_N) + 1);
			N_transition = ((long)ceilf((float)eps_ftransition / (float)fs * X_N));
			N_lowtransition = (N_low - 1 >= N_transition) ? N_transition : N_low - 1; // +1 as DC offset
			N_hightransition = (X_N - N_high > N_transition) ? N_transition : X_N - N_high;

			// set regularizer to maximal damping
			eps[0] = 0.0f; // regularization parameter epsilon at DC offset
			for (n = 1; n < X_N; n++) {
				eps[n] = eps_max;
			}
			// calculate regularizer gain for every bin of left sided spectrum
			if (N_low + 2 <= N_high) // if bandwith is existant ( + 2 as we have complex array (real and imaginary value))
			{

				// left transition band
				fade_out(eps + N_low - N_lowtransition, N_lowtransition, "hann", 1.0f, false);
				for (n = N_low - N_lowtransition; n < N_low; n++)
				{
					eps[n] = eps[n] * fade_scale_factor + eps_min;
				}
				// pass band
				for (n = N_low; n < N_high; n++)
				{
					eps[n] = eps_min;
				}
				// righ transition band
				fade_in(eps + N_high, N_hightransition, "hann", 1.0f, false);
				for (n = N_high; n < N_high + N_hightransition; n++)
				{
					eps[n] = eps[n] * fade_scale_factor + eps_min;
				}
				// set regularizer gain for every bin of right sided spectrum
				for (n = X_N / 2 + 1; n < X_N; n++) {
					eps[n] = eps[X_N - n];
				}
			}
		}

		// Apply regularizor to FFT bins with strongest signal energy
		if (!strcmp(xinvtype, "kirkeby")) {
			// init
			long NXflat = X_N / 2;
			float* Xflat;
			float* Xflatmeanleft;
			float* Xflatmeanright;
			if (!strcmp(xtype, "exponential")) {
				Xflat = ar_linearsquareroot_int(1.0f, 1, NXflat);
			}
			else {
				Xflat = ar_value(NXflat, 1.0f);
			}
			Xflatmeanleft = ar_declare(NXflat);
			Xflatmeanright = ar_declare(NXflat);
			if (Xflat && Xflatmeanleft && Xflatmeanright) {
				float mflat;
				long mflatN;
				float eps_minimum;
				float eps_n;
				float magnitude;
				long n2;
				for (n = 0; n < NXflat; n++) {
					n2 = 2 * (n + 1);  // + 2 as DC offset complex value (and left sided spectrum X_N long, 1 bin longer than right siged specturm X_N - 2 complex bins long)
					nreal = n2;
					nimag = nreal + 1;
					magnitude = cpx_abs(&X[nreal], &X[nimag]);
					Xflat[n] = Xflat[n] * 1.0f;
				}
				movingmean_double(Xflat, NXflat, Xflatmeanleft, Mleft, Mright);
				movingmean_double(Xflat, NXflat, Xflatmeanright, Mright, Mleft);
				for (n = 0; n < NXflat; n++) {
					if(Xflatmeanleft < Xflatmeanright){
						Xflat[n] = Xflatmeanleft[n];
					}
					else {
						Xflat[n] = Xflatmeanright[n];
					}
				}
				ar_max(Xflat, NXflat, &mflatN, &mflat);
				eps_minimum = eps_min_weight * mflat;

				// set regularizer to minimal damping
				eps[0] = 0.0f; // regularization parameter epsilon at DC offset
				for (n = 1; n < X_N; n++) {
					eps[n] = eps_minimum;
				}
				// calculate regularizer gain for every bin of left sided spectrum
				for (n = 1; n <= X_N / 2 + 1; n++) {
					eps_n = eps_max_weight * mflat - Xflat[n - 1];
					if (eps_n <= eps_minimum) {
						eps[n] = eps_minimum;
					}
					else {
						eps[n] = eps_n;
					}
				}

				// set regularizer gain for every bin of right sided spectrum
				for (n = X_N / 2 + 1; n < X_N; n++) {
					eps[n] = eps[X_N - n];
				}
				free(Xflat);
				free(Xflatmeanleft);
				free(Xflatmeanright);
			}
		}

		//write_txt_debug(eps, X_N, 0);
		//write_txt_debug(X, X_Ncpx, 0);

		// apply Kirkeby inverse filter
		for (n = 0; n < X_N + 2; n += 2) // + 2 as DC offset (with that left sided spectrum 1 bin longer) and complex values
		{
			nreal = n;
			nimag = nreal + 1;
			neps = n / 2;
			X_imag_conj = cpx_conj(&X[nimag]);
			cpx_multiply(&X[nreal], &X_imag_conj, &X[nreal], &X[nimag], &tmp_real, &tmp_imag);
			cpx_add(&tmp_real, &tmp_imag, &eps[neps], &zero, &tmp_real, &tmp_imag);
			cpx_divide(&X[nreal], &X_imag_conj, &tmp_real, &tmp_imag, &C_real, &C_imag);
			X[nreal] = C_real;
			X[nimag] = C_imag;
		}

		// perform IFFT

		fast_fft(X, X_N, fft, -1, fft_library);
		//write_txt_debug(X, X_N, 1);

		// write back spectrum to time signal, no cutting leading trailing zeros (as inverse) and adding shift of signal as inverse rings
		memcpy(x_inv, X, X_N * sizeof(float));

		// free memory
		free(X);
		free(eps);

		// debug
		//write_txt_debug(x_inv, X_N, 1);

		// denormalize (as inverse filter, division instead of multiplication)
		if (x_max != 1.0f) {
			for (n = 0; n < x_N; n++) {
				x_inv[n] = x_inv[n] / x_max;
			}
		}
	}
}

void movingmean(float* x, long x_N, float* y, long Mleft, long Mright) {
	long Mall;
	long M;
	long m;
	long n;
	long nm;
	Mall = Mleft + 1 + Mright;
	for (n = 0; n < x_N; n++) {
		M = Mall;
		y[n] = 0.0f;
		for (m = -Mleft; m <= Mright; m++) {
			nm = n + m;
			if (nm >= 0 && nm < x_N) {
				y[n] += x[nm];
			}
			else {
				M -= 1;
			}
		}
		y[n] = y[n] / (float)M;
	}
}

void movingmean_double(float* x, long x_N, float* y, long Mleft, long Mright) {
	long Mall;
	long M;
	long m;
	long n;
	long nm;
	double* y_double =  ar_declare_double(x_N);

	Mall = Mleft + 1 + Mright;
	for (n = 0; n < x_N; n++) {
		M = Mall;
		y_double[n] = 0.0f;
		for (m = -Mleft; m <= Mright; m++) {
			nm = n + m;
			if (nm >= 0 && nm < x_N) {
				y_double[n] += (double)x[nm];
			}
			else {
				M -= 1;
			}
		}
		y_double[n] = y_double[n]  / (double)M;
	}
	for (n = 0; n < x_N; n++) {
		y[n] = (float)y_double[n];
	}
	free(y_double);

}

void correlate_positive(float *x1, float *x2, long x_N, float *y) {
	long k, n;
	long N = 2 * x_N - 1;
	for (k = 0; k < N; k++){
        y[k] = 0;
        for (n = 0; n < x_N - k; n++) {
            y[k] += x1[n + k] * x2[n] / x_N;
        }
    }			
};

void crosscorrelate(float* x1, float* x2, long x_N, float* y, long maxdelayN, char * shift) {
	// adapted from: http://paulbourke.net/miscellaneous/correlate/
	long n, j, delayN;
	float mx1, mx2, sx, sy, sxy, denom, r;

	/* Calculate the mean of the two series x[], y[] */
	mx1 = 0;
	mx2 = 0;
	for (n = 0; n < x_N; n++) {
		mx1 += x1[n];
		mx2 += x2[n];
	}
	mx1 /= x_N;
	mx2 /= x_N;

	/* Calculate the denominator */
	sx = 0;
	sy = 0;
	// To do: maybe x_N not correct here but instaed -maxdelayN, maxdelayN (this would assume signal only goes to delay)
	for (n = 0; n < x_N; n++) {
		sx += (x1[n] - mx1) * (x1[n] - mx1);
		sy += (x2[n] - mx2) * (x2[n] - mx2);
	}
	denom = sqrtf(sx * sy);

	// Shift signal if needed
	long shiftN = 0;
	if (!strcmp(shift, "delay")) {
		shiftN = maxdelayN;
	}
	if (!strcmp(shift, "middle")) {
		shiftN = (long)ceilf((float)x_N / 2.0f);
	}
	// check if no shift would be better (shift in case of tie), best shift is x_N2 -> penality if strong deviation from it
	//if(abs_long(x_N2 - shiftN) > (long)floorf((float)maxdelayN/2)) {
		//shiftN = 0;
	//}
	if(x_N - shiftN <= (long)ceilf((float)maxdelayN / 2.0f)) {
		shiftN = 0;
	}

	/* Calculate the correlation series */
	for (delayN = -maxdelayN; delayN <= maxdelayN; delayN++) {
		sxy = 0;
		// To do: maybe start at n + delayN, than more values to correlate
		for (n = 0 + shiftN; n < x_N; n++) {
			j = n + delayN;
			if (j < 0 || j >= x_N)
				continue;
			else
				sxy += (x1[n] - mx1) * (x2[j] - mx2);
			/* Or should it be (?): assumed the signal is 0 around existing signal
			if (j < 0 || j >= n)
			   sxy += (x1[n] - mx1) * (-mx2);
			else
			   sxy += (x1[n] - mx1) * (x2[j] - mx2);
			*/
		}
		r = sxy / denom;
		
		/* r is the correlation coefficient at "delayN" */
		y[maxdelayN -delayN] = r;
	}
	//write_txt_debug(x1,  x_N, 0);
	//write_txt_debug(x2,  x_N, 0);
	//write_txt_debug(y,  maxdelayN * 2 + 1, 0);
}

void crosscorrelate_maxn(float *y, long y_N, long N_lim, long *max_N, float *max_amp) {
    long N_0delay = ((long)floorf(((float)y_N) / 2.0f));
	long N_Startdelay = N_0delay - N_lim;
	if (N_Startdelay >= 0) {
		long N = 2 * N_lim + 1;
		ar_max(y + N_Startdelay, N, max_N, max_amp);
		//*max_N = ar_maxn(y + n_start, N);
		*max_N = N_Startdelay + *max_N - N_0delay;
	}
	else {
		*max_N = 0;
		*max_amp = 0.0f;
        console_logi(PRIO_WARNINGDEACTIVATED, "Frame length to small for set cross-correlation delay limit, set time difference maximum to 0 seconds delay %i\n", abs((int)N_Startdelay));
	}
};

void crosscorrelate_maxntotd(long maxn, long samplingrate, float *td) {
	*td = ((float)maxn) / samplingrate;
};

void convolve2(float* x, long x_N, float *h, long h_N, float *y)
{
	long n, m;
	long N = x_N + h_N;
	for (n = 0; n < N; n++)
		for (n = 0, m = maximum_long(0, n - x_N + 1); m <= minimum_long(n, h_N); m++){
			y[n] += h[m] * x[n - m];
		}
}

void convolve(float *x, long x_N, float *h, long h_N, float *y)
{
    // init
    long N = x_N+h_N-1;
    long i,j,h_start,x_start,x_end;

    // calculate
    for (i=0; i<N; i++)
    {
        x_start = MAX(0,i-h_N+1);
        x_end   = MIN(i+1,x_N);
        h_start = MIN(i,h_N-1);
        for(j=x_start; j<x_end; j++)
        {
            y[i] += h[h_start--]*x[j]; // exception at 1s signal and 2s recording
        }
    }
}


//void convolve_intelmkl_fft(float *x, long x_N, float *h, long h_N, float *y, char* fft_library, bool cpxconjugate)
//{
//	// Find minimal possible zeropadded length for fast convolution
//	long y_N = x_N + h_N - 1; 
//
//	// Find length with smallest power of 2 for faster computation of FFT
//	long Y_N = (long)pow(2, ceil(log(y_N)/log(2))); 
//
//	//write_txt_debug(x,  x_N);
//	//write_txt_debug(h,  h_N);
//
//	// allocate complex signals
//	long Y_Ncpx = y_N * 2;
//	float* X = (float*)malloc(Y_Ncpx * sizeof(float));
//	float* H = (float*)malloc(Y_Ncpx * sizeof(float));
//	float* Y = (float*)malloc(Y_Ncpx * sizeof(float));
//
//	struct cpxs *X = (struct cpxs*)malloc(Y_Ncpx * sizeof(struct cpxs));
//	struct cpxs *H = (struct cpxs*)malloc(Y_Ncpx * sizeof(struct cpxs));
//	X->real = (float*)malloc(Y_Ncpx);
//	X->imag = (float*)malloc(Y_Ncpx);
//	H->real = (float*)malloc(Y_Ncpx);
//	H->imag = (float*)malloc(Y_Ncpx);
//
//	// zeropad signals
//	memset(X+, 0, sizeof(mx2array)); 
//	memset(H, 0, sizeof(mx2array)); 
//
//	struct cpx *H = (struct cpx*)malloc(Y_N * sizeof(struct cpx));
//	for (long n = 0; n < Y_N; n++) {
//		if (n < x_N) {
//			X[n].real = x[n];
//			X[n].imag = 0.0f;
//		}
//		else {
//			X[n].real = 0.0f;
//			X[n].imag = 0.0f;
//		}
//		if (n < h_N) {
//			H[n].real = h[n];
//			H[n].imag = 0.0f;
//
//		}
//		else {
//			H[n].real = 0.0f;
//			H[n].imag = 0.0f;
//		}
//	}
//
//	// perform FFT
//	fft(H, Y_N, 1, fft_library);
//	fft(X, Y_N, 1, fft_library);
//
//	// multiply FFT bins (could be made more performant by considering symetric structure of FFT)
//	struct cpx *Y = (struct cpx*)malloc(Y_N * sizeof(struct cpx));
//	for (long n = 0; n < Y_N; n++) {
//		Y[n].real = X[n].real * H[n].real - X[n].imag * H[n].imag;
//		Y[n].imag = X[n].real * H[n].imag + X[n].imag * H[n].real;
//	}
//	// perform inverse FFT
//	fft(Y, Y_N, -1, fft_library);
//
//	// remove trailing zeros and produce output and divide by fourier transformation scaling factor
//	for (long n = 0; n < y_N; n++) {
//		y[n] = Y[n].real/Y_N;
//	}
//
//	//write_txt_debug(y,  y_N);
//	//write_txt_debug_complex(Y, Y_N, 1);
//	//write_txt_debug_complex(H, Y_N, 1);
//	//write_txt_debug_complex(X, Y_N, 1);
//
//
//	// free memory
//	free(X);
//	free(H);
//}


void level_diff(float *x1, long x1_N, float *x2, long x2_N, float * leveldiff) {
	float x1_sum = ar_sumsquared(x1, x1_N);
	float x2_sum = ar_sumsquared(x2, x2_N);
	*leveldiff = 10 * (logf(x2_sum) - logf(x1_sum));
};



// aggregated mid level functions
void convolve_fft(float *x1, long x1_N, float *x2, long x2_N, float *y, long y_N, long y_Nshift, struct ffts *fft, char* fft_library, char * fft_type){
	bool fast_instead_of_full = true;
	if (fast_instead_of_full) {
		fast_fft_convolution(x1, x1_N, x2, x2_N, y, y_N, y_Nshift, fft, fft_library, fft_type, 0, 0, 0);
	}
	else {
		full_fft_convolution(x1, x1_N, x2, x2_N, y, fft_library);
	}
}


void crosscorrelate_fft(float *x1, long x1_N, float *x2, long x2_N, float *y, long y_N, struct ffts *fft, char* fft_library, char* cor_type, long fft_flim_low, long fft_flim_high, long sampling_rate, bool normalized){
	float x1_energy;
	float x2_energy;
	float x_energy;
	bool fast_instead_of_full = true;
	if (fast_instead_of_full) {
		fast_fft_crosscorrelation(x1, x1_N, x2, x2_N, y, y_N, fft, fft_library, cor_type, fft_flim_low, fft_flim_high, sampling_rate);
	}
	else {
		full_fft_crosscorrelation(x1, x1_N, x2, x2_N, y, fft_library);
	}
	if (normalized) {
		x1_energy = ar_sumsquared(x1, x1_N);
		x2_energy = ar_sumsquared(x2, x2_N);
		x_energy = sqrtf(x1_energy * x2_energy);
		if (x_energy != 0.0f) {
			ar_divide(y, y_N, x_energy);
		}
	}
}


// aggregated high level functions
void rec_to_recbeam(float* rec_1, float* rec_2, long rec_N, float steering_ang, float rec_distance_m, float sos, float fs, float * rec_beamformed) {
	float t_shift;
	float N_shift_precise;
	long N_shift;
	long N_shift_delay;

	// calculate shift for signal 2 needed to reach steering angle
	t_shift = 1.0f / sos * sinf(steering_ang / 360.0f * 2 * PI) * rec_distance_m;
	N_shift_precise = t_shift * fs;
	N_shift_delay = (long)roundf(N_shift_precise/2);
	N_shift = N_shift_delay * 2;
	//write_txt_debug(rec_2, rec_N, 1);

	// shift signal 2
	shift_linear(rec_2, rec_N, N_shift);

	// save signal 1 and 2 to beamformed signal 
	// if signal 2 was shifted to the right side
	for (long n = 0; n < rec_N; n++) {
		rec_beamformed[n] = (rec_1[n] + rec_2[n]) / 2.0f;
	}
	// apply delay to compensate shift of signal 2
	shift_linear(rec_beamformed, rec_N, -N_shift_delay);

	//if (N_shift_delay >= 0) {
	//	for (long n = N_shift_delay; n < rec_N; n++) {
	//		rec_beamformed[n - N_shift_delay] = (rec_1[n] + rec_2[n]) / 2.0f;
	//	}
	//	for (long n = 0; n < N_shift_delay; n++) {
	//		rec_beamformed[n] = 0.0f;
	//	}
	//}
	//// if signal 2 was shifted to the left side
	//else {
	//	for (long n = 0; n < rec_N + N_shift_delay; n++) {
	//		rec_beamformed[n - N_shift_delay] = (rec_1[n] + rec_2[n]) / 2.0f;
	//	}
	//	for (long n = rec_N + N_shift_delay; n < rec_N; n++) {
	//		rec_beamformed[n] = 0.0f;
	//	}
	//}
	//write_txt_debug(rec_beamformed, rec_N, 1);
};

void convolve_rec_to_ir(float *rec, long rec_N, float *is_inv, long isinv_N, float *ir, long ir_N, struct ffts *fft, char* fft_library) {
	convolve_fft(rec, rec_N, is_inv, isinv_N, ir, ir_N, 0, fft, fft_library, "ir");
};

void corf_to_tdmax(float* y, long y_N, long N_lim, long samplingrate, float *td, float *td_amp){
	long max_N = 0;
	crosscorrelate_maxn(y, y_N, N_lim, &max_N, td_amp);
	crosscorrelate_maxntotd(max_N, samplingrate, td);
};


void irf_to_ld(float *x_l, float *x_r, long x_N, float *ld) {
	float energy_l;
	float energy_r;
	energy_l = ar_sumsquared(x_l, x_N);
	energy_r = ar_sumsquared(x_r, x_N);
	*ld = indb(energy_r / energy_l);
	if (isnan(*ld)) {
		*ld = 0.0f;
	}

};

void ang_windowed_in_cor(float* cor, long cor_N, long cor_Nlim, float ang, float* window, long window_N_oneside, bool cor_merge) {
	long N_0delay;
	long N_ld;
	long N_ld_left;
	long N_ld_right;
	float stepsize;

	N_0delay = ((long)floorf(((float)cor_N) / 2.0f));
	ang_to_tdn(ang, cor_Nlim, &N_ld);
	//ang_to_tdn(ld_window_length_ang / 2.0f, cor_Nlim, &N_ld_window_length_oneside);
	N_ld = N_0delay - N_ld; // adapted, before + (seems like correlation is flipped)
	N_ld_left = N_ld - window_N_oneside;
	N_ld_right = N_ld + window_N_oneside;

	if (N_ld_left < 0) {
		N_ld_left = 0;
		console_log(PRIO_WARNING, "Warning: level difference windowed into correlation bigger than left correlation limit. Window only to left correlation limit.\n");
	}
	if (N_ld_right >= cor_N) {
		N_ld_right = cor_N - 1;
		console_log(PRIO_WARNING, "Warning: level difference windowed into correlation bigger than right correlation limit. Window only to rigth correlation limit.\n");

	}

	// zero signal part outside of window
	for (long n = 0; n < N_ld_left; n++) {
		cor[n] = 0.0f;
	}
	for (long n = N_ld_right; n < cor_N; n++) {
		cor[n] = 0.0f;
	}

	// apply window
 	if (window_N_oneside > 0) {
		// stepsize defining how much window bins needs to be jumped if ang is at edge of correlation limits
		stepsize = (float)window_N_oneside / (float)(N_ld - N_ld_left);

		// left of peak
		kaiserf_apply(cor + N_ld_left, window, window_N_oneside, -1, stepsize, cor_merge); //N_ld - N_ld_left
		//write_txt_debug(cor, cor_N, 1);

		// right of peak
		kaiserf_apply(cor + N_ld, window, window_N_oneside, 1, stepsize, cor_merge); //N_ld_right - N_ld
		//write_txt_debug(cor, cor_N, 1);
	}
}


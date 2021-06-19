// Copyright 2020, Felix Pfreundtner, All rights reserved.

#include "impulse.h"


void generate_impulse_double(double * is_send, long is_N, long fs, long is_f1, long is_f2, const char * is_type, double is_amp) { //double* is_inv

	// init
	double start = 0.0;
	double incr = 1.0 / ((double)fs);
	double* t = ar_linear_double(start, incr, is_N);
	double T = (double)is_N / (double)fs;
	double arg;
	double arg2;

	// calculate impulse
	// linear sine sweep (at 192khz and 0.1ms finishes at 0)
	if (strcmp(is_type, "linear") == 0) {
		// impulse

		for (long n = 0; n < is_N; n++) {
			// sine conversion: has to be in double otherwise hearable noise is produced
			//is_send[n] = sin(2.0 * PI_D * ((double)is_f1 + ((double)is_f2 - (double)is_f1) / (2.0 * T) * t[n]) * t[n]) * is_amp; 
			arg =  ((double)is_f1 + ((double)is_f2 - (double)is_f1) / (2.0 * (double)is_N / (double)n)) * ((double)n / (double)fs);
			arg2 = arg - floor(arg);
			is_send[n] = sin(arg2 * 2.0 * PI_D) * is_amp; 
			//if (n == isinv_N - 2) {
				//float a = 1;
			//}
		}
		// impulse inverse
		//for (long n = 0; n < is_N; n++) {
			//is_inv[is_N - 1 - n] = is_send[n];
		//}
	}
	// exponential sine sweep (at 192khz and 0.1ms finishes at 0)
	if (strcmp(is_type, "exponential") == 0) {
		double R = log(((double)is_f2) / (double)is_f1);
		// impulse
		for (long n = 0; n < is_N; n++) {
			// sine conversion: has to be in double otherwise hearable noise is produced
			arg = (double)is_f1 * T / R * (exp(t[n] * R / T) - 1.0);
			arg2 = arg - floor(arg);
			is_send[n] = sin(arg2 * 2.0 * PI_D) * is_amp; //fmod(arg, 2 * PI_D)
			//if (n > 19099) { // correct for 0.1ms 192khz 28khz-96khz pulse
				//is_send[n] = 0.0;
			//}
		}

		// impulse inverse
		//double k;
		//for (long n = 0; n < is_N; n++) {
			//k = exp(t[is_N - 1 - n] * R / T);
			//is_inv[is_N - 1 - n] = is_send[n] / k;
		//}
	}

	// free memory
	free(t);

	//write_txt_debug_double(is_send,  isinv_N);

};

void calculate_inverse_impulse(float* is_send, long is_N, float* is_inv, long isinv_N, char * is_type, const char* isinv_type, long fs, long is_f1, long is_f2, struct ffts* fft, char * fft_library) {
	

	if (!strcmp(is_type, "linear")) {
		// theoretical optimal inverse weighting (problem loudspeaker might not have flat freqeuency reponse)
		for (long n = 0; n < is_N; n++) {
			is_inv[is_N - 1 - n] = is_send[n];
		}
	}

	if (!strcmp(is_type, "exponential")) {

		// no inverse spectrum gain compensation to reach flat impulse response
		if (!strcmp(isinv_type, "no")) {
			for (long n = 0; n < is_N; n++) {
				is_inv[is_N - 1 - n] = is_send[n];
			}
		}
		// theoretical optimal inverse weighting (problem loudspeaker might not have flat freqeuency reponse)
		if (!strcmp(isinv_type, "analytical")) {
			float k;
			float start = 0.0;
			double incr = 1.0 / ((double)fs);
			float* t = ar_linear(start, incr, is_N);
			float R = logf(((float)is_f2) / (float)is_f1);
			float T = (float)is_N / (float)fs;

			for (long n = 0; n < is_N; n++) {
				k = expf(t[is_N - 1 - n] * R / T);
				is_inv[is_N - 1 - n] = is_send[n] / k;
			}

			// free memory
			free(t);
		}
		// empirical optimal inverse weighting using Kirkeby inverse filter to cancel out loudspeaker and microphone characteristic
		if (!strncmp(isinv_type, "kirkeby", 7)) {
			inverse_kirkeby(is_send, is_N, is_inv, isinv_N,is_type, isinv_type, fs, is_f1, is_f2, fft, fft_library);
		}
	}


}



void fade_in_double(double* x, long x_N, char* set_fade_window, double fade_percent, bool unfade) {
	long fade_N = (long)(fade_percent * (double)x_N);
	if (strcmp(set_fade_window, "none") && fade_percent > 0.0f && fade_N > 1) {
		// init
		double* w = (double*)malloc(fade_N * sizeof(double));
		long n_start = 0l;
		if (w) {
			// calculate window
			if (strcmp(set_fade_window, "none") == 0) {
				return;
			}

			if (strcmp(set_fade_window, "hann") == 0) {
				for (long n = 0; n < fade_N; n++) {
					w[n] = 0.5 * (1.0 - cos(2.0 * PI_D * (double)n / ((double)(fade_N)-1.0) / 2.0));
				}
			}

			if (strcmp(set_fade_window, "blackman") == 0) {
				for (long n = 0; n < fade_N; n++) {
					w[n] = 0.42 - 0.5 * cos(2.0 * PI_D * (double)n / ((double)(fade_N)-1.0) / 2.0) + 0.08 * cos(4.0 * PI_D / ((double)(fade_N)-1.0) / 2.0);
				}
			}

			if (strcmp(set_fade_window, "linear") == 0) {
				double fade_t_step = 1.0 / ((double)(fade_N)-1.0);
				w[0] = 0.0;
				for (long n = 1; n < fade_N; n++)
				{
					w[n] = (double)(w[0] + fade_t_step * n);

				};
			}
			//write_txt_debug_double(w,  fade_N, 0);


			// calculate signal
			if (unfade) {
				x[n_start] = 0.0;
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


void fade_out_double(double* x, long x_N, char* set_fade_window, double fade_percent, bool unfade) {
	long fade_N = (long)(fade_percent * (double)x_N);
	if (strcmp(set_fade_window, "none") && fade_percent > 0.0f && fade_N > 1) {
		// init
		double* w = (double*)malloc(fade_N * sizeof(double));
		long n_start = x_N - fade_N;
		if (w) {
			// calculate window
			if (strcmp(set_fade_window, "hann") == 0) {
				for (long n = 0; n < fade_N; n++) {
					w[n] = 0.5 * (1.0 + cos(2.0 * PI_D * (double)n / ((double)(fade_N)-1.0) / 2.0));
				}
			}

			if (strcmp(set_fade_window, "blackman") == 0) {
				for (long n = 0; n < fade_N; n++) {
					w[n] = 0.42 + 0.5 * cos(2.0 * PI_D * (double)n / ((double)(fade_N)-1.0) / 2.0) + 0.08 * cos(4.0 * PI_D / ((double)(fade_N)-1.0) / 2.0);
				}
			}

			if (strcmp(set_fade_window, "linear") == 0) {
				double fade_t_step = 1.0 / ((double)(fade_N)-1.0);
				w[0] = 1.0;
				for (long n = 1; n < fade_N; n++)
				{
					w[n] = (double)(w[0] - fade_t_step * n);
				};
			}
			//write_txt_debug_double(w,  fade_N, 0);

			// calculate signal
			if (unfade) {
				x[fade_N - 1 + n_start] = 0.0;
				for (long n = 0; n < fade_N - 1; n++) {
					x[n + n_start] = x[n + n_start] / w[n];
				}
			}
			else {
				for (long n = 0; n < fade_N; n++) {
					x[n + n_start] = x[n + n_start] * w[n];
				}
			}

			//write_txt_debug_double(x,  x_N, 1);

			// free memory
			free(w);
			//write_txt_debug_double(x,  x_N);
			//write_txt_debug_double(x,  x_N);
		}
	}
};


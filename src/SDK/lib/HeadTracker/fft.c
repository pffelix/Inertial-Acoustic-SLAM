// Copyright 2020, Felix Pfreundtner, All rights reserved.
#include "fft.h"

// Single FFT functions


void fast_fft(float *x, long x_N, struct ffts *fft, int flag, char * fft_library) {
	// timer start
	long timer_start = clock_tick();

	// select FFT library: Intel MKL FFT with Assembler code for x86 processors called over JUCE C Wrapper (fastest)
	if (!strcmp(fft_library, "intelmkl")) {
		if (flag == 1) {

			// run FFT
			JuceCWrapperFFTForward(x, x_N, true);
		}

		if (flag == -1) {

			// run FFT
			JuceCWrapperFFTInverse(x, x_N);
		}
	}

	// select FFT library: Native Cooley-Tukey Radix 2 C code (slower)
	if (!strcmp(fft_library, "native")) {

		// pre-format data from float data to complex data format
		//long Xnative_N = x_N;
		struct cpx* Xnative = fft->NATIVEtmp; //(struct cpx*)malloc((Xnative_N * sizeof(struct cpx)));
		if (Xnative) {
			if (flag == 1) {
				for (long n = 0; n < x_N; n++) {
					Xnative[n].real = x[n];
					Xnative[n].imag = 0.0f;
				}

			}

			if (flag == -1) {
				for (long n = 0; n < x_N; n++) {
					Xnative[n].real = x[2*n];
					Xnative[n].imag = x[2*n + 1];

				}
			}
		}

		//write_txt_debug_complex(Xnative,  Xnative_N, 1);

		// run FFT
		nativefft(Xnative, x_N, flag);

		//write_txt_debug_complex(Xnative,  Xnative_N, 1);

		// back-format data from complex data format to float data format
		if (Xnative) {
			if (flag == 1) {
				for (long n = 0; n < x_N; n++) {
					x[2*n] = Xnative[n].real;
					x[2*n + 1] = Xnative[n].imag;
				}
			}
			if (flag == -1) {
				for (long n = 0; n < x_N; n++) {
					x[n] = Xnative[n].real;
				}
			}
		}
		//write_txt_debug(x, 2*x_N, 1);
		// free memory
		//free(Xnative);
	}

	// stop timer
	double timer_diff = clock_stop(timer_start);
	timer_start = clock_tick();

	console_logi(PRIO_FFTTIMERDETAIL, "Single FFT done in: %d us\n", (int)(timer_diff * 1000.0));
	timer_diff = clock_stop(timer_start);
}

void fast_fft_convolution(float* x, long x_N, float* h, long h_N, float* y, long y_N, long y_Nshift, struct ffts* fft, char* fft_library, char* fft_type, long fft_flim_low, long fft_flim_high, long sampling_rate) {
	// timer stop
	//long timer_start;
	long timer_start_all;
	//double timer_diff;
	double timer_diff_all;

	timer_start_all = clock_tick();
	// Find minimal possible zeropadded length for fast convolution
	//long y_N = x_N + h_N - 1; 
	//timer_start = clock_tick();

	// Find length with smallest power of 2 for faster computation of (zeropadded) FFT 
	long Y_N = next_power_of_2(x_N + h_N - 1); //(debugger: 8us @ 2^16)

	//timer_diff = clock_stop(timer_start);

	// Find length of upsampled (zeropadded) signal (the same if y_N = x_N + h_N - 1, no upsampling)
	long Y_Nup = next_power_of_2(y_N);


	// Set length to two times length of signal to allow for complex expansion of float array
	//long Y_Ncpx = 2 * Y_N;

	// declare spectrum signals //(debugger: 750us @ 2^16)
	float* X = fft->Xtmp; //( float*)malloc(Y_Ncpx * sizeof(float));
	float* H;
	// if not impulse response convolution: use temporary array for fourier transformation
	if (strcmp(fft_type, "ir")) {
		H = fft->Htmp;
	}
	// else, use pre-set array
	else {
		//memcpy(H, fft->ISinverse, Y_Ncpx * sizeof(float));
		H = fft->ISinverse; // (float*)malloc(Y_Ncpx * sizeof(float));
	}
	float* Y = fft->Ytmp; // (float*)malloc(Y_Ncpx * sizeof(float));
	//timer_diff = clock_stop(timer_start);

	//timer_start = clock_tick();
	// copy signal into zeropadded spectrum signal (debugger: 30us @ 2^16)
	if (X && H) {
		// if crosscorrelation
		if (!strncmp(fft_type, "cor", 3)) {
			// shift input signal such that FFT DC = 0 delay becomes middle of output signal 
			if (x_N >= h_N) {
				// signal
				memcpy(X + h_N - 1, x, x_N * sizeof(float));
				memcpy(H, h, h_N * sizeof(float));
				// zeros
				memcpy(X, fft->zeros, (h_N - 1l) * sizeof(float));
				memcpy(X + h_N - 1 + x_N, fft->zeros, (Y_N - (h_N - 1l + x_N)) * sizeof(float));
				memcpy(H + h_N, fft->zeros, (Y_N - h_N) * sizeof(float));
			}
			else {
				// signal
				memcpy(H + x_N - 1, h, h_N * sizeof(float));
				memcpy(X, x, x_N * sizeof(float));
				// zeros
				memcpy(H, fft->zeros, (x_N - 1l) * sizeof(float));
				memcpy(H + x_N - 1 + h_N, fft->zeros, (Y_N - (x_N - 1l + h_N)) * sizeof(float));
				memcpy(X + x_N, fft->zeros, (Y_N - x_N) * sizeof(float));
			}
		}
		else {
			// signal
			memcpy(X, x, x_N * sizeof(float));
			// zeros
			memcpy(X + x_N, fft->zeros, (Y_N - x_N) * sizeof(float));

			// if not impulse response convolution: set fourier array
			if (strcmp(fft_type, "ir")) {
				// signal
				memcpy(H, h, h_N * sizeof(float));
				// zeros
				memcpy(H + h_N, fft->zeros, (Y_N - h_N) * sizeof(float));
			}

		}
	}

	//write_txt_debug(X,  Y_Ncpx, 1);
	//write_txt_debug(H,  Y_Ncpx, 1);
	//timer_diff = clock_stop(timer_start);
	//timer_start = clock_tick();

	fast_fft(X, Y_N, fft, 1, fft_library);
	// if not impulse response convolution: calculate fourier array
	if (strcmp(fft_type, "ir")) {
		fast_fft(H, Y_N, fft, 1, fft_library); //(debugger: 600us @ 2^16)
	}

	//timer_diff = clock_stop(timer_start);
	//write_txt_debug(X,  Y_Ncpx, 1);
	//write_txt_debug(H,  Y_Ncpx, 1);
	//write_txt_debug(Y,  Y_Ncpx, 0);

	// multiply FFT bins (debugger: 400us @ 2^16)
	//timer_start = clock_tick();
	if (Y && X && H) {
		long nreal;
		long nimag;
		// if crosscorrelation
		if (!strncmp(fft_type, "cor", 3)) {
			for (long n = 0; n <= Y_N; n += 2) { // <= as DC on left sided spectrum and signal length always even number (tested: is correct, even though last complex value is 0)
				nreal = n;
				nimag = nreal + 1;
				Y[nreal] = X[nreal] * H[nreal] + X[nimag] * H[nimag]; // real part
				Y[nimag] = -X[nreal] * H[nimag] + X[nimag] * H[nreal]; // imaginary part
			}
			if (!strcmp(fft_type, "cor-phat")) {
				float absY;
				for (long n = 0; n <= Y_N; n += 2) {
					nreal = n;
					nimag = nreal + 1;
					absY = sqrtf(Y[nreal] * Y[nreal] + Y[nimag] * Y[nimag]);
					Y[nreal] = Y[nreal] / absY;  // Phase Transform Weighting
					Y[nimag] = Y[nimag] / absY;  // Phase Transform Weighting
				}
			}
			if (!strcmp(fft_type, "cor-phatlim")) {
				float absY;
				// Apply Phase Transform Weighting only to FFT bins where impulse signal bandwith is located
				long N_low = 2 * ((long)floorf((float)fft_flim_low / (float)sampling_rate * Y_N) + 1); // 2 * as we have complex array (real and imaginary value)
				long N_high = 2 * ((long)ceilf((float)fft_flim_high / (float)sampling_rate * Y_N) + 1);
				if (N_low + 2 <= N_high) { // if bandwith is existant ( + 2 as we have complex array (real and imaginary value))
					for (long n = N_low; n < N_high; n += 2) {
						nreal = n;
						nimag = nreal + 1;
						absY = sqrtf(Y[nreal] * Y[nreal] + Y[nimag] * Y[nimag]);
						Y[nreal] = Y[nreal] / absY;  // Phase Transform Weighting
						Y[nimag] = Y[nimag] / absY;  // Phase Transform Weighting
					}
					for (long n = 0; n < N_low; n += 2) {
						nreal = n;
						nimag = nreal + 1;
						Y[nreal] = 0;
						Y[nimag] = 0;
					}
					for (long n = N_high; n <= Y_N; n += 2) {
						nreal = n;
						nimag = nreal + 1;
						Y[nreal] = 0;
						Y[nimag] = 0;
					}
				}
				else {
					console_log(PRIO_WARNING, "GCC-PHAT: selected frequency band too small. Cannot select FFT bins. Conventional cross-correlation is done.");
				}
			}
		}
		else {
			for (long n = 0; n <= Y_N; n += 2) {
				nreal = n;
				nimag = nreal + 1;
				Y[nreal] = X[nreal] * H[nreal] - X[nimag] * H[nimag]; // real part
				Y[nimag] = X[nreal] * H[nimag] + X[nimag] * H[nreal]; // imaginary part
			}
		}

		// if upsampling (add zeros after left spectrum)
		if (Y_Nup > Y_N) {
			long Naddedzeros =  Y_Nup - Y_N;
			//y_N = ceilf((float)(x_N + h_N - 1) / ((float)Y_N) * Y_Nup);
			memcpy(Y + Y_N + 2, fft->zeros, Naddedzeros * sizeof(float)); // zeropadded rest is also upsampled, amplitude is decreased by up_N
			//write_txt_debug(Y,  Y_Nup + 2, 0);
		}
	}
	//timer_diff = clock_stop(timer_start);
	//write_txt_debug(Y,  Y_N, 1);

	// perform inverse FFT
	fast_fft(Y, Y_Nup, fft, -1, fft_library);
	//write_txt_debug(Y,  Y_Nup, 0);

	//write_txt_debug(Y,  Y_Ncpx, 1);
	//timer_start = clock_tick();
	// write back spectrum to time signal while removing trailing zeros
	if (Y) {
		//if (!strcmp(fft_type, "cor")) {
			//memcpy(y, Y, y_N * sizeof(float));
		//}
		//else {
		memcpy(y, Y + y_Nshift, y_N * sizeof(float));
		//}
	}

	// norm fft (intelmkl is already automatically normed, native has not normed higher output than intelmkl but somehow leads to error when divided at first long convolution, later division by y_N at cross-correlation works to get same result as intel mkl)
	bool norm_fft = false;
	if (norm_fft && !strncmp(fft_library, "native", 3) && !strncmp(fft_type, "cor", 3)) {
		ar_divide(y, y_N, (float)y_N);
	}

	//write_txt_debug(y,  y_N, 0);
	//timer_diff = clock_stop(timer_start);

	//timer_start = clock_tick();
	// free memory (debugger: 150us @ 2^16)
	//free(X);
	//free(H);
	//free(Y);
	//timer_diff = clock_stop(timer_start);

	// timer
	timer_diff_all = clock_stop(timer_start_all);
	console_logi(PRIO_FFTTIMER, "Single FFT convolution done in: %d us\n", (int)(timer_diff_all*1000.0));

}


void full_fft(struct cpx *x, long x_N, int flag, char * fft_library)
{
	// mufft (easy syntax, only 2^n, maybe only windows) or pffft (2^n, 3, 5, difficutl for android) or MKL (fastest, but inside FFTW framework) or FFTS (fastest according to Juce forum)

	// select FFT library and run FFT
	if (!strcmp(fft_library, "native")) {
		// run FFT

		//write_txt_debug_complex(x, x_N, 0);
		long timer_start = clock_tick();
		nativefft(x, x_N, flag);
		double timer_diff = clock_stop(timer_start);
		console_logi(PRIO_FFTTIMERDETAIL, "FFT internal done in: %d us\n", (int)(timer_diff*1000.0));

	}

	if (!strcmp(fft_library, "intelmkl")) {
		// generate suitable mkl float* array format as input for intel mkl fft
		long xmkl_N = 2 * x_N;
		float* xmkl = (float*)malloc((xmkl_N * sizeof(float) * 2));

		long timer_start_wrapper = clock_tick();
		//write_txt_debug_complex(x, x_N, 1);


		// start fft
		long timer_start_internal = clock_tick();
		bool ignore_negative_freqs = true;
		if (xmkl) {
			if (flag == 1) {
				for (long n = 0; n < x_N; n++) {
					xmkl[n] = x[n].real;
				}

				JuceCWrapperFFTForward(xmkl, x_N, ignore_negative_freqs);

			}
			if (flag == -1) {
				//xmkl = (float*)(x); // watch out, pointer on xmkl cannot be deleted in this way as only adress mapping changes and with this returned signal x has new adress
				for (long n = 0; n < x_N; n++) {
					xmkl[2 * n] = x[n].real;
					xmkl[2 * n + 1] = x[n].imag;
				}
				JuceCWrapperFFTInverse(xmkl, x_N);
			}
			double timer_diff_internal = clock_stop(timer_start_internal);
			console_logi(PRIO_FFTTIMERDETAIL, "FFT internal done in: %d us\n", (int)(timer_diff_internal * 1000.0));
			//OutputDebugString(str);

			// write back suitable array
			//write_txt_debug(xmkl, xmkl_N);
			//x = (struct cpx*)(xmkl); // watch out, pointer on xmkl cannot be deleted in this way as only adress mapping changes and with this returned signal x has new adress
			if (ignore_negative_freqs && flag == -1) {
				for (long n = 0; n < x_N; n++) {
					x[n].real = xmkl[n];
					x[n].imag = 0.0f;
				}
				//write_txt_debug_complex(x, x_N, 1);
			}
			else {
				for (long n = 0; n < x_N; n++) {
					x[n].real = xmkl[2 * n];
					x[n].imag = xmkl[2 * n + 1];
				}
			}

			double timer_diff_wrapper = clock_stop(timer_start_wrapper);
			console_logi(PRIO_FFTTIMERDETAIL, "FFT wrapper done in: %d us\n", (int)(timer_diff_wrapper * 1000.0));
			//OutputDebugString(str);

			// free mkl array

			free(xmkl);
		}
	}

	if (!strcmp(fft_library, "pocketfft")) {

		// make FFT plan

		//// select forward or backward fourier transformation
		//if (flag == 1) {
		//	// make plan
		//	//x_N = x_N / 2;
		//	cfft_plan plan = make_rfft_plan (2 * (int)x_N);

		//	// perform complex float to interleaved double conversion
		//	double *xdouble = (double*)malloc(2 * (int)x_N * sizeof(double));
		//	for (long n = 0; n < x_N; n++) {
		//		xdouble[2*n] = (double)(x[n].real);
		//		xdouble[2*n + 1] = (double)(x[n].imag);
		//	}
		//	long a = clock_tick();

		//	// run FFT
		//	rfft_forward(plan, xdouble, 1.0);
		//	double b = clock_stop(a);

		//	//nativefft(x, x_N, flag);

		//	// write back to complex  array
		//	for (long n = 0; n < x_N; n++) {
		//		x[n].real = (float)xdouble[2*n];
		//		x[n].imag = (float)xdouble[2*n+1];
		//	}

		//	// free memory
		//	free(xdouble);
		//	destroy_cfft_plan (plan);
		//	double c = 0.1;
		//}
		//if (flag == -1) {
		//	// make plan
		//	cfft_plan plan = make_rfft_plan (2 * (x_N-1));

		//	// perform complex float to interleaved double conversion
		//	double *xdouble = (double*)malloc(2 * x_N * sizeof(double));
		//	for (long n = 0; n < x_N; n++) {
		//		xdouble[2*n] = (double)(x[n].real);
		//		xdouble[2*n + 1] = (double)(x[n].imag);
		//	}
		//	// run FFT
		//	rfft_backward(plan, xdouble, 1.0);


		//	// write back to complex struct array
		//	for (long n = 0; n < x_N; n++) {
		//		x[n].real = (float)xdouble[2*n];
		//		x[n].imag = (float)xdouble[2*n+1];
		//	}

		//	// free memory
		//	free(xdouble);
		//	destroy_cfft_plan (plan);
		//}
	}
}



void full_fft_convolution(float *x, long x_N, float *h, long h_N, float *y, char* fft_library){
	// Find minimal possible zeropadded length for fast convolution
	long y_N = x_N + h_N - 1; 

	// Find length with smallest power of 2 for faster computation of FFT
	long Y_N = (long)pow(2, ceil(log(y_N)/log(2))); 
	//write_txt_debug(x,  x_N);
	//write_txt_debug(h,  h_N);

	// zeropad signal
	struct cpx *X = (struct cpx*)malloc(Y_N * sizeof(struct cpx));
	struct cpx *H = (struct cpx*)malloc(Y_N * sizeof(struct cpx));
	for (long n = 0; n < x_N; n++) {
		X[n].real = x[n];
		X[n].imag = 0.0f;
	}
	for (long n = x_N; n < Y_N; n++) {
		X[n].real = 0.0f;
		X[n].imag = 0.0f;
	}
	for (long n = 0; n < h_N; n++) {
		H[n].real = h[n];
		H[n].imag = 0.0f;
	}
	for (long n = h_N; n < Y_N; n++) {
		H[n].real = 0.0f;
		H[n].imag = 0.0f;
	}
	//write_txt_debug_complex(X, Y_N, 0);
	//write_txt_debug_complex(H, Y_N, 0);

	// perform FFT
	full_fft(H, Y_N, 1, fft_library);
	full_fft(X, Y_N, 1, fft_library);


	// multiply FFT bins (could be made more performant by considering symetric structure of FFT)
	struct cpx *Y = (struct cpx*)malloc(Y_N * sizeof(struct cpx));
	if (Y) {
		for (long n = 0; n < Y_N; n++) {
			Y[n].real = X[n].real * H[n].real - X[n].imag * H[n].imag;
			Y[n].imag = X[n].real * H[n].imag + X[n].imag * H[n].real;
		}
	}

	//write_txt_debug_complex(Y, Y_N, 1);

	//long N_sym = (long)ceil(Y_N / 2);
	//long n_sym;
	//for (long n = 0; n < N_sym; n++) {
	//	Y[n].real = X[n].real * H[n].real - X[n].imag * H[n].imag;
	//	Y[n].imag = X[n].real * H[n].imag + X[n].imag * H[n].real;
	//}

	//for (long n = 1; n < N_sym; n++) {
	//	n_sym = Y_N - n;
	//	Y[n_sym].real = Y[n].real;
	//	Y[n_sym].imag = - Y[n].imag;
	//}


	// perform inverse FFT
	full_fft(Y, Y_N, -1, fft_library);

	// remove trailing zeros and produce output and divide by fourier transformation scaling factor
	if (Y) {
		for (long n = 0; n < y_N; n++) {
			y[n] = Y[n].real / Y_N;
		}
	}
	//write_txt_debug(y,  y_N);


	// free memory
	free(X);
	free(H);
	free(Y);
}

// Crosscorrelation

void fast_fft_crosscorrelation(float* x, long x_N, float* h, long h_N, float* y, long y_N, struct ffts *fft, char* fft_library, char* cor_type, long fft_flim_low, long fft_flim_high, long sampling_rate) {
	fast_fft_convolution(x, x_N, h, h_N, y, y_N, 0, fft, fft_library, cor_type, fft_flim_low, fft_flim_high, sampling_rate);
}

void full_fft_crosscorrelation(float* x, long x_N, float* h, long h_N, float* y, char* fft_library) {

	// Find minimal possible zeropadded length for fast convolution
	long y_N = x_N + h_N - 1; // varies from convolution FFT //x_N + x_N - 1

	// Find length with smallest power of 2 for faster computation of FFT
	long Y_N = (long)pow(2, ceil(log(y_N)/log(2)));  // varies from convolution FFT
	//write_txt_debug(x,  x_N, 0);
	//write_txt_debug(h,  h_N, 0);


	// varies from convolution FFT by shifting the signal for getting 0 delay to middle of signal (DC value, after it lowest frequency in FFT)
	// zeropad signal
	struct cpx *X = (struct cpx*)malloc(Y_N * sizeof(struct cpx));
	struct cpx *H = (struct cpx*)malloc(Y_N * sizeof(struct cpx));
	for (long n = 0; n < h_N - 1; n++) { // varies from convolution FFT //n = 0; n < x_N - 1;
		X[n].real = 0.0f; // varies from convolution FFT
		X[n].imag = 0.0f;
	}
	for (long n = h_N - 1; n < y_N; n++) { // varies from convolution FFT //n = x_N - 1; n < y_N;
		X[n].real = x[n - h_N + 1]; // varies from convolution FFT //n - x_N + 1
		X[n].imag = 0.0f;
	}
	for (long n = y_N; n < Y_N; n++) { // varies from convolution FFT
		X[n].real = 0.0f; // varies from convolution FFT
		X[n].imag = 0.0f; // varies from convolution FFT
	}
	for (long n = 0; n < h_N; n++) {
		H[n].real = h[n];
		H[n].imag = 0.0f;
	}
	for (long n = h_N; n < Y_N; n++) {
		H[n].real = 0.0f;
		H[n].imag = 0.0f;
	}
	//write_txt_debug_complex(X, Y_N, 0);
	//write_txt_debug_complex(H, Y_N, 0);

	// perform FFT
	full_fft(H, Y_N, 1, fft_library);
	full_fft(X, Y_N, 1, fft_library);

	// multiply FFT bins (could be made more performant by considering symetric structure of FFT)
	struct cpx *Y = (struct cpx*)malloc(Y_N * sizeof(struct cpx));
	if (Y) {
		for (long n = 0; n < Y_N; n++) {
			Y[n].real = X[n].real * H[n].real + X[n].imag * H[n].imag; // varies from convolution FFT
			Y[n].imag = -X[n].real * H[n].imag + X[n].imag * H[n].real; // varies from convolution FFT
		}
	}
	//write_txt_debug_complex(Y, Y_N, 1);

	//long N_sym = (long)ceil(Y_N / 2);
	//long n_sym;
	//for (long n = 0; n < N_sym; n++) {
	//	Y[n].real = X[n].real * H[n].real - X[n].imag * H[n].imag;
	//	Y[n].imag = X[n].real * H[n].imag + X[n].imag * H[n].real;
	//}

	//for (long n = 1; n < N_sym; n++) {
	//	n_sym = Y_N - n;
	//	Y[n_sym].real = Y[n].real;
	//	Y[n_sym].imag = - Y[n].imag;
	//}


	// perform inverse FFT
	full_fft(Y, Y_N, -1, fft_library);

	// remove trailing zeros and produce output and divide by fourier transformation scaling factor
	//long zerosfront = floorf((float)(Y_N-y_N) / 2);
	//zerosfront = 0;
	if (Y) {
		for (long n = 0; n < y_N; n++) {
			y[n] = Y[n].real; // varies from convolution FFT // n + zerosfront
		}
	}
	//write_txt_debug(y,  y_N, 1);


	// free memory
	free(X);
	free(H);
	free(Y);
}






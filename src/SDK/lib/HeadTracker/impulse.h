// Copyright 2020, Felix Pfreundtner, All rights reserved.
#pragma once

#include <math.h>
#include <string.h>
#include "array.h"
#include <stdlib.h>
#include "debugging.h"
#define PI_D 3.1415926535897932384626433832795028841971693993751058209749445923078164062//3.141592653589793238463
#define PI_F 3.14159265358979f

#ifdef __cplusplus
extern "C" {
#endif

	/** Initialises a single sine sweep impulse (linear or exponential) with double precision.
		flag = 1: impulse
		flag = -1: inverse impulse
		formula: https://www.realcordingblogs.com/wiki/sine-sweep
		formula: https://dsp.stackexchange.com/questions/41696/calculating-the-inverse-filter-for-the-exponential-sine-sweep-method
	*/
	void generate_impulse_double(double * is_send, long is_N, long fs, long is_f1, long is_f2, const char* is_type, double is_amp);

	// calculate inverse impulse is_inv to impulse is_send
	void calculate_inverse_impulse(float* is_send, long is_N, float* is_inv, long isinv_N, char* is_type, const char* isinv_type, long fs, long is_f1, long is_f2, struct ffts* fft, char * fft_library);


	/** Fades in a signal with double precision.

	*/
	void fade_in_double(double* x, long x_N, char* set_fade_window, double fade_percent, bool unfade);

	/** Fades out a signal with double precision.

	*/
	void fade_out_double(double* x, long x_N, char* set_fade_window, double fade_percent, bool unfade);


#ifdef __cplusplus
};
#endif
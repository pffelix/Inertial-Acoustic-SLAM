// Copyright 2020, Felix Pfreundtner, All rights reserved.
#pragma once
#include "params.h"
#include "state.h"
#include "array.h"
#include "fft.h"
#include <math.h>
#include <stdlib.h>
#include "tools.h"
#include "debugging.h"
#include "filter.h"

#define MIN(X, Y) (((X) < (Y)) ? (X) : (Y))
#define MAX(X, Y) (((X) < (Y)) ? (Y) : (X))

#ifdef __cplusplus
extern "C" {
#endif
	
	// standard functions

	// normalize signal
	void mono_normalize(float *x, long x_N);

	// binaural normalize signal
	void binaural_normalize(float* x1, float* x2, long x_N);

	// maximum of binaural signal
	void binaural_maximum(float* x1, float* x2, long x_N, long* max_N, float* max_amp);

	// multichannel pointer signal normalize
	void multichannel_normalize(float** x, long x_N, int Nchannels);

	// multichannel pointer signal normalize and return norm
	float multichannel_normalize_norm(float** x, long x_N, int Nchannels);

	// get signal energy
	void get_average_energy_per_sample(float* x, long x_N, float * energy);

	// compare energy of two (pressure) values and derive energy gain
	void get_sample_gain(float x1, float x2, float* gain_db);

	// compare energy of two (pressure) arrays and derive energy gain
	void get_array_gain(float* x1, float* x2, long x_N, float* gain_db);

	// square signal
	void square(float* x, long x_N);

	// window signal with defined window_type
	void window(float* x, long x_N, char* window_type);

	// flip signal in its direction
	void flip(float* x, long x_N);

	// shift signal linear in array for x_Nshift samples to the right or left
	void shift_linear(float* x, long x_N, long x_Nshift);

	// shift signal circular in array for x_Nshift samples to the right
	void shift_circular(float* x, long x_N, long x_Nshift);

	// upsamle signal about factor Nup and weight for each value 1...Nup
	void linupsample(float *x, long x_N, float* weight, long Nup, float *y);

	// upsample signal, given in M frames, about factor Nup  and weight for each value 1...Nup (warning: m/n notation means first pointer position m (Frames), second ointer position n (length signal)
	void linupsample2d(float** x, long x_N, long x_M, float* weight, long Nup, float** y);

	// fades in a signal or unfades a signal
	void fade_in(float* x, long x_N, char* set_fade_window, float fade_percent, bool unfade);

	// fades out a signal or unfades a signal
	void fade_out(float* x, long x_N, char* set_fade_window, float fade_percent, bool unfade);

	// lowpass filtering of signal x with varying filter types and frequency positions
	void lowpass(float* x, long x_N, enum TWindowType fir_type, long fir_N, float fir_alpha, long fir_fcut, long fir_fs, struct ffts* fft, char* fft_library);

	// highpass filtering of signal x with varying filter types and frequency positions
	void highpass(float* x, long x_N, enum TWindowType fir_type, long fir_N, float fir_alpha, long fir_fcut, long fir_fs, struct ffts* fft, char* fft_library);

	// bandpass filtering of signal x with varying filter types and frequency positions
	void bandpass(float* x, long x_N, enum TWindowType fir_type, long fir_N, float fir_alpha, long fir_f1, long fir_f2, long fir_fs, struct ffts* fft, char* fft_library);

	// calculate inverse kirkeby filter to signal x
	void inverse_kirkeby(float* x, long x_N, float* x_inv, long xinv_N, const char* xtype, const char* xinvtype, long fs, long flow, long fhigh, struct ffts* fft, char* fft_library);

	// moving mean with Mleft samples to left side and Mright samples to right side
	void movingmean(float* x, long x_N, float* y, long Mleft, long Mright);

	// moving mean with Mleft samples to left side and Mright samples to right side at internal double precision(needed if x values large and Mleft and Mright large)
	void movingmean_double(float* x, long x_N, float* y, long Mleft, long Mright);

	// correlation in positive direction between x1 and x2 with upsampling factor N_up and output y (depreciated)
	void correlate_positive(float *x1, float *x2, long x_N, float *y);

	// crosscorrelation in between x1 and x2 with upsampling factor N_up and output y up to delay sample maxdelayN
	void crosscorrelate(float* x1, float* x2, long x_N, float* y, long maxdelayN, char * cor_shift);

	// Maximum of crosscorrelation output y between limit in samples with framerate fs
	void crosscorrelate_maxn(float* y, long y_N, long N_lim, long* max_N, float* max_amp);

	// crosscorrelation peak to time delay
	void crosscorrelate_maxntotd(long maxn, long samplingrate, float *td);

	// calculate convolution y between signal x and filter h in time domain (own implementation)
	void convolve2(float* x, long x_N, float *h, long h_N, float *y);

	// calculate convolution y between signal x and filter h in time domain (requirement: x_N>=h_N)
	void convolve(float* x, long x_N, float *h, long h_N, float *y);

	// calculate fft convolution y between signal x and filter h in frequency domain
	// fft_type can be "" for normal convolution between x and h, "ir" with pre-set inverse impulse h, "cor" for crosscorrelation between x and h
	void convolve_fft(float* x, long x_N, float *h, long h_N, float *y, long y_N, long y_Nshift, struct ffts *fft, char * fft_library,  char * fft_type);

	// calculate fft cross-correlation y between signal x and signal y in frequency domain, if signal arrives at x earlier than y peak is on the left side of cross-correlation
	void crosscorrelate_fft(float* x1, long x1_N, float* x2, long x2_N, float* y, long y_N, struct ffts *fft, char* fft_library, char *cor_type, long fft_flim_low, long fft_flim_high, long sampling_rate, bool normalized);

	// level difference between left and right signal
	void level_diff(float *irf1, long irf1_N, float *irf2, long irf2_N, float * leveldiff);



	// aggregated  functions
	// steer 2 channel recording of an linear array with specified distance in metre to beamformed recording with specified beamform angle
	void rec_to_recbeam(float* rec_1, float* rec_2, long rec_N, float steering_ang, float rec_distance_m, float sos, float fs, float * rec_beamformed);

	// convert recording of impulse to impulse response
	void convolve_rec_to_ir(float *rec, long rec_N, float *is_inv, long isinv_N, float *ir, long ir_N, struct ffts *fft, char * fft_library);

	// convert correlation to time delay difference
	void corf_to_tdmax(float* y, long y_N, long N_lim, long samplingrate, float *td, float *td_amp);

	// convert binaural impulse response frame to level difference
	void irf_to_ld(float *x_l, float *x_r, long x_N, float * ld);

	// merge angle (degree) with window to model cross-correlation result
	void ang_windowed_in_cor(float* cor, long cor_N, long cor_Nlim, float ang, float* window, long window_N_oneside, bool cor_merge);

#ifdef __cplusplus
};
#endif
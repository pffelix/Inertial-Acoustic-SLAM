// Copyright 2020, Felix Pfreundtner, All rights reserved.
// tools.h: Toolkits for algorithm excecution.

#pragma once
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "params.h"
#include "threadmain.h"
#include "state.h"
#include "tools.h"
#include <stdbool.h>
#include <time.h>
#include <stdarg.h>
#include "signals.h"
#include "timing.h"
#if(FRAMEWORK_WINDOWSUSED == 1)
#include <windows.h> 
#endif

#ifdef __cplusplus
extern "C" {
#endif

	// write array with float values to disk at filepath
	void write_txt(float* ar, long N, char *filepath);

	// write 2d array with float values to disk at filepath
	void write_txt2d(float* ar, long N, char *filepath, bool newfile);

	// write array with stereo float values to disk at filepath
	void write_txt_complex(struct cpx *x,long n, char *filepath);

	// write array with double values to disk at filepath
	void write_txt_double(double* x, long N, char* filepath);

	// write array with long values to disk at filepath
	void write_txt_long(long* x, long N, char* filepath);

	// write array with float values to disk at standard debug filepath in params.h
	void write_txt_debug(float* x, long N, bool normalize);

	// write 2d array with float values to disk at standard multichannel debug filepath in params.h using multiple files
	void write_txt_debug2dmultifile(float** x, long M, long N, bool normalize);

	// write 2d array with float values to disk at standard multichannel debug filepath in params.h
	void write_txt_debug2d(float** x, long M, long N, bool normalize);

	// write stereo signal to disk at standard multichannel debug filetpath in params.h
	void write_txt_debug_stereo(float* x_l, float* x_r, long N, bool normalize);

	// write array with stereo float values to disk at standard debug filepath in params.h
	void write_txt_debug_complex(struct cpx *x, long N, bool normalize);

	// write array with double values to disk at standard debug filepath in params.h
	void write_txt_debug_double(double* x, long N);
	
	// write array with long values to disk at standard debug filepath in params.h
	void write_txt_debug_long(long* x, long N);

	/* write multichannel array to wave file consisting of N samples for Nchannels (byte writing, faster than write_txt)
	   normalize options: 0: no, 1: normalize compared to loudest channel, 2: normalize every channel one by one, 3: normalize every frame one by one, 4: normalize compared to loudest channel and set sampling rate, >=10: multiply each sample with long value given in flag
	*/ 
	void write_wav_debug(float** x, int Nchannels, long Nsamples, float sampling_rate, const char* filepath, const char* metadata, long flag);

	/* write multichannel array to wave file consisting of Nsamples for Nchannels and Nframes (byte writing, faster than write_txt)
	   normalize options: 0: no, 1: normalize compared to loudest channel, 2: normalize every channel one by one, 3: normalize every frame one by one, 4: normalize compared to loudest channel and set sampling rate
	*/ 
	void write_wav_frame_debug(float*** x, int Nchannels, int Nframes, long Nsamples, float sampling_rate, const char* filepath, const char* metadata, long flag);

	// debug variables of particle slam filter (same thread)
	// flag: 1: write additional info txt file with dimensions of array
	void write_particleslam_debug(struct par6s* p, const char* channel, const char* frame, const char* sample, int Nchannels, int Nframes, int Nsamples, int flag);
	
	// mulithreading disk debug
	/////////////// 

	// debug main functions (multithread) (deb_Nmax, deb and pool are used from state)
	void write_wav_debug_multithread(float** x, int Nchannels, long Nsamples, float sampling_rate, const char* filepath, const char* metadata, long flag, bool multithread_on);
	void write_wav_frame_debug_multithread(float*** x, int Nchannels, int Nframes, long Nsamples, float sampling_rate, const char* filepath, const char* metadata, long flag, bool multithread_on);
	void write_particleslam_debug_multithread(struct par6s* p, const char* channel, const char* frame, const char* sample, int Nchannels, int Nframes, int Nsamples, int flag, bool multithread_on);
	
	// conventional debug calling functions (struct as input)
	void write_wav_debug_struct(struct debs* d);
	void write_wav_frame_debug_struct(struct debs* d);
	void write_particleslam_debug_struct(struct debs* d);

	// get open debugging struct
	struct debs* get_deb_struct(struct debs** dd, int deb_Nmax);

	// leave debugging struct
	void leave_deb_struct(struct debs* d);
	///////////////



	// start (get) timer based on clock cycle (if windows 0.1us accuracy, else 1ms accuracy and platform independent)
	long clock_tick();
	
	// stop time in ms basd on clock cycle (if windows 0.1us accuracy, else 1ms accuracy and platform independent)
	double clock_stop(long start);

	// get millisecond difference of clock cycle timer (if windows 1us accuracy, else 1ms accuracy and platform independent)
	double clock_diff(long start, long stop);

	// add console that print stdout for debugging (only Windows)
	void console_add();

	// log to console with certain priority flag and dynamic number of arguments
	void console_logdyn(int priority, const char *format, ...);

	// log to console with certain priority flag and one double value as argument
	void console_logd(int priority, const char* format, double arg);

	// log to console with certain priority flag and one float value as argument
	void console_logf(int priority, const char* format, float arg);

	// log to console with certain priority flag and 3 float values as argument
	void console_logf3d(int priority, const char* format, float arg1, float arg2, float arg3);

	// log to console with certain priority flag and one long value as argument
	void console_logl(int priority, const char* format, long arg);

	// log to console with certain priority flag and one int value as argument
	void console_logi(int priority, const char* format, int arg);

	// log to console with certain priority flag and two int values as argument
	void console_logii(int priority, const char* format, int arg1, int arg2);

	// log to console with certain priority flag and one string value as argument
	void console_logs(int priority, const char* format, char * arg);

	// log to console with certain priority flag and no argument
	void console_log(int priority, const char* format);

	// is float value unregular
	bool is_unregular_float(float* x, bool DEN, bool zero);

#ifdef __cplusplus
};
#endif
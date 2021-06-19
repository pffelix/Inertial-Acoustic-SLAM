// Copyright 2020, Felix Pfreundtner, All rights reserved.
#include "timing.h"


long timer_tick() {
	long timer  = 0l;
	// if Windows is used
	#if(FRAMEWORK_WINDOWSUSED == 1)
		// Get 1/10th microsecond precision time
		// source: https://www.qtcentre.org/threads/1606-Current-Time-with-microsecond-precision-on-windows

		//struct timeval* tv;
		time_t rawtime;
		time(&rawtime);
		timer = (long)rawtime;

		// here starts the microsecond resolution:

		LARGE_INTEGER tickPerSecond;
		LARGE_INTEGER tick; // a point in time

		// get the high resolution counter's accuracy
		QueryPerformanceFrequency(&tickPerSecond);

		// what time is it ?
		QueryPerformanceCounter(&tick);

		// and here we get the current microsecond! \o/
		timer = (long)(tick.QuadPart % tickPerSecond.QuadPart);

		// All other operating systems
	#else
		// Get millisecond precision time
		timer = (long)clock();
	#endif to de
	return timer;

};

long long timer_tick_ll() {
	long long timer = 0;
	// if Windows is used
	#if(FRAMEWORK_WINDOWSUSED == 1)
		// Get 1/10th microsecond precision time
		// source: https://www.qtcentre.org/threads/1606-Current-Time-with-microsecond-precision-on-windows

		//struct timeval* tv;
		time_t rawtime;
		time(&rawtime);
		timer = (long long)rawtime;

		// here starts the microsecond resolution:

		LARGE_INTEGER tickPerSecond;
		LARGE_INTEGER tick; // a point in time

		// get the high resolution counter's accuracy
		QueryPerformanceFrequency(&tickPerSecond);

		// what time is it ?
		QueryPerformanceCounter(&tick);

		// and here we get the current microsecond! \o/
		timer = (long long)(tick.QuadPart % tickPerSecond.QuadPart);

		// All other operating systems
	#else
		// Get millisecond precision time
		timer = (long)clock();
	#endif to de
	return timer;

};


double timer_stop(long start) {
	double timer = 0.0;
	long stop = timer_tick();
	timer = timer_diff(start, stop);
	return timer;

};


double timer_stop_ll(long long start) {
	double timer = 0.0;
	long long stop = timer_tick_ll();
	timer = timer_diff(start, stop);
	return timer;

};



double timer_diff(long start, long stop) {
	double time_diff_ms = 0.0;
	// if Windows is used
	#if(FRAMEWORK_WINDOWSUSED == 1)
			time_diff_ms = ((double)stop - (double)start) / 10000.0; // in ms
	// All other operating systems
	#else
			time_diff_ms = (double)(stop - start) / (double)CLOCKS_PER_SEC * 1000.0; // in ms
	#endif
	return time_diff_ms;
};

double timer_diff_ll(long long start, long long stop) {
	double time_diff_ms = 0.0;
	// if Windows is used
	#if(FRAMEWORK_WINDOWSUSED == 1)
			time_diff_ms = ((double)stop - (double)start) / 10000.0; // in ms
	// All other operating systems
	#else
			time_diff_ms = (double)(stop - start) / (double)CLOCKS_PER_SEC * 1000.0; // in ms
	#endif
	return time_diff_ms;
};

void timerdiff_update(long* timer_start, float* timer_diff, float max_diff_ms) {
	double timer_diff_new;
	if (*timer_start != 0) {
		timer_diff_new = timer_stop(*timer_start);
		timer_diff_new /= 1000.0;
		if (timer_diff_new > 0.0f && timer_diff_new <= max_diff_ms) {
			*timer_diff = (float)timer_diff_new;
		}
	}
	*timer_start = timer_tick();
}
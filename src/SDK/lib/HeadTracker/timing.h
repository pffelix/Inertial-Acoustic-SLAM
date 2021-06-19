// Copyright 2020, Felix Pfreundtner, All rights reserved.
// timing.h: Toolkits for measuring code excecution time.

#pragma once
#define FRAMEWORK_WINDOWSUSED 1 //Has to be set here as otherwise error in JUCECWrapper. Windows system used (1) or not used (0), has impact on selected clock timer functions in debugging.c, more precision in windows
#include <time.h>
#include <stdbool.h>
#if(FRAMEWORK_WINDOWSUSED == 1)
#include <windows.h> 
#endif

#ifdef __cplusplus
extern "C" {
#endif


	// start (get) timer based on clock cycle (if windows 0.1us accuracy, else 1ms accuracy and platform independent)
	long timer_tick();
	long long timer_tick_ll();

	// stop time in ms basd on clock cycle (if windows 0.1us accuracy, else 1ms accuracy and platform independent)
	double timer_stop(long start);
	double timer_stop_ll(long long start);

	// get millisecond difference of clock cycle timer (if windows 1us accuracy, else 1ms accuracy and platform independent)
	double timer_diff(long start, long stop);
	double timer_diff_ll(long long start, long long stop);

	// update continually running timmer to get difference between two looped timepoints with maximal difference in ms
	void timerdiff_update(long* timer_start, float* timer_diff, float max_diff_ms);

#ifdef __cplusplus
};
#endif
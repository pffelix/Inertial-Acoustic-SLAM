// head_tracker.h: Main head tracker start script.
// Copyright 2020, Felix Pfreundtner, All rights reserved.

#pragma once
#define EXPORT __declspec(dllexport) 
#include "params.h"
#include "signals.h"
#include "state.h"
#include "head.h"
#include "offline.h"
#include "debugging.h"
#include "kalman.h"
#include "statistics.h"
#include "threadmain.h"

#ifdef __cplusplus
extern "C" {
#endif

	//////////////////////////////////////////
	// external head tracker functions (public)
	//////////////////////////////////////////

	// initilize head tracker parameters and program state
	void init_head_tracker(); // EXPORT

	// Update head tracker program state with audio buffer and return features with information about head orientation
	void update_head_tracker_state(); // EXPORT


	//////////////////////////////////////////
	// internal head tracker functions (private)
	//////////////////////////////////////////

	// initilize head tracker paramters
	void init_head_tracker_params(); // EXPORT

	// initilize head tracker program state
	void init_head_tracker_state();

	// clean recording buffer by windowing any signal edges
	void rec_clean();

	// synchronize playback and recording device by positioning first impulse response peak in l/r channel at fixed position
	void rec_sync();

	// convert recording to impulse response
	void rec_to_ir();

	// convert impulse response to sparse impulse response
	void ir_to_irsparse();

	// convert impulse response to impulse response frames
	void irsparse_to_irf();

	// convert impulse response to features
	void irf_to_fe();

	// convert features for slam
	void fe_to_slam();

	// predict from slam head position
	void slam_to_pos();

	// save selected state for next update routine
	void save_state();

	// finish update loop
	void finish_update();


	//////////////////////////////////////////
	// multithreading subroutines (private)
	//////////////////////////////////////////

	// calculate impulse response for single frame number m given as pointer argument
	void rec_to_ir_multithread(void *arg);

	// extract frames response for single frame number m given as pointer argument
	void extract_frames_multithread(void* arg);

	// calculate feature for single frame number m given as pointer argument
	void irf_to_fe_multithread(void *arg);



#ifdef __cplusplus
};
#endif
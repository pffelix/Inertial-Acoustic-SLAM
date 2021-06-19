// frame.h: Frame extraction from impulse response.
// Copyright 2020, Felix Pfreundtner, All rights reserved.

#pragma once
#include "params.h"

#ifdef __cplusplus
extern "C" {
#endif
	// find first sample in signal exceeding absolute threshold
	void get_start_sample(float* x, long x_N, float threshold, long * Nstart);

	// find impulse delay position (-isinv_N, rec_N - isinv_N) centralised around center_N using derived matched filter in x1 and x2;
	void get_binaural_delay_position(float* x1, float* x2, long x_N, long isinv_N, long rec_N, long center_N, long* Ndelay);

	// find impulse playback position (0...isinv_N) centralised around center_N + Ndelay using derived matched filter in x1 and x2;
	void get_binaural_playback_position(float* x1, float* x2, long x_N, long isinv_N, long center_N, long *Ndelay, long *Nplay);

	// find first sample in binaural signal exceeding absolute threshold
	void get_binaural_start_sample(float* x1, float* x2, long x_N, long x_N_start, long Nstartdynamicsearch, float start_threshold, long start_offset, long* Nstart);
	
	// check whether signal begin was not found properly and reset start sample if needed
	void check_start_sample(long N, long frame_M, long frame_N, long frame_Noverlap, long * Nstart);
	
	// extract frames from signal considering overlap and a hann window
	void extract_frames(struct fras* f, float* x, long x_N, float* window, long frame_N, long frame_M, long frame_Mon, long frame_Mstart, long Nstartframesample, long Nstart, long frame_Noverlap, bool frame_window, bool frame_normalize, int ch, int ch_reference, float** y);

	// select frames array of frames y with M frames of lenght y_N considering Mon frames with maximal energy (currently A_r is reference for frame selection, not multichannel)
	void select_frames(struct fras* f, long y_N, long M, long Mon);

	// get weight of cross-correlation peaks in each frame (currently, A_ll2, A_rr2, A_ff2 frame based, not multichannel)
	void tdamp_to_frame_corpeak(struct fras* f, float** tdamp, int Nch, long m);
	
	// get weight of elevation position in each frame
	void el_to_frame_el(struct fras* f, struct heads* h, long m);

	// normalize weight of cross-correlation peaks in each frame 
	void tdamp_to_frame_corpeak_normalize(struct fras* f, long Mframe);

	// get x,y,z position of source or image source from frame time of arrival and frame incidence angle (if loudspeaker on body, half of the output is the wall distance in case of first reflection)
	// Nstartm: absolute: maximum peak of frame m
	// Nstart0: abosolute: first frame 
	// Nstart: absolute: empirical first start sample exceeding threshold
	// Nlatency: relative: latency of audio device
	void Nstart_to_frame_xyz(struct fras* f, struct heads* h, struct par6s* p, long Nstartm, long Nstart0, long Nstart, long Nlatency, bool sync, long m, long sample_rate, float speed_of_sound);

#ifdef __cplusplus
};
#endif
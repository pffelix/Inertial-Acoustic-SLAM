// offline.h: Interface for saving and reading sensor data for offline usage
// Copyright 2020, Felix Pfreundtner, All rights reserved.

#pragma once
#include "params.h"
#include <stdio.h>
#include "array.h"
#include <stdlib.h>
#include "debugging.h"

#ifdef __cplusplus
extern "C" {
#endif

	// save headtracker recording and imu data to binary wav files
	void save_head_tracker_disk();

	// read headtracker recording and imu data from binary wav files
	void read_head_tracker_disk();

	// depreciated (slow, binaural txt read in)
	/////////////////////////////////////////////////////////////

	// read headtracker recording data from txt files
	void read_head_tracker_disk_old(); // depreciated

	// read params input from disk interface
	void read_interface_disk_params_old(long *rec_N_ptr, char *interface_path_params);  // depreciated

	// read recording input from disk interface
	void read_interface_disk_rec_old(float* rec_l, float *rec_r, char *interface_path_rec_l, char *interface_path_rec_r);  // depreciated

#ifdef __cplusplus
};
#endif
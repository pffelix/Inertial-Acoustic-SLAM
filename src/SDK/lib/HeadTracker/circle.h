// circle.h: Circle
// Copyright 2020, Felix Pfreundtner, All rights reserved.

#pragma once
//#include "debugging.h"
#include "params.h"
#include "head.h"
#include "rotation.h"

#ifdef __cplusplus
extern "C" {
#endif

	// initialize mapping between bins of x and y cross-correlation with lenght N, with relevant bins Nlim left and right of (N-1)/2 and probablity
	int** init_cir_az_mapping(int Nxy, int Nxylim, int* prob_az_N, bool flip_x_plane, bool flip_y_plane);

	// initialize linear mapping between bins of cross-correlation with lenght N and probablity
	int* init_cir_linear_mapping(int N, int Nlim, int* prob_N, bool flip_plane);

	// add absolute probability observations (cor_x, cor_y, cor_z for azimuth, elevation dimension)
	void add_cir_ref(struct cirs* c, float* cor_x, float* cor_y, float* cor_z, int frame_m, float ymax_ang, bool binarymax_instead_of_y, bool flip_y_plane);

	// add differential probability observations (mtd for x,y,z dimension, differential cor_x, cor_y, cor_z for azimuth, elevation dimension)
	void add_cir_diff(struct cirs* c, float* cor_mtd_x, float* cor_mtd_y, float* cor_mtd_z, struct ffts* fft_tmp, char* fft_library, int frame_m, bool delta_instead_of_diff);

	// get head position and statistics
	void get_cir_head_pos(struct cirs* c, struct heads* h, int frame_m, bool diffmax_instead_of_diffmean, bool delta_instead_of_diff);


#ifdef __cplusplus
};
#endif
// vicon.h: Funcationality for reading vicon ground truth from Vicon Nexus Server with CPP datastream library
// Copyright 2020, Felix Pfreundtner, All rights reserved.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif
	// read position from vicon datastream server
	void get_vicon_pos(float** pos, float** pos_p, int vicon_object_n, int Ndof, int* Ndim_routing, int Nderivative, float* update_Tdiff);

#ifdef __cplusplus
};
#endif
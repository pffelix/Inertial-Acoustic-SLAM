// Copyright 2020, Felix Pfreundtner, All rights reserved.
#include "ViconMain.h"
#include "ViconCWrapper.h"

	void get_vicon_pos(float** pos, float** pos_p, int vicon_object_n, int Ndof, int* Ndim_routing, int Nderivative, float* update_Tdiff){	int d;

	for (d = 0; d < Ndof; d++) {
		// copy old position
		memcpy(pos_p[Ndim_routing[d]], pos[Ndim_routing[d]], Nderivative * sizeof(float));

		// get new position
		pos[Ndim_routing[d]][0] = viconPosition[vicon_object_n][d];

		// conver yaw-pitch-roll coordinates to spherical coordinates
		switch (d){
			case viconEulerYaw:  // azimuth
				if (pos[Ndim_routing[d]][0] < 0.0f) {
					pos[Ndim_routing[d]][0] = 360.0f + pos[Ndim_routing[d]][0];
				}
				break;
			case viconEulerPitch: // elevation
				pos[Ndim_routing[d]][0] = -pos[Ndim_routing[d]][0];
				break;
		}

	}

	// set latency pointer
	update_Tdiff = &viconLatencyTotal;
}
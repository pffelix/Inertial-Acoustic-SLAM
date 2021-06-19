// Copyright 2020, Felix Pfreundtner, All rights reserved.

#include "inertial.h"

void get_imu_pos(struct imus* i) {
	int d;
	float T_convert;

	for (d = 0; d < A_Ndof; d++) {
		// copy old position
		memcpy(i->pos_p[d], i->pos[d], A_Nderivative * sizeof(float));
		 
		// convert acceleration in m/s^2 between two loop updates with measured time update_Tdiff into differential time between 2 update steps with T = 1;
		//T_convert = (*i->update_Tdiff * *i->update_Tdiff) / (i->T * i->T);
		T_convert = 1.0f; //rec_ms;
		switch (A_dim_r[d]){
			case A_x: 
				i->pos[d][A_acc] = ngimuLinearXMean * T_convert; 
				//i->pos[d][A_ref] = ngimuLinearPosX->x[A_ref];
				//i->pos[d][A_vel] = i->pos[d][A_ref] - i->pos_p[d][A_ref];
				break; 
			case A_y:
				i->pos[d][A_acc] = ngimuLinearYMean * T_convert; 
				//i->pos[d][A_ref] = ngimuLinearPosY->x[A_ref];
				//i->pos[d][A_vel] = i->pos[d][A_ref] - i->pos_p[d][A_ref];
				break;
			case A_z:
				i->pos[d][A_acc] = -ngimuLinearZMean * T_convert;
				//i->pos[d][A_acc] = ngimuLinearZ * T_convert; 
				//i->pos[d][A_vel] = i->pos[d][A_ref] - i->pos_p[d][A_ref];
				break;
			case A_az:
				yaw_to_az(ngimuEulerYaw, &i->pos[d][A_ref]); 
				if (!update_first) {
					ang_to_angdiff(i->pos_p[d][A_ref], i->pos[d][A_ref], &i->pos[d][A_vel]);
					angdiff_to_circular360(&i->pos[d][A_vel]);
				}
				break;
			case A_el:
				pitch_to_el(-ngimuEulerPitch, &i->pos[d][A_ref]); 
				if (!update_first) {
					i->pos[d][A_vel] = i->pos[d][A_ref] - i->pos_p[d][A_ref];
				}
				break;
		}
	}
}


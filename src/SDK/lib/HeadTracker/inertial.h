// inertial.h: Funcationality for reading 10 DOF inertial measurement unit (accelerometer, magnetometer, gyroscope, barometer).
// Copyright 2020, Felix Pfreundtner, All rights reserved.

#pragma once
#include "state.h"

#ifdef __cplusplus
extern "C" {
#endif
	void get_imu_pos(struct imus* i);

#ifdef __cplusplus
};
#endif
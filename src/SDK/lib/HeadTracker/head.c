// Copyright 2020, Felix Pfreundtner, All rights reserved.

#include "head.h"


void td_to_ang(float td, float td_maximal_sec, float *ang){
	bool nonlinear = true;
	if (td >= td_maximal_sec) {
		td = td_maximal_sec;
	}
	if (td <= -td_maximal_sec) {
		td = -td_maximal_sec;
	}
	if (nonlinear) {
		*ang = asinf(td / td_maximal_sec);
		*ang = *ang * RAD_TO_DEG;
	}
	else {
		*ang = td / td_maximal_sec * 90.0f;
	}
};

void ld_to_ang(float ld, float ld_maximal_db, float *ang){
	bool nonlinear = false;
	if (ld > ld_maximal_db) {
		ld = ld_maximal_db;
	}
	if (ld < -ld_maximal_db) {
		ld= -ld_maximal_db;
	}
	if (nonlinear) {
		*ang = asinf(ld / ld_maximal_db);
		*ang = *ang * RAD_TO_DEG;
	}
	else {
		*ang = ld / ld_maximal_db * 90.0f;
	}
};

void ang_to_angdiff(float ang1, float ang2, float *angdiff) {
	*angdiff = ang2 - ang1;
}

void angdiff_to_circular360(float* angdiff){
	// modified from: https://stackoverflow.com/questions/7570808/how-do-i-calculate-the-difference-of-two-angle-measures/30887154
	float r, sign;
	//calculate circular difference 
	modulusf(*angdiff, 360, &r); 
	r = r > 180.0f ? 360.0f - r : r;

	//calculate sign 
	sign = (*angdiff >= 0.0f && *angdiff <= 180.0f) || (*angdiff <=-180.0f && *angdiff>= -360.0f) ? 1.0f : -1.0f; 
	*angdiff = r * sign;
}

void ang_to_tdn(float ang, long N_lim, long* tdn) {
	*tdn = (long)roundf(ang / 90.0f * (float)N_lim);
}

void ang_sph_to_ipc(float sph_az, float sph_el, float* ipc_az, float* ipc_el) {
	//sph_az = 60.0f;
	//sph_el = -10.0f;

	// 360 degree azimuth range
	modulusf(sph_az, 360, &sph_az);

	// to radians
	float sph_az_rad = deg_to_rad(sph_az);
	float sph_el_rad =  deg_to_rad(sph_el);

	//if back plane
	if (sph_az > 90.0f && sph_az < 270.0f) {
		*ipc_az = - asinf(cosf(sph_el_rad) * sinf(sph_az_rad));
		if (sph_el == 90.0f || sph_el == -90.0f) {
			sph_el_rad = -sph_el_rad; // numerical error at edge of acot
		}
		*ipc_el = PI - acot(1.0f / tanf(sph_el_rad) * cosf(sph_az_rad - PI));
	}

	//if front plane
	else {
		*ipc_az = asinf(cosf(sph_el_rad) * sinf(sph_az_rad));
		if (sph_el == 90.0f || sph_el == -90.0f) {
			sph_el_rad = -sph_el_rad; // numerical error at edge of acot
		}
		*ipc_el = acot(1.0f / tanf(sph_el_rad) * cosf(sph_az_rad));
	}

	// to degree
	*ipc_az = rad_to_deg(*ipc_az);
	*ipc_el = rad_to_deg(*ipc_el);

}
void ang_ipc_to_sph(float ipc_az, float ipc_el, float* sph_az, float* sph_el) {

	// to radians
	float ipc_az_rad = deg_to_rad(ipc_az);
	float ipc_el_rad =  deg_to_rad(ipc_el);


	//if back plane
	if (ipc_el > 90.0f) {
		*sph_el = asinf(cosf(ipc_az_rad) * sinf(ipc_el_rad));
		*sph_az = acot(1.0f / tanf(ipc_az_rad) * cosf(-ipc_el_rad + PI)) + PI;
		*sph_el = rad_to_deg(*sph_el);
		*sph_az = rad_to_deg(*sph_az);
	}

	//if front plane
	else {
		*sph_el = asinf(cosf(ipc_az_rad) * sinf(ipc_el_rad));
		if (ipc_az == 90.0f || ipc_az == -90.0f) {
			ipc_az_rad = -ipc_az_rad; // numerical error at edge of acot
		}
		*sph_az = acot(1.0f / tanf(ipc_az_rad) * cosf(ipc_el_rad));
		*sph_el = rad_to_deg(*sph_el);
		*sph_az = rad_to_deg(*sph_az);
		modulusf(*sph_az, 360, sph_az);
	}
}

void atan2_to_az(float* az) {
	modulusf(-*az + 90.0f, 360, az);
}

void az90_to_az(float* az) {
	modulusf(*az - 90.0f, 360, az);
}

void az_to_az90(float* az) {
	modulusf(*az + 90.0f, 360, az);
}

void az_to_azsigned(float* ang_az) {
	if (*ang_az > 180.0f) {
		*ang_az = *ang_az - 360.0f;
	}
}

void azsigned_to_az(float* ang_az) {
	if (*ang_az < 0.0f) {
		*ang_az = 360.0f + *ang_az;
	}
}

void el_to_elsigned(float* ang_el) {
	*ang_el = -*ang_el + 90.0f;
}

void elsigned_to_el(float* ang_el) {
	*ang_el = -*ang_el + 90.0f;
}

void elbin_to_elsigned(long bin_el, float* ang_el, int sph_ang_stepsize) {
	*ang_el = (float)(-bin_el * sph_ang_stepsize + 90);
}

void yaw_to_az(float yaw, float* ang_az) {
	*ang_az = -yaw;
	azsigned_to_az(ang_az);
}

void pitch_to_el(float pitch, float* ang_el) {
	*ang_el = -pitch;
	elsigned_to_el(ang_el);
}

void yawpitch_to_sph_slow(float yaw, float pitch, float* az, float* el) {
	
	// change signs
	yaw = yaw;

	// to radians
	float sph_yaw_rad = deg_to_rad(yaw);
	float sph_pitch_rad =  deg_to_rad(pitch);

	float x = sinf(sph_pitch_rad) * sinf(sph_yaw_rad); //sinf()* instead of cosf()* as yaw not starting at 90°
	float y = sinf(sph_pitch_rad) * cosf(sph_yaw_rad); //cosf()* instead of sinf()* as yaw not starting at 90°
	float z = cosf(sph_pitch_rad);

	float rd;
	cart_to_sph(x, y, z, az, el, &rd);
	//modulusf(yaw, 360, az);
	//*el = -pitch;
}

void cart_to_sph(float x, float y, float z, float* az, float* el, float* rd)
{
    //Convert from cartesian to spherical
    *rd = sqrtf(x*x + y*y + z*z);
    *el = acosf(z / *rd); // asinf(x / *rd);
    *az = atan2f(y, x); // atan2f(z, y)
	//if (*az < 0.0f) { // y < 0.0f && z < 0.0f
		//*az += 2 * PI; // *el += 2 * PI
	//}

	// to degree
	*az = rad_to_deg(*az);
	*el = rad_to_deg(*el);

	// to azimuth [0...360]
	atan2_to_az(az);

}

void sph_to_cart(float az, float el, float rd, float* x, float* y, float* z)
{
	// to radians
	float sph_az_rad;
	float sph_el_rad;
	sph_az_rad = deg_to_rad(az);
	sph_el_rad = deg_to_rad(el);

    //convert from spherical to cartesian
    *x = rd * sinf(sph_el_rad) * sinf(sph_az_rad);
    *y = rd * sinf(sph_el_rad) * cosf(sph_az_rad);
    *z = rd * cosf(sph_el_rad);

	// correct edge cases (with numerical error)
	if (el == 180.0f || el == 0.0f ) {
		//if (az >= 0.0f && az <= 180.0f && *x < 0.0f) {
		//	*x = -*x;
		//}
		//if (az >= 180.0f && az < 360.0f && *x > 0.0f) {
		//	*x = -*x;
		//}
		//if (az >= 270.0f || az <= 90.0f && *y < 0.0f) {
		//	*y = -*y;
		//}
		//if (az >= 90.0f && az <= 270.0f && *y > 0.0f) {
		//	*y = -*y;
		//}
		*x = 0.0f;
		*y = 0.0f;
	}
}
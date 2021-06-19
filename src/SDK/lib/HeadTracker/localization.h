// localization.h: Calculation of wave source location based on time and level differences
// Copyright 2020, Felix Pfreundtner, All rights reserved.
#pragma once
#include "array.h"
#include "debugging.h"

#ifdef __cplusplus
extern "C" {
#endif

	
	//functions
	//////////////////////////////


	// Initialize Potluri (2002) / Fang (1990) image source localization algorithm with position of 4 microphones
	void init_localization(struct locs* loc, float xi1, float xi2, float xi3, float xi4, float yi1, float yi2, float yi3, float yi4, float zi1, float zi2, float zi3, float zi4, float zero_offset, bool td_absolute);
	
	/* Update Potluri (2002) / Fang (1990) image source localization algorithm with position of 4 microphones
	   calculate x,y,z location of source from time of flight differences between 4 microphones and x,y,z position of 4 microphones
	   The 4 time of flight differences Rij expressed in metre are set with loc->Rij and must be derived by the time of arrival ti multiplied by the sound speed sos of following 4 microphone configurations:
	   Rij(1) =  (ti(1) - ti(2)) * sos; // td_ij
	   Rij(2) =  (ti(1) - ti(3)) * sos; // td_ik
	   Rij(3) =  (ti(3) - ti(2)) * sos; // td_kj
	   Rij(4) =  (ti(3) - ti(4)) * sos; // td_kl
	*/
	void update_localization(struct locs * loc);

#ifdef __cplusplus
};
#endif
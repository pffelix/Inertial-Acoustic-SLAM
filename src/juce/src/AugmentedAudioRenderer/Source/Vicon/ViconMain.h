// ViconMain.h: Main script allowing real-time access to Vicon datastream server (compatible with Datastream SDK 1.3)
// Copyright 2020, Felix Pfreundtner, All rights reserved.

#pragma once
#include <sstream> 
#include "Client.h"

#define viconRAD_TO_DEG 57.29577951308232087685F
#define viconGlobalX 0 // x position in global vicon coordinates (metre)
#define viconGlobalY 1 // y position in global vicon coordinates (metre)
#define viconGlobalZ 2 // z position in global vicon coordinates (metre)
#define viconEulerYaw 3 // pitch orientation of the device relative to global vicon coordinates (degree)
#define viconEulerPitch 4 // pitch orientation of the device relative to global vicon coordinates (degree)
#define viconEulerRoll 5 // roll orientation of the device relative to global vicon coordinates (degree)
#define viconNpos 6 // dimension of elements in position pointer
#define viconNobjects 2 // number of objects to be tracked (each object has a combined segement/subject name)

// variables
///////////////////////////////////////////

// paramters
extern char viconHostName[128];
extern char* viconSegmentName[viconNobjects]; // segment names of the objects
extern char* viconSubjectName[viconNobjects];// subject names of the objects

// client
extern ViconDataStreamSDK::CPP::Client* MyClient; // connection client to vicon server
extern bool viconConnected; // connection to vicon server has been established sucessfully

// position of object
extern float** viconPosition; // pointer including all position variables
		
// meta information
extern float viconFrameRate; // frame rate (Hz)
extern float viconLatencyTotal; // latency (s)

// functions
///////////////////////////////////////////

// initialize client to viconHostName server
extern void viconInit();

// update position of objectName
void viconUpdate();

// destroy client to viconHostName server
extern void viconDestroy();

// initialize vicon position reading
float** viconInitPosition();


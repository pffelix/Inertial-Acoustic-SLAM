// Copyright 2020, Felix Pfreundtner, All rights reserved.

#include "ViconMain.h"

// variables
///////////////////////////////////////////

// parameters
char viconHostName[128] = "192.168.1.243:801";
char* viconSegmentName[viconNobjects] = {"loudspeaker", "headtop"};
char* viconSubjectName[viconNobjects] = {"loudspeaker", "headtop"};

// client
ViconDataStreamSDK::CPP::Client* MyClient;
bool viconConnected = false;

// position of object
float** viconPosition = viconInitPosition();

// meta information
float viconFrameRate = 0.0f;
float viconLatencyTotal = 0.0f;


// functions
///////////////////////////////////////////

void viconInit() {

	// generate vicon client
	MyClient = new ViconDataStreamSDK::CPP::Client();

	// connect via Wi-Fi
	ViconDataStreamSDK::CPP::Output_Connect outConnect = MyClient->Connect(viconHostName);
	ViconDataStreamSDK::CPP::Output_IsConnected outIsConnected = MyClient->IsConnected();
	viconConnected = outIsConnected.Connected;

	// set stream mode
	ViconDataStreamSDK::CPP::Output_SetStreamMode outSetStreamMode = MyClient->SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ClientPull);

	// axis mapping
	ViconDataStreamSDK::CPP::Output_SetAxisMapping outSetAxisMapping = MyClient->SetAxisMapping(ViconDataStreamSDK::CPP::Direction::Forward, ViconDataStreamSDK::CPP::Direction::Left, ViconDataStreamSDK::CPP::Direction::Up); // 'Z-up'

	// enable data streams
	ViconDataStreamSDK::CPP::Output_EnableSegmentData outEnableSegmentData = MyClient->EnableSegmentData();
	ViconDataStreamSDK::CPP::Output_EnableMarkerData outEnableMarkerData = MyClient->EnableMarkerData();
	ViconDataStreamSDK::CPP::Output_EnableUnlabeledMarkerData outEnableUnlabeledMarkerData = MyClient->EnableUnlabeledMarkerData();

}

void viconUpdate() {
	// get current frame
	ViconDataStreamSDK::CPP::Output_GetFrame outGetFrame = MyClient->GetFrame();

	for (int n = 0; n < viconNobjects; n++) {
		// get x,y,z translation 
		ViconDataStreamSDK::CPP::Output_GetSegmentGlobalTranslation outGetSegmentGlobalTranslation = MyClient->GetSegmentGlobalTranslation(viconSubjectName[n], viconSegmentName[n]);
		viconPosition[n][viconGlobalX] = (float)outGetSegmentGlobalTranslation.Translation[0] / 1000.f;
		viconPosition[n][viconGlobalY] = (float)outGetSegmentGlobalTranslation.Translation[1] / 1000.f;
		viconPosition[n][viconGlobalZ] = (float)outGetSegmentGlobalTranslation.Translation[2] / 1000.f;


		// get euler angle
		ViconDataStreamSDK::CPP::Output_GetSegmentGlobalRotationEulerXYZ outGetSegmentGlobalRotationEulerXYZ = MyClient->GetSegmentGlobalRotationEulerXYZ(viconSubjectName[n], viconSegmentName[n]);
		viconPosition[n][viconEulerPitch] = (float)outGetSegmentGlobalRotationEulerXYZ.Rotation[0] * viconRAD_TO_DEG;
		viconPosition[n][viconEulerRoll] = (float)outGetSegmentGlobalRotationEulerXYZ.Rotation[1] * viconRAD_TO_DEG;
		viconPosition[n][viconEulerYaw] = (float)outGetSegmentGlobalRotationEulerXYZ.Rotation[2] * viconRAD_TO_DEG;
	}
	// get frame rate
	ViconDataStreamSDK::CPP::Output_GetFrameRate outGetFrameRate = MyClient->GetFrameRate();
	viconFrameRate = (float)outGetFrameRate.FrameRateHz;

	// get latency
	ViconDataStreamSDK::CPP::Output_GetLatencyTotal outGetLatencyTotal = MyClient->GetLatencyTotal();
	viconLatencyTotal = (float)outGetLatencyTotal.Total;
};


void viconDestroy() {

	// disconnect from vicon client
	ViconDataStreamSDK::CPP::Output_Disconnect outDisconnect = MyClient->Disconnect();

	// destroy vicon client
	delete MyClient;

	// free vicon position
	for (int n = 0; n < viconNobjects; n++) {
		free(viconPosition[n]);
	}
}


float** viconInitPosition() {
	float** y = (float**)malloc(viconNobjects * sizeof(float*));
	for (int n = 0; n < viconNobjects; n++) {
		if (y[n]) {
			y[n] = (float*)malloc(viconNpos * sizeof(float));
			for (int d = 0; d < viconNpos; d++) {
				if (y[n][d]) {
					y[n][d] = 0.0f;
				}
			}
		}
	}
	return y;
}
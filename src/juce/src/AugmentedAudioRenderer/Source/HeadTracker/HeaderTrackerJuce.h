// HeaderTrackerJuce.h: Class containg audio buffer loop and routine to regularly start update of head tracker.
// Copyright 2019, Felix Pfreundtner, All rights reserved.

#pragma once
#include "head_tracker.h"
#include "JuceHeader.h"
#include <io.h> 
#include <fcntl.h>
#include "PluginProcessor.h"
#include "PluginEditor.h"
#include "ViconMain.h"



class HeadTracker :    public Thread,
					   public AudioIODeviceCallback,
                       private Timer,
					   public ChangeListener
{
public:
	// constructor
	HeadTracker(TextEditor& resultsBox_r, HrtfBiAuralAudioProcessor& binauralAudioProcessor_r, HrtfBiAuralAudioProcessorEditor& binauralAudioProcessorGui_r)// HrtfBiAuralAudioProcessor& binauralAudioProcessor_r
		: Thread("JUCE Head Tracker Thread"),
		resultsBox(resultsBox_r),
		binauralAudioProcessor(binauralAudioProcessor_r),
		binauralAudioProcessorGui(binauralAudioProcessorGui_r)
    {
		//binauralAudioProcessorGui = binauralAudioProcessorGui_ptr;
	}
	
	// destructor
	~HeadTracker() override
    {
        // give the head tracker thread some milliseconds to stop cleanly
        stopThread (threadStopMs);
    }

public:

	// parameters
	/////////////////////////////////////////////////////////////////////////////
	long count = 0;
	int trackerType = Mixed; // 0: Mixed, 1: Acoustic, 2: Imu
	int guiShow = GUI_Slam; // GUI_Imu, GUI_Slam, GUI_Truth_is, GUI_Truth_rec (defintion see params.h)
	
	// vicon
	bool viconOn = false; // activate Vicon infrared ground thruth
	bool viconInitialized = false; // status variable: vicon active
	bool viconDataTestWait = false; // test vicon data whether it is 0
	bool viconDataTestPlot = true; // plot whether vicon data is 0
	bool viconTruthShow = true; // show truth head orientation in GUI
	float viconTruthFirstValue = 0.0f; // first value truth head orientation in GUI

	// acoustic head tracker
	bool recordingDataAvailable = false; // status variable: recording buffer has been filled
	bool recordingDataFirstLoopFinished = false; // status variable: first recording loop finished
	bool recordingDataSecondLoopFinished = false; // status variable: first recording loop finished
	bool trackerIsRunning = false; // status variable: head tracker has been started
	bool trackerUpdateIsRunning = false; // status variable: head tracker is currently updating estimation
	bool impulsePlayback = false; // status variable: playback of impulse
	
	// intertial measurement unit (imu) head tracker
	String imuSerialPortName = "COM11"; // name of imu serial port 
	bool ngimuBiasCorrection = true; // correct bias (When "start head tracker button is pressed" NGIMU hast to be in rest and in the correct pitch and roll coordinate)
	bool ngimuWaitInitialized = FRAMEWORK_OFFLINEREAD ? false : true; // wait until NGIMU has initalized before acoustic head tracking is started
	bool ngimu = true; // X-IO NGIMU is used (true), otherwise Movsense TrackIMU (false)
	bool imuUdp = false; // Imu read protocol: udp (true), otherwise Serial (false)
	int imuUdpPort  = 8001; // UDP (opentrack) input port of TrackIMU: 4242, NGIMU: 8001

	// threads
	int threadPriority = 10; // priority of head tracker thread: 10 highest, 1 lowest
	int threadStopMs = 1000; // time to let thread finish after stop call has been performed


    // Methods to begin and stop head tracking after GUI button has been pressed.
	////////////////////////////////////////////////////////////////////////////////////////////////////
	void beginTracking()
	{

		// init parameters and program state of head tracker
		insertText(newLine + newLine + "Run head tracking" + newLine);
		insertText("Initialize Head Tracker");

		// starthead tracker update routine
		if (trackerType == Imu || trackerType == Mixed)
		{
			beginImuTracking();
		}
		if (trackerType == Acoustic || trackerType == Mixed)
		{
			// start computational intensive acoustic head tracker update routine in a separated thread
			init_head_tracker();
			startThread(threadPriority);
		}

		// connect Vicon
		if (viconOn) {
			console_log(PRIO_MAIN, "Connect to Vicon\n");
			viconInit();
			if (!viconConnected) {
				console_log(PRIO_MAIN, "Connect to Vicon failed\n");
			}
			else {
				viconInitialized = true;
			}

		}

		// regularly update GUI with new head position (JUCE messaging thread, lower priority and shared ressources with GUI)
		startTimer((int)update_ms);


		// show that head tracker is running
		trackerIsRunning = true;

		// show that first and second recording buffer loop has not finished
		recordingDataFirstLoopFinished = false;
		recordingDataSecondLoopFinished = false;
    }
	
	void stopTracking()
    {
		// stop starthead tracker update routine
		if (trackerType == Acoustic || trackerType == Mixed) 
		{
			stopAcousticTracking();
		}
		if (trackerType == Imu || trackerType == Mixed)
		{
			stopImuTracking();
		}

		// disconnect Vicon
		if (viconOn) {
			viconDestroy();
		}

		// print that head tracking has finished
		insertText(newLine + "Head tracking finished" + newLine);
    }


	
	// Time thread callback function, getting called every update_ms milliseconds, can be used to start a second thread beside head tracker thread (runs on JUCE messaging thread, lower priority and shared ressources with GUI)
	void timerCallback() override
	{

		// update imu udp tracking (acoustic tracking update via callback):
		if (trackerType == Imu || trackerType == Mixed)
		{
			updateImuTracking();
		}

		// update GUI and binaural renderer
		binauralAudioProcessorGui.updateSourcePosition(headX, headY, headZ, deg_to_rad(headAz), deg_to_rad(headEl), headRd, wall, par6->obs_weight, wallN);  // x,y,z= [0...100] azimuth=[0...pi,-pi..0], elevation=[-pi/2...pi/2]
	}


	// Methods to control acoustic head tracker update procedure
	////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// Head tracker thread, called once, to start the head tracker update routine in a while loop (runs on own thread, priority can be set)
	void run() override
	{
		// if Vicon is connected
		if (viconOn) {
			while (!viconInitialized) {
				Time::waitForMillisecondCounter(Time::getMillisecondCounter() + 1);
				console_log(PRIO_MAIN, "Wait for VICON to connect \n");
			}
		}

		// if IMU is connected
		if (trackerType == Imu || trackerType == Mixed){
			// wait until imu has initialized
			if (ngimuWaitInitialized) {
				while (!ngimuInitialized) {
					Time::waitForMillisecondCounter(Time::getMillisecondCounter() + 1);
					console_log(PRIO_MAIN, "Wait for IMU to connect \n");
				}
			}
			// calibrate imu
			if (ngimuBiasCorrection) {
				ngimuLinearCalibrateInit();
				for (int n = 0; n < ngimuLinearBiasBufferN; n++) {
					ngimuLinearCalibrateUpdate();
					Time::waitForMillisecondCounter(Time::getMillisecondCounter() + (int)ngimuUpdateRate_ms);
				}
				ngimuLinearCalibrateFinish();
			}
		}

		// Continually update
		while (!threadShouldExit())
		{
			// vicon ground truth
			if (viconOn && viconConnected) {
				viconUpdate();
				if (viconDataTestWait) {
					for (int n = 0; n < viconNobjects; n++) {
						if (viconPosition[n][viconGlobalX] == 0.0f) {
							while (viconPosition[n][viconGlobalX] == 0.0f) {
								Time::waitForMillisecondCounter(Time::getMillisecondCounter() + 1);
								console_logi(PRIO_MAIN, "Vicon cannot read object %d\n", n);
							}
						}
					}
				}
			}
			// head tracker estimation
			updateAcousticTracking();
			//Time::waitForMillisecondCounter(Time::getMillisecondCounter() + 1);

		}
	}
	
	void updateAcousticTracking() {
		
		// if new recording data is available
		if (recordingDataAvailable) {
				// start head tracker update
				// audio device playback and recroding: via audioDeviceIOCallback()

				// show that head tracker update is running
				//trackerUpdateIsRunning = true;

				// debug
				//write_wav_debug(rec->ptr, rec_N, ch_Nrec, 0);
				
				// start timer measuring the time in milliseconds each head tracker update takes
				long long start = timer_tick_ll();
				// show that head tracker update is running
				trackerUpdateIsRunning = true;

				// Perfom head position calculation update
				update_head_tracker_state();

				// stop timer
				double timerDiff = timer_stop_ll(start);
				
				// show that head position calculation is no waiting for new recording data anymore
				recordingDataAvailable = false;

				// show that head tracker update is not running anymore
				trackerUpdateIsRunning = false;

				// update GUI head position information
				switch (guiShow) {
					case GUI_Imu:
						if (ngimu) {
							headAz = imu->pos[A_dim[A_az]][A_ref];
							headEl = imu->pos[A_dim[A_el]][A_ref];
						}
						else {
							yawpitch_to_sph_slow(imuYaw, imuPitch, &headAz, &headEl);
						}
						break;
					case GUI_Slam:
						headAz = par6->pos[A_dim[A_az]][A_ref]; // kalaz->x[0];
						headEl = par6->pos[A_dim[A_el]][A_ref]; // kalel->x[0];
						headRd = par6->pos[A_dim[A_rd]][A_ref]; // kalel->x[0];
						headAz = isinf(headAz) || isnan(headAz) ? 0.0f : headAz;
						headEl = isinf(headEl) || isnan(headEl) ? 0.0f : headEl;
						if (A_Ndof == 6) {
							if (viconTruthShow) {
								if (viconTruthFirstValue == 0.0f) {
									viconTruthFirstValue = vic->pos_rec[A_dim[A_az]][A_ref];
								}
								headX = sinf(deg_to_rad(vic->pos_rec[A_dim[A_az]][A_ref] - viconTruthFirstValue)) * 2.0f;
								headY = cosf(deg_to_rad(vic->pos_rec[A_dim[A_az]][A_ref] - viconTruthFirstValue)) * 2.0f;
								headZ = 0.0f;
							}
							else {
								headX = par6->pos[A_dim[A_x]][A_ref];
								headY = par6->pos[A_dim[A_y]][A_ref];
								headZ = par6->pos[A_dim[A_z]][A_ref];
							}
							int w = 0;
							if (A_Ndof == A_Ndim) {
								for (int m = 0; m < frame_M; m++) {
									if (fra->m[m]) { // frame active
										for (int d = 0; d < A_Ndof; d++) {
											wall[w][d] = par6->obs[m].kal[d].x[A_ref];
										}
										w += 1;
									}
								}
							}
						}
						break;
					case GUI_Truth_is:
						headAz = vic->pos_is[A_dim[A_az]][A_ref];
						headEl = vic->pos_is[A_dim[A_el]][A_ref];
						headX = vic->pos_is[A_dim[A_x]][A_ref];
						headY = vic->pos_is[A_dim[A_y]][A_ref];
						headZ = vic->pos_is[A_dim[A_z]][A_ref];
						break;
					case GUI_Truth_rec:
						headAz = vic->pos_rec[A_dim[A_az]][A_ref];
						headEl = vic->pos_rec[A_dim[A_el]][A_ref];
						headX = vic->pos_rec[A_dim[A_x]][A_ref];
						headY = vic->pos_rec[A_dim[A_y]][A_ref];
						headZ = vic->pos_rec[A_dim[A_z]][A_ref];
						break;
				}
				el_to_elsigned(&headEl);
				az_to_azsigned(&headAz);

				// print tracking result
				console_logf(PRIO_MAIN, "Current head orientation az: %3.1f degree\n", headAz);
				console_logf(PRIO_MAIN, "Current head orientation el: %3.1f degree\n", headEl);
				//console_logf(PRIO_MAIN, "Estimated head orientation: %3.1f degree\n", kal->x[0]);
				console_logf(PRIO_LATENCY, "Latency: %3.0f ms\n", fabsf((float)update_Tdiff * 1000.0f));
				console_logi(PRIO_MAIN, "Frames active: %i\n", fra->m_Non);
				console_logi(PRIO_MAIN, "Frame start: %i\n", fra->m_energyfirst + 1);
				if (viconDataTestPlot) {
					for (int n = 0; n < viconNobjects; n++) {
						if (viconPosition[n][viconGlobalX] == 0.0f) {
							console_logi(PRIO_MAIN, "Vicon cannot read object %d\n", n);
						}
					}
				}

				//insertText(newLine + "Energy=" + String(fe->energy_global) + ", " + newLine);


		}
	};

	// Function, stops acoustic head tracking
	void stopAcousticTracking()
    {
		// set the head tracker state variables
		trackerIsRunning = false;		
		trackerUpdateIsRunning = false;

		// stop timer thread
		stopTimer();

		// stop separated head tracker thread
		stopThread (threadStopMs);
    }


	// JUCE callback function, getting called when audio device has been succesfully initialized and is read for receiving samples in audioDeviceIOCallback()
    void audioDeviceAboutToStart (AudioIODevice* device) override
    {
		// prepare head tracker
        trackerIsRunning = false;
		playBuffer_n = 0;
		recBuffer_n = 0;

		// perpare binauralization
		if (binaural_on) {
			binauralAudioProcessor.prepareToPlay(device->getCurrentSampleRate(), device->getDefaultBufferSize());
			binauralBuffer.setSize(2, device->getDefaultBufferSize(), false, true, false); // generate binaural block
		}

    }

	// JUCE callback function, getting called when audio device has stopped working
    void audioDeviceStopped() override {}

	// JUCE callback function, getting called after audio buffer needs to be refreshed, allows to play and record new audio samples
	void audioDeviceIOCallback(const float** inputChannelData, int numInputChannels,
		float** outputChannelData, int numOutputChannels, int numSamples) override
	{

		if (trackerIsRunning)
		{
			// binaural spatialize play buffer
			/////////////////////////////////////////////////////////////////////////////
			if (binaural_on && (numInputChannels - 1 >= A_sound)){ // sound channel must be behind the microphone channels
				// copy input signal block to be binauralized to channel 0 of binaural output block
				binauralBuffer.copyFrom(0, 0, inputChannelData[A_sound], numSamples);
				binauralAudioProcessor.processBlock(binauralBuffer, midiBuffer);
			}

			// rapid impulse response measurement technique
			/////////////////////////////////////////////////////////////////////////////
			if (rapid_on && !trackerUpdateIsRunning && recordingDataSecondLoopFinished) {
				if (rapid_edge_on && (recBuffer_n < rapid_edge_N || recBuffer_n >= recbuffer_N - rapid_edge_N)) {
					// pass
				}
				else {
					// flush the recording buffer to the internal head tracker recording pointer
					// ask_lock(&lock_rec);
					for (int ch = 0; ch < ch_Nrec; ch++) {
						memcpy(rec->x[R_rec[ch]], recbuffer->x[ch], recbuffer_N * sizeof(float));
						if (rapid_snrfull) {
							memcpy(rec->x[R_rec[ch]] + recbuffer_N, recbuffer->x[ch], recbuffer_N * sizeof(float));
						}
					}
					// close_lock(&lock_rec);
					// save index expressing the position where the impulse has stopped playing 
					rapid_n = recBuffer_n;
					// show that new recording data is available and can be used for head tracker calculations in timerCallback()
					recordingDataAvailable = true;
				}
			}

			// conventional impulse response measurement technique
			/////////////////////////////////////////////////////////////////////////////

			// update recording pointer after the amount of audio buffers have been passed that correspond to the impulse response length
			for (sampleN = 0; sampleN < numSamples; ++sampleN)
			{

				// for debugging purposes
				/////////////////////////////////////////////////////////////////////////////
				// stop impulse playback and recording while head position calculation runs
				if (deb_on && deb_stopimpulse && recordingDataAvailable && !rapid_on) {
					// reset play buffer to 0 position to stop impulse being played back
					playBuffer_n = 0;
					if (!sync_on) {
						recBuffer_n = 0; // before: deactivated (then click noise as play is in middle when recbuffer_N is full and resetted, but for first samples was working for sync_on)
						//impulsePlayback = false;
					}
				}

				// recording buffer:
				/////////////////////////////////////////////////////////////////////////////

				// read from JUCE audio input buffer
				for (int ch = ch_Nrec; --ch >= 0;)
					if (inputChannelData[ch] != nullptr) {
						recbuffer->x[ch][recBuffer_n] = inputChannelData[ch][sampleN];
					}

				// play buffer
				/////////////////////////////////////////////////////////////////////////////

				// calculate output sample: play sample or play nothing
				// write to JUCE audio output buffer
				for (int ch = numOutputChannels; --ch >= 0;)
					if (outputChannelData[ch] != nullptr){
						outputChannelData[ch][sampleN] = 0.0f; //0.0f;
						if (ch == A_is) {
							outputChannelData[ch][sampleN] = (playBuffer_n < is_N) ? is->send[playBuffer_n] : 0.0f; // && impulsePlayback
						}
						if (binaural_on) {
							if (ch == A_bisoundl) {
								outputChannelData[ch][sampleN] = binauralBuffer.getSample(0, sampleN);
							}
							if (ch == A_bisoundr) {
								outputChannelData[ch][sampleN] = binauralBuffer.getSample(1, sampleN);
							}
						}
					}


				// increment recording and play buffer index
				/////////////////////////////////////////////////////////////////////////////
				++recBuffer_n;
				++playBuffer_n;

				// if recording buffer is full : update recording pointer, loop the playback and restart the recording
				/////////////////////////////////////////////////////////////////////////////
				if (recBuffer_n == recbuffer_N) {

					// reset recording and play buffer index
					playBuffer_n = 0;
					recBuffer_n = 0;
					if (recordingDataFirstLoopFinished) {
						recordingDataSecondLoopFinished = true;
					}
					recordingDataFirstLoopFinished = true;

					// flushing the recording buffer to the internal head tracker recording pointer
					if (!rapid_on) {
						ask_lock(&lock_rec);
						// flush the recording buffer to the internal head tracker recording pointer
						for (int ch = 0; ch < ch_Nrec; ++ch) {
							memcpy(rec->x[R_rec[ch]], recbuffer->x[ch], recbuffer_N * sizeof(float));
						}
						close_lock(&lock_rec);

						// show that new recording data is available and can be used for head tracker calculations in timerCallback()
						recordingDataAvailable = true;
					}
				}

			}
		}
		else
		{
			// We need to clear the output buffers, in case they're full of junk..
			for (int i = 0; i < numOutputChannels; ++i)
				if (outputChannelData[i] != nullptr)
					ar_set(outputChannelData[i], (long)numSamples, 0.0f);
		}
	}

	// Methods to control inertial measurement unit head tracker start and update procedure
	////////////////////////////////////////////////////////////////////////////////////////////////////
	void beginImuTracking() {
		// add console for debugging (only windows)
		console_add();

		// init imu
		if (imuUdp) {
			beginImuUdpTracking();
		}
		else {
			beginImuSerialTracking();
		}
	}
	void updateImuTracking() {
		
		// get current yaw pitch roll angles
		if (imuUdp) {
			updateImuUdpTracking();
		}
		else {
			// not needed: via updateImuSerialTracking() triggered by changeListenerCallback() at every new serial line
		}

		// print tracking result
		console_logf(PRIO_IMU," Current head orientation yaw: %3.1f degree\n", imuYaw);
		console_logf(PRIO_IMU," Current head orientation pitch: %3.1f degree\n", imuPitch);
		console_logf(PRIO_IMU," Current head orientation roll: %3.1f degree\n", imuRoll);
		console_logf(PRIO_IMU," Current acceleration x: %3.4f m/s^2\n", imuX);
		console_logf(PRIO_IMU," Current acceleration y: %3.4f m/s^2\n", imuY);
		console_logf(PRIO_IMU," Current acceleration z: %3.4f m/s^2\n", imuZ);

	}

	void stopImuTracking() {
		if (imuUdp) {
			// not needed
		}
		else {
			stopImuSerialtracking();
		}
	}

	// Methods to control imu serial interface start and update procedure
	////////////////////////////////////////////////////////////////////////////////////////////////////

	void beginImuSerialTracking() {
		imuSerialOn = false;
		imuSerialPort = beginSerialPort(imuSerialPortName);
		if (imuSerialPort) {
			imuSerialInputStream = beginSerialPortInputStream(imuSerialPort);
			if (imuSerialInputStream) {
				insertText(newLine + "IMU Serial stream was found at port " + String(imuSerialPortName) + newLine);
				if (ngimu) {
					//Initialize and assign NGIMU receive callback functions
					NgimuReceiveInitialise();
					//NgimuReceiveSetReceiveErrorCallback(ngimuReceiveErrorCallback);
					//NgimuReceiveSetSensorsCallback(ngimuSensorsCallback);
					//NgimuReceiveSetQuaternionCallback(ngimuQuaternionCallback);
					NgimuReceiveSetEulerCallback(ngimuEulerCallback);
					NgimuReceiveSetLinearCallback(ngimuLinearCallback);
					NgimuReceiveSetEarthCallback(ngimuEarthCallback);
					//ngimuLinearPosCallbackInit();
					ngimuLinearMeanCallbackInit();
				}
				imuSerialOn = true;


			}
			else {
				console_log(PRIO_WARNING, "IMU head tracker not connected via specified serial port");
			}
		}
	};

	void updateImuSerialTracking() {
		if (imuSerialOn) {
			if (ngimu) {
				char c;
				while (!imuSerialInputStream->isExhausted()) { // for NGIMU end character is (char)0xC0
					imuSerialInputStream->read(&c, 1);
					NgimuReceiveProcessSerialByte(c);
				}
				imuYaw = ngimuEulerYaw;
				imuPitch = ngimuEulerPitch;
				imuRoll = ngimuEulerRoll;
				imuX = ngimuLinearX;
				imuY = ngimuLinearY;
				imuZ = ngimuLinearZ;
			}
			else {
				String serialString = imuSerialInputStream->readNextLine();
				auto serialStringSplit = StringArray::fromTokens(serialString, "\t", "");
				int serialStringSplitN = serialStringSplit.size();
				if (serialStringSplitN == 4) {
					for (int i = 1; i < serialStringSplitN; i++)
					{
						String s = serialStringSplit[i]; // holds next token
						float f = s.getFloatValue();
						if (i == 1) {
							imuYaw = f;
						}
						if (i == 2) {
							imuPitch = f;
						}
						if (i == 3) {
							imuRoll = f;
						}
					}
				}
			}
		}
	}

	void stopImuSerialtracking() {
		if (imuSerialOn) {
			stopSerialTracking(imuSerialPort, imuSerialInputStream);
			imuSerialOn = false;
		}
	}
	

	// Methods to control serial interface start and update procedure
	////////////////////////////////////////////////////////////////////////////////////////////////////
	
	SerialPort* beginSerialPort(String serialPortName) {
		SerialPort* serialPort;
		//get a list of serial ports installed on the system, as a StringPairArray containing a friendly name and the port path
		StringPairArray serialPortList = SerialPort::getSerialPortPaths();
		StringArray serialPortDevices = serialPortList.getAllValues();
		int serialPortDevicesN = serialPortList.size();
		if (serialPortDevicesN)
		{
			int serialPortNumber;
			//open the selected port on the system
			for (serialPortNumber = 0; serialPortNumber < serialPortDevicesN; serialPortNumber++) {
				String aaa = serialPortDevices[serialPortNumber];
				if (serialPortDevices[serialPortNumber].containsWholeWord(serialPortName)) {
					break;
				}
			}
			serialPort = new SerialPort(serialPortDevices[serialPortNumber], SerialPortConfig(115200, 8, SerialPortConfig::SERIALPORT_PARITY_NONE, SerialPortConfig::STOPBITS_2, SerialPortConfig::FLOWCONTROL_NONE));
			return serialPort;
		}
		else {
			return 0;
		}
	}

	SerialPortInputStream* beginSerialPortInputStream(SerialPort* serialPort) {
		SerialPortInputStream* serialInputStream;
		bool serialPortExists = serialPort->exists();
		if(serialPortExists)
		{
			//create streams for reading/writing
			serialInputStream = new SerialPortInputStream(serialPort);

			//wait for imu run() thread to attach
			Time::waitForMillisecondCounter(10);

			//or ask to be notified when a new line is available:
			serialInputStream->addChangeListener(this); //we must be a ChangeListener to receive notifications, takes long to initialize callback > 100ms as waiting for next (first) line
			serialInputStream->setNotify(SerialPortInputStream::NOTIFY_ON_CHAR, '\n');
			return serialInputStream;

		}
		else {
			return 0;
		}
	}

	void updateSerialTracking() {
		if (imuSerialOn) {
			updateImuSerialTracking();
		}
	}

	void changeListenerCallback(ChangeBroadcaster* source)
	{
		ignoreUnused(source);
		updateSerialTracking();
	}

	void stopSerialTracking(SerialPort* serialPort, SerialPortInputStream* serialInputStream) {
		imuSerialOn = false;
		Time::waitForMillisecondCounter(10);
		serialInputStream->removeChangeListener(this);
		serialPort->~SerialPort();
		serialInputStream->~SerialPortInputStream();
	}

	// Methods to control imu udp interface head tracker start and update procedure
	////////////////////////////////////////////////////////////////////////////////////////////////////

	void beginImuUdpTracking() {

		connectImuUdp();
		if (imuUdpPortOpened) {
			insertText(newLine + "IMU UDP stream was found at localhost port " + String(imuUdpPort) + newLine);
			if (ngimu) {
				//Initialize and assign NGIMU receive callback functions
				NgimuReceiveInitialise();
				//NgimuReceiveSetReceiveErrorCallback(ngimuReceiveErrorCallback);
				//NgimuReceiveSetSensorsCallback(ngimuSensorsCallback);
				//NgimuReceiveSetQuaternionCallback(ngimuQuaternionCallback);
				NgimuReceiveSetEulerCallback(ngimuEulerCallback);
				NgimuReceiveSetLinearCallback(ngimuLinearCallback);
				NgimuReceiveSetEarthCallback(ngimuEarthCallback);
				//ngimuLinearPosCallbackInit();
				ngimuLinearMeanCallbackInit();


			}
		}
		else {
			insertText(newLine + "IMU UDP stream could not be found at localhost port " + String(imuUdpPort) + newLine);
		}
		disconnectImuUdp();
	}



	void updateImuUdpTracking() {

		int bytesReadN;
		char* udpBufferPtr;
		// read from udp connection into buffer
		//imuUdpPortOpened = imuUdpSocket->bindToPort(imuUdpPort);
		connectImuUdp();
		if(imuUdpPortOpened)
		{
			udpBufferPtr = udpBuffer->begin();
			bytesReadN = imuUdpSocket->read(udpBufferPtr, udpBufferBytesN, false); 
			if (bytesReadN > 0) { // before bytesReadN == udpBufferBytesN, but number of bytes varies at ngimu, so set to > 0
				imuBytesOk = true;
			}
			else {
				imuBytesOk = false;
				console_log(PRIO_IMU, "IMU UDP stream could not be read. Restart computer might help.\n");
			}
		}

		if (ngimu) {
			NgimuReceiveProcessUdpPacket(udpBufferPtr, udpBufferBytesN);
			imuYaw = ngimuEulerYaw;
			imuPitch = ngimuEulerPitch;
			imuRoll = ngimuEulerRoll;
			imuX = ngimuLinearX;
			imuY = ngimuLinearY;
			imuZ = ngimuLinearZ;
		}
		else {
			//else
			if (count == 3)
			{
				udpBuffer->fillWith(0);
			}
			count = count + 1;

			// read binary input buffer into input stream to calculate double head position
			udpBufferStream->setPosition(0);
			imuX = (float)udpBufferStream->readDouble();
			imuY = (float)udpBufferStream->readDouble();
			imuZ = (float)udpBufferStream->readDouble();
			imuYaw = (float)udpBufferStream->readDouble();
			imuPitch = (float)udpBufferStream->readDouble();
			imuRoll = (float)udpBufferStream->readDouble();
		}
		disconnectImuUdp();

	}

	void connectImuUdp() {
		bool portReused = false;
		imuUdpSocket = new DatagramSocket();
		imuUdpPortOpened = imuUdpSocket->bindToPort(imuUdpPort);
		if (imuUdpPortOpened) {
			imuUdpSocket->waitUntilReady(true, 100);
			portReused = imuUdpSocket->setEnablePortReuse(true);
		}
	}

	void disconnectImuUdp(){
		// imuUdpSocket->shutdown();
		imuUdpSocket->~DatagramSocket();
		imuUdpPortOpened = false;
	}



	// Tools
	////////////////////////////////////////////////////////////////////////////////////////////////////
	// print new text on the GUI text editor interface (slows down GUI).
	void insertText(String text) {
		resultsBox.moveCaretToEnd();
        resultsBox.insertTextAtCaret (text);
        resultsBox.moveCaretToEnd();
	};


	// convert float value to string value with defined precision
	String floatToStr(float x) {
		char buffer[32];
		sprintf(buffer, "%3.1f", x);
		String xString = String(buffer);
		return xString;
	}

	
private:
	// initialize
	/////////////////////////////////////////////////////////////////////////////

	// generalobjects
	TextEditor& resultsBox;
	HrtfBiAuralAudioProcessor& binauralAudioProcessor;
	HrtfBiAuralAudioProcessorEditor& binauralAudioProcessorGui;

	// general variables
	float headX = 0.0f; // current GUI x head position [0...100]
	float headY = 0.0f; // current GUI y head position [0...100]
	float headZ = 0.0f; // current GUI z head position [0...100]
	float headAz = 0.0f; // current GUI azimuth head position [0...pi,-pi..0]
	float headEl = 0.0f; // current GUI elevation head position [-pi/2...pi/2]
	float headRd = 0.0f; // current GUI radius head position [0...100]
	int wallN = frame_Mon; // number of considered image sources (walls)
	float** wall = ar_zeros2d(wallN, A_Ndimxyz); // current GUI x,y,z wall position

	// acoustic head tracker
	long playBuffer_n  = 0; // playback buffer sample position
	long recBuffer_n = 0; // recording buffer sample position
	long sampleN = 0; // device callback buffer sample position
	long syncBufferN = 0; // synchronization buffer sample position
	long syncBufferNoffset = 0; // detected synchronization offset
	long syncBufferNoffset_p = 0; // detected past synchronization offset
	long syncBufferNtrail = 0;  // detected synchronization start (and number of bins in trailing signal part)
	long syncBufferNlead = 0;  // detected synchronization number of bins in leading signal part

	
	// imu head tracker (head position variables)
	float imuX = 0.0f; // acceleration
	float imuY = 0.0f;
	float imuZ = 0.0f;
	float imuYaw = 0.0f; // [0...180,-180...0] degree
	float imuPitch = 0.0f; // [-90...0,0...90] degree
	float imuRoll = 0.0f; // [0...180,-180...0] degree


	// imu head tracker (serial communication)
	SerialPort* imuSerialPort; // serial port of imu
	SerialPortInputStream* imuSerialInputStream; // serial input stream of imu
	bool imuSerialOn = false; // serial port of imu is connected

	// imu head tracker (udp communication)
	DatagramSocket* imuUdpSocket;
	int udpBufferBytesN = ngimu ? 512 : sizeof(double) * 6; // number of bytes in udp buffer, NGIMU: 512 (variable, often 500 or 276 or 224), TrackIMU 6 double values from opentrack (x, y, z, yaw, pitch, roll)
	MemoryBlock* udpBuffer = new MemoryBlock(udpBufferBytesN, true); // udp buffer with bytes
	MemoryInputStream* udpBufferStream = new MemoryInputStream(*udpBuffer, false); // udp buffer used for converting bytes to double values
	bool imuUdpPortOpened = false; // status: imu port could be openend
	bool imuBytesOk = false; // status: imu bytes at port were read correct




	// binaural spatializer
	AudioSampleBuffer binauralBuffer;
	MidiBuffer midiBuffer;






    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (HeadTracker)
};
/*
  ==============================================================================

   This file is part of the JUCE examples.
   Copyright (c) 2017 - ROLI Ltd.

   The code included in this file is provided under the terms of the ISC license
   http://www.isc.org/downloads/software-support-policy/isc-license. Permission
   To use, copy, modify, and/or distribute this software for any purpose with or
   without fee is hereby granted provided that the above copyright notice and
   this permission notice appear in all copies.

   THE SOFTWARE IS PROVIDED "AS IS" WITHOUT ANY WARRANTY, AND ALL WARRANTIES,
   WHETHER EXPRESSED OR IMPLIED, INCLUDING MERCHANTABILITY AND FITNESS FOR
   PURPOSE, ARE DISCLAIMED.

  ==============================================================================
*/

/*******************************************************************************
 The block below describes the properties of this PIP. A PIP is a short snippet
 of code that can be read by the Projucer and used to generate a JUCE project.

 BEGIN_JUCE_PIP_METADATA

 name:             HeadTrackerGui
 version:          1.0.0
 vendor:           JUCE
 website:          http://juce.com
 description:      Tests the audio latency of a device.

 dependencies:     juce_audio_basics, juce_audio_devices, juce_audio_formats,
                   juce_audio_processors, juce_audio_utils, juce_core,
                   juce_data_structures, juce_events, juce_graphics,
                   juce_gui_basics, juce_gui_extra
 exporters:        xcode_mac, vs2019, linux_make, androidstudio, xcode_iphone

 moduleFlags:      JUCE_STRICT_REFCOUNTEDPOINTER=1

 type:             Component
 mainClass:        HeadTrackerGui

 useLocalCopy:     1

 END_JUCE_PIP_METADATA

*******************************************************************************/
//HeadTrackerGui.h: Class containg graphical user interface for starting and stopping head tracker.
// Copyright 2019, Felix Pfreundtner, All rights reserved.
#pragma once

#include "HeaderTrackerJuce.h"
#include "DemoUtilities.h"
#include "AudioLiveScrollingDisplay.h"
#include "LatencyTester.h"
#include "PluginEditor.h"


class HeadTrackerGui  : public Component
{
public:
	// Constructor of HeadTracker class
    HeadTrackerGui()
    {
		// generate graphical user interface (GUI) Window
        setOpaque (true);





		// Add GUI window for results box
        addAndMakeVisible (resultsBox);
        resultsBox.setMultiLine (true);
        resultsBox.setReturnKeyStartsNewLine (true);
        resultsBox.setReadOnly (true);
        resultsBox.setScrollbarsShown (true);
        resultsBox.setCaretVisible (false);
        resultsBox.setPopupMenuEnabled (true);
        resultsBox.setColour (TextEditor::outlineColourId, Colour (0x1c000000));
        resultsBox.setColour (TextEditor::shadowColourId,  Colour (0x16000000));
        resultsBox.insertTextAtCaret ("Running this app sends an impulse, measures the round-trip latency between the audio output and microphone input"
                            ", tracks the head position, maps the reflection boundary and spatializes binaural signals using HRTFs.\n\n");

		// Add GUI button to test latency
        addAndMakeVisible (startTestButton);
        startTestButton.onClick = [this] { startTest(); };


		// Add GUI button to start visualizing audio signal
		addAndMakeVisible (startLiveAudioScrollerButton);
        startLiveAudioScrollerButton.onClick = [this] { startLiveAudioScroller(); };

		// Add GUI button to stop visualizing audio signal
		addAndMakeVisible (stopLiveAudioScrollerButton);
        stopLiveAudioScrollerButton.onClick = [this] { stopLiveAudioScroller(); };

		// Add GUI button to load and prepare HRTF library
		addAndMakeVisible (prepareBinauralizationButton);
        prepareBinauralizationButton.onClick = [this] { prepareBinauralization(); };

		// Add GUI button to start head tracking
		addAndMakeVisible (startHeadTrackerButton);
        startHeadTrackerButton.onClick = [this] { startHeadTracker(); };

		// Add GUI button to stop head tracking
		addAndMakeVisible (stopHeadTrackerButton);
        stopHeadTrackerButton.onClick = [this] { stopHeadTracker(); };

		// Add GUI window for binaural spatializer
		binauralAudioProcessorGui.reset(new HrtfBiAuralAudioProcessorEditor(binauralAudioProcessor));
		addAndMakeVisible(binauralAudioProcessorGui.get());

		// List all available audio devices:
		resultsBox.insertTextAtCaret(newLine + newLine + "Available devices:" + newLine);
		//int id = 1;
		for (int i = 0; i < audioDeviceManager.getAvailableDeviceTypes().size(); ++i)
		{
			const AudioIODeviceType* t = audioDeviceManager.getAvailableDeviceTypes()[i];
			const StringArray deviceNames = t->getDeviceNames();

			for (int j = 0; j < deviceNames.size(); ++j)
			{
				const String deviceName = deviceNames[j];

				String menuName;
				menuName << deviceName << " (" << t->getTypeName() << ")";
				resultsBox.insertTextAtCaret(menuName + newLine);
			}
		}

		// Select and initialize audio device

       #ifndef JUCE_DEMO_RUNNER

        RuntimePermissions::request (RuntimePermissions::recordAudio,
                                     [this] (bool granted)
                                     {	

										// set audio device parameters
										/////////////////////////////////////////////////////////////////////////////////////
										init_head_tracker_params();
										String selected_audio_device_type = "ASIO"; // ASIO (lowest latency), Windows Audio, DirectSound, WASAPI
										String selected_audio_device = "Focusrite USB ASIO"; // Focusrite USB ASIO, ASIO4ALL v2 , Yamaha Steinberg USB ASIO
										int bufferSize = 512;
										double sampleRate = (double)fs;
										int numInputChannels = granted ? ch_Nmic + ch_Nsound: 0;
										int numOutputChannels = granted ? ch_Nis + ch_Nbisound : 0;


										//if no "real" audio device is connected set audio device to "ASIO4ALL v2"
										/////////////////////////////////////////////////////////////////////////////////////
										if (FRAMEWORK_OFFLINEREAD || FRAMEWORK_OFFLINEREADSAVE) {
											selected_audio_device_type = "ASIO";
											selected_audio_device = "ASIO4ALL v2"; //ASIO4ALL v2
										}

										// initialize audio device with JUCE framework
										/////////////////////////////////////////////////////////////////////////////////////
										audioDeviceManager.initialise(numInputChannels, numOutputChannels, nullptr, true, {}, nullptr); //"Yamaha Steinberg USB ASIO", "Lautsprecher (3- Realtek(R) Audio)", "ASIO4ALL v2"
										audioDeviceManager.setCurrentAudioDeviceType(selected_audio_device_type, true);
										audioDeviceManager.initialise(numInputChannels, numOutputChannels, nullptr, true, selected_audio_device, nullptr); //"Yamaha Steinberg USB ASIO", "Lautsprecher (3- Realtek(R) Audio)", "ASIO4ALL v2"
										AudioIODevice* audioDevice = audioDeviceManager.getCurrentAudioDevice();
										AudioIODeviceType* audioDeviceType = audioDeviceManager.getCurrentDeviceTypeObject();
										AudioDeviceManager::AudioDeviceSetup audioDeviceSetup = audioDeviceManager.getAudioDeviceSetup();
										audioDeviceSetup.sampleRate = sampleRate;
										audioDeviceSetup.bufferSize = bufferSize;
										String deviceSetupError = audioDeviceManager.setAudioDeviceSetup(audioDeviceSetup, false);
										
										// control what device parameters have been set by JUCE framework at initialization
										/////////////////////////////////////////////////////////////////////////////////////
										double deviceSampleRate = audioDevice->getCurrentSampleRate();
										String deviceName = audioDeviceSetup.outputDeviceName;
										String deviceType = audioDeviceType->getTypeName();
										int deviceInputLatency = audioDevice->getInputLatencyInSamples();
										int deviceOutputLatency = audioDevice->getOutputLatencyInSamples();
										int deviceNumInputChannels	= (int)audioDevice->getActiveInputChannels().toInteger();
										int deviceNumOutputChannels	= (int)audioDevice->getActiveOutputChannels().toInteger();
										int deviceBufferSize = audioDevice->getCurrentBufferSizeSamples();

										// print the device parameters set by JUCE framework to GUI
										/////////////////////////////////////////////////////////////////////////////////////
										resultsBox.insertTextAtCaret(newLine + newLine + "Selected device:" + newLine);
										resultsBox.insertTextAtCaret(deviceName + newLine);
										resultsBox.insertTextAtCaret(deviceType + ": ");
										resultsBox.insertTextAtCaret("Input channels: " + String(deviceNumInputChannels) +  ", ");
										resultsBox.insertTextAtCaret("Output channels: " + String(deviceNumOutputChannels) + newLine);
										resultsBox.insertTextAtCaret("Sampling rate: " + String(deviceSampleRate) + " Hz" + newLine);
										resultsBox.insertTextAtCaret("Buffer size: " + String(deviceBufferSize) + " samples" + newLine);
										resultsBox.insertTextAtCaret("Input latency: " + String(deviceInputLatency) + " samples, " + String(deviceInputLatency/deviceSampleRate*1000) + " ms" + newLine);
										resultsBox.insertTextAtCaret("Output latency: " + String(deviceOutputLatency) + " samples, " + String(deviceOutputLatency/deviceSampleRate*1000) + " ms" + newLine);
										
										// in case the set JUCE parameters differ from the selected parameters print warnings
										/////////////////////////////////////////////////////////////////////////////////////
										if (selected_audio_device_type != deviceType) {
											String debugText = "Warning: Wrong device selected by system: " + String(deviceType) + "(set), " + String(selected_audio_device_type) + "(asked)" + newLine;
											DBG(debugText);
											resultsBox.insertTextAtCaret (debugText);
										};

										if (selected_audio_device != deviceName) {
											String debugText = "Warning: Wrong device selected by system: " + String(deviceName) + "(set), " + String(selected_audio_device) + "(asked)" + newLine;
											DBG(debugText);
											resultsBox.insertTextAtCaret (debugText);
										};

										if (fs != deviceSampleRate) {
											String debugText = "Warning: Sampling rate reseted to device value: " + String(deviceSampleRate) + "(set), " + String(sampleRate) + "(asked)" + newLine;
											DBG (debugText);
											resultsBox.insertTextAtCaret (debugText);
										};

										if (bufferSize != deviceBufferSize) {
											String debugText = "Warning: Buffer size reseted to device value: " + String(deviceBufferSize) + "(set), " + String(bufferSize) + "(asked)" + newLine;
											DBG (debugText);
											resultsBox.insertTextAtCaret (debugText);
										};

										bool printMatchingIOError = false;
										if (printMatchingIOError) {
											if (numInputChannels != deviceNumInputChannels) {
												String debugText = "Warning: Number of input channels do not match: " + String(deviceNumInputChannels) + "(set), " + String(numInputChannels) + "(asked)" + newLine;
												DBG(debugText);
												resultsBox.insertTextAtCaret(debugText);

											};
											if (numOutputChannels != deviceNumOutputChannels) {
												String debugText = "Warning: Number of output channels do not match: " + String(deviceNumOutputChannels) + "(set), " + String(numOutputChannels) + "(asked)" + newLine;
												DBG(debugText);
												resultsBox.insertTextAtCaret(debugText);
											};
										}

									  });
       #endif
		// Add audio callback loop audioDeviceIOCallback() for regularly updating the audio buffer

		// Set window size of GUI
        setSize (pixels_x, pixels_y);



		// For Debug purposes
		//startHeadTracker();
    }


	// Destructor of HeadTracker class
    ~HeadTrackerGui() override
    {

		if (liveAudioScroller.get() == nullptr) {
			audioDeviceManager.removeAudioCallback (liveAudioScroller.get());
			liveAudioScroller.reset();
		}
		if (latencyTester.get() == nullptr) {
			audioDeviceManager.removeAudioCallback(latencyTester.get());
			latencyTester.reset();
		}
		if (headTracker.get() == nullptr) {
			audioDeviceManager.removeAudioCallback(headTracker.get());
			headTracker.reset();
		}

    }
	// Method that starts the latency testing routine, after GUI button has been pressed
    void startTest()
    {
        if (latencyTester.get() == nullptr)
        {
            latencyTester.reset (new LatencyTester (resultsBox));
            audioDeviceManager.addAudioCallback (latencyTester.get());
        }

        latencyTester->beginTest();
    }


	// Method that starts the live audio scroller routine, after GUI button has been pressed
    void startLiveAudioScroller()
    {
        if (liveAudioScroller.get() == nullptr)
        {
			// Add GUI window for live recording signal plot
			liveAudioScroller.reset (new LiveScrollingAudioDisplay());
			addAndMakeVisible (liveAudioScroller.get());
            audioDeviceManager.addAudioCallback (liveAudioScroller.get());
			resized();
        }

    }

	// Method that stops the live audio scroller routine, after GUI button has been pressed
    void stopLiveAudioScroller()
    {
        if (liveAudioScroller.get() != nullptr)
        {
			// Remove GUI window for live recording signal plot
            audioDeviceManager.removeAudioCallback(liveAudioScroller.get());
			removeChildComponent(liveAudioScroller.get());
			liveAudioScroller.reset ();
			resized();
        }
    }

	void prepareBinauralization()
	{
		binauralAudioProcessor.loadHRTF(hrtf_filepath);
	}

	// Method that starts the head tracking routine, after GUI button has been pressed
    void startHeadTracker()
    {
        if (headTracker.get() == nullptr)
        {
            headTracker.reset (new HeadTracker (resultsBox, binauralAudioProcessor, *binauralAudioProcessorGui.get()));
			if (headTracker->trackerType == Mixed || headTracker->trackerType == Acoustic) {
				audioDeviceManager.addAudioCallback(headTracker.get());
			}
        }

        headTracker->beginTracking();
    }
	// Method that stops the head tracking routine, after GUI button has been pressed
	void stopHeadTracker()
	{
		if (headTracker.get() != nullptr){
			headTracker->stopTracking();
			if (headTracker->trackerType == Mixed || headTracker->trackerType == Acoustic) {
				audioDeviceManager.removeAudioCallback(headTracker.get());
			}
			headTracker.reset ();
		}
    }
	// Method that paints the GUI window
    void paint (Graphics& g) override
    {
        g.fillAll (findColour (ResizableWindow::backgroundColourId));
    }

	// Method that resizes the GUI window
    void resized() override
    {
        auto b = getLocalBounds().reduced (5);

        if (liveAudioScroller.get() != nullptr)
        {
            liveAudioScroller->setBounds (b.removeFromTop (b.getHeight() / 8));
            b.removeFromTop (10);
        }

		binauralAudioProcessorGui->setBounds(b.removeFromBottom (b.getHeight() / 2));
		b.removeFromBottom (10);
		stopHeadTrackerButton.setBounds (b.removeFromBottom (b.getHeight() / 10));
		b.removeFromBottom (10);
		startHeadTrackerButton.setBounds (b.removeFromBottom (b.getHeight() / 10));
		b.removeFromBottom (10);
		prepareBinauralizationButton.setBounds (b.removeFromBottom (b.getHeight() / 10));
		b.removeFromBottom (10);
		stopLiveAudioScrollerButton.setBounds (b.removeFromBottom (b.getHeight() / 10));
		b.removeFromBottom (10);
		startLiveAudioScrollerButton.setBounds (b.removeFromBottom (b.getHeight() / 10));
		b.removeFromBottom (10);
        startTestButton.setBounds (b.removeFromBottom (b.getHeight() / 10));
        b.removeFromBottom (10);



        resultsBox.setBounds (b);
    }


private:
	// parameters
	int pixels_x = GUI_x_pixel;
	int pixels_y= GUI_y_pixel;
	String hrtf_filepath = File::getSpecialLocation(File::currentExecutableFile).getParentDirectory().getFullPathName() + "/hrir/kemar.bin";

    // if this PIP is running inside the demo runner, we'll use the shared device manager instead
   #ifndef JUCE_DEMO_RUNNER
    AudioDeviceManager audioDeviceManager;
   #else
    AudioDeviceManager& audioDeviceManager { getSharedAudioDeviceManager (1, 2) };
   #endif

	// Add live audio livestream window, latency tester, headtracker, binaural spatializer and binaural spatializer window to GUI
    std::unique_ptr<LiveScrollingAudioDisplay> liveAudioScroller;
	std::unique_ptr<LatencyTester> latencyTester;
	std::unique_ptr<HrtfBiAuralAudioProcessorEditor> binauralAudioProcessorGui;
	std::unique_ptr<HeadTracker> headTracker;
	TextEditor resultsBox;
	HrtfBiAuralAudioProcessor binauralAudioProcessor;
	//HrtfBiAuralAudioProcessor& binauralAudioProcessor = HrtfBiAuralAudioProcessor();

	// Add buttons to GUI
    TextButton startTestButton  { "Test Latency" };
	TextButton startLiveAudioScrollerButton  { "Start Overall Input Signal Plot" };
	TextButton stopLiveAudioScrollerButton{ "Stop Overall Input Signal Plot" };
	TextButton prepareBinauralizationButton{ "Initialize HRTF database for binauralization" };
	TextButton startHeadTrackerButton  { "Start Head Tracking" };
	TextButton stopHeadTrackerButton  { "Stop Head Tracking" };
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (HeadTrackerGui)
};




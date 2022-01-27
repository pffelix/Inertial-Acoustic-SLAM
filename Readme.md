# Inertial-Acoustic-SLAM
<p align="center">
<img src="images/app.png">
</p>


Inertial-acoustic SLAM implementation usable for localization of a robot mannequin while mapping the surrounding boundaries.

Source-Code folder structure:

"src/juce/src" folder contains platform independent JUCE framework C++-code for GUI elements and the user interface to control binaural mixed audio playback based on the robot position in real-time. The JUCE code also allows to read x-io technologies NGIMU inertial measurement unit data via Serial Interface and Vicon Nexus measurement data for ground truth position calculation. 
"src/sdk" folder contains C-code to perform inertial-acoustic SLAM. It allows to send and record sine-sweeps using Steinberg ASIO or Windows audio drivers, convolve the output with minimal latency using the Intel MKL library, extract impulse responses, process the impulse responses, map boundaries and calculate the position of a robot equiped with microphones.

The program can be run by loading JUCE Projucer.exe in "src\juce\lib\JUCE".
Then open the JUCE project via File/Open -> "src\juce\src\AugmentedAudioRenderer.jucer".
If not automatically set, the JUCE Global Paths have to be defined in Projucer via "File/Global Paths" to include the JUCE library located in "src\juce\lib\".
In addition, if not automatically set, the JUCE project settings have to be adapted to link to the correct Header Search Paths in the repository. 
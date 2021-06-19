<!-- #######  Augmented Audio Renderer #########-->
![image_setup](images/closetocorner.png?raw=true)
<h1 style="color: #5e9ca0;">Augmented Audio Renderer</h1>
<h2 style="color: #2e6c80;">Implementation usable for Localization of a Robot Mannequin using real-time acoustic impulse recordings</h2>
<p>&nbsp;</p>
<p><span style="color: #0000ff;"><strong>&nbsp;Source-Code folder structure: (/src/)</strong></span></p>
<ol style="list-style: none; font-size: 14px; line-height: 32px; font-weight: bold;">
<li style="clear: both;">"/sdk" folder contains C-code to record and playback audio (Sine-Sweeps in the project), convolve the output with minimal latency, extract impulse responses and calculate position of robot.<br /></li>
<li style="clear: both;">"/juce" folder contains platform independent JUCE framework C++-code for GUI generation, real-time user interface and binaural audio playback. <br /></li>

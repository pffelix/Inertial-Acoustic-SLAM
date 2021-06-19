#pragma once
#include <cmath>
#include "../JuceLibraryCode/JuceHeader.h"
#include "Util.h"
#include "images/head_top.h"
#include "images/source_icon.h"


class HrtfBiAuralAudioProcessorEditor;

class MainDisplay :
	public Component,
	public Slider::Listener
{
public:
	MainDisplay(HrtfBiAuralAudioProcessorEditor* editor);
	~MainDisplay();

	void paint(Graphics& g) override;
	void mouseDrag(const MouseEvent& event) override;
	void sliderValueChanged(Slider* slider) override;
	void updateSourcePosition(float x, float y, float z, float az, float el, float rd, float** wall, float* wallWeight, int wallN); // added

private:
	void drawGridLines(Graphics& g);
	void drawHead(Graphics& g); // added
	void drawSource(Graphics& g);
	void drawWall(Graphics& g, int wallNr); //added
	void updateHRTF();

	Image headImage_;
	Image sourceImage_;
	Point3DoublePolar<float> sourcePos_;
	Point3Cartesian<float> sourceXYZ_; // added
	float ** wallXYZ_; // added
	float* wallXYZWeight_; // added
	int wallXYZN_; // added
	HrtfBiAuralAudioProcessorEditor* editor_;

	JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(MainDisplay)
};

#include "MainDisplay.h"
#include "PluginEditor.h"


MainDisplay::MainDisplay(HrtfBiAuralAudioProcessorEditor* editor)
	:
	editor_(editor),
	sourcePos_({1.0, 0.0, 0.0}),
	sourceXYZ_({0.0, 0.0, 0.0})
{
	wallXYZN_ = 0;
	sourceImage_ = ImageFileFormat::loadFrom(source_icon_png, source_icon_png_size);
	headImage_ = ImageFileFormat::loadFrom(head_top_png, head_top_png_size);
}

MainDisplay::~MainDisplay()
{
}

void MainDisplay::paint(Graphics& g)
{
	// parameter
	bool draw_switch_source = false;

	g.fillAll(Colours::transparentBlack);

	g.setOpacity(1.0f);
	g.setColour(Colours::black);
	auto w = getWidth();
	auto h = getHeight();
	g.fillEllipse(0.f, 0.f, static_cast<float>(w), static_cast<float>(h));

	for (int w = 0; w < wallXYZN_; w++) {
		drawWall(g, w); // added;
	}

	if (draw_switch_source) {
		if (sourcePos_.elevation < 0)
			drawSource(g);
		g.setOpacity(1.f);
		drawHead(g); // added
		drawGridLines(g);
		if (sourcePos_.elevation >= 0)
			drawSource(g);
	}
	else {
		drawHead(g); // added
		drawGridLines(g);
		drawSource(g);
	}

}

void MainDisplay::mouseDrag(const MouseEvent& event)
{
	auto pos = event.getPosition();
	auto x = pos.x - getWidth() * 0.5f;
	auto y = pos.y - getHeight() * 0.5f;
	sourcePos_.azimuth = std::atan2(x, -y);
	updateHRTF();
}

void MainDisplay::sliderValueChanged(Slider* slider)
{
	sourcePos_.elevation = static_cast<float>(deg2rad(slider->getValue()));
	updateHRTF();
}

void MainDisplay:: updateSourcePosition(float x, float y, float z, float az, float el, float rd, float** wall, float* wallWeight, int wallN)
{
	sourceXYZ_.x = x;
	sourceXYZ_.y = y;
	sourceXYZ_.z = z;
	sourcePos_.azimuth = az;
	sourcePos_.elevation = el;
	sourcePos_.radius = rd;
	wallXYZ_ = wall;
	wallXYZWeight_ = wallWeight;
	wallXYZN_ = wallN;
	updateHRTF();
}

void MainDisplay::drawGridLines(Graphics& g)
{
	g.setColour(Colours::white);
	g.setOpacity(0.5f);
	auto w = static_cast<float>(getWidth());
	auto h = static_cast<float>(getHeight());
	auto lineHorizontal = Line<float>(0.f, h * 0.5f, w, h * 0.5f);
	auto lineVertical = Line<float>(w * 0.5f, 0.f, w * 0.5f, h);
	float dashes[] = {3, 2};
	g.drawDashedLine(lineHorizontal, dashes, 2);
	g.drawDashedLine(lineVertical, dashes, 2);
}

void MainDisplay::drawHead(Graphics& g) {
	// parameters
	bool draw_head_xyz_pos = false; // true:  head at xyz position, else in middle
	float xyz_cm_max = 100.0f; // scaling of x,y,z plot (pixel per meter), maximal 100 pixel to display
	float head_scaling = 0.5f; // 1.0f... 0.0f;

	// init

	float xOffset = 0.0f;
	float yOffset = 0.0f;
	float rMax;
	float x_y_2;
	auto w = static_cast<float>(getWidth());
	auto h = static_cast<float>(getHeight());
	float pos_x;
	float pos_y;
	float x;
	float y;

	// draw head
	if (draw_head_xyz_pos) {
		x = sourceXYZ_.x * xyz_cm_max;
		y = sourceXYZ_.y * xyz_cm_max;

		// correct limits
		rMax = (w * 0.5f - 30) * sinf(float_Pi / 2);
		x_y_2 = (x + y);
		x_y_2 *= x_y_2;
		x_y_2 = sqrtf(x_y_2);
		x = x_y_2 > rMax? x * rMax / x_y_2: x;
		y = x_y_2 > rMax? y * rMax / x_y_2 : y;

		xOffset = x;
		yOffset = -y;
	}


    Rectangle<int> pathBounds = headImage_.getBounds(); // degreesToRadians((float)Random::getSystemRandom().nextInt (359))
	pos_x = w / 2.0f - (float)pathBounds.getCentreX() * head_scaling + xOffset;
	pos_y = h / 2.0f - (float)pathBounds.getCentreY() * head_scaling + yOffset;
	AffineTransform translation = AffineTransform().translated(pos_x, pos_y); //.followedBy(afrot)
	AffineTransform rotation = AffineTransform().rotated(sourcePos_.azimuth, (float)pathBounds.getCentreX(), (float)pathBounds.getCentreY());
	AffineTransform scaling = AffineTransform().scaled(head_scaling);

	Graphics::ScopedSaveState sss(g);
	g.addTransform(translation);
	g.drawImageTransformed(headImage_, rotation.followedBy(scaling));

	//g.drawImageWithin(headImage_, w/2, -30, w, h, RectanglePlacement::centred | RectanglePlacement::doNotResize);
}

void MainDisplay::drawSource(Graphics& g)
{
	// parameters
	bool draw_xyz_pos = true; // true: source is x,y,z position
	bool draw_az_pos = false; // true: source is az position
	bool draw_el_pos = false; // true: source is elevation plot, else azimuth plot or x,y,z plot
	bool draw_el_color = false; // true: elevation is changing color of source
	float xyz_cm_max = 30.0f; // scaling of x,y,z plot (pixel per meter), maximal 100 pixel to display

	// init
	auto w = static_cast<float>(getWidth());
	auto h = static_cast<float>(getHeight());
	float rMax;
	float x_y_2;
	auto color = editor_->fgColor_;
	float radius;
	float x;
	float y;
	float scaleFactor = 0.5f; // added 0.5, before 0.75
	auto sw = sourceImage_.getWidth();
	auto sh = sourceImage_.getHeight();
	
	// select drawing
	if (draw_xyz_pos) {
		x = sourceXYZ_.x * xyz_cm_max;
		y = sourceXYZ_.y * xyz_cm_max;
		if (sourceXYZ_.x == 0.0f && sourceXYZ_.y == 0.0f) {
			y = xyz_cm_max * 2.0f;
		};

		// correct limits
		rMax = (w * 0.5f - 2); //* sinf(float_Pi / 2);
		x_y_2 = sqrtf(x*x + y*y);
		//x_y_2 *= x_y_2;
		//x_y_2 = sqrtf(x_y_2);
		if (x_y_2 > rMax) {
			return;
		}
	}
	if (draw_az_pos){
		radius = w * 0.5f - 30; // w = 260, maximal displayable radius is then 100
		x = radius * std::sin(sourcePos_.azimuth);
		y = radius * std::cos(sourcePos_.azimuth);

		if (draw_el_pos) {
			x = x * std::cos(sourcePos_.elevation);
			y = y * std::cos(sourcePos_.elevation);
		}
		if (draw_el_color) {
			if (sourcePos_.elevation < 0)
			{
				color = color.darker();
				g.setOpacity(0.8f);
			}
			scaleFactor = scaleFactor * (std::sin(sourcePos_.elevation) * 1.0f + 1); // added * 1.0f, before * 0.25f
		}
	}

	// draw source
	x = w * 0.5f + x;
	y = h * 0.5f - y;

	ColourGradient grad(color, x, y, Colours::transparentBlack, x + w * 0.25f, y + h * 0.25f, true);
	g.setGradientFill(grad);
	g.fillEllipse(0.f, 0.f, w, h);
	if (!isnan(x) && !isnan(y)) {
		g.drawImageWithin(sourceImage_,
			static_cast<int>(x - sw * 0.5f * scaleFactor),
			static_cast<int>(y - sh * 0.5f * scaleFactor),
			static_cast<int>(sw * scaleFactor),
			static_cast<int>(sh * scaleFactor),
			RectanglePlacement::centred,
			true);
	}

}

void MainDisplay::drawWall(Graphics& g, int wallNr)
{
	// parameters
	bool draw_xyz_wall = false;
	float xyz_cm_max = 30.0f; // scaling of x,y,z plot (pixel per meter), maximal 100 pixel to display

	// init
	if (draw_xyz_wall) {
		auto w = static_cast<float>(getWidth());
		auto h = static_cast<float>(getHeight());
		float rMax;
		float x_y_2;
		auto color = editor_->wallgColor_;
		float x;
		float y;
		float diffEl = 30.0f; // Maximal plotted Elevation deviation from horizontal
		float minEl = 90.0f - diffEl; // maximal plotted elevaton angle [0(top)...90(horizontal)...180(bottom)]
		float maxEl = 90.0f + diffEl; // maximal plotted elevaton angle [0(top)...90(horizontal)...180(bottom)]
		float scaleVariable = 2.5f;
		float strange = powf(wallXYZWeight_[wallNr], 1.0f / scaleVariable);
		float scaleFactor2 = 0.01f + powf(wallXYZWeight_[wallNr], 1.0f/scaleVariable) / 10.0f;
		float scaleFactor = scaleFactor2;// *wallXYZWeight_[wallNr]; // added 0.5, before 0.75
		if (scaleFactor > 0.11f) {
			return;
		}
		auto sw = sourceImage_.getWidth();
		auto sh = sourceImage_.getHeight();

		// set drawing
		if (wallXYZ_[wallNr][4] < minEl || wallXYZ_[wallNr][4] > maxEl) {
			return;
		}
		x = wallXYZ_[wallNr][0] * xyz_cm_max;
		y = wallXYZ_[wallNr][1] * xyz_cm_max;

		if (!isnan(x) && !isnan(y)) {
			// correct limits
			rMax = (w * 0.5f - 2); //* sinf(float_Pi / 2);
			x_y_2 = sqrtf(x*x + y*y);
			//x_y_2 *= x_y_2;
			//x_y_2 = sqrtf(x_y_2);
			if (x_y_2 > rMax) {
				return;
			}
			//x = x_y_2 > rMax ? x * rMax / x_y_2 : x;
			//y = x_y_2 > rMax ? y * rMax / x_y_2 : y;

			// draw source
			x = w * 0.5f + x;
			y = h * 0.5f - y;

			ColourGradient grad(color, x, y, Colours::transparentBlack, x + w * scaleFactor2, y + h * scaleFactor2, true); // smallest 0.01
			g.setGradientFill(grad);
			g.fillEllipse(0.f, 0.f, w, h);
			g.drawImageWithin(sourceImage_,
				static_cast<int>(x - sw * 0.5f * scaleFactor),
				static_cast<int>(y - sh * 0.5f * scaleFactor),
				static_cast<int>(sw * scaleFactor),
				static_cast<int>(sh * scaleFactor),
				RectanglePlacement::centred,
				true);
		}

	}

}
void MainDisplay::updateHRTF()
{
	sourcePos_.radius = 1.0f;
	auto p = sphericalToInteraural(sourcePos_);
	editor_->processor_.updateHRTF(rad2deg(p.azimuth), rad2deg(p.elevation));
	repaint();
}

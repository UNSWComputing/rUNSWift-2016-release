/*
Copyright 2010 The University of New South Wales (UNSW).

This file is part of the 2010 team rUNSWift RoboCup entry. You may
redistribute it and/or modify it under the terms of the GNU General
Public License as published by the Free Software Foundation; either
version 2 of the License, or (at your option) any later version as
modified below. As the original licensors, we add the following
conditions to that license:

In paragraph 2.b), the phrase "distribute or publish" should be
interpreted to include entry into a competition, and hence the source
of any derived work entered into a competition must be made available
to all parties involved in that competition under the terms of this
license.

In addition, if the authors of a derived work publish any conference
proceedings, journal articles or other academic papers describing that
derived work, then appropriate academic citations to the original work
must be included in that publication.

This rUNSWift source is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along
with this source code; if not, write to the Free Software Foundation,
Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
*/

#include <QMenu>
#include <QMenuBar>
#include <QFileDialog>
#include <QDebug>
#include <QPainter>
#include <QMouseEvent>

#include <iostream>
#include <unistd.h>
#include "blackboard/Blackboard.hpp"
#include "cameraTab.hpp"
#include "yuvHistogram.hpp"
#include "perception/vision/yuv.hpp"
#include "perception/vision/NaoCamera.hpp"

using namespace std;

extern __u32 controlIds[NUM_CONTROLS];

// For manual tuning
static const char* ControlNames[NUM_CONTROLS] = {
   "Horizontal Flip",
   "Vertical Flip",
   "Exposure Auto",
   "Brightness",
   "Contrast",
   "Saturation",
   "Hue",
   "Sharpness",
   "Auto White Balance",
   "Backlight Compensation",
   "Exposure Auto",
   "Exposure",
   "Gain",
   "White Balance"
};
static const char* shortControlNames[NUM_CONTROLS] = {
   "hflip",
   "vflip",
   "autoexp",
   "brightness",
   "contrast",
   "saturation",
   "hue",
   "sharpness",
   "autowb",
   "backlightcompensation",
   "autoexp",
   "exposure",
   "gain",
   "whitebalance"
};
static const int cNUM_CAM_CONTROLS = sizeof(ControlNames)/sizeof(char *);
__u16 controlMin[NUM_CONTROLS] = {
	0,
	0,
	0,
	0,
	16,
	0,
	0,
	0,
	0,
	0,
	0,
	1,
	0,
	2700
};

 __u16 controlMax[NUM_CONTROLS] = {
	1,
	1,
	1,
	255,
	64,
	255,
	44,
	7,
	1,
	4,
	1,
	512,
	255,
	6500
};
struct controlAttributes {
    __u32 control;
    string name;
    __s32 min;
    __s32 max;
    __s32 current;
    bool tryIncrease;
};

// For autotuning
static struct controlAttributes ControlValues[] = {
   // Note: Exposure can be set upto 2500 (manually), but for auto-tuning, we limit it to 140
    {V4L2_CID_EXPOSURE,                "Exposure",                  1,  140, 0, false},
    {V4L2_CID_BRIGHTNESS,              "Brightness",                0,  255, 0, false},
    {V4L2_CID_CONTRAST,                "Contrast",                 16,   64, 0, false},
    {V4L2_CID_GAIN,                    "Gain",                     35,  255, 0, false},
    // Note: Saturation ranges from 0 - 255, but anything below 60 is losing its colour
    {V4L2_CID_SATURATION,              "Saturation",               60,  255, 0, false},
    {V4L2_CID_SHARPNESS,               "Sharpness",                 0,    7, 0, false},
    {V4L2_CID_BACKLIGHT_COMPENSATION,  "Backlight Compensation",    0,    4, 0, false},
    {V4L2_CID_DO_WHITE_BALANCE,        "White Balance",          2700, 6500, 0, false},
};

static const int cNUM_CONTROL_ATTRIBUTES = sizeof(ControlValues)/sizeof(struct controlAttributes);

CameraTab::CameraTab(QTabWidget *parent,
      QMenuBar *menuBar, Vision *vision)  {
   init();
   this->vision = vision;
   currentFrame = NULL;
   topFrame = NULL;
   botFrame = NULL;
   this->parent = parent;
   framesSinceReset = INT_MAX;

   numLocalMax = 0;
}

void CameraTab::init() {
   int i;

   // Lay them all out
   layout = new QGridLayout();
   this->setLayout(layout);

   // Set up pixmaps for camera image and RGB, YUV, HSV histograms
   for (i = 0; i < NUM_IMG_TYPES; i++) {
      // Create pixmap and label
      imagePixmaps[i] = QPixmap(320, 240);
      imagePixmaps[i].fill(Qt::darkGray);
      imageLabels[i] = new QLabel();
      imageLabels[i]->setPixmap(imagePixmaps[i]);

      // Set alignment and size of pixmaps
      imageLabels[i]->setAlignment(Qt::AlignTop);
      imageLabels[i]->setMinimumSize(BOT_IMAGE_COLS, BOT_IMAGE_ROWS);
      imageLabels[i]->setMaximumSize(BOT_IMAGE_COLS, BOT_IMAGE_ROWS);
   }

   // Position image and histogram pixmaps
   layout->addWidget(imageLabels[CAMERA], 0,0,1,1); // Camera image top left
   //layout->addWidget(imageLabels[RGB], 1,0,1,1); // RGB histogram below camera
   layout->addWidget(&pointCloud,      1,0,1,1);   // Point cloud to bottom left
   layout->addWidget(imageLabels[YUV], 0,1,1,1); // YUV histogram to right
   //layout->addWidget(imageLabels[HSV], 1,1,1,1); // HSV histogram below YUV

   // Add controls
   QVBoxLayout *controlLayout = new QVBoxLayout();
   // TODO: don't need??? (aneita)
   // QButtonGroup *cameraGroup = new QButtonGroup();

   // Camera selector
   QGroupBox *cameraBox = new QGroupBox(tr("Camera"));
   QVBoxLayout *cameraGroupLayout = new QVBoxLayout();
   cameraBox->setLayout(cameraGroupLayout);
   controlLayout->addWidget(cameraBox);
   cameraGroupLayout->setAlignment(Qt::AlignTop);

   topCam = new QRadioButton(QString("Top Camera"), cameraBox);
   bottomCam = new QRadioButton(QString("Bottom Camera"), cameraBox);
   topCam->setChecked(true);
   cameraGroupLayout->addWidget(topCam);
   cameraGroupLayout->addWidget(bottomCam);

   connect(topCam, SIGNAL(toggled(bool)), this, SLOT(toggleChange()));
   connect(bottomCam, SIGNAL(toggled(bool)), this, SLOT(toggleChange()));

   // Camera controls
   QGroupBox *controlsBox = new QGroupBox(tr("Camera Controls"));
   QGridLayout *controlsGroupLayout = new QGridLayout();
   controlsBox->setLayout(controlsGroupLayout);
   controlLayout->addWidget(controlsBox);


   for (i = 0; i < cNUM_CAM_CONTROLS-1; i++) {
      //QHBoxLayout *settingsGroupLayout = new QHBoxLayout();

      // Skip the control if it is an auto setting
      if (strcmp(ControlNames[i],"Exposure Auto") == 0) {
        if(i != 2){
        QLabel *nameLabel = new QLabel(ControlNames[i]);
        controlsGroupLayout->addWidget(nameLabel, i * 6, 0, 3, 1);
        QCheckBox *checkbox = new QCheckBox(this);
        checkbox->setCheckable(true);
        controlsGroupLayout->addWidget(checkbox, i *6, 3, 3, 1);
        int exposureControl = i;
        i++;
        //Exposure
        QLabel *enameLabel = new QLabel(ControlNames[i]);
        QPushButton *decButton = new QPushButton("-");
        decButton->setMaximumSize(30,30);
        QLineEdit *currentValue =
              new QLineEdit(QString::number(controlValues[TOP_CAMERA][i]));
        QPushButton *incButton = new QPushButton("+");
        incButton->setMaximumSize(30,30);
        QSlider *slider = new QSlider( Qt::Horizontal,this);
        slider->setMaximum(controlMax[i]);
        slider->setMinimum(controlMin[i]);
        slider->setTracking(true);
        slider->setSliderPosition(controlValues[TOP_CAMERA][i]);
        CameraButtonHandler* cbh = new CameraButtonHandler(this, i, NULL, currentValue, NULL);
        valueInputBoxes.push_back(cbh);
        connect(decButton, SIGNAL(clicked(bool)),
                new CameraButtonHandler(this, i, decButton, currentValue, slider),
                SLOT(clicked(bool)));
        connect(incButton, SIGNAL(clicked(bool)),
                new CameraButtonHandler(this, i, incButton, currentValue, slider),
                SLOT(clicked(bool)));
        connect(currentValue, SIGNAL(editingFinished()),
                cbh, SLOT(clicked()));
        connect(slider, SIGNAL(valueChanged(int)),
                      new SliderHandler(this, i, slider, currentValue),
                      SLOT(setValue(int)));
        controlsGroupLayout->addWidget(enameLabel, i * 6, 0, 3, 1);
        controlsGroupLayout->addWidget(decButton, i * 6, 2, 3, 1);
        controlsGroupLayout->addWidget(currentValue, i * 6, 3, 3, 1);
        controlsGroupLayout->addWidget(incButton, i * 6, 4, 3, 1);
        controlsGroupLayout->addWidget(slider,(i*6)+3,0,3,5);
        i++;
        //Gain
        QLabel *gnameLabel = new QLabel(ControlNames[i]);
        QPushButton *decButton2 = new QPushButton("-");
        decButton2->setMaximumSize(30,30);
        QLineEdit *currentValue2 =
              new QLineEdit(QString::number(controlValues[TOP_CAMERA][i]));
        QPushButton *incButton2 = new QPushButton("+");
        incButton2->setMaximumSize(30,30);
        QSlider *slider2 = new QSlider( Qt::Horizontal,this);
        slider2->setMaximum(controlMax[i]);
        slider2->setMinimum(controlMin[i]);
        slider2->setTracking(true);
        slider2->setSliderPosition(controlValues[TOP_CAMERA][i]);
        CameraButtonHandler* cbh2 = new CameraButtonHandler(this, i, NULL, currentValue2, NULL);
        valueInputBoxes.push_back(cbh2);
        connect(decButton2, SIGNAL(clicked(bool)),
                new CameraButtonHandler(this, i, decButton2, currentValue2, slider2),
                SLOT(clicked(bool)));
        connect(incButton2, SIGNAL(clicked(bool)),
                new CameraButtonHandler(this, i, incButton2, currentValue2, slider2),
                SLOT(clicked(bool)));
        connect(currentValue2, SIGNAL(editingFinished()),
                cbh, SLOT(clicked()));
        connect(slider2, SIGNAL(valueChanged(int)),
                      new SliderHandler(this, i, slider2, currentValue2),
                      SLOT(setValue(int)));
        controlsGroupLayout->addWidget(gnameLabel, i * 6, 0, 3, 1);
        controlsGroupLayout->addWidget(decButton2, i * 6, 2, 3, 1);
        controlsGroupLayout->addWidget(currentValue2, i * 6, 3, 3, 1);
        controlsGroupLayout->addWidget(incButton2, i * 6, 4, 3, 1);
        controlsGroupLayout->addWidget(slider2,(i*6)+3,0,3,5);
        connect(checkbox, SIGNAL(clicked(bool)),new CheckboxHandler(this, slider,decButton,currentValue,incButton,exposureControl),SLOT(toggleState(bool)));
        connect(checkbox, SIGNAL(clicked(bool)),new CheckboxHandler(this, slider2,decButton2,currentValue2,incButton2,exposureControl),SLOT(toggleState(bool)));
        }

      } else if (strcmp(ControlNames[i],"Auto White Balance") == 0){
          QLabel *nameLabel = new QLabel(ControlNames[i]);
          controlsGroupLayout->addWidget(nameLabel, (cNUM_CAM_CONTROLS-1) *6, 0, 3, 1);
          QCheckBox *checkbox = new QCheckBox(this);
          checkbox->setCheckable(true);
          controlsGroupLayout->addWidget(checkbox, (cNUM_CAM_CONTROLS-1) *6, 3, 3, 1);
          int whitebaltoggle = i;
          i = cNUM_CAM_CONTROLS-1;
          QLabel *wnameLabel = new QLabel(ControlNames[i]);
          QPushButton *decButton = new QPushButton("-");
          decButton->setMaximumSize(30,30);
          QLineEdit *currentValue =
                new QLineEdit(QString::number(controlValues[TOP_CAMERA][i]));
          QPushButton *incButton = new QPushButton("+");
          incButton->setMaximumSize(30,30);
          QSlider *slider = new QSlider( Qt::Horizontal,this);
          slider->setMaximum(controlMax[i]);
          slider->setMinimum(controlMin[i]);
          slider->setTracking(true);
          slider->setSliderPosition(controlValues[TOP_CAMERA][i]);
          CameraButtonHandler* cbh = new CameraButtonHandler(this, i, NULL, currentValue, NULL);
          valueInputBoxes.push_back(cbh);
          connect(decButton, SIGNAL(clicked(bool)),
                  new CameraButtonHandler(this, i, decButton, currentValue, slider),
                  SLOT(clicked(bool)));
          connect(incButton, SIGNAL(clicked(bool)),
                  new CameraButtonHandler(this, i, incButton, currentValue, slider),
                  SLOT(clicked(bool)));
          connect(currentValue, SIGNAL(editingFinished()),
                  cbh, SLOT(clicked()));
          connect(slider, SIGNAL(valueChanged(int)),
                        new SliderHandler(this, i, slider, currentValue),
                        SLOT(setValue(int)));
          i++;
          controlsGroupLayout->addWidget(wnameLabel, i * 6, 0, 3, 1);
          //controlsGroupLayout->addWidget(minLabel, i * 3, 1, 3, 1);
          controlsGroupLayout->addWidget(decButton, i * 6, 2, 3, 1);
          controlsGroupLayout->addWidget(currentValue, i * 6, 3, 3, 1);
          controlsGroupLayout->addWidget(incButton, i * 6, 4, 3, 1);
          controlsGroupLayout->addWidget(slider,(i*6)+3,0,3,5);
          connect(checkbox, SIGNAL(clicked(bool)),new CheckboxHandler(this, slider,decButton,currentValue,incButton,whitebaltoggle),SLOT(toggleState(bool)));
          i = whitebaltoggle;
          //controlsGroupLayout->addWidget(maxLabel, i * 3, 5, 3, 1);
          //controlsGroupLayout->addWidget(settingsGroupLayout);
      } else if (strcmp(ControlNames[i],"Horizontal Flip") == 0||
              strcmp(ControlNames[i],"Vertical Flip") == 0) {
    	  QLabel *nameLabel = new QLabel(ControlNames[i]);
    	  controlsGroupLayout->addWidget(nameLabel, i * 4, 0, 3, 1);
    	  QCheckBox *checkbox = new QCheckBox(this);
    	  controlsGroupLayout->addWidget(checkbox, i * 4, 3, 3, 1);
    	  connect(checkbox, SIGNAL(clicked(bool)),new CheckboxHandler(this, NULL,NULL,NULL,NULL,i),SLOT(toggleState(bool)));
      }else {
      QLabel *nameLabel = new QLabel(ControlNames[i]);
      //QLabel *minLabel = new QLabel("0");
      QPushButton *decButton = new QPushButton("-");
      decButton->setMaximumSize(30,30);
      QLineEdit *currentValue =
            new QLineEdit(QString::number(controlValues[TOP_CAMERA][i]));
      //currentValue->setValue(controlValues[TOP_CAMERA][i]);
      QPushButton *incButton = new QPushButton("+");
      incButton->setMaximumSize(30,30);
      //QLabel *maxLabel = new QLabel("255");
      QSlider *slider = new QSlider( Qt::Horizontal,this);
      slider->setMaximum(controlMax[i]);
      slider->setMinimum(controlMin[i]);
      slider->setTracking(true);
      slider->setSliderPosition(controlValues[TOP_CAMERA][i]);
      CameraButtonHandler* cbh = new CameraButtonHandler(this, i, NULL, currentValue, NULL);
      valueInputBoxes.push_back(cbh);
      connect(decButton, SIGNAL(clicked(bool)),
              new CameraButtonHandler(this, i, decButton, currentValue, slider),
              SLOT(clicked(bool)));
      connect(incButton, SIGNAL(clicked(bool)),
              new CameraButtonHandler(this, i, incButton, currentValue, slider),
              SLOT(clicked(bool)));
      connect(currentValue, SIGNAL(editingFinished()),
              cbh, SLOT(clicked()));
      connect(slider, SIGNAL(valueChanged(int)),
                    new SliderHandler(this, i, slider, currentValue),
                    SLOT(setValue(int)));
      controlsGroupLayout->addWidget(nameLabel, i * 6, 0, 3, 1);
      //controlsGroupLayout->addWidget(minLabel, i * 3, 1, 3, 1);
      controlsGroupLayout->addWidget(decButton, i * 6, 2, 3, 1);
      controlsGroupLayout->addWidget(currentValue, i * 6, 3, 3, 1);
      controlsGroupLayout->addWidget(incButton, i * 6, 4, 3, 1);
      controlsGroupLayout->addWidget(slider,(i*6)+3,0,3,5);

      //controlsGroupLayout->addWidget(maxLabel, i * 3, 5, 3, 1);
      //controlsGroupLayout->addWidget(settingsGroupLayout);
      }
   }
   i++;
   i++;
   // Add load values button
   QPushButton *loadValuesButton = new QPushButton("Load Values");
   controlsGroupLayout->addWidget(loadValuesButton, i * 6, 0, 3, 1);
   connect(loadValuesButton, SIGNAL(clicked(bool)), this, SLOT(valueFetcher()));

   // Add auto tuning button
   QPushButton *autoTuneButton = new QPushButton("Auto Tune");
   controlsGroupLayout->addWidget(autoTuneButton, i * 6, 2, 3, 3);
   connect(autoTuneButton, SIGNAL(clicked(bool)), this, SLOT(autoTuner()));

   layout->addLayout(controlLayout, 0,2,1,1);

   // Initialise variables
   bestEntropy = 0.0;
   globalBestEntropy = 0.0;
   autoTuneOn = false;
   currentCamera = TOP_CAMERA;
   cameraToggled = false;
}

// Called when user has switched cameras
void CameraTab::toggleChange() {
   cameraToggled = true;
   if (topCam->isChecked()) {
      currentCamera = TOP_CAMERA;
   } else {
      currentCamera = BOTTOM_CAMERA;
   }
   if((topFrame!= NULL) && (botFrame != NULL)){
   updateCameraValues();
   }
}

void CameraTab::redraw() {
   QImage image;
   unsigned int imageRows, imageCols;
   const uint8_t* frame;

   if (currentCamera == TOP_CAMERA) {
      image = QImage(TOP_IMAGE_COLS, TOP_IMAGE_ROWS, QImage::Format_RGB32);
      frame = topFrame;
      imageRows = TOP_IMAGE_ROWS;
      imageCols = TOP_IMAGE_COLS;
   } else {
      image = QImage(BOT_IMAGE_COLS, BOT_IMAGE_ROWS, QImage::Format_RGB32);
      frame = botFrame;
      imageRows = BOT_IMAGE_ROWS;
      imageCols = BOT_IMAGE_COLS;
   }

   pointCloud.points.resize(0);

   if (topFrame && botFrame) {
      //display RAW image from selected camera
      for (unsigned int row = 0; row < imageRows; ++row) {
         for (unsigned int col = 0; col < imageCols; ++col) {
            QRgb rgb = getRGB(col, row, frame, imageCols);
            image.setPixel(col, row, rgb);
            int r, g, b;
            QColor::fromRgb(rgb).getRgb(&r, &g, &b);
            pointCloud.points.push_back(make_pair(qglviewer::Vec(r / 255.0,
                                                                 g / 255.0,
                                                                 b / 255.0),
                                                  qglviewer::Vec
                                       (gety(frame, row, col) / 255.0,
                                        getu(frame, row, col) / 255.0,
                                        getv(frame, row, col) / 255.0)));
         }
      }

      // If we are auto-tuning or have switched cameras, update the parameters
      if (autoTuneOn || cameraToggled) {
         updateCameraValues();
         cameraToggled = false;
      }

   } else {
      image.fill(Qt::darkGray);
   }

   QPixmap imagePixmap = QPixmap(QPixmap::fromImage(image.scaled(BOT_IMAGE_COLS, BOT_IMAGE_ROWS)));

   imageLabels[CAMERA]->setPixmap(imagePixmap);

   // Create histograms
   //Histogram RGBHistogram(0x00000000U, 0x00ffffffU);
   YUVHistogram yuvHistogram(0x00000000U, 0x00ffffffU);

   // Process image
   for (unsigned int row = 0; row < imageRows; ++row) {
      for (unsigned int col = 0; col < imageCols; ++col) {
         unsigned int YUVValue = 0x00ffffff &
                                 ((gety(frame, row, col) << 16) |
                                  (getu(frame, row, col) << 8) |
                                   getv(frame, row, col));

         //RGBHistogram.addDatapoint(getRGB(col, row, currentFrame) & MAX_RGB_VALUE);
         yuvHistogram.addDatapoint(YUVValue);
      }
   }

   // Set initial background of pixmaps
   imagePixmaps[YUV].fill(Qt::darkGray);
   // Draw Histogram
   yuvHistogram.drawHistogram(imagePixmaps[YUV], (unsigned int)(IMAGE_COLS/2), (unsigned int)(IMAGE_ROWS/2), YUVHistogram::eYUV);
   // Display Histograph in tab
   imageLabels[YUV]->setPixmap(imagePixmaps[YUV]);

   if (autoTuneOn) {
      //tuneValue(yuvHistogram.getEntropyValue());
      tuneValue(getEntropyValue());
   }
}

bool CameraTab::outOfBounds(int row, int col) {
   int imageRows, imageCols;
   if (currentCamera == TOP_CAMERA) {
      imageRows = TOP_IMAGE_ROWS;
      imageCols = TOP_IMAGE_COLS;
   } else {
      imageRows = BOT_IMAGE_ROWS;
      imageCols = BOT_IMAGE_COLS;
   }

   if (row < 0 || row > imageRows) {
      return true;
   }

   if (col < 0 || col > imageCols) {
      return true;
   }

   return false;
}

//TODO: new entropy calculation function
double CameraTab::getEntropyValue() {

   // This is to stop entropy getting calculated too many times
   if (framesSinceReset < 3) {
      return 0;
   }

   unsigned int imageRows, imageCols;
   const uint8_t* frame;
   unsigned int numCorrect = 0;

   // Initialise correct frame and image size depending on selected camera
   if (currentCamera == TOP_CAMERA) {
      frame = topFrame;
      imageRows = TOP_IMAGE_ROWS;
      imageCols = TOP_IMAGE_COLS;
   } else {
      frame = botFrame;
      imageRows = BOT_IMAGE_ROWS;
      imageCols = BOT_IMAGE_COLS;
   }

   for (unsigned int row = 0; row < imageRows; ++row) {
      for (unsigned int col = 0; col < imageCols; ++col) {

         int y = gety(frame, row, col);
         int u = getu(frame, row, col);
         int v = getv(frame, row, col);

         // If white
         if (160 <= y && y <= 255 &&
            120 <= u && u <= 155 &&
            95 <= v && v <= 130) {
            numCorrect++;
         }

         // If green
         if (55 <= y && y <= 85 &&
            46 <= u && u <= 120 &&
            90 <= v && v <= 115) {
            numCorrect++;
         }
      }
   }

   return numCorrect;
}

// TODO(brockw): see if this can be genericized into tab.cpp, so it's not in
// every tab
void CameraTab::newNaoData(NaoData *naoData) {
   if (!naoData || !naoData->getCurrentFrame().blackboard) {
      topFrame = NULL;
      botFrame = NULL;
     // currentFrame = NULL;
   } else {
      Blackboard *blackboard = naoData->getCurrentFrame().blackboard;
      topCameraSettings = readFrom(vision,topCameraSettings);
      botCameraSettings = readFrom(vision,botCameraSettings);
      if (((topFrame = readFrom(vision, topFrame)) != NULL) && ((botFrame = readFrom(vision, botFrame)) != NULL)) {
         if (parent->currentIndex() == parent->indexOf(this)) {
            // Reading from the Blackboard (data written from VisionAdapter)
            if (currentCamera == TOP_CAMERA) {
               currentSettings = readFrom(vision, topCameraSettings);
            } else {
               currentSettings = readFrom(vision, botCameraSettings);
            }
            redraw();
         }
      }
   }
   // framesSinceReset++;
}

// Update camera control values in OffNao to reflect robot values
void CameraTab::updateCameraValues() {
	/** This is used for autotuning
   for (vector<CameraButtonHandler*>::iterator it = valueInputBoxes.begin(); it != valueInputBoxes.end(); ++it) {
      CameraButtonHandler *cbh = *it;
      int controlIndex = cbh->getControl();
      int value = getParamValue(ControlNames[controlIndex]);
      cbh->setValue(value);
   }
   **/
	CameraSettings currentSetting;
	if (currentCamera == TOP_CAMERA){
		currentSetting = topCameraSettings;
	} else {
		currentSetting = botCameraSettings;
	}
	int i = 0;
	for (vector<CameraButtonHandler*>::iterator it = valueInputBoxes.begin(); it != valueInputBoxes.end(); ++it) {
	    CameraButtonHandler *cbh = *it;
		switch(i){
	       //brightness
		  case 0:
			  cbh->setValue(currentSetting.brightness);
			  break;
		  //contrast
		  case 1:
			  cbh->setValue(currentSetting.contrast);
			  break;
	      //saturation
		  case 2:
			  cbh->setValue(currentSetting.saturation);
			  break;
		  //hue
		  case 3:
			  cbh->setValue(currentSetting.hue);
			  break;
		  //sharpness
		  case 4:
			  cbh->setValue(currentSetting.sharpness);
			  break;
	      //white bal
		  case 5:
			  cbh->setValue(currentSetting.whiteBalance);
			  break;

	      //backlight
		  case 6:
			  cbh->setValue(currentSetting.backlightCompensation);
			  break;
	      //exposure
		  case 7:
			  cbh->setValue(currentSetting.exposure);
			  break;

		  //gain
		  case 8:
			  cbh->setValue(currentSetting.gain);
			  break;
	   }
	   i++;
	}

}

void CameraTab::setControls(const vector<struct v4l2_control> &controls){
    string selectedCam = (topCam->isChecked() ? "top" : "bot");
    CameraSettings *currentSetting;
    stringstream ss;
    if(topCam->isChecked()){
    	currentSetting = &topCameraSettings;
    } else {
    	currentSetting = &botCameraSettings;
    }

    for(vector<struct v4l2_control>::const_iterator ci = controls.begin();
       ci != controls.end(); ++ci) {
    	switch(ci->id){
    		 	case V4L2_CID_BRIGHTNESS:
    		 		currentSetting->brightness = ci->value;
    		 		break;
    		 	case V4L2_CID_CONTRAST:
    		 		currentSetting->contrast = ci->value;
    		 		break;
    		 	case V4L2_CID_SATURATION:
    		 	    currentSetting->saturation= ci->value;
    		 	   break;
    		 	case V4L2_CID_HUE:
    		 		currentSetting->hue= ci->value;
    		 		break;
    		 	case V4L2_CID_SHARPNESS:
    		 		currentSetting->sharpness= ci->value;
    		 		break;
    		 	case V4L2_CID_BACKLIGHT_COMPENSATION:
    		 		currentSetting->backlightCompensation= ci->value;
    		 		break;
    		 	case V4L2_CID_EXPOSURE:
    		 		currentSetting->exposure= ci->value;
    		 		break;
    		 	case V4L2_CID_GAIN:
    		 		currentSetting->gain= ci->value;
    		 		break;
    		 	case V4L2_CID_DO_WHITE_BALANCE:
    		 		currentSetting->whiteBalance= ci->value;
    		 		break;
    	}
        ss << "--camera." << selectedCam << "." << getCameraControlName(ci->id)
           << "=" << ci->value << " ";
    }

    emit sendCommandToRobot(QString(ss.str().c_str()));
}

string CameraTab::getCameraControlName(__u32 controlName) {
    switch (controlName) {
        case V4L2_CID_HFLIP: return "hflip";
        case V4L2_CID_VFLIP: return "vflip";
        //TODO: remove the auto adjusters
        case V4L2_CID_EXPOSURE_AUTO: return "autoexp";
        case V4L2_CID_BRIGHTNESS: return "brightness";
        case V4L2_CID_CONTRAST: return "contrast";
        case V4L2_CID_SATURATION: return "saturation";
        case V4L2_CID_HUE: return "hue";
        case V4L2_CID_SHARPNESS: return "sharpness";
        // TODO: remove
        case V4L2_CID_AUTO_WHITE_BALANCE: return "autowb";
        case V4L2_CID_BACKLIGHT_COMPENSATION: return "backlightcompensation";
        case V4L2_CID_EXPOSURE: return "exposure";
        case V4L2_CID_GAIN: return "gain";
        case V4L2_CID_DO_WHITE_BALANCE: return "whitebalance";
    }
    return "unknownparam";
}

// TODO: find better way to do this (aneita)
int CameraTab::getParamValue(const char* controlName) const {
   if (strcmp(controlName, "Horizontal Flip") == 0) {
      return currentSettings.hflip;
   }
   if (strcmp(controlName, "Vertical Flip") == 0) {
      return currentSettings.vflip;
   }
   if (strcmp(controlName, "Brightness") == 0) {
      return currentSettings.brightness;
   }
   if (strcmp(controlName, "Contrast") == 0) {
      return currentSettings.contrast;
   }
   if (strcmp(controlName, "Saturation") == 0) {
      return currentSettings.saturation;
   }
   if (strcmp(controlName, "Hue") == 0) {
      return currentSettings.hue;
   }
   if (strcmp(controlName, "Sharpness") == 0) {
      return currentSettings.sharpness;
   }
   if (strcmp(controlName, "Backlight Compensation") == 0) {
      return currentSettings.backlightCompensation;
   }
   if (strcmp(controlName, "Exposure") == 0) {
      return currentSettings.exposure;
   }
   if (strcmp(controlName, "Gain") == 0) {
      return currentSettings.gain;
   }
   if (strcmp(controlName, "White Balance") == 0) {
      return currentSettings.whiteBalance;
   }
   return 0;
}

void CameraTab::valueFetcher(void) {
   updateCameraValues();
}

void CameraTab::autoTuner(void) {
   if (autoTuneOn) { // Then turn it off
      qDebug() << "Autotuning off!";
      numLocalMax = 0;
   } else { // Otherwise turn it on
      qDebug() << "Autotuning on!";
      // Reset best entropy value
      bestEntropy = 0.0;

      // Set the values we begin tuning from
      for (int i = 0; i < cNUM_CONTROL_ATTRIBUTES; i++) {
         ControlValues[i].current = getParamValue(ControlValues[i].name.c_str());
      }
   }
   autoTuneOn = !autoTuneOn;
   srandom(time(NULL)); // (Re)Seed random number generator
   framesSinceReset = 0;
}

void CameraTab::tuneValue(float entropy) {
   vector<struct v4l2_control> vals;
   static unsigned int iterations = 0;
   static int control = -1;
   static unsigned int temperature = 12;
   static unsigned int iterationsSinceChange = 0;

   // wait for new frame with updated image
   if (framesSinceReset > 5) {
      qDebug() << "Pre change: ";
      printCameraControlValues();

      // Get new entropy value
      if (entropy < bestEntropy) {
          // If new entropy value is worse, revert back to previous attribute value for control
          struct v4l2_control val = {ControlValues[prevControl].control, prevControlValue};
          vals.push_back(val);
          // Next time we try this control, try the other direction
          ControlValues[prevControl].tryIncrease = !ControlValues[prevControl].tryIncrease;
          ControlValues[prevControl].current = prevControlValue;
          qDebug() << "Decreased entropy: " << entropy;
          printCameraControlValues();
      } else {
          bestEntropy = entropy;
          iterationsSinceChange = 0;
          qDebug() << "Increased entropy: " << entropy;
          printCameraControlValues();
      }

      // If we have reached one full iteration of the controls
      if ((iterations % cNUM_CONTROL_ATTRIBUTES) == 0) {
         iterationsSinceChange++;
         if (iterationsSinceChange == 2) {
            // If this local max is better than our global max, set values
            if (bestEntropy > globalBestEntropy) {
               qDebug()  << "Better set of settings!";
               globalBestEntropy = bestEntropy;
               globalBestSettings = currentSettings;
            }

            // TODO: turn this back to < 2
            // Restart auto-tuning if we haven't found max of 3 local maxima
            if (numLocalMax < 1) {
               numLocalMax++;
               qDebug() << "Restarting tuning #" << numLocalMax;
               // Turn off auto-tuner and start it again
               autoTuneOn = false;

               // Reset all values randomly
               vector<struct v4l2_control> newVals;
               for (int i = 0; i < cNUM_CONTROL_ATTRIBUTES; i++) {
                  int value = ControlValues[i].min + (random() % ((ControlValues[i].max - ControlValues[i].min)/2));
                  ControlValues[i].current = value;
                  struct v4l2_control val = {ControlValues[i].control, value};
                  newVals.push_back(val);
               }

               // Send these values to the robot
               setControls(newVals);

               // Toggle autoTuner
               autoTuneOn = true;
               bestEntropy = 0.0;
               iterationsSinceChange = 0;
               srandom(time(NULL)); // (Re)Seed random number generator
               framesSinceReset = 0;
               return;
            } else {
               cout << "Reached 2 local max" << endl;
               cout << "Turning autotuning off" << endl;
               autoTuneOn = !autoTuneOn;

               // Revert to our best settings
               vector<struct v4l2_control> bestVals;
               for (int i = 0; i < cNUM_CONTROL_ATTRIBUTES; i++) {
                  struct v4l2_control val = {ControlValues[i].control, getBestSettingsValue(ControlValues[i].control)};
                  bestVals.push_back(val);
               }
               setControls(bestVals);
               return;
            }
         } // If iterationsSinceChange == 2
      }

      // Select next control to manipulate
      control = (control + 1) % cNUM_CONTROL_ATTRIBUTES;
      // Save current values in case they need to be reset
      prevControl = control;
      prevControlValue = ControlValues[control].current;
      // Value to adjust parameter by
      int degreeOfChange = max((ControlValues[control].max - ControlValues[control].min)/temperature, static_cast<unsigned int>(1));
      // If change is too large, pick random value from 1-49
      if (degreeOfChange > 100) {
         degreeOfChange = random() % 50;
      }
      int newValue = prevControlValue + (ControlValues[control].tryIncrease ? degreeOfChange : -degreeOfChange);
      int clippedNewValue = min(newValue, ControlValues[control].max);
      clippedNewValue = max(clippedNewValue, ControlValues[control].min);
      if (clippedNewValue == prevControlValue)
          ControlValues[control].tryIncrease = !ControlValues[control].tryIncrease;
      struct v4l2_control val = {ControlValues[control].control, clippedNewValue};
      ControlValues[control].current = clippedNewValue;
      vals.push_back(val);
      setControls(vals);

      framesSinceReset = 0;
      iterations++;

   } else {
      framesSinceReset++;
   }

   // Increase the value we alter each param after 2 full iterations
   if (iterations != 0 && (iterations % (2 * cNUM_CONTROL_ATTRIBUTES)) == 0) {
      temperature *= 2;
   }
}

// Retrieves the best settings
int CameraTab::getBestSettingsValue(__u32 controlName) {
   switch (controlName) {
      case V4L2_CID_EXPOSURE: return globalBestSettings.exposure;
      case V4L2_CID_BRIGHTNESS: return globalBestSettings.brightness;
      case V4L2_CID_CONTRAST: return globalBestSettings.contrast;
      case V4L2_CID_GAIN: return globalBestSettings.gain;
      case V4L2_CID_SATURATION: return globalBestSettings.saturation;
      case V4L2_CID_SHARPNESS: return globalBestSettings.sharpness;
      case V4L2_CID_BACKLIGHT_COMPENSATION: return globalBestSettings.backlightCompensation;
      case V4L2_CID_DO_WHITE_BALANCE: return globalBestSettings.whiteBalance;
   }
   return INT_MIN;
}

void CameraTab::printCameraControlValues(void) {
   qDebug() << "Camera control settings";
   for (int i = 0; i < cNUM_CONTROL_ATTRIBUTES; i++) {
      qDebug() << "\t" << ControlValues[i].name.c_str() << "=" << ControlValues[i].current;
   }
}

CameraButtonHandler::CameraButtonHandler(CameraTab *camtab, int control,
                                          QPushButton *qpb, QLineEdit *qle, QSlider *sl) :
      camtab(camtab), control(control), qpb(qpb), qle(qle), sl(sl) {
}

SliderHandler::SliderHandler(CameraTab *camtab, int control, QSlider *sl, QLineEdit *qle) :
	  camtab(camtab), control(control), sl(sl), qle(qle) {

}
CheckboxHandler::CheckboxHandler(CameraTab *camtab,QSlider *sl,QPushButton *db,QLineEdit *qle,QPushButton *ib,int control):
		camtab(camtab),sl(sl),db(db),qle(qle),ib(ib),control(control){

}
void CheckboxHandler::toggleState(bool enable){
	if(control > 1){
		sl->setEnabled(!enable);
		db->setEnabled(!enable);
		qle->setEnabled(!enable);
		ib->setEnabled(!enable);
	}
	int toggle = enable ? 1:0;
	if(control <= 1)
		toggle = enable ? 0:1;
	struct v4l2_control val = {controlIds[control], toggle};
	vector<struct v4l2_control> vals(1, val);
	camtab->setControls(vals);

}
 void SliderHandler::setValue(int newValue){
	 qle->setText(QString::number(newValue));
	 struct v4l2_control val = {controlIds[control], newValue};
	 vector<struct v4l2_control> vals(1, val);
	 camtab->setControls(vals);
 }

void CameraButtonHandler::setValue(unsigned int val) {
   qle->setText(QString::number(val));
}

int CameraButtonHandler::getControl() const {
   return control;
}

void CameraButtonHandler::clicked(bool) {
   __s32 diff = qpb ? (qpb->text() == "-" ? -1 : 1) : 0;
   __s32 newval = qle->text().toInt() + diff;
   if (newval >= controlMin[control] && newval <= controlMax[control]){
	   qle->setText(QString::number(newval));
	   sl->setSliderPosition(newval);
   }
}


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

#include "CameraToRR.hpp"
#include "utils/SPLDefs.hpp"
#include "utils/body.hpp"
#include "utils/Logger.hpp"
#include "utils/basic_maths.hpp"

using namespace std;

CameraToRR::CameraToRR()
{
   camera = 0;

   for (int i = 0; i < IMAGE_COLS; i++) {
      topEndScanCoords[i] = IMAGE_ROWS;
      botEndScanCoords[i] = IMAGE_ROWS*2;
   }
}

CameraToRR::~CameraToRR()
{
}

void CameraToRR::setCamera(Camera *cam) {
   camera = cam;
}

void CameraToRR::updateAngles(SensorValues val)
{
   values = val;
}

RRCoord CameraToRR::convertToRR(const Point &p, bool isBall) const
{
   return convertToRR(p.x (), p.y (), isBall);
}

RRCoord CameraToRR::convertToRR(int16_t i, int16_t j, bool isBall) const
{
   RRCoord myloc = pose.imageToRobotRelative(i, j, isBall ? 35 : 0);
   return myloc;
}

Point CameraToRR::convertToRRXY(const Point &p) const
{
   Point myloc = pose.imageToRobotXY(p, 0);
   return myloc;
}

RANSACLine CameraToRR::convertToRRLine(const RANSACLine &l) const
{
   return RANSACLine(convertToRRXY(l.p1), convertToRRXY(l.p2));
}

Point CameraToRR::convertToImageXY(const Point &p) const
{
   Point myloc = pose.robotToImageXY(p, 0);
   return myloc;
}

// TODO: Sean make this not just use top pixel size
float CameraToRR::pixelSeparationToDistance(int pixelSeparation,
      int realSeparation) const
{
   return (FOCAL_LENGTH * realSeparation) /
      (TOP_PIXEL_SIZE * pixelSeparation);
}

float CameraToRR::ballDistanceByRadius(int radius, bool top) const
{
   const float PIXEL = (top) ? TOP_PIXEL_SIZE : BOT_PIXEL_SIZE;
   extern bool offNao;
   radius += 2;
   bool isTopCamera = true;
   float height_to_neck = 440;
   float neck_pitch = 0;
   if (!offNao) {
      if (camera->getCamera() == BOTTOM_CAMERA) {
         isTopCamera = false;
      }
      const float *angles = values.joints.angles;
      neck_pitch = angles[Joints::HeadPitch];
   }
   float height;
   if (isTopCamera) {
      height = height_to_neck + NECK_TO_TOP_CAMERA *
            -sin(neck_pitch + ANGLE_OFFSET_TOP_CAMERA);
   } else {
      height = height_to_neck + NECK_TO_BOTTOM_CAMERA *
         -sin(neck_pitch + ANGLE_OFFSET_BOTTOM_CAMERA);
   }
   height -= BALL_RADIUS;
   float distance = (FOCAL_LENGTH * BALL_RADIUS * 1.3) /
      (PIXEL * radius);

   if (distance < height) {
      return -1;
   }
   distance = sqrt(SQUARE(distance) - SQUARE(height));
   return distance;
}

float CameraToRR::ballRadiusByDistance(Point p) const
{
   bool top = (p.y() < 960);
   Point q = Point(top? 640 : 320, p.y());
   
   Point r1 = pose.imageToRobotXY(q, BALL_RADIUS);

   // Another fix is to normalise the x cooridnate to the centre of the image

   // Hack to fix random large radii due to curvature
   /*if (r1.y() < -200) {
      r1.y() += 200;
   } else if (r1.y() > 200) {
      r1.y() -= 200;
   }*/

   Point r2 = r1 + Point(0, BALL_RADIUS);
   Point p2 = pose.robotToImageXY(r2, BALL_RADIUS);
   Point p3 = pose.robotToImageXY(r1, BALL_RADIUS);
   float pixels = fabs(p2.x() - p3.x());
   //cout << "___________\nRadius: " << pixels << endl;
   //cout << "Image: " << p3.x() << "," << p3.y() << " - " << p2.x() <<","<<p2.y() << " <=> " << r1.x()<<","<<r1.y() << " - " << r2.x() <<","<< r2.y() << endl;
   return pixels;
   /*
   
   
   bool top = p.y() > 960? false : true; //
   extern bool offNao;
   bool isTopCamera = top;//true;
   float height_to_neck = 440; /// does this need to change? I can fix this
   float neck_pitch = 0;
   if (!offNao) {
      if (camera->getCamera() == BOTTOM_CAMERA) {
         isTopCamera = false;
      }
      const float *angles = values.joints.angles;
      neck_pitch = angles[Joints::HeadPitch];
   }
   float height;
   if (isTopCamera) {
      height = height_to_neck + NECK_TO_TOP_CAMERA *
            -sin(neck_pitch + ANGLE_OFFSET_TOP_CAMERA);
   } else {
      height = height_to_neck + NECK_TO_BOTTOM_CAMERA *
         -sin(neck_pitch + ANGLE_OFFSET_BOTTOM_CAMERA);
   }
   height -= BALL_RADIUS;

   RRCoord ballCentre = pose.imageToRobotRelative(p, BALL_RADIUS);
   float distance = ballCentre.distance();


   float hypotenuse = sqrt(SQUARE(distance) + SQUARE(height));
   float theta = atan2f(BALL_RADIUS, hypotenuse);
   float pixels = (theta/1.064) * (top? 1280 : 640);

   // Compensation for error in bottom camera
   if (!top) pixels += 4;

   return pixels;*/
}

bool CameraToRR::isRobotMoving() const
{
   return true;
}

void CameraToRR::findEndScanValues() {
   const int exclusionRes = Pose::EXCLUSION_RESOLUTION;
   const int16_t *topPoints = pose.getTopExclusionArray();
   const int16_t *botPoints = pose.getBotExclusionArray();
   for (int i = 0; i < TOP_IMAGE_COLS; ++i) {
      if (topPoints[(i * exclusionRes)/TOP_IMAGE_COLS] < IMAGE_ROWS) {
         topEndScanCoords[i] = topPoints[(i * exclusionRes)/TOP_IMAGE_COLS];
      } else {
         topEndScanCoords[i] = IMAGE_ROWS;
      }
   }
   for (int i = 0; i < BOT_IMAGE_COLS; ++i) {
      if (botPoints[(i * exclusionRes)/BOT_IMAGE_COLS] < IMAGE_ROWS) {
         botEndScanCoords[i] = botPoints[(i * exclusionRes)/BOT_IMAGE_COLS];
      } else {
         botEndScanCoords[i] = IMAGE_ROWS*2;
      }
   }
}

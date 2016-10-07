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

#include "teamBallTab.hpp"

#include "utils/SPLDefs.hpp"
#include "utils/angles.hpp"
#include "types/ActionCommand.hpp"
#include "types/RobotObstacle.hpp"

#include <QGridLayout>
#include <QRect>
#include <QGroupBox>
#include <QLabel>
#include <QPixmap>
#include <QPainter>
#include <QColor>
#include <QTransform>

#include <iostream>
#include <vector>

#include "blackboard/Blackboard.hpp"

#define DEBUG_IMAGE_ROWS (IMAGE_ROWS / 4)
#define DEBUG_IMAGE_COLS (IMAGE_COLS / 4)

#define MAX_LAST_SEEN               10

using namespace std;

TeamBallTab::TeamBallTab(QTabWidget *parent, QMenuBar *menuBar) : image(":/images/spl_field.svg"),
                                                                      blackboard(0) {
   initMenu(menuBar);
   init();
   this->parent = parent;
}


void TeamBallTab::initMenu(QMenuBar *) {
}

void TeamBallTab::init() {
   fieldLabel = new QLabel();
   imagePixmap = QPixmap(640, 480);
   QRectF target(0.0, 0.0, 640.0, 480.0);
   QRectF source(0.0, 0.0, 740, 540.0);
   QPainter painter(&imagePixmap);
   painter.drawImage(target, image, source);

   QGridLayout *layout = new QGridLayout();
   this->setLayout(layout);
   layout->setAlignment(Qt::AlignTop);
   layout->addWidget(fieldLabel, 1, 1, 4, 1);

   /* draw the field with nothing on it */
   QPixmap fieldPixmap(imagePixmap);
   FieldPainter fieldPainter(&fieldPixmap);
   setTransform(fieldPixmap.width(), fieldPixmap.height());
   fieldLabel->setPixmap(fieldPixmap);
}

void TeamBallTab::setTransform(int device_width, int device_height) {
   transform = QTransform();
   // determine the size of the bitmap
   // int device_width = width;
   // int device_height = height;
   // translate to center of field
   transform.translate(device_width/2, device_height/2);
   // make positive y point upwards
   transform.scale(1,-1);
   // scale so we can draw using field coordinates
   transform.scale((float)device_width/OFFNAO_FULL_FIELD_LENGTH,
         (float)device_height/OFFNAO_FULL_FIELD_WIDTH);
}

void TeamBallTab::redraw() {
   if (!blackboard) return;

   const std::vector<BallInfo> hypotheses = readFrom(vision, balls);
   const std::vector<BallInfo> receivedBalls = readFrom(localisation, receivedBalls);
   const AbsCoord botPos = readFrom(localisation, robotPos);
   const AbsCoord ballAbs = readFrom(localisation, ballPos);

   QPixmap fieldPixmap(imagePixmap);

   FieldPainter fieldPainter(&fieldPixmap);
   setTransform(fieldPixmap.width(), fieldPixmap.height());

   std::cout << "Rec balls: " << receivedBalls.size() << std::endl;

   for (int i = 0; i < hypotheses.size(); i++) {
      int lifetime = hypotheses[i].lifetime;
      int lastSeen = hypotheses[i].lastSeen;

      cout << "Ball " << i << " - " << "Lifetime: "  << lifetime << ", Last seen: " << lastSeen << endl;
      QColor col = QColor(0, 255, 0, 255);
      fieldPainter.drawBallRR(hypotheses[i], botPos, col);
   }

   for (int i = 0; i < receivedBalls.size(); i++)
   {
      int lifetime = receivedBalls[i].lifetime;
      int lastSeen = receivedBalls[i].lastSeen;

      cout << "RECEIVED Ball " << i << " - " << "Lifetime: "  << lifetime << ", Last seen: " << lastSeen << endl;
      QColor col = QColor(0, 0, 255, 255);
      fieldPainter.drawBallRR(receivedBalls[i], botPos, col);
   }   
      

   // ball with 
   fieldPainter.drawRobotAbs(botPos, "yellow");
   fieldPainter.drawRobotAbs(ballAbs, "red");
   fieldLabel->setPixmap(fieldPixmap);

   return;
}



// TODO(brockw): see if this can be genericized into tab.cpp, so it's not in
// every tab
void TeamBallTab::newNaoData(NaoData *naoData) {

   if (naoData) {

if (naoData->getFramesTotal() != 0) {
    blackboard = (naoData->getCurrentFrame().blackboard);
    if (parent->currentIndex() == parent->indexOf(this)) {
      redraw();
    }
  }
   }
}

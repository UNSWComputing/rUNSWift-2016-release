/*
Copyright 2016 The University of New South Wales (UNSW).

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

#pragma once

#include "tabs/tab.hpp"
#include "mediaPanel.hpp"
#include "utils/FieldPainter.hpp"
#include "utils/FieldObject.hpp"
#include "types/AbsCoord.hpp"

#include <vector>

#include <QGroupBox>
#include <QMenuBar>
#include <QGridLayout>
#include <QPixmap>
#include <QLabel>
#include <QImage>

#include <cstdio>
#include <deque>



class Vision;

class Blackboard;

/*
 * This is the default/initial tab that shows on entering Off-Nao.
 * As the name suggests it gives an overview of most of the important
 * aspects of the nao. Good for general gameplay watching etc.
 *
 * Also will be used for localization debugging unless the localization
 * people need more info.
 */
class TeamBallTab : public Tab {
   Q_OBJECT
   public:
      TeamBallTab(QTabWidget *parent, QMenuBar *menuBar
                  );
   private:
      void init();
      void initMenu(QMenuBar *menuBar);
      void redraw();

      QTransform transform;
      void setTransform(int device_width, int device_height);

      QGridLayout *layout;

      /* These variables are used to present the debug variables from the nao*/
      QLabel *fieldLabel;
      QPixmap imagePixmap;
      QImage image;

      // Data
      Blackboard *blackboard;

   public slots:
      void newNaoData(NaoData *naoData);
};
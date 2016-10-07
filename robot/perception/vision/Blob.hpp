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

#pragma once

#include <utility>
#include <vector>

#include <boost/shared_ptr.hpp>

#include "CameraToRR.hpp"
#include "Fovea.hpp"
#include "ImageRegion.hpp"
#include "VisionConstants.hpp"
#include "VisionDefs.hpp"

#include "types/BallInfo.hpp"
#include "types/SensorValues.hpp"

// boundaries on blob size at lowest density
#define MAX_BLOB_SIZE 12
#define MIN_BLOB_SIZE 2

class Blob
{
   public:

      explicit Blob();

      /**
       * findColourBlobs
       * Find all blobs of a specified colour in a fovea
       * within the given bounds. These bounds are to reduce
       * the processing time of creating a new fovea.
       *
       * @param fovea     : region of image to search
       * @param t       : upper left of bounding box in fovea
       * @param width     : size of said bounding box
       * @param height    : size of said bounding box
       * @param c         : colour to search for
       */
      static std::vector<Blob> findBlobs(
            const Fovea &fovea,
            Colour c,
            Point offset);

      // Add a point to the blob
      void addPoint(Point p);

      // return the centre of the blob - average of all points
      Point getCentre();

      // return the size - number of pixels in the blob
      int getSize();

            // ADD IN SIZE RESTRICTIONS
      void setOffset(Point p);
      Point getOffset();

      /*
       * findBlobFromPoint
       * Given a point in a fovea that is of the specified colour (this is checked),
       * create the blob that that point is part of. Not that it must be within certain
       * size bounds to be considered a blob, dependent upon fovea and density
       *
       * @param fovea     : region of image to search
       * @param p         : the starting point for the blob
       * @param markedPixels         : marked map
       * @param c         : colour to search for
       */
      static Blob findBlobFromPoint(
            const Fovea &fovea,
            Point p,
            int markedPixels[],// 2d array
            Colour c);

   private:
      /*
       * A blob has a vector of points in the blob,
       * a centre, a size (the number of pixels)
       * and ...
       *
       */
      Point offset;
      Point centre;
      std::vector<Point> points;
      int size;

};


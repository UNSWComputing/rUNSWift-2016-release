#include "BallDetection.hpp"
#include "Blob.hpp"

#include <string.h>
#include <algorithm>
#include <limits>
#include <vector>

#include "utils/Logger.hpp"
#include "utils/basic_maths.hpp"
#include "utils/SPLDefs.hpp"
#include "utils/Timer.hpp"

#include "types/Point.hpp"
#include "types/XYZ_Coord.hpp"

//#include "motion/touch/FilteredTouch.hpp"

using namespace std;
using namespace boost::numeric::ublas;

#define MARKED 1
#define UNMARKED 0


Blob::Blob()
{
   centre = Point(-1,-1);
   offset = Point(0,0);
   size = 0;
   points = std::vector<Point>();
}

// Return the centre of the blob - average of all the points
// In the case of an empty blob, it returns (-1,-1)
Point Blob::getCentre()
{
   if (points.size() != 0 && centre.x() == -1 && centre.y() == -1) {
      centre.x() = 0;
      centre.y() = 0;
      // Average the points
      std::vector<Point>::iterator i;
      for (i = points.begin(); i != points.end(); ++ i) {
         centre.x() += i->x();
         centre.y() += i->y();
      }
      centre.x() /= points.size();
      centre.y() /= points.size();
   }
   return centre;
}

// Return the size of the blob - the number of pixels in it
int Blob::getSize()
{
   return points.size();
}

// Add a point to the blob
void Blob::addPoint(Point p)
{
   points.push_back(p);
}

// Set the offset of the blob in the fovea
void Blob::setOffset(Point p)
{
   offset = p;
}

// Get the blob offset
Point Blob::getOffset()
{
   return offset;
}

/*
 * findBlobFromPoint
 * Given a starting point and a colour in a fovea, create the blob that
 * starts from there of that colour
 * This has size restrictions on what is a valid blob, dependent on fovea and density
 * Upon failure, this returns a blob with centre -1,-1
 */
Blob Blob::findBlobFromPoint(
         const Fovea &fovea,
         Point p,
         int markedPixels[],
         Colour c)
{
   Blob* b = new Blob();
   // sanity check that the colour of the given pixel is correct
   if (fovea.colour(p.x(),p.y()) != c)
      return *b;

   // Mark the first point as seen on the markedPixels map
   // Remember, the same point can be used to index both the fovea and the
   // markedPixels array, as they are the same size
   // Also need to ensure we do not go outside the array bounds
   int height = fovea.bb.height();
   int width = fovea.bb.width();
   Point curr;
   std::vector<Point> stack = std::vector<Point>();
   stack.push_back(p);
   while (stack.size() > 0) {
      // Get the top pixel, mark it, add it to the blob
      curr = stack.back();
      stack.pop_back();
      b->addPoint(curr);
      markedPixels[curr.y()*width + curr.x()] = MARKED;

      // Get the pixels adjacent to curPoint that are
      // of the given colour AND unmarked
      // Ensure that each surrounding pixel is within the fovea
      // First, check above the pixel
      if (curr.y() - 1 >= 0) {
         if (fovea.colour(curr.x(), curr.y()-1) == c 
               && markedPixels[(curr.y()-1)*width+curr.x()] == UNMARKED) {
            markedPixels[(curr.y()-1)*width+curr.x()] = MARKED;
            stack.push_back(Point(curr.x(),curr.y()-1));
         }
      }
      // To the right
      if (curr.x() + 1 < width) {
         if (fovea.colour(curr.x()+1,curr.y()) == c 
               && markedPixels[curr.y()*width+(curr.x()+1)] == UNMARKED) {
            markedPixels[curr.y()*width+curr.x()+1] = MARKED;
            stack.push_back(Point(curr.x()+1,curr.y()));
         }
      }
      // Below
      if (curr.y() + 1 < height) {
         if (fovea.colour(curr.x(),curr.y()+1) == c 
               && markedPixels[(curr.y()+1)*width+curr.x()] == UNMARKED) {
            markedPixels[(curr.y()+1)*width+curr.x()] = MARKED;
            stack.push_back(Point(curr.x(),curr.y()+1));
         }
      }
      // To the left
      if (curr.x() - 1 >= 0) {
         if (fovea.colour(curr.x()-1,curr.y()) == c 
               && markedPixels[curr.y()*width+(curr.x()-1)] == UNMARKED) {
            markedPixels[curr.y()*width+curr.x()-1] = MARKED;
            stack.push_back(Point(curr.x()-1,curr.y()));
         }
      }
   }
   b->getCentre();
   return *b;
}


/**
 * findBlobs
 * Given a fovea and a colour, find all blobs in that fovea of that colour
 * This has size restrictions on what is a valid blob, dependent on fovea and density
 * Upon failure, this returns a blob with centre -1,-1
 */
std::vector<Blob> Blob::findBlobs(
         const Fovea &fovea,
         Colour c,
         Point offset)
{
   int width = fovea.bb.width();
   int height = fovea.bb.height();
   // Create the array of pixels we have seen
   int markedPixels[height*width];
   // Zero the array
   for (int i = 0; i < height*width; ++i) {
      markedPixels[i] = UNMARKED;
   }


   // Search through the fovea for pixels of the given colour,
   // and then create blobs from there
   std::vector<Blob> blobList = std::vector<Blob>();
   for (int x = 0; x < width; ++x) {
      for (int y = 0; y < height; ++y) {
         // If we find a pixel of the given colour that has not been marked,
         // create a blob from here
         if (fovea.colour(x,y) == c && markedPixels[y*width+x] == UNMARKED) {
            Blob b = Blob::findBlobFromPoint(fovea, Point(x,y), markedPixels, c);
            // Do blob size checking
            // Add checks for different densities
            if (b.getSize() >= MIN_BLOB_SIZE*fovea.density
                  && b.getSize() <= MAX_BLOB_SIZE*fovea.density) {
               b.setOffset(offset);
               blobList.push_back(b);
            }
         }
      }
   }

   return blobList;
}
















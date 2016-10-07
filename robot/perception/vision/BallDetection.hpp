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
#include <fstream>

#include <boost/shared_ptr.hpp>

#include "CameraToRR.hpp"
#include "Fovea.hpp"
#include "ImageRegion.hpp"
#include "VisionConstants.hpp"
#include "VisionDefs.hpp"

#include "types/BallInfo.hpp"
#include "types/RobotInfo.hpp"
#include "types/RobotObstacle.hpp"
#include "types/SensorValues.hpp"
#include "types/Cluster.hpp"

#include "robotdetection/types/PossibleRobot.hpp"

#include "utils/Timer.hpp"

#define ANGLE_DELAY 4

#define NUM_ANGLE_BINS 4
#define NUM_CCD_POINTS 16

#define MAX_DENSITY 8
#define MIN_DENSITY 1


// Struct for a Region of Interest around a ball
typedef struct {
   Point tl;
   int density;
   int width;
   int height;
} Roi;


class Vision;

class BallDetection
{
   public:
      /**
       * The coordinates that have been detected to be on
       * the edge of the ball. For debugging display
       **/
      std::vector<Point> ballEdgePoints;

      explicit BallDetection();

      Timer timer;


      /**
       * findBalls
       * Find all balls in an image. Can potentially return more than
       * one ball.
       *
       * @param frame   : VisionFrame associated with current data
       * @param fovea   : Region of image to search
       * @param seed    : Seed value past to rand_r
       * @param terminal: Prevent ball detector from recursing on smaller foveas
       */
      void findBalls(
            VisionFrame &frame, 
            const Fovea &topFovea,
            const Fovea &botFovea,
            unsigned int *seed);


      /* Pretty stuff for offnao */
      std::vector<boost::shared_ptr<FoveaT<hNone, eGrey> > > ballFoveas;
      std::vector<boost::shared_ptr<FoveaT<hNone, eNone> > > trackingFoveas;;
      std::vector<Roi> ballRois;

      std::vector<Point> clusterCentres;
      std::vector<Point> darkSpots;
      std::vector<Point> ccdCentres;
      std::vector<Point> maximums;

      /* Given a frame and an image y coord, return the expected ball size */
      static float getRadiusInImage(
            VisionFrame &frame,
            Point p);

      // Localisation ball and ball velocity
      RRCoord localisationBall;
      RRCoord ballVelRR;


      // Positions of robots from localisation
      std::vector<RobotObstacle> localisationRobots;

      // vector of points from line edges from FieldFeatureDetection
      std::vector<Point> lineEdges;

      // vector of detected robots and possible robots from robot detection
      std::vector<RobotInfo> robots;
      std::vector<PossibleRobot> topPossRobots;
      std::vector<PossibleRobot> botPossRobots;

      // Robot Rock Angle (for smoothing ball distances)
      float latestAngleX;

   private:

      int ballHintAge; // how many frames since we last saw the ball
      int ball_dx; // pixel speed of the ball based on 1 observation 
      // TODO can actual put a decent filter ddon this

      // Vector of past balls
      std::vector<BallInfo> prevBalls;

      float angleXDelay[ANGLE_DELAY];

      static const int dontTrustKinematicsBeyond;
      static const int ballEdgeThreshold;
      static const int ballCloseThreshold;
      static const int ballColourRatio;
      static const int maxPixelsAboveFieldEdge;
      static const int trackingSizeX;
      static const int trackingSizeY;
      static const int maxTrackBallRadius;
      static const int maxBallHintAge;
      static const int ballHintEdgeThreshold;

      static const int topImageHeight;
      static const int topImageWidth;
      static const int botImageHeight;
      static const int botImageWidth;

      static const int angleLimit;
      static const float headTiltLimit;
      static const float blackSpotVar;
      static const float halfPI;
      static const int minInitialDarkness;
      static const int minDarknessPercentage;
      static const int minBallDarkness;
      static const int maxDarkness;
      static const int maxBallDarkness;
      static const int secondaryDarkThreshold;

      static const float hardVarianceBound;

      static const int hardMinColourRatio;
      static const int hardInsideColourBound;
      static const int maxColourRatio;
      static const int minCCDRating;

      // the darkest point in the current image
      int globalDarkest;

      // The binarised, normalised image of the ball
      char * bits;

      // array of points for the bottoms of robots
      // Populated by detected robots bounding boxes
      // only scan below these points
      // if no robot, value is 0
      int robotBottomsTopFrame[TOP_IMAGE_COLS / MAX_DENSITY];
      int robotBottomsBotFrame[BOT_IMAGE_COLS / MAX_DENSITY];


      struct ball_seed_t
      {
         Point centre;
         int radius;
         int count;
      };

      /**
       * findBallsR
       * Recursive helper for findBalls
       *
       * @param frame   : VisionFrame associated with current data
       * @param fovea   : Region of image to search
       * @param seed    : Seed value past to rand_r
       */
      void findBallsR(
            VisionFrame &frame, 
            const Fovea &fovea,
            unsigned int *seed,
            bool terminal = false);

      /**
       * findBall
       * Find all balls in an image. Can potentially return more than
       * one ball.
       *
       * @param frame   : VisionFrame associated with current data
       * @param fovea   : Fovea in which seed location exists
       * @param ball    : Seed location to search for ball
       * @param seed    : Seed value passed to rand_r
       */
      void findBall(
            VisionFrame &frame,
            const Fovea &fovea,
            ball_seed_t &ball,
            unsigned int *seed);

      /**
       * findWhiteBalls
       * Find all white soccer balls (2016) in an image.
       * Can potentially return more than one ball.
       *
       * @param frame   : VisionFrame associated with current data
       * @param fovea   : Fovea of the image
       * @param seed    : Seed value passed to rand_r
       */
      void findWhiteBalls_DecisionTree(
            VisionFrame &frame,
            const Fovea &fovea);

      void findWhiteBalls(
            VisionFrame &frame,
            const Fovea &fovea);

      void checkRobotOcclusion(
            VisionFrame &frame);

      void updateBallHeatMap(
            VisionFrame &frame);

      void populateRobotArrays(
            VisionFrame &frame,
            bool top);

      void findWhiteBallsTopFovea(
            VisionFrame &frame,
            const Fovea &fovea);

      Point revisedClusterCentre(
            const Fovea &fovea,
            int radius,
            Point p);

      std::vector<Point> findSaliencyRegions(
            VisionFrame &frame,
            const Fovea &fovea);

      std::vector<Cluster> clusterPointsCLC(
            VisionFrame &frame,
            const Fovea &fovea,
            std::vector<Point>);

      Point findDarkestPoint(
            VisionFrame &frame,
            const Fovea &fovea,
            BBox box,
            int * darkestVal);

      BallInfo ccdNativeRes(
            VisionFrame &frame,
            const Fovea &fovea,
            bool top,
            BallInfo ball);

      BallInfo ccdBallDetector(
            VisionFrame &frame,
            const Fovea &fovea,
            BBox box,
            BallInfo ball);

      bool runDecisionTree(
            VisionFrame &frame,
            const Fovea &fovea,
            bool tracking,
            BallInfo &ball);

      int getYValue(
            VisionFrame &frame,
            bool top,
            Point p);

      int getUValue(
            VisionFrame &frame,
            bool top,
            Point p);

      int getVValue(
            VisionFrame &frame,
            bool top,
            Point p);

      int * createBitStringFromBall(
            VisionFrame &frame,
            bool top,
            BallInfo &ball);

      bool insideRobot(
            VisionFrame &frame,
            Point p);

      /**
       * Helper functions
       */
      static bool isHeadTiltedForward(
            VisionFrame &frame);


      /**
       * trackLastWhiteBall
       * Uses the location of the last seen white soccer ball (2016)
       * to search again for a ball in the current frame
       *
       * @param frame   : VisionFrame associated with current data
       * @param ball    : Ball from previous frame
       */
      void trackLastWhiteBall_DecisionTree(
            VisionFrame &frame,
            const Fovea &fovea,
            BallInfo b);

      void trackLastWhiteBall(
            VisionFrame &frame,
            const Fovea &fovea,
            BallInfo b);

      /**
       * classifyBallsInFovea
       * Search through a sub-fovea and apply the white ball classfiers to it
       *
       * @param frame   : VisionFrame associated with current data
       * @param fovea   : Sub-fovea of the image
       * @param seed    : Seed value passed to rand_r
       */
      void classifyBallsInFovea(
            VisionFrame &frame,
            const Fovea &fovea,
            unsigned int *seed);

      /**
       * updateBallHypotheses
       * Perform a sanity check on each ball hypothesis and update
       * its current weighting accordingly
       *
       * @param frame   : VisionFrame associated with current data
       * @param fovea   : Fovea in which the hypotheses are located
       */
      void updateBallHypotheses(
            VisionFrame &frame);

      /**
       * collateBlackHypotheses
       * Combine overlapping black spot hypotheses
       *
       * @param frame   : VisionFrame associated with current data
       * @param fovea   : Fovea in which the hypotheses are located
       * @param balls   : List of black spot ball hypotheses
       */
      void collateBlackHypotheses(
            VisionFrame &frame,
            const Fovea &fovea,
            std::vector<BallInfo> balls);

      /**
       * findStrongEdges
       * Find local maxima in the edge saliency to get a point cloud
       * over the image of worthwhile points
       *
       * @param frame   : VisionFrame associated with current data
       * @param fovea   : Fovea to scan over
       * @return edges  : a vector of candidate points found
       */
      std::vector<Point> findStrongEdges(
            VisionFrame &frame,
            const Fovea &fovea);

      /**
       * findStrongEdgesHighRes
       * Given a higher res sub-fovea, search for stronger edge points
       * on which to RANSAC for a ball
       *
       * @param frame   : VisionFrame associated with current data
       * @param fovea   : Fovea to scan over
       * @return edges  : vector of candidate points
       */
      std::vector<Point> findStrongEdgesHighRes(
            VisionFrame &frame,
            const Fovea &fovea);

      /**
       * clusterPoints
       * Given a set of points in a fovea, group them into clusters of size
       * about a ball diameter wide, to give candidate regions upon which to search further
       *
       * @param frame   : VisionFrame associated with current data
       * @param fovea   : Fovea that the points are in
       * @param seed    : Seed value passed to rand_r
       * @param points  : vector of the points to cluster
       * @return cluster: vector of the clusters of the points
       */
      std::vector<Cluster> clusterPoints(
            VisionFrame &frame,
            const Fovea &fovea,
            std::vector<Point> points);

      float hogClassifierInFovea(
            const Fovea &fovea,
            BallInfo ball,
            BBox box);

      /**
       * findBlackSpotsInFovea
       * Search through a fovea for connected components of spots below
       * the black level threshold and mark these as balls (recursive)
       *
       * @param frame   : VisionFrame associated with current data
       * @param fovea   : Fovea to scan through
       * @param box     : Bounding box within fovea to scan through
       */
      void findBlackSpotsInFovea(
            VisionFrame &frame,
            const Fovea &fovea,
            BBox box);

     /**
       * isPointInRobot
       * Check through all previously found robots to see if the given
       * point is within their bounds
       *
       * @param frame   : VisionFrame associated with current data
       * @param p       : Point to check
       */
      bool isPointInRobot(
            VisionFrame &frame,
            Point p);

      /**
       * isPointNearFieldFeature
       * UNUSED
       * Given a point, is it near any previously-found field features?
       *
       * @param frame   : VisionFrame associated with current data
       * @param p       : Point to check
       */
      bool isPointNearFieldFeature(
            VisionFrame &frame,
            Point p);

      /**
       * createFieldBall
       * Derive the real-world coordinates and parameters for a ball given its
       * position in an image and the frame for the image
       *
       * @param frame   : VisionFrame associated with current data
       * @param fovea   : Fovea in which the ball is located
       * @param ball    : The ball
       * @return ball   : The ball with updated parameters
       */
      BallInfo createFieldBall(
            VisionFrame &frame,
            const Fovea &Fovea,
            BallInfo ball);

      /**
       * findEdgeRegions
       * Find all white-black balls in a given fovea (2016 ball)
       * This is intended for zoomed-in foveas on white balls
       * We scan for strong edges next to no edges, and then RANsAC from here.
       *
       * @param frame   : VisionFrame associated with current data
       * @param fovea   : Fovea in which to search for ball
       */
      std::vector<BBox> findEdgeRegions(
            VisionFrame &frame,
            const Fovea &fovea);

      /**
       * trackLastBall
       * Attempt to relocate a previously seen ball. Only really useful for
       * balls of small radii 
       *
       * @param frame   : VisionFrame associated with current data
       * @param fovea   : Fovea in which seed location exists
       * @param ball    : Previously known BallInfo
       * @param seed    : Seed value past to rand_r
       */
      void trackLastBall(
            VisionFrame &frame, 
            const Fovea &fovea, 
            const BallInfo &ball,
            unsigned int *seed);


      /**
       * ransacBall
       * Search vector of points for circle. If good match is found,
       * add found circle to list of detected balls
       *
       * @param frame      : VisionFrame associated with current data
       * @param fovea      : Region of image to search
       * @param ballEdges  : Vector of edge points to search
       * @param radius     : Guessed radius of ball to search for
       * @param seed       : Seed value past to rand_r
       */
      void ransacBall(
            VisionFrame &frame,
            const Fovea &fovea,
            const std::vector<Point> &ballEdges,
            int radius,
            unsigned int *seed);


      /**
       * ransacWhiteBall
       * Search vector of points for circle. If good match is found,
       * add found circle to list of detected balls
       *
       * @param frame      : VisionFrame associated with current data
       * @param fovea      : Region of image to search
       * @param ballEdges  : Vector of edge points to search
       * @param radius     : Guessed radius of ball to search for
       * @param seed       : Seed value past to rand_r
       */
      void ransacWhiteBall(
            VisionFrame &frame,
            const Fovea &fovea,
            const std::vector<Point> &ballEdges,
            unsigned int *seed);

      /**
       * populateSeedLocations
       * Generate list of viable hypothises for ball locations
       * to seed into findBall()
       *
       * @param frame   : VisionFrame associated with current data
       * @param fovea   : Region of image to search
       * @param seeds   : Vector to fill with seed locations
       * @see findBall
       */
      void populateSeedLocations(
            VisionFrame &frame,
            const Fovea &fovea,
            std::vector<ball_seed_t> &seeds);


      /**
       * guessRadiusAndValidateSeed
       * Perform an initial set of sanity checks on a potential ball. If
       * seed passes initial checks, guess the radius of the ball from
       * either saliency or kinematics.
       *
       * @param frame      : VisionFrame associated with current data
       * @param fovea      : Region of image to search
       * @param xhistogram : orange x histogram below field edge
       * @param xhistogram : orange y histogram below field edge
       * @param ball       : Seed location of ball
       *
       * @return true if seed is valid
       * @see findBall
       */
      bool guessRadiusAndValidateSeed(
            VisionFrame &frame,
            const Fovea &fovea,
            const Histogram<int, cNUM_COLOURS> &xhistogram,
            const Histogram<int, cNUM_COLOURS> &yhistogram,
            ball_seed_t &ball);

      /**
       * ballRadiusFromHistogram
       * Guess the radius of the ball based on the histograms
       *
       * @param ball    : Seed location to search for ball
       * @param xhistogram : orange x histogram below field edge
       * @param xhistogram : orange y histogram below field edge
       *
       * @return radius of ball
       */
      int ballRadiusFromHistogram(
            const Fovea &fovea,
            const Histogram<int, cNUM_COLOURS> &xhistogram,
            const Histogram<int, cNUM_COLOURS> &yhistogram,
            const ball_seed_t &ball);

      /**
       * ballRadiusFromHistogram
       * Guess the radius of the ball based on kinematic projection
       *
       * @param distance: Distance to ball as calculated from
       *                  the kinematic chain
       *
       * @return radius of ball
       */
      int ballRadiusFromDistance(int distance, bool top);

      /**
       * findSmallBallEdges
       * Generate a list of ball edge points from a small ball.
       *
       * @param fovea   : Region of image to search for ball edges.
       *                  This is highly intensive and should be performed on
       *                  the smallest fovea possible. The fovea must also
       *                  have been calculated using the eBall edge weightings
       * @param edges   : Vector onto which edges will be appended
       */
      void findSmallBallEdges(const Fovea &fovea, std::vector<Point> &edges);

      /**
       * findBallEdges
       * Generate a list of ball edge points from a small ball.
       *
       * @param fovea   : Region of image to search for ball edges.
       *                  This is highly intensive and should be performed on
       *                  the smallest fovea possible. The fovea must also
       *                  have been calculated using the eBall edge weightings
       * @param edges   : Vector onto which edges will be appended
       */
      void findBallEdges(const Fovea &fovea, std::vector<Point> &edges);

      /**
       * scanBoxForEdges
       * Scan a box for edges in a configurable scan pattern
       *
       * @param fovea   : Region of image to search for ball edges.
       *                  This is highly intensive and should be performed on
       *                  the smallest fovea possible. The fovea must also
       *                  have been calculated using the eBall edge weightings
       * @param start   : Point to start scanning from
       * @param size    : Length of scan box edges
       * @param dx      : dx component of top edge
       * @param dy      : dy component of top edge
       * @param edges   : Vector onto which edges will be appended
       */
      void scanBoxForEdges(
            const Fovea &fovea,
            const Point start,
            const int size,
            const int dx,
            const int dy,
            std::vector<Point> &edges);
      /**
       * Count the number orange pixels in a square region. Assumes entire
       * image region is a valid part of the fovea. 
       * 
       * @param fovea   : Region of image to search.
       * @param ball    : Seed location to search around.
       * @param radius  : Distance to search around ball seed. A radius of 1
       *                  would test 1 pixel, radius of 2 would test 9 pixels.
       *
       * return number of orange pixels around the seed point
       */
      int countOrangeAround(
            const Fovea &fovea, 
            const ball_seed_t &ball,
            const int radius);

      /**
       * makeBallHistsBelowFieldEdge
       * 
       * @param frame   : VisionFrame associated with current data
       * @param fovea   : Region of image to search.
       * @param xhist   : XHistogram to output to
       * @param yhist   : YHistogram to output to
       *
       * return true if ball is below field edge 
       */
      void makeBallHistsBelowFieldEdge(
            VisionFrame &frame,
            const Fovea &fovea,
            Histogram<int, cNUM_COLOURS> &xhist,
            Histogram<int, cNUM_COLOURS> &yhist);



      /**
       * doSeedsOverlap
       * 
       * @param a       : first ball seed
       * @param b       : second ball seed
       *
       * return true if ball seeds overlap
       */
      bool doSeedsOverlap(
            const ball_seed_t &a,
            const ball_seed_t &b);


      int checkSeed(
            const Fovea       &fovea,
            const ball_seed_t &seed);

      int countColourInBox(
            const Fovea &fovea,
            BBox         roi,
            int          density,
            Colour       colour);

      int countColourAroundBox(
            const Fovea &fovea,
            BBox         roi,
            int          density,
            Colour       colour);
      
      /**
       * Sanity Checks
       */
      bool isBallRadiusTooBig(const VisionFrame &frame, const BallInfo &ball);
      /* TODO(carl) port these sanity checks over to the new architecture */
      /**
       * A dirty hack to delete cases where some of the background is below
       * the field edge, mainly due to looking side on to the goal posts
       * causing the field edge detectino to be wrong, and balls get seen
       * in this area
       **/
      bool isBallAboveMissedEdge(ImageRegion *ballRegion, Vision *vision,
            const std::pair<int, int> &horizon);
      /**
       * Another dirty hack to remove balls from robots that are
       * occassionally missed
       **/
      bool isBallInMissedRobot(Vision *vision);

      /**
       * Tests to see if the detected ball contains the original
       * ballRegion
       **/
      bool isBallInsideBallRegion(ImageRegion *ballRegion);
};


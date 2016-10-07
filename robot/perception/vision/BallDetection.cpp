#include "BallDetection.hpp"

#include <algorithm>
#include <limits>
#include <vector>

#include "Ransac.hpp"
#include "yuv.hpp"
#include "J48Tree.hpp"

#include "utils/Logger.hpp"
#include "utils/basic_maths.hpp"
#include "utils/BresenhamPtr.hpp"
#include "utils/SPLDefs.hpp"
#include "utils/body.hpp"
#include "CameraToRR.hpp"
#include "utils/Timer.hpp"
#include "types/Cluster.hpp"
#include "types/FieldFeatureInfo.hpp"

#include "types/Point.hpp"
#include "types/XYZ_Coord.hpp"
#include "types/RobotInfo.hpp"

/* ################################# */
/*  TO USE WHITE BALL DETECTION      */
/*  INSTEAD OF ORANGE, UNCOMMENT     */
#define WHITE_BALL

/*
   One of the ball detectors uses a decision tree
   The other uses a series of filters, and foveating
   The latter is slower, but more robust to lighting changes
   Uncomment the below line to use this latter detection
 */
#define DECISION_TREE_DETECTION

// To debug white ball detection, uncomment
//#define BALL_DEBUG_POINTS
//#define BALL_DEBUG_PRINTOUT
//#define DATA_COLLECTION

//#include "motion/touch/FilteredTouch.hpp"


#define UNDEFINED                   (-1)
#define LARGE_NUMBER                1000000

#define HD_STRONG_EDGE_THRESHOLD    (400*400)
#define REALLY_STRONG_EDGE_THRESHOLD (1000*1000)
#define STRONG_EDGE_THRESHOLD       (500*500)
#define MED_EDGE_THRESHOLD          (300*300)
#define WEAK_EDGE_THRESHOLD         (200*100)

#define YUV_Y_BOUND                 100
#define NUM_CCD_ITERATIONS          5
#define NUM_BUCKETS                 6
#define FIELD_EDGE_BALL_OFFSET      3

#define COLOUR_VAL                  1
#define WHITE_Y                     130 // This is tweaked for lighting conditions
#define Y_THRESH                    72
#define U_THRESH                    135
#define V_THRESH                    118
#define MIN_LINE_DARKNESS           45
#define DARK_Y                      64
#define BRIGHT_LEVEL                180
#define MIN_BRIGHT_PIXELS           80
#define MAX_BRIGHT_PIXELS           500

#define EXPECTED_FRAME_RATE         30

#define MIN_ACCEPTABLE_RADIUS       5

#define A_FOOT                      (300*300)
#define TWO_METRES                  2000

#define MAX_LAST_SEEN               12 //5
#define TWENTY_CM                   (200*200)
#define MIN_NEEDED_FRAMES           2
#define MIN_NEEDED_LIFETIME         50

#define ROBOT_ERROR                 (-4)
#define ROBOT_BOTTOM_ERROR          (ROBOT_ERROR*MAX_DENSITY)


using namespace std;
using namespace boost::numeric::ublas;

/* Old orange ball constants */
const int BallDetection::trackingSizeX             = 160;//160;
const int BallDetection::trackingSizeY             = 160;//160;

const int BallDetection::ballEdgeThreshold         = 150;
const int BallDetection::ballCloseThreshold        = 450;
const int BallDetection::ballColourRatio           = 512; /* ratio * 1024 */

const int BallDetection::dontTrustKinematicsBeyond = 100;
const int BallDetection::maxPixelsAboveFieldEdge   = 6;// * IMAGE_COLS / 640;

const int BallDetection::maxTrackBallRadius        = 8;

const int BallDetection::maxBallHintAge            = 80;
const int BallDetection::ballHintEdgeThreshold     = 100;
/* ################################### */

const int BallDetection::topImageHeight            = 960;
const int BallDetection::topImageWidth             = 1280;
const int BallDetection::botImageHeight            = 480;
const int BallDetection::botImageWidth             = 640;

// These should vary depending on y-position
const int BallDetection::angleLimit                = 60;
const float BallDetection::headTiltLimit           = 0.2;
const int BallDetection::minInitialDarkness        = 88;
const int BallDetection::minBallDarkness           = 98;
const int BallDetection::minDarknessPercentage     = 98;
const int BallDetection::maxDarkness               = 255;
const int BallDetection::maxBallDarkness           = 255;
const int BallDetection::secondaryDarkThreshold    = 95;

const float BallDetection::halfPI                  = 1.57;

const float BallDetection::hardVarianceBound       = 1;

const int BallDetection::hardMinColourRatio        = 1;
const int BallDetection::hardInsideColourBound     = 100;
const int BallDetection::maxColourRatio            = 100;
const int BallDetection::minCCDRating              = 50;

/* ################
   Helper functions
   ################ */

// Given a fovea and a y-coord in image-coords,
// what density should we use?
// Values derived by SCIENCE
int getDensity(int y)
{
   // If we are in the lower two thirds of bottom fovea, density 8
   if (y > 960) { // top image height
      return MAX_DENSITY;
   } else {
      // These boundaries were derived experimentally, after simply
      // setting them and testing until it worked well in enough settings
      if (y > 700) return MAX_DENSITY/2;
      if (y > 500) return MAX_DENSITY/4;
      if (y > 300) return MIN_DENSITY;
      return MIN_DENSITY;
   }
   // default
   return MAX_DENSITY;
}

// Given a fovea and a point, return the minimum of the point and the field edge
Point getFieldEdgePoint(VisionFrame& frame, const Fovea& fovea, Point p) {
   Point start = fovea.mapFoveaToImage(p);
   int fieldTop = frame.topStartScanCoords[start.x()];
   if (!fovea.top) {
      fieldTop = frame.botStartScanCoords[start.x()];
      if (fieldTop == 480) // botImageHeight
         fieldTop = 0;
   }
   start.y() = std::max(fieldTop, start.y());
   start = fovea.mapImageToFovea(start);
   return start;
}

// Given a fovea and a y-coord in image coords, get the approximate radius we expect.
// The function below comes from measuring the diameter of the ball in pixels
// at different heights in each image, and then linearly interpolating over
// these data points. The resulting affine functions have little error, as
// the measurements were very linear, except when going from half a field away to
// the full field length away - but at that distance, the ball is unlikely to be
// seen anyway. The constants are from the affine approximations, derived in
// OS X's program Grapher
float BallDetection::getRadiusInImage(VisionFrame &frame, Point p)
{
   //float radius = frame.cameraToRR.ballRadiusByDistance(p);

   int y = p.y();
   double a = 0, b = 0;
   if (y < BallDetection::topImageHeight) {
      if (isHeadTiltedForward(frame)) {
         a = 0.218;
         b = -8.86;
      } else {
         a = 0.21;
         b = -43.84;
      }
   } else {
      // Remove offset of top image coords
      y -= BallDetection::topImageHeight;

      // if neck is tilted, use different parameters
      if (isHeadTiltedForward(frame)) {
         a = 0.146;
         b = 82.5;
      } else {
         a = 0.151;
         b = 76.77;
      }
   }
   double diameter = a*y + b;
   if (diameter < 0) return 0;

   // Compensation for kinematics error
   diameter += 8;

   return diameter/2;
}


// Is the robot's head tilted forward?
// Normal pitch is about 1.8, tilted is close to 0.5
// I use 0.3 as a threshold
bool BallDetection::isHeadTiltedForward(VisionFrame &frame)
{
   double neck_pitch = frame.cameraToRR.values.joints.angles[Joints::HeadPitch];
   if (!isnan(neck_pitch) && neck_pitch > BallDetection::headTiltLimit) {
      return true;
   }
   return false;
}

// Return the squared Euclidean distance between two points
inline int squaredDist(Point a, Point b)
{
   return (a.x()-b.x())*(a.x()-b.x()) + (a.y()-b.y())*(a.y()-b.y());
}

/* ########################### */
/* Constructor and comparators */
/* ########################### */

BallDetection::BallDetection()
{
   ballHintAge = 0;
   ball_dx = 0;

   bits = (char *) malloc(SIZE*SIZE*sizeof(char));
}

/* Sort balls according to likelyhood of truth */
struct BallInfoCmp
{
   bool operator() (const BallInfo &a, const BallInfo &b) const
   {
      return a.visionVar < b.visionVar;
   }
};

/* Sort balls according to lifetime and # frames dropped */
struct PastBallInfoCmp
{
   bool operator() (const BallInfo &a, const BallInfo &b) const
   {
      //return a.lifetime > b.lifetime;
      return a.lifetime-a.lastSeen > b.lifetime-b.lastSeen;
   }
};

/* ################################### */
/* Ball Detection for 2016 soccer ball */
/* ################################### */

void BallDetection::findBalls(
      VisionFrame &frame,
      const Fovea &topFovea,
      const Fovea &botFovea,
      unsigned int *seed)
{
   ballFoveas.clear();
   clusterCentres.clear();
   darkSpots.clear();
   ccdCentres.clear();
   maximums.clear();

   // Update angleX to take into account motion rocking and sensor delay
   for(int i = ANGLE_DELAY-1; i > 0; --i) {
      angleXDelay[i] = angleXDelay[i-1];
   }
   angleXDelay[0] = latestAngleX;

#ifdef WHITE_BALL
   // We try to find the ball in the bottom fovea
   // then update the hypotheses we get with various sanity checks
   // This then sorts the hypotheses according to variance,
   // and throws away all but the lowest variance. If its
   // variance is still too high, then we chuck this too
   // and repeat on the top fovea
   // Use previous ball estimate if we have one
   if (frame.last) {
      if (frame.last->balls.size() > 0) {
         // Grab the coordinates of the last ball and foveate
         BallInfo b = frame.last->balls[0];
         bool top = b.topCamera;
#ifdef DECISION_TREE_DETECTION
         trackLastWhiteBall_DecisionTree(frame, top? topFovea:botFovea, b);
#else
         trackLastWhiteBall(frame, top? topFovea:botFovea, b);
#endif
      }
   }

   // If neither previous ball worked, search whole image
   if (frame.balls.empty()) {
      // Try to find the ball in the bottom fovea
      populateRobotArrays(frame, false);
#ifdef DECISION_TREE_DETECTION
      findWhiteBalls_DecisionTree(frame, botFovea);
#else
      findWhiteBalls(frame, botFovea);
      updateBallHypotheses(frame);
#endif

      if (frame.balls.size() == 0) {
         // Try to find the ball in the top fovea
         populateRobotArrays(frame, true);
         findWhiteBallsTopFovea(frame, topFovea);
      }
   }

   // If previous ball didn't work, try the previous global ball
   // only if it's further than a metre away
   const int min_teamball_dist = 1000;
   const int other_end_of_field = 9000;
   if (frame.balls.empty() &&
         localisationBall.distance() > min_teamball_dist &&
         localisationBall.distance() < other_end_of_field) {
      float x = localisationBall.distance() * cos(localisationBall.heading());
      float y = localisationBall.distance() * sin(localisationBall.heading());

      Point lBall = frame.cameraToRR.convertToImageXY(Point(x, y));
      if (lBall.x() > 0 && lBall.x() < TOP_IMAGE_COLS &&
         lBall.y() > 0 && lBall.y() < TOP_IMAGE_ROWS) {
         BallInfo ball;
         ball.imageCoords = lBall;
#ifdef DECISION_TREE_DETECTION
         trackLastWhiteBall_DecisionTree(frame, topFovea, ball);
#else
         trackLastWhiteBall(frame, top? topFovea:botFovea, b);
#endif
      }
   }

   // Sort balls according to visionVar, whose meaning varies depending on detection
   std::sort(frame.balls.begin(), frame.balls.end(), BallInfoCmp());

#ifdef BALL_DEBUG_PRINTOUT
   if (frame.balls.size() > 0) {
      cout << "Ball: " << frame.balls[0].imageCoords.x() << "," << frame.balls[0].imageCoords.y() << " and " << frame.balls[0].radius << endl;
   }
#endif

   // Use the robot positions from localisation to filter out
   // false positives that we found on robots, since robot vision is patchy
   checkRobotOcclusion(frame);

   // Add hypotheses to ball history, update global hypotheses
   updateBallHeatMap(frame);

#else

   findBallsR(frame, botFovea, seed);
   if (frame.balls.size() == 0) {
      findBallsR(frame, topFovea, seed);
   }
#endif
}


void BallDetection::checkRobotOcclusion(
      VisionFrame &frame)
{
   // Here we check that no robot is occluding our view of the ball
   // If it is - there is a robot where the ball is or in the way of the ball
   // - this means that we *most probably* have a false positive on the robot.
   // Throw away the ball in this case, as NO FALSE POSITIVES is the highest priority

   // For each ball
   //    for each robot
   //       is it with 20cm of ball? (or something like that)
   //          chuck it
   //       does it have the same heading as the ball and is closer?
   //          chuck it
   std::vector<BallInfo>::iterator balls;
   for (balls = frame.balls.begin(); balls != frame.balls.end(); ) {
      Point ball = balls->rr.toCartesian();
      std::vector<RobotObstacle>::iterator bots;
      bool breakFlag = false;
      for (bots = localisationRobots.begin(); bots != localisationRobots.end(); ++bots) {
         Point bot = bots->rr.toCartesian();
         int sqDist = squaredDist(ball, bot);
         if (sqDist < 100*100 && balls->rr.distance() >= bots->rr.distance() - 50) {
            // 10cm and 5cm respectively

            balls = frame.balls.erase(balls);
            breakFlag = true;
            break;
         }

         // If the ball has a similar orientation to the robot,
         // and if they are close enough, then kill the ball
         if (fabs(bots->rr.orientation() - balls->rr.orientation()) <= 0.1) { // fix this boundary
            if (balls->rr.distance() > bots->rr.distance()) {

               // Throw the ball away
               balls = frame.balls.erase(balls);
               breakFlag = true;
               break;
            }
         }
      }

      if (!breakFlag)
         ++ balls;
   }

}


void BallDetection::updateBallHeatMap(
      VisionFrame &frame)
{
   bool clustered = false;

   std::vector<BallInfo>::iterator pastBalls;
   for (pastBalls = prevBalls.begin(); pastBalls != prevBalls.end(); ++pastBalls) {
      // For each past ball hypothesis, find any new hypotheses that cluster with it
      Point b1 = pastBalls->rr.toCartesian();

      std::vector<BallInfo>::iterator curBalls;
      for (curBalls = frame.balls.begin(); curBalls != frame.balls.end(); ) {

         // If this ball is a dud, remove it
         if (curBalls->imageCoords == Point(-1,-1)) {
            curBalls = frame.balls.erase(curBalls);
            continue;
         }

         // Ball is not a dud. Check if it clusters
         Point b2 = curBalls->rr.toCartesian();
         int squDist = squaredDist(b1, b2);
         if (squDist < A_FOOT) { // 15cm
            // If this new hypothesis is close enough, cluster and update
            pastBalls->imageCoords = curBalls->imageCoords;
            pastBalls->rr = curBalls->rr;
            pastBalls->radius = curBalls->radius;
            pastBalls->lastSeen = 0;
            pastBalls->neckRelative = curBalls->neckRelative;
            pastBalls->topCamera = curBalls->topCamera;
            clustered = true;

            curBalls = frame.balls.erase(curBalls);
         } else {
            // Ball didn't cluster. Increment number of frames dropped
            pastBalls->lastSeen ++;
            ++ curBalls;
         }
      }
      // If there were no current balls, increment framesDropped
      if (frame.balls.size() == 0)
         pastBalls->lastSeen ++;
   }

   // All remaining balls in frame.balls couldn't be clustered - add these to the list
   prevBalls.insert(prevBalls.end(), frame.balls.begin(), frame.balls.end());

   // Now go through and update all the past balls
   std::vector<BallInfo>::iterator curBalls;
   for (curBalls = prevBalls.begin(); curBalls != prevBalls.end(); ) {
      curBalls->lifetime ++;
      if (curBalls->lastSeen > MAX_LAST_SEEN) {
         curBalls = prevBalls.erase(curBalls);
      } else {
         ++ curBalls;
      }
   }

   // If we have no balls after this, then just quit and keep what we had
   if (prevBalls.empty()) return;

   // If we couldn't cluster our estimates earlier, then we have no good ball
   // Otherwise, choose the strongest ball
   frame.balls.clear();
   std::sort(prevBalls.begin(), prevBalls.end(), PastBallInfoCmp());
   if (clustered) {
      if (prevBalls[0].lifetime >= MIN_NEEDED_FRAMES)
         // We need to see a ball for 3 consecutive frames
         // for it to be worthwhile
         frame.balls.push_back(prevBalls[0]);
   } else if (prevBalls[0].lifetime > MIN_NEEDED_LIFETIME) {
      //frame.balls.push_back(prevBalls[0]);
   }
}



void BallDetection::populateRobotArrays(
      VisionFrame &frame,
      bool top)
{
   // Note that I tried to use memset for speed here,
   // but it didn't work for whatever reason
   if (top) {
      int width = TOP_IMAGE_COLS / MAX_DENSITY;
      for (int i = 0; i < width; ++i) {
         robotBottomsTopFrame[i] = 0;
      }
   } else {
      int width = BOT_IMAGE_COLS / MAX_DENSITY;
      for (int i = 0; i < width; ++i) {
         robotBottomsBotFrame[i] = 0;
      }
   }

   // First populate with the known robots
   // Extend each of these aboxes 3 out to each side, and 2 below,
   // since the boxes do not entirely cover robots
   std::vector<RobotInfo>::iterator i;
   for (i = robots.begin(); i != robots.end(); ++i) {
      if (top) {
         if (i->cameras == RobotInfo::TOP_CAMERA || i->cameras == RobotInfo::BOTH_CAMERAS) {
            int start = std::max(i->topImageCoords.a.x()/MAX_DENSITY, 0);
            int end = std::min(i->topImageCoords.b.x()/MAX_DENSITY, TOP_IMAGE_COLS);
            int error = i->cameras == RobotInfo::TOP_CAMERA? i->topImageCoords.height()/10 : 0;
            for (int k = start; k < end; ++k) {
               robotBottomsTopFrame[k] = i->topImageCoords.b.y() - error;
            }
         }
      } else {
         if (i->cameras == RobotInfo::BOT_CAMERA ||
               i->cameras == RobotInfo::BOTH_CAMERAS) {
            int start = std::max(i->botImageCoords.a.x()/MAX_DENSITY, 0);
            int end = std::min(i->botImageCoords.b.x()/MAX_DENSITY, BOT_IMAGE_COLS);
            int error = i->botImageCoords.height()/10;
            for (int k = start; k < end; ++k) {
               robotBottomsBotFrame[k] = i->botImageCoords.b.y() - topImageHeight - error;
            }
         }
      }
   }

   // Now populate with the possible robots - boxes around robot parts
   // Yeah, this loses us a few balls, but it cuts down on many a false positive,
   // which is more than worth while
   // Only change an edge if there isn't already a robot detected in it
   // Note that this is implemented without an iterator. Dunno why, but it makes it work
   if (top) {
      std::vector<PossibleRobot>::iterator i;
      for (unsigned int n = 0; n < topPossRobots.size(); ++n) {
         PossibleRobot r = topPossRobots[n];
         int start = r.region.a.x();
         int end = r.region.b.x();

         // Check the next PossibleRobot - if within this width of b.x
         // then extend this b to that a
         if (n < topPossRobots.size() - 1) {
            PossibleRobot j = topPossRobots[n+1];
            int dist = std::max(r.region.width(), j.region.width());
            if (j.region.a.x() - end <= dist) {
               end = j.region.a.x();
            }
         }

         for (int k = start; k < end; ++k) {
            robotBottomsTopFrame[k] = std::max(r.region.b.y()*MAX_DENSITY, robotBottomsTopFrame[k]);
            maximums.push_back(Point(k,r.region.b.y() + 1)*MAX_DENSITY);
         }
      }
   }

   // If top then it's the last run, so remove everything afterwards
   if (top) {
      // Clear robots
      robots.clear();
      topPossRobots.clear();
      botPossRobots.clear();
   }
}


void BallDetection::findWhiteBallsTopFovea(
      VisionFrame &frame,
      const Fovea &fovea)
{
   // The top fovea contains all the points from the field lines
   // as detected in FieldLineDetection. Create a new fovea and
   // remove these points before searching.

#ifndef DECISION_TREE_DETECTION
   // create a new fovea
   BBox box(Point(0,0), Point(fovea.bb.width(), fovea.bb.height()));
   boost::shared_ptr<FoveaT<hNone, eGrey> > ballFovea(
         new FoveaT<hNone, eGrey>(box, fovea.density, 0, fovea.top));
   ballFovea->actuateWithoutColour(frame);

   // For each line point we have, remove it from the fovea
   // And remove the 8 pixels around it
   // SPPED BOTTLENECK
   std::vector<Point>::iterator i;
   Point prev(0,0);
   for (i = lineEdges.begin(); i != lineEdges.end(); ++i) {
      Point p = fovea.mapImageToFovea(*i);

      int k1 = -1;
      int k2 = 1;
      int j1 = -1;
      int j2 = 1;

      prev = p;
      // Remove points around it
      for (int k = k1; k <= k2; ++k) {
         for (int j = j1; j <= j2; ++j) {
            Point q = fovea.mapImageToFovea(*i) + Point(k,j);
            if (fovea.bb.validIndex(q)) {
               ballFovea->zeroEdge(q.x(), q.y());
            }
         }
      }
   }

   Fovea f = ballFovea->asFovea();


   // Now search through the top fovea for balls
   // and weight them, etc etc
   findWhiteBalls(frame, f);
   updateBallHypotheses(frame);
#else
   findWhiteBalls_DecisionTree(frame, fovea);
#endif
}

// Weighting the hypotheses
void BallDetection::updateBallHypotheses(
      VisionFrame &frame)
{
   // Weight hypotheses according to their classifier scores
   std::vector<BallInfo>::iterator balls;
   for (balls = frame.balls.begin(); balls != frame.balls.end(); ) {

      // Throw away fake balls
      if (balls->imageCoords == Point(-1,-1)) {
         balls = frame.balls.erase(balls);
      } else {
         int newVar = balls->ccdRating;
         balls->visionVar = newVar;
#ifdef BALL_DEBUG_PRINTOUT
         cout << "\tCCD = " << balls->ccdRating << endl;
         cout << "Ball " << balls->imageCoords.x()<<","<<balls->imageCoords.y() << " has var " << balls->visionVar << " and radius " << balls->radius <<  endl;
#endif
         ++balls;
      }
   }

   // Sort by hypothesis variance in ascending order
   std::sort(frame.balls.begin(), frame.balls.end(), BallInfoCmp());

   // Remove all but the best 3 estimates
   if (frame.balls.size() > 3)
      frame.balls.resize(3);
}

// Search for white soccer balls in the given fovea
void BallDetection::findWhiteBalls(
      VisionFrame &frame,
      const Fovea &fovea)
{
   std::vector<Point> maxima = findSaliencyRegions(frame, fovea);
#ifdef BALL_DEBUG_POINTS
   maximums.insert(maximums.end(), maxima.begin(), maxima.end());
#endif
   std::vector<Cluster> clusters = clusterPointsCLC(frame, fovea, maxima);

   std::vector<Cluster>::iterator ci;
   for (ci = clusters.begin(); ci != clusters.end(); ++ ci) {
      Point p = ci->centre;
      double radius = BallDetection::getRadiusInImage(frame, p);
      radius = 1.5*radius;
      int error = (int)(0.05*radius);
      int bound = (int)radius + error;

      // Account for total image coordinates, create bounding box for fovea
      int height = fovea.top ? topImageHeight : botImageHeight;
      int width = fovea.top ? topImageWidth : botImageWidth;
      int density = getDensity(p.y());
      if (density < MIN_DENSITY || density > MAX_DENSITY) density = MAX_DENSITY/2;
      p.y() = fovea.top ? p.y() : p.y() - topImageHeight;
      Point tl(std::max(0, p.x()-bound), std::max(0, p.y()-bound));
      Point br(std::min(width, p.x()+bound), std::min(height, p.y()+bound));

      // Sanity checks on area and size of fovea
      if (br.x() <= tl.x() || br.y() <= tl.y()) continue;
      if (br.x() - tl.x() < 10 || br.y() - tl.y() < 10) continue;

      // If the fovea has any point past the endScan, then
      // we throw away the fovea
      // If the head is tilted down, avoid this step
      if (!isHeadTiltedForward(frame)) {
         bool skip = false;
         for (int x = tl.x(); x < br.x(); ++x) {
            int endScan;
            if (fovea.top) {
               endScan = frame.cameraToRR.topEndScanCoords[x];
            } else {
               endScan = frame.cameraToRR.botEndScanCoords[x];
            }
            if (endScan < br.y() && endScan > 0) {
               skip = true;
               break;
            }
         }
         if (skip) continue;
      }

      // If we're in the bottom fovea, use the current fovea and and the bounding box
      // if the top fovea, create a sub-fovea
      tl /= density;
      br /= density;
      BBox box(tl,br);
      boost::shared_ptr<FoveaT<hNone, eGrey> > ballFovea(
            new FoveaT<hNone, eGrey>(box, density, 0, fovea.top));
      // Search for darkest spot in the fovea here
      // Before we foveate, do a sanity check on the dark points in this region
      if (fovea.top) {
         box.a /= MAX_DENSITY/density;
         box.b /= MAX_DENSITY/density;
      }

#ifdef BALL_DEBUG_PRINTOUT
      cout << "Ball at: " << p.x() << "," << p.y() << endl;
#endif

      /* The Classifiers and Filters start here! */

      // Before foveating, run darkness filter
      int darkestPoint = 0;
      Point darkest = findDarkestPoint(frame, fovea, box, &darkestPoint);
#ifdef BALL_DEBUG_PRINTOUT
      cout << "\tpre-zoom Darkness: " << darkestPoint << endl;
#endif
      if (darkestPoint < minInitialDarkness) continue;


      // Foveate: zoom in and re-render at a higher res
      if (fovea.top) {
         // Only actuate the new fovea if we're in the top frame
         ballFovea->actuate(frame);
         // Set the box to represent sub-fovea coordinates
         box.a = tl;
         box.b = br;
         int width = box.width();
         int height = box.height();
         box.a = Point(0,0);
         box.b = Point(width,height);
#ifdef BALL_DEBUG_POINTS
         ballFoveas.push_back(ballFovea);
#endif
      }
      Fovea f = fovea.top? ballFovea->asFovea() : fovea;

      // Create a ball in the centre of the fovea
      BallInfo ball;
      p.y() = fovea.top ? p.y() : p.y() + topImageHeight;
      ball.imageCoords = p.cast<int>();
      ball.radius      = (int)getRadiusInImage(frame, p);
      ball.visionVar   = 1;
      ball.topCamera   = fovea.top;


      // Secondary darkness filter on higher resolution
      darkestPoint = 0;
      darkest = findDarkestPoint(frame, f, box, &darkestPoint);
      if (darkestPoint < minBallDarkness) continue;
      ball.darkestPoint = darkestPoint;
#ifdef BALL_DEBUG_PRINTOUT
      cout << "\tDarkness: " << ball.darkestPoint << endl;
#endif


      // HoG filter on high res, to remove lines
      float angleVar = hogClassifierInFovea(f, ball, box);
#ifdef BALL_DEBUG_PRINTOUT
      cout << "\tVague Var: " << angleVar << endl;
#endif
      if (angleVar <= 0) continue;
      ball.angleVar = angleVar;

      // CCD classifier to accurately determine ball position
      ball = ccdBallDetector(frame, f, box, ball);
#ifdef BALL_DEBUG_PRINTOUT
      cout << "\tCCD: "  << ball.ccdRating << endl;
#endif
      if (ball.ccdRating < minCCDRating) continue;


      // Final check - if the darkest point is too far from the ball
      // centre, then chances are it's not a ball. Kill it
      double r = 1.2*getRadiusInImage(frame, ball.imageCoords);
      if (squaredDist(ball.imageCoords, darkest) > r*r) {
#ifdef BALL_DEBUG_PRINTOUT
         cout << "\tDark spot too far away" << endl;
#endif
         continue;
      }

      // We got this far. Create a ball
      ball = createFieldBall(frame, fovea, ball);
      frame.balls.push_back(ball);
   }

   maxima.clear();
   clusters.clear();
}


void BallDetection::trackLastWhiteBall(
      VisionFrame &frame,
      const Fovea &fovea,
      BallInfo b)
{
#ifdef BALL_DEBUG_PRINTOUT
   cout << "Tracking" << endl;
#endif
   // Get a fovea around the area and apply our classifiers
   Point p = b.imageCoords;
   bool top = (p.y() < topImageHeight);

   // Sanity check - if p is in a robot, stop here
   if (fovea.top) {
      if (robotBottomsTopFrame[p.x()/MAX_DENSITY] > p.y()) return;
   } else {
      if (robotBottomsBotFrame[p.x()/MAX_DENSITY] + topImageHeight > p.y()) return;
   }

   double radius = 2*b.radius;
   int error = (int)(0.05*radius);
   int bound = (int)radius + error;

   // Account for total image coordinates, create bounding box for fovea
   int height = top ? topImageHeight : botImageHeight;
   int width = top ? topImageWidth : botImageWidth;
   int density = getDensity(p.y());
   if (density < MIN_DENSITY || density > MAX_DENSITY) density = MAX_DENSITY/2;
   p.y() = top ? p.y() : p.y() - topImageHeight;
   Point tl(std::max(0, p.x()-bound), std::max(0, p.y()-bound));
   Point br(std::min(width, p.x()+bound), std::min(height, p.y()+bound));

   // Sanity checks on area and size of fovea
   if (br.x() <= tl.x() || br.y() <= tl.y()) return;
   if (br.x() - tl.x() < 10 || br.y() - tl.y() < 10) return;

   // If the centre of the fovea is within a robot, throw this away
   Point centre = b.imageCoords;

   // If it's the bottom fovea, don't create a sub-fovea
   tl /= density;
   br /= density;
   BBox box(tl,br);
   boost::shared_ptr<FoveaT<hNone, eGrey> > ballFovea(
         new FoveaT<hNone, eGrey>(box, density, 0, fovea.top));
   if (fovea.top) {
      // Only create the new fovea if we're in the top frame
      ballFovea->actuate(frame);
      int width = box.width();
      int height = box.height();
      box.a = Point(0,0);
      box.b = Point(width,height);
#ifdef BALL_DEBUG_POINTS
      ballFoveas.push_back(ballFovea);
#endif
   }
   Fovea f = fovea.top? ballFovea->asFovea() : fovea;

   // Create a ball in the centre of the fovea
   BallInfo newBall;
   p.y() = f.top ? p.y() : p.y() + topImageHeight;
   newBall.imageCoords = p.cast<int>();
   newBall.radius      = (int)getRadiusInImage(frame, p);
   newBall.visionVar   = 1;
   newBall.topCamera   = f.top;

   // Run each of the classifiers over the balls
   // and perform the sanity checks at each stage
   // Reset global darkest point
   // Since darkness is a comparison against the rest of the image,
   // and we aren't checking that, it doesn't make sense to have it here
   newBall.darkestPoint = 100; // % of darkness in image

   // HoG filter
   float angleVar = hogClassifierInFovea(f, newBall, box);
   if (angleVar <= 0) return;
   newBall.angleVar = angleVar;

   // CCD Classifier with easier bounds, since we are tracking
   newBall = ccdBallDetector(frame, f, box, newBall);
   if (newBall.ccdRating < minCCDRating/2) return;

   // We got this far. Create a ball
   newBall = createFieldBall(frame, f, newBall);
   frame.balls.push_back(newBall);
   updateBallHypotheses(frame);
}

// Search for white soccer balls in the given fovea
void BallDetection::findWhiteBalls_DecisionTree(
      VisionFrame &frame,
      const Fovea &fovea)
{
   std::vector<Point> maxima = findStrongEdges(frame, fovea);
#ifdef BALL_DEBUG_POINTS
   maximums.insert(maximums.end(), maxima.begin(), maxima.end());
#endif
   std::vector<Cluster> clusters = clusterPointsCLC(frame, fovea, maxima);

   std::vector<Cluster>::iterator ci;
   for (ci = clusters.begin(); ci != clusters.end(); ++ ci) {

      Point p = ci->centre;
      double radius = (ci->br.x() - ci->tl.x())/2;
      double sanity = BallDetection::getRadiusInImage(frame, p);
if (radius < 0.7*(sanity) || radius > sanity*3) radius = sanity;
      //radius = sanity;

#ifdef BALL_DEBUG_PRINTOUT
      cout << "Radius is " << radius << endl;
#endif
      int bound = (int)(1.55*radius);

      if (radius <= MIN_ACCEPTABLE_RADIUS) continue;

      // Account for total image coordinates, create bounding box for fovea
      int height = fovea.top ? topImageHeight-1 : topImageHeight+botImageHeight-1;
      int minHeight = fovea.top? 0 : topImageHeight;
      int width = fovea.top ? topImageWidth-1 : botImageWidth-1;
      Point tl(std::max(0, p.x()-bound), std::max(minHeight, p.y()-bound));
      Point br(std::min(width, p.x()+bound), std::min(height, p.y()+bound));

      // Sanity checks on area and size of fovea
      if (br.x() <= tl.x() || br.y() <= tl.y()) continue;
      if (br.x() - tl.x() < 10 || br.y() - tl.y() < 10) continue;

      BBox bb(tl, br);

#ifdef BALL_DEBUG_PRINTOUT
      cout << "Ball at: " << p.x() << "," << p.y() << endl;
#endif

      /* The Classifiers and Filters start here! */

      // Before foveating, run darkness filter
      int darkestPoint = 0;
      Point darkest = findDarkestPoint(frame, fovea, bb, &darkestPoint);
#ifdef BALL_DEBUG_PRINTOUT
      cout << "\tpre-zoom Darkness: " << darkestPoint << endl;
#endif
      if (darkestPoint < minInitialDarkness) {
         continue;
      }

      // Create a ball in the centre of the fovea
      BallInfo ball;
      ball.imageCoords = p.cast<int>();
      ball.radius      = (int)radius;
      ball.visionVar   = 1;
      ball.topCamera   = fovea.top;

      // Sanity check globalDarkest
      if (globalDarkest == 0) {
         globalDarkest = MIN_LINE_DARKNESS;
      }

      timer.restart();
      ball = ccdNativeRes(frame, fovea, fovea.top, ball);
llog(VERBOSE) << "Native res CCD took " << timer.elapsed_us() << endl;


#ifdef BALL_DEBUG_PRINTOUT
      cout << "\tCCD: "  << ball.ccdRating << endl;
#endif

      // If the new centre of the ball is off the frame, reject this ball
      Point centre = ball.imageCoords;
      if (centre.x() <= 0 || centre.x() >= width) {
         continue;
      } else if (centre.y() <= minHeight || centre.y() >= height) {
         continue;
      }

      bool decision = runDecisionTree(frame, fovea, false, ball);
#ifdef BALL_DEBUG_PRINTOUT
      cout << "\tTree result: " << decision << endl;
#endif
      if (!decision) continue;

      // We got this far. Create a ball
      ball = createFieldBall(frame, fovea, ball);
      frame.balls.push_back(ball);
   }

   maxima.clear();
   clusters.clear();
}


void BallDetection::trackLastWhiteBall_DecisionTree(
      VisionFrame &frame,
      const Fovea &fovea,
      BallInfo b)
{
#ifdef BALL_DEBUG_PRINTOUT
   cout << "Tracking" << endl;
#endif
   // Get a fovea around the area and apply our classifiers
   Point p = b.imageCoords;

   // Get the real-world position of this ball, then add the velocity to it
   Point ballRR = frame.cameraToRR.pose.imageToRobotXY(p, BALL_RADIUS);
   // Velocity is in mm/s, so we scale it down by 30 cos at 30fps, and x mm/s,
   // we expect it to move by x/30 mm/frame
   ballRR += ballVelRR.toCartesian()/EXPECTED_FRAME_RATE;
   Point ballInImage = frame.cameraToRR.pose.robotToImageXY(ballRR, BALL_RADIUS);
   // NOTE: this may have the same kinematics error as the ball radius computations

   // Sanity check - if p is in a robot, stop here
   if (fovea.top) {
      if (robotBottomsTopFrame[p.x()/MAX_DENSITY] > p.y()) return;
   } else {
      if (robotBottomsBotFrame[p.x()/MAX_DENSITY] + topImageHeight > p.y()) return;
   }

   // Create a ball in the centre of the fovea
   BallInfo newBall;
   newBall.imageCoords = ballInImage.cast<int>(); // p
   newBall.radius      = (int)getRadiusInImage(frame, ballInImage);
   newBall.visionVar   = 1;
   newBall.topCamera   = fovea.top;

   if (newBall.radius <= 0) return;

   // Sanity check globalDarkest
   if (globalDarkest == 0) {
      globalDarkest = MIN_LINE_DARKNESS;
   }

   // CCD Classifier with easier bounds, since we are tracking
   newBall = ccdNativeRes(frame, fovea, fovea.top, newBall);

   bool decision = runDecisionTree(frame, fovea, true, newBall);
   if (!decision) return;

   // We got this far. Create a ball
   newBall = createFieldBall(frame, fovea, newBall);
   frame.balls.push_back(newBall);
}



/*
 * Collect all strong edges in the fovea
 * to find candidate regions for the ball
 */
std::vector<Point> BallDetection::findStrongEdges(
      VisionFrame &frame,
      const Fovea &fovea)
{
   int width = fovea.bb.width();
   int height = fovea.bb.height();
   std::vector<Point> edges;

   globalDarkest = maxDarkness;

   for (int x = 0; x < width; ++x) {

      int yStart = 0;
      // If top fovea, check for field edges
      if (fovea.top) {
         Point start = getFieldEdgePoint(frame, fovea, Point(x, yStart));
         yStart = start.y() + FIELD_EDGE_BALL_OFFSET;
         // Check for robot exclusion zone
         yStart = std::max(robotBottomsTopFrame[x]/MAX_DENSITY, yStart);
      } else {
         // Bottom frame - just check for robot exclusion zone
         yStart = std::max(robotBottomsBotFrame[x]/MAX_DENSITY, yStart);
      }

      // If bottom fovea, check for exclusion zone on ourself
      int yEnd = height;
      if (!fovea.top) {
         int worldX = x*fovea.density;
         int endScan = (fovea.top? frame.cameraToRR.topEndScanCoords[worldX] : frame.cameraToRR.botEndScanCoords[worldX]);
         yEnd = std::min(endScan/fovea.density, height);
      }

      int max = 0;
      for (int y = yStart; y < yEnd; ++y) {

         // Collect edges that are local maxima in the y-direction
         int mag = fovea.magnitude(x,y);
         if (mag > STRONG_EDGE_THRESHOLD && mag > max) {
            max = mag;
         } else if (max > 0) {
            max = 0;
            edges.push_back(fovea.mapFoveaToImage(Point(x,y-1)));
         }

         // Find the darkest edge in the image
         int yVal = getYValue(frame, fovea.top, fovea.mapFoveaToImage(Point(x,y)));
         if (yVal < globalDarkest) {
            globalDarkest = yVal;
         }
      }
   }

   // Reverse globalDarkest so that we can maximise
   globalDarkest = maxDarkness - globalDarkest;

   return edges;
}


/*
 * Perform Complete-Linkage Clustering on
 * the strong edges to get clusters to sub-foveate on
 */
std::vector<Cluster> BallDetection::clusterPointsCLC(
      VisionFrame &frame,
      const Fovea &fovea,
      std::vector<Point> points)
{
   // For each point, combine it with the point closest to it
   // that is within a diameter of it
   // Repeat for a while until this happens no more
   // Find clusters amongst the points

   std::vector<Cluster> preClusters = std::vector<Cluster>();
   for (unsigned int i = 0; i < points.size(); ++i) {
      Cluster c;
      c.addPoint(points[i]);
      preClusters.push_back(c);
   }

   std::vector<Cluster> clusters = std::vector<Cluster>();
   Cluster c;
   std::vector<Cluster>::iterator index;
   std::vector<Cluster> goodPoints;
   while (preClusters.size() > 0) {
      c = preClusters[0];
      preClusters.erase(preClusters.begin());

      // Get the closest point within the diameter
      int bound = 2*BallDetection::getRadiusInImage(frame, c.centre);
      int minDist = topImageWidth*topImageWidth;
      std::vector<Cluster>::iterator minPoint;
      std::vector<Cluster>::iterator it;
      for (it = preClusters.begin(); it != preClusters.end(); ++it) {
         int d = squaredDist(it->centre,c.centre);
         // find closest points
         if (d <= bound*bound) {
            if (d < minDist) {
               minDist = d;
               minPoint = it;
            }
         }
      }

      // Combine the two points
      if (minDist < topImageWidth*topImageWidth) {
         c.addPoint(minPoint->centre);
         c.addPoint(minPoint->tl);
         c.addPoint(minPoint->br);
         c.centre = (c.centre + minPoint->centre)/2;
         minPoint = preClusters.erase(minPoint);
         preClusters.push_back(c);
      } else {
         // We've combined this point all we can.
         // Add it to the list of finished points
         goodPoints.push_back(c);
      }
   }

   Point p(0,0);
   for (index = goodPoints.begin(); index != goodPoints.end(); ++index) {

      // Check for dupes with previous point
      if (index->centre == p) continue;

      // No change to this point. Create a cluster
      Cluster cl;
      cl.addPoint(index->centre);
      cl.addPoint(index->tl);
      cl.addPoint(index->br);
      cl.centre = index->centre;
      clusters.push_back(cl);
#ifdef BALL_DEBUG_POINTS
      clusterCentres.push_back(cl.centre);
#endif
      p = cl.centre;
   }
   goodPoints.clear();

   return clusters;
}

/*
 * This works by creating a histogram of strong edges
 * over each dimension, and searching for maxima.
 * This will generate a lot of FPs.
 * Also, we may need to downsample to reduce sparseness
 */
std::vector<Point> BallDetection::findSaliencyRegions(
      VisionFrame &frame,
      const Fovea &fovea)
{
   int width = fovea.bb.width();
   int height = fovea.bb.height();
   int xHist[width/2];
   int yHist[height/2];
   int x,y;
   std::vector<Point> xyMaxima;

   // reset global darkest point
   globalDarkest = maxBallDarkness;

   int minX = 0;
   int minY = 0;

   // Init y array:
   for (y = 0; y < height/2; ++y) yHist[y] = 0;

   for (x = 0; x < width; ++x) {

      int yStart = 0;
      // If top fovea, check for field edges
      if (fovea.top) {
         Point start = getFieldEdgePoint(frame, fovea, Point(x, yStart));
         yStart = start.y();
         // Check for robot exclusion zone
         yStart = std::max(robotBottomsTopFrame[x]/MAX_DENSITY, yStart);
         if (yStart == robotBottomsTopFrame[x]/MAX_DENSITY) {
         }
      } else {
         yStart = std::max(robotBottomsBotFrame[x]/MAX_DENSITY, yStart);
      }

      // If bottom fovea, check for exclusion zone
      int yEnd = height;
      if (!fovea.top) {
         int worldX = x*fovea.density;
         int endScan = (fovea.top? frame.cameraToRR.topEndScanCoords[worldX] : frame.cameraToRR.botEndScanCoords[worldX]);
         yEnd = std::min(endScan/fovea.density, height);
      }

      for (y = yStart; y < yEnd; ++y) {

         // Accept all edges strong enough
         if (fovea.magnitude(x,y) > STRONG_EDGE_THRESHOLD) {
            yHist[y/2] ++;
         }

         // Find the darkest point in the image
         if (fovea.grey(x,y) < globalDarkest) {
            globalDarkest = fovea.grey(x,y);
            minX = x;
            minY = y;
         }

      }
   }

   // Invert the globalMinDelta to be a maximum
   globalDarkest = maxBallDarkness - globalDarkest;
   Point q = fovea.mapFoveaToImage(Point(minX,minY));

   // Find local maxima in the histogram array
   std::vector<Point> maxima;
   int max = 0;
   for (y = 0; y < height/2; ++y) {
      if (yHist[y] > max) {
         max = yHist[y];
      } else if (max > 0) {
         maxima.push_back(Point(0, y*2));
         max = 0;
      }
   }

   // In regions around these maxima
   // populate the x histogram and search for x maxima
   for (unsigned int i = 0; i < maxima.size(); ++i) {
      int yTop = maxima[i].y();
      int y = fovea.density*yTop + (fovea.top? 0 : topImageHeight);
      int r = getRadiusInImage(frame, Point(0,y))/fovea.density;
      yTop -= r;
      int yBot = yTop + 2*r;


      int max = 0;
      for (x = 0; x < width; ++x) {

         // Reset y1 and y2 to the above each iteration
         int y1 = yTop;
         int y2 = yBot;

         xHist[x/2] = 0;
         // if top fovea, check field edges
         if (fovea.top) {
            Point start = getFieldEdgePoint(frame, fovea, Point(x,y1));
            if (y2 < start.y()) continue;
            y1 = std::max(y1, start.y());

            // Check for robot exclusion zone
            y1 = std::max(y1, robotBottomsTopFrame[x]/MAX_DENSITY);
            if (y1 == robotBottomsTopFrame[x]/MAX_DENSITY) continue;
         } else {
            y1 = std::max(y1, robotBottomsBotFrame[x]/MAX_DENSITY);
            if (y1 < robotBottomsBotFrame[x]/MAX_DENSITY) continue;
         }

         // If bottom fovea, check for own body exclusion zone
         if (!fovea.top) {
            int worldX = x*fovea.density;
            int endScan = (fovea.top? frame.cameraToRR.topEndScanCoords[worldX] : frame.cameraToRR.botEndScanCoords[worldX]) - fovea.density*5;
            y2 = std::min(y2, endScan/fovea.density);
         }
         y2 = std::min(height, y2);

         for (y = y1; y < y2; ++y) {
            if (fovea.magnitude(x,y) > STRONG_EDGE_THRESHOLD) {
               xHist[x/2] ++;
            }

            if (fovea.magnitude(x,y) > MED_EDGE_THRESHOLD) {
               // Add sufficiently dark points immediately as maxima
               int grey = maxBallDarkness - fovea.grey(x,y);
               grey = (grey*100)/globalDarkest;
               if (grey >= minInitialDarkness) {
                  xyMaxima.push_back(fovea.mapFoveaToImage(Point(x,y)));
               }
            }
         }
      }

      // Search for local maxima in the x histogram. Collect these points
      max = 0;
      for (x = 0; x < width/2; ++x) {
         if (xHist[x] > max) {
            max = xHist[x];
         } else if (max > 0) {
            max = 0;
            maxima[i].x() = x*2;
            Point p = fovea.mapFoveaToImage(maxima[i]);
            xyMaxima.push_back(p);
         }
      }
   }

   return xyMaxima;
}


/*
 * Search over a sub-fovea for the darkest point in the greyscale image
 * Return the image coordinates of this point, and save the value in the param
 */
Point BallDetection::findDarkestPoint(
      VisionFrame &frame,
      const Fovea &fovea,
      BBox box,
      int * darkestVal)
{
   int x,y;
   int min = maxDarkness;
   int minX = 0;
   int minY = 0;

   for (x = box.a.x(); x < box.b.x(); ++x) {
      for (y = box.a.y(); y < box.b.y(); ++y) {
         // Find the darkest point in the image

         int yVal = getYValue(frame, fovea.top, Point(x,y));

         if (yVal < min) {
            min = yVal;//fovea.grey(x,y);
            minX = x;
            minY = y;
         }
      }
   }

   // Invert the minimum to be a maximum
   min = maxDarkness - min;

   // If we don't have a global minimum for whatever reason,
   // set it to the local min
   if (globalDarkest == 0 || globalDarkest == maxDarkness)
      globalDarkest = min;

   // Get this value as a % of the darkest point in the image
   *darkestVal = (100*min)/globalDarkest;

   return fovea.mapFoveaToImage(Point(minX, minY));
}

float BallDetection::hogClassifierInFovea(
      const Fovea &fovea,
      BallInfo ball,
      BBox box)
{
   // Create a histogram of the orientations of edges
   // We now do this just within the ball, instead of the entire fovea
   // Compare this to a benchline for the ball
   // We use 4 buckets for angles

   Point centre = ball.imageCoords;
   int angleCounts[NUM_ANGLE_BINS] = {0, 0, 0, 0};
   for (int x = box.a.x(); x < box.b.x(); ++x) {
      for (int y = box.a.y(); y < box.b.y(); ++y) {

         // Only consider strong enough edge gradients
         if (fovea.magnitude(x,y) < STRONG_EDGE_THRESHOLD) continue;

         Point edge = fovea.edge(x, y);
         float theta = atan2(edge.y(), edge.x());

         // Assign edges to orientation buckets
         theta += 2*halfPI;
         theta *= (360/halfPI);
         int phi = ((int)theta % 360) / 90;
         angleCounts[phi] ++;
      }
   }

   // Normalise the values to the lowest resolution
   // and compute the variance across the bins
   int avg = 0;
   int avgSqu = 0;
   int minBinVal = LARGE_NUMBER;
   int minAllowedVal = NUM_ANGLE_BINS;
   for (int i = 0; i < NUM_ANGLE_BINS; ++i) {
      // Minimum required for each bin
      if (angleCounts[i] < minAllowedVal) {
         // If failure, set values to ensure var == -1
         minBinVal = 0;
         avg = LARGE_NUMBER;
         break;
      }

      angleCounts[i] *= fovea.density;

      if (angleCounts[i] < minBinVal) {
         minBinVal = angleCounts[i];
      }

      avg += angleCounts[i];
      avgSqu += angleCounts[i]*angleCounts[i];
   }
   // Sanity check - if the smallest bin is < 30% of the average,
   // then this clearly isn't a ball. Scrap it
   float var = UNDEFINED;
   avg /= NUM_ANGLE_BINS;

   if (minBinVal < avg/3 ) {
      var = UNDEFINED;
   } else {
      avgSqu /= NUM_ANGLE_BINS;
      var = avgSqu - avg*avg;
      var /= 1000.0;
   }

   return var;
}

// Helper function for the CCD
// We change what each colour is rated here
int colourValue(Colour c, bool allowed) {
   int retVal = 0;
   int multiplier = allowed? 1:-1;

   // do we want to consider white and black more strongly?
   // if so, change these
   int white_multiplier = 2;
   int black_multiplier = 4;

   switch(c) {
      case cWHITE: retVal = white_multiplier*COLOUR_VAL;
                   break;
      case cBLACK: retVal = black_multiplier*COLOUR_VAL;
                   break;
      case cBACKGROUND: retVal = COLOUR_VAL;
                        break;
      // background colour gets no special consideration
      default: retVal = -1*COLOUR_VAL;
   }

   return retVal*multiplier;
}

/*
 * This uses the Contracting Curve Density (CCD) algorithm
 * or an approximation of it to fit a curve to the ball.
 * We perform this for several iterations, and get a measure
 * of how well it converges.
 * The measure is the balance of inside points and outside points
 * around the circle
 * We use 16 points around the circle edge to test. Maybe this is
 * too much, maybe too little. Experiments can decide.
 * We also start off with a region size of just the 3x3 aorund the point
 * and pick certain ones for different points
 */
BallInfo BallDetection::ccdBallDetector(
      VisionFrame &frame,
      const Fovea &fovea,
      BBox box,
      BallInfo ball)
{
   Point c = Point((box.a.x()+box.b.x())/2, (box.b.y() + box.a.y())/2);
   float r = getRadiusInImage(frame, fovea.mapFoveaToImage(c))/fovea.density - 1;
   // Compute the number of points we will need around each annulus
   // Note that the annulus has a thickness of 1, and for lower
   // densities I also want a thicker annulus
   float outR = r + (fovea.density == MAX_DENSITY? 1:2);
   float inR = r - (fovea.density == MAX_DENSITY? 1:2);
   int numOuterPoints = 4*outR;
   int numInnerPoints = 4*inR;

   // Arrays for caching the trig values to avoid repeat calculations
   std::vector<float> outerCos(numOuterPoints, 2*UNDEFINED);
   std::vector<float> outerSin(numOuterPoints, 2*UNDEFINED);
   std::vector<float> innerCos(numInnerPoints, 2*UNDEFINED);
   std::vector<float> innerSin(numInnerPoints, 2*UNDEFINED);

   // Rating the final iteration of the CCD
   int validCCDPoints = 0;
   int countedPoints = 0;
   for (int i = 0; i < NUM_CCD_ITERATIONS; ++i) {

      // The cumulative convergence vector
      PointF vector(0.0, 0.0);

      // Update the radius each iteration. Have it smaller than what we need
      r = getRadiusInImage(frame, fovea.mapFoveaToImage(c))/fovea.density - 1;

      // reset scores
      validCCDPoints = 0;
      countedPoints = 0;

      // Compute outer annulus
      outR = r + 2;
      for (int j = 0; j < numOuterPoints; ++j) {
         if (outerCos[j] < UNDEFINED) {
            float theta = ((float)j) * 4*halfPI / (float)numOuterPoints;
            outerCos[j] = cos(theta);
            outerSin[j] = sin(theta);
         }
         float cosVal = outerCos[j];
         float sinVal = outerSin[j];
         int x = outR * cosVal + c.x();
         int y = outR * sinVal + c.y();
         Point p(x,y);

         // Sanity check that the coords are within the bounding box
         if (!box.validIndex(p - box.a)) continue;

         // Grab the colour value
         Colour colour = fovea.colour(p);

         // Colours outside that are bad:
         // white
         // background and black IF we're not at the bottom quarter of the circle
         // this is allowed due to the shadow
         bool bottom = false;
         if (j > 3*numOuterPoints/8 && j < 6*numOuterPoints/8) bottom = true;
         bool badColour = !bottom && (colour == cBACKGROUND || colour == cBLACK);
         badColour = badColour || colour == cWHITE;
         if (badColour) {
            // BAD. Move towards colour to get it inside
            vector += PointF(cosVal, sinVal);
            if (colour == cWHITE) validCCDPoints -= 2;
         } else {
            // Colour is something that should be outside the ball.
            // This is good, so no motion
            validCCDPoints ++;
         }

         countedPoints ++;
      }

      // Compute inner annulus
      inR = r - (fovea.density == MAX_DENSITY? 2:3);
      for (int j = 0; j < numInnerPoints; ++j) {
         if (innerCos[j] < UNDEFINED) {
            float theta = ((float)j) * 4*halfPI / (float)numInnerPoints;
            innerCos[j] = cos(theta);
            innerSin[j] = sin(theta);
         }
         float cosVal = innerCos[j];
         float sinVal = innerSin[j];
         int x = inR * cosVal + c.x();
         int y = inR * sinVal + c.y();
         Point p(x,y);

         // Sanity check that the coords are within the bounding box
         if (!box.validIndex(p - box.a)) continue;

         // Grab the colour value
         Colour colour = fovea.colour(p);

         // Colours allowed inside: black, baackground, white. Always. Nothing else.
         int value = colourValue(colour, true);
         validCCDPoints += value;
         vector += value > 0? PointF(0, 0) : PointF(-cosVal, -sinVal);

         countedPoints ++;
      }

      // If at any stage the score is sufficient, stop
      countedPoints = std::max(countedPoints, 1); // Ensure no div by zero
      int score = (validCCDPoints*100)/countedPoints;
      if (score > minCCDRating) {
#ifdef BALL_DEBUG_PRINTOUT
         cout << "\tNum iterations: " << i << " with score " << score << endl;
#endif
         break;
      }

      // scale vector down to avoid overshoot
      vector /= 4;

      // Update the ball centre
      c += Point((int)vector.x(), (int)vector.y());
   }

   // Clear the vectors
   outerCos.clear();
   outerSin.clear();
   innerCos.clear();
   innerSin.clear();

   // Normalise the number of good points
   ball.ccdRating = (validCCDPoints*100) / (countedPoints);

   // Perform a colour scan inside the ball as a sanity check
   // Check the green to white ratio is right
   // If too much other colour, chuck it
   int goodInside = 0;
   int greenInside = 0;
   int otherInside = 0;
   for (int x = c.x() - r; x < c.x() + r; ++x) {
      for (int y = c.y() - r; y < c.y() + r; ++y) {
         // don't check point if it isn't in fovea
         if (!box.validIndex(Point(x,y) - box.a)) continue;

         // don't check point if it isn't in the circle
         if (squaredDist(Point(x,y), c) > r*r) continue;

         Colour colour = fovea.colour(x,y);
         if (colour == cWHITE || colour == cBACKGROUND)
            goodInside ++;
         else if (colour == cBLACK)
            goodInside += 2;
         else if (colour == cFIELD_GREEN)
            greenInside ++;
         else
            otherInside ++;
      }
   }

   // Check ratios
   // Ratios derived by SCIENCE
   // if InsideColour / Green ~= 2 - 8, then good. Also, if other > 100, BAD
   // TODO: fix this whole ratio business to just be a sanity check
   int ratio = 0; // What if there isn't green inside? Hmmmm.
   if (greenInside != 0) {
      ratio = goodInside / greenInside;
   }
   if (otherInside > hardInsideColourBound) {
      // Set ratio to 0 to signal dud ball
      ratio = 0;
   }
   // TODO: have the sanity check here instead of outside the function
   if (ratio < hardMinColourRatio ||
         ratio > maxColourRatio) {
      ball.ccdRating = 0;
   }

   // Otherwise, the ratio is used as a rating
   // Move the ball to the new position
#ifdef BALL_DEBUG_POINTS
   ccdCentres.push_back(fovea.mapFoveaToImage(c));
#endif
   ball.imageCoords = fovea.mapFoveaToImage(c);

   return ball;
}


int BallDetection::getYValue(VisionFrame &frame, bool top, Point p) {

   int x = p.x();
   int y = p.y();

   if (!top) y -= topImageHeight;

   int cols = top? TOP_IMAGE_COLS : BOT_IMAGE_COLS;

   // future: the considered value is the median of the 4 around it
   int yVal = 0;
   if (top) {
      yVal = gety(frame.topImage, y, x, cols);
   } else {
      yVal = gety(frame.botImage, y, x, cols);
   }

   return yVal;
}

int BallDetection::getUValue(VisionFrame &frame, bool top, Point p) {

   int x = p.x();
   int y = p.y();

   if (!top) y -= topImageHeight;

   int cols = top? TOP_IMAGE_COLS : BOT_IMAGE_COLS;

   // future: the considered value is the median of the 4 around it
   int uVal = 0;
   if (top) {
      uVal = getu(frame.topImage, y, x, cols);
   } else {
      uVal = getu(frame.botImage, y, x, cols);
   }

   return uVal;
}

int BallDetection::getVValue(VisionFrame &frame, bool top, Point p) {

   int x = p.x();
   int y = p.y();

   if (!top) y -= topImageHeight;

   int cols = top? TOP_IMAGE_COLS : BOT_IMAGE_COLS;

   // future: the considered value is the median of the 4 around it
   int vVal = 0;
   if (top) {
      vVal = getv(frame.topImage, y, x, cols);
   } else {
      vVal = getv(frame.botImage, y, x, cols);
   }

   return vVal;
}

/*
 * NATIVE RES VERSION WITH ON-THE-FLY COLOUR CLASSIFICATION
 * This uses the Contracting Curve Density (CCD) algorithm
 * or an approximation of it to fit a curve to the ball.
 * We perform this for several iterations, and get a measure
 * of how well it converges.
 * The measure is the balance of inside points and outside points
 * around the circle
 * We use 16 points around the circle edge to test. Maybe this is
 * too much, maybe too little. Experiments can decide.
 * We also start off with a region size of just the 3x3 aorund the point
 * and pick certain ones for different points
 */
BallInfo BallDetection::ccdNativeRes(
      VisionFrame &frame,
      const Fovea &fovea,
      bool top,
      BallInfo ball)
{
   Point c = ball.imageCoords;
   float r = ball.radius - 1;

   darkSpots.push_back(c);

   // Create another bbox of the whole frame to do sanity checking on coords
   BBox image;
   if (top) {
      image = BBox(Point(0,0), Point(topImageWidth-1, topImageHeight-1));
   } else {
      image = BBox(Point(0, topImageHeight), Point(botImageWidth-1, topImageHeight + botImageHeight-1));
   }

   // Compute the number of points we will need around each annulus
   // Note that the annulus has a thickness of 1
   float outR = r + 4;
   float inR = r - 4;
   int numOuterPoints = 4*outR;
   int numInnerPoints = 4*inR;

   // Arrays for caching the trig values to avoid repeat calculations
   float * outerCos = (float *) malloc(numOuterPoints*sizeof(float));
   float * outerSin = (float *) malloc(numOuterPoints*sizeof(float));
   float * innerCos = (float *) malloc(numInnerPoints*sizeof(float));
   float * innerSin = (float *) malloc(numInnerPoints*sizeof(float));
   // Clear the arrays initially
   for (int n = 0; n < numOuterPoints; ++n) {
      outerCos[n] = 2*UNDEFINED;
      outerSin[n] = 2*UNDEFINED;
   }
   for (int n = 0; n < numInnerPoints; ++n) {
      innerCos[n] = 2*UNDEFINED;
      innerSin[n] = 2*UNDEFINED;
   }



   // Rating the final iteration of the CCD
   int validCCDPoints = 0;
   int countedPoints = 0;
   for (int i = 0; i < NUM_CCD_ITERATIONS; ++i) {

      // The cumulative convergence vector
      PointF vector(0.0, 0.0);

      // reset scores
      validCCDPoints = 0;
      countedPoints = 0;

      // Compute outer annulus
      outR = r + 4;//2;
      for (int j = 0; j < numOuterPoints; ++j) {
         if (outerCos[j] < UNDEFINED) {
            float theta = ((float)j) * 4*halfPI / (float)numOuterPoints;
            outerCos[j] = cos(theta);
            outerSin[j] = sin(theta);
         }
         float cosVal = outerCos[j];
         float sinVal = outerSin[j];
         int x = outR * cosVal + c.x();
         int y = outR * sinVal + c.y();
         Point p(x,y);

         // Sanity check that the coords are within the image
         if (!image.within(p)) continue;

         bool topHalf = false;
         if (sinVal < 0) {
            topHalf = true;
         }

         int yVal = getYValue(frame, top, p);
         int uVal = top? getUValue(frame, top, p) : 255;
         int vVal = top? getVValue(frame, top, p) : 255;
         bool badColour = (yVal >= 130 && uVal >= 80 && vVal >= 80); // y >= WHITE_Y

         if (badColour) {
            // BAD. Move towards colour to get it inside
            vector += PointF(cosVal, sinVal);
            validCCDPoints --;
         } else {
            // Colour is something that should be outside the ball.
            // This is good, so no motion
            validCCDPoints ++;
         }

         countedPoints ++;
      }

      // Compute inner annulus
      inR = r - 4;//2;
      // int j = 0 to start
      for (int j = numInnerPoints/2; j < numInnerPoints; ++j) {
         if (innerCos[j] < UNDEFINED) {
            float theta = ((float)j) * 4*halfPI / (float)numInnerPoints;
            innerCos[j] = cos(theta);
            innerSin[j] = sin(theta);
         }
         float cosVal = innerCos[j];
         float sinVal = innerSin[j];
         int x = inR * cosVal + c.x();
         int y = inR * sinVal + c.y();
         Point p(x,y);

         // Sanity check that the coords are within the bounding box
         if (!image.within(p)) continue;

         // Colours allowed inside: black, baackground, white. Always. Nothing else.
         int value;

         bool topHalf = false;
         if (sinVal < 0) {
            topHalf = true;
         }

         if (topHalf) {

            int yVal = getYValue(frame, top, p);
            //value = (val < WHITE_Y)? -1:1; // val < WHITE_Y
            int uVal = top? getUValue(frame, top, p) : 255;
            int vVal = top? getVValue(frame, top, p) : 255;
            value = (yVal < 130 || uVal < 80 || vVal < 80)? -1:1;
            validCCDPoints += value;
            vector += value > 0? PointF(0, 0) : PointF(-cosVal, -sinVal);


            countedPoints ++;
         }
      }

      // If at any stage the score is sufficient, stop
      countedPoints = std::max(countedPoints, 1); // Ensure no div by zero
      int score = (validCCDPoints*100)/countedPoints;
      if (score > 90) {//minCCDRating) {
#ifdef BALL_DEBUG_PRINTOUT
         cout << "\tNum iterations: " << i << " with score " << score << endl;
#endif
         break;
      }

      // scale vector down to avoid overshoot
      vector /= 4;

      // Update the ball centre
      c += Point((int)vector.x(), (int)vector.y());
   }

   // Clear the vectors
   free(outerCos);
   free(outerSin);
   free(innerCos);
   free(innerSin);

   // Normalise the number of good points
   ball.ccdRating = (validCCDPoints*100) / (countedPoints);

   // Otherwise, the ratio is used as a rating
   // Move the ball to the new position
#ifdef BALL_DEBUG_POINTS
   ccdCentres.push_back(c);
#endif
   ball.imageCoords = c;

   return ball;
}


bool BallDetection::runDecisionTree(
      VisionFrame &frame,
      const Fovea &fovea,
      bool tracking,
      BallInfo &ball)
{
   bool truth = false;

   // Get the bitstring of the ball
   int * attributes = createBitStringFromBall(frame, fovea.top, ball);

   // First, run the decision tree, if we have any islands at all:
   // We want an island of at least 10, cos now we deal with spotty noise
   if (attributes[0] > 10) { // min size 1
      truth = runJ48Tree(attributes);
   }

#ifdef BALL_DEBUG_PRINTOUT
   cout << "\tNum Islands: " << attributes[13] << endl;
   cout << "\tDark points: " << (int)(1.0/ball.visionVar) << endl;
   cout << "\tBright points: " << attributes[NUM_ATTRIBUTES-1]  << endl;
#endif

   // If decision tree failed, then we get to re-attempt in the case of tracking,
   // or extreme distance + darkness
   // Get the darkness in this window as a percentage of the global darkest point
   int darkness = attributes[12];
   // Note that when we are tracking, globalDarkest is taken from the previous frame
   if (globalDarkest == 0) globalDarkest = MIN_LINE_DARKNESS;
   int darkPercent = 100*(maxDarkness-darkness)/globalDarkest;
   int distance = frame.cameraToRR.pose.imageToRobotRelative(ball.imageCoords, BALL_RADIUS).distance();

   if (!tracking && distance > TWO_METRES) return false;

   if (tracking || distance > TWO_METRES) {
      // If we're tracking, or its sufficiently distant,
      // Then we need sufficient brightness AND sufficient darkness
      if (attributes[NUM_ATTRIBUTES-1] > MIN_BRIGHT_PIXELS &&
            attributes[NUM_ATTRIBUTES-1] < MAX_BRIGHT_PIXELS &&
            darkPercent >= minDarknessPercentage) {
         truth = true;
      }
   }

   // Only if we aren't tracking
   // Compare the darkest point here to the global darkest
   // This should hopefully weed out any FPs on lines
   // We need enough darkness
   // WHAT IF WE LOSE BALLS HERE?
#ifdef BALL_DEBUG_PRINTOUT
   cout << "\tDark %: " << darkPercent << "%" << endl;
#endif
   if (!fovea.top ||  (!tracking && distance > TWO_METRES)) {
      int numDarkPoints = (int)(1.0/ball.visionVar);
      if (numDarkPoints < 15 || darkPercent < secondaryDarkThreshold) {
#ifdef BALL_DEBUG_PRINTOUT
         cout << "Not enough dark with " << numDarkPoints << endl;
#endif
         truth = false;
      }
   }

#ifdef DATA_COLLECTION
   if (attributes[0] > 1) {

      cout << "Attributes: ";
      for (int n = 0; n < NUM_ATTRIBUTES-1; ++n) {
         cout << attributes[n] << ",";
      }
      cout << "P"/*(truth? "P" :"N")*/ << endl;
   //}
   }
#endif

   // We no longer need attributes. Free the memory
   free(attributes);

   return truth;

}


Cluster * getCluster(
      std::vector<Cluster *> &clusters,
      unsigned int index)
{
   if (index < clusters.size()) {
      Cluster * c = clusters[index];
      if (c == NULL) {
         c = (Cluster *)malloc(sizeof(Cluster));
         c->size = 0;
         c->centre = Point(0,0);
         c->tl = Point(-1,-1);
         c->br = Point(-1,-1);
         clusters.push_back(c);
         c->id = index;
         return c;
      }
      return clusters[index];
   } else {
      Cluster * c = (Cluster *)malloc(sizeof(Cluster));
      c->size = 0;
      c->centre = Point(0,0);
      c->tl = Point(-1,-1);
      c->br = Point(-1,-1);
      clusters.push_back(c);
      c->id = clusters.size();
      return c;
   }
}


int * BallDetection::createBitStringFromBall(
      VisionFrame &frame,
      bool top,
      BallInfo &ball)
{
   // Get all the pixels within the ball
   // downsample to 30x30, so the centre is 14,14
   Point centre(SIZE/2 - 1,SIZE/2 - 1);

   // Vertical jump amount
   double jump = 1;
   double diameter = (double)(2*ball.radius);
   if (diameter >= SIZE) {
      // Round up or down, whichever is appropriate
      int roundUp = diameter/(double)SIZE >= (floor(diameter/(double)SIZE) + 0.5);
      jump = roundUp ? ceil(diameter/(double)SIZE) : floor(diameter/(double)SIZE);
   }

   // TODO: do we lower the jump amount when at low resolutions? Jump == 1 works well for
   // really small balls at a distance, but for radius 28, we need jump == 2
   // Also, we can't rreally modify this much or it screws over the learner
   // Also, jump == 1 is REALLY precise
   Point b = ball.imageCoords;

   // Create a 30x30 to know how many are in each row, so we know what we have to fill
   // Reset out 30x30 array of the binarised, normalised image
   memset(bits, 0, SIZE*SIZE*sizeof(char));

   int darkestPoint = maxDarkness;
   int numWhitePixels = 0;
   int numBlackPixels = 0;
   int numBrightPixels = 0;

   // This forms the 'visionVar', or the certainty of anything that passes the decision tree layer
   int numDarkPoints = 0;

   // Debug info
   bool printout = false;
#ifdef BALL_DEBUG_PRINTOUT
   printout = true;
   cout << "\tJump for radius " << ball.radius << ": " << jump << endl;
#endif

   // Note that our IDs start at 1, so that we can use 0 as a NO_ID type thing
   std::vector<Cluster *> clusters;
   for (int y = 0; y < SIZE; ++y) {
      for (int x = 0; x < SIZE; ++x) {
         if (squaredDist(Point(x,y), centre) <= (SIZE/2)*(SIZE/2)) {

            // Get the pixel value
            int pixel = 0;
            int u = 0;
            int v = 0;
            // Check that coordinates aren't gonna segfault
            int frameX = b.x() + (x - centre.x())*jump;
            int frameY = b.y() + (y - centre.y())*jump;
            int height = top? topImageHeight-1 : botImageHeight+topImageHeight-1;
            int width = top? topImageWidth-1 : botImageWidth-1;
            int bottom = top? 0+1 : topImageHeight+1;
            if ((frameX < 1 || frameX > width-1) ||
               (frameY < bottom || frameY > height-1)) {
               pixel = LARGE_NUMBER; // bogus value
            } else {
               pixel = getYValue(frame, top, Point(frameX, frameY));
               u = getUValue(frame, top, Point(frameX, frameY));
               v = getVValue(frame, top, Point(frameX, frameY));
            }

            // We only care to label it if it's 1
            if (pixel < WHITE_Y) {
               int darkPercent = 100*(maxDarkness-pixel)/globalDarkest;
               if (darkPercent >= secondaryDarkThreshold) numDarkPoints ++;

               // Find the darkest point of this entire region of interest
               if (pixel < darkestPoint) darkestPoint = pixel;

               numBlackPixels ++;

               if (printout) cout << 0;
               // check the ID's of square to left and above
               int leftID = x > 0? bits[y*SIZE + x - 1] : 0;
               int upID = y > 0? bits[(y-1)*SIZE + x] : 0;
               if (leftID <= UNDEFINED || leftID > 500) leftID = 0;
               if (upID <= UNDEFINED || upID > 500) upID = 0;

               // Four cases:
               // They are equal and are 0 - create a new cluster
               // They are equal and nonzero - add this point to that cluster
               // They are not equal, and one isn't zero - add this point to that cluster
               // They are neither zero - add this to the lower number cluster, combine the clusters,
               // and link the smaller-ID cluster from the larger
               // cluster.size keeps track of the highest id we can choose
               if (leftID == upID) {
                  if (leftID == 0) {
                     // Create a new cluster
                     Cluster * c = getCluster(clusters, clusters.size());
                     if (c == NULL) continue;
                     c->addPoint(Point(x,y));
                     bits[y*SIZE+x] = c->id;
                  } else {
                     // Add this point to the cluster of that ID
                     Cluster * c = getCluster(clusters, leftID-1);
                     if (c == NULL) continue;
                     c->addPoint(Point(x,y));
                     bits[y*SIZE+x] = leftID;
                  }
               } else if (leftID == 0 && upID != 0) {
                  Cluster * c = getCluster(clusters, upID-1);
                  if (c == NULL) continue;
                  c->addPoint(Point(x,y));
                  bits[y*SIZE+x] = c->id;
               } else if (leftID != 0 && upID == 0) {
                  Cluster * c = getCluster(clusters, leftID-1);
                  if (c == NULL) continue;
                  c->addPoint(Point(x,y));
                  bits[y*SIZE+x] = c->id;
               } else {
                  // They are different. 
                  // Check that we haven't done the swap before
                  // sanity check the IDs ... 
                  Cluster * one = getCluster(clusters, leftID-1);
                  Cluster * two =getCluster(clusters, upID-1);
                  if (one == NULL) continue;
                  if (two == NULL) continue;
                  if (one->id != two->id) {


                     //Find the lowest
                     int id = leftID < upID? leftID : upID;
                     int other = leftID < upID? upID : leftID;

                     Cluster * cLowerID = getCluster(clusters, id-1);
                     Cluster * cHigherID = getCluster(clusters, other-1);
                     if (cLowerID == NULL) continue;
                     if (cHigherID == NULL) continue;
                     if (cHigherID == NULL || cHigherID->id <= UNDEFINED || 
                           cLowerID->id <= UNDEFINED || cHigherID->id > 500 || cLowerID->id > 500) {
                        continue;
                        exit(1);
                     }

                     // Add this point
                     cLowerID->addPoint(Point(x,y));

                     // Combine with the other cluster
                     cLowerID->addPoint(cHigherID->centre);
                     cLowerID->size += cHigherID->size;
                     cLowerID->size --;

                     // free the other cluster
                     int prev = cHigherID->id;
                     // Link lower cluster from higher one
                     // Loop over all clusters, and for all that share id with cHigherID,
                     // make all these point to cLowerID
                     for (unsigned int k = 0; k < clusters.size(); ++k) {
                        if (clusters[k] != NULL && clusters[k]->id == prev) {
                           clusters[k] = cLowerID;
                        }
                     }
                     clusters[other-1] = cLowerID;
                     bits[y*SIZE+x] = cLowerID->id;
                     bits[y*SIZE+x] = cLowerID->id;

                  } else {
                     Cluster * c = getCluster(clusters, leftID-1);
                     if (c == NULL) continue;
                     c->addPoint(Point(x,y));
                     bits[y*SIZE+x] = c->id;
                  }
               }
            } else {
               if (printout) cout << 1;
               numWhitePixels ++;

               // Count the really bright pixels, for tracking purposes
               if (pixel > BRIGHT_LEVEL) numBrightPixels ++;
            }
         } else {
            if (printout) cout << " ";
         }
      }
      if (printout) cout << endl;
   }

#ifdef BALL_DEBUG_PRINTOUTS
   cout << "\tDarkest point: " << darkestPoint << endl;
#endif

   ball.visionVar = 1;
   if (numDarkPoints > 0)
      ball.visionVar = 1.0/(float)numDarkPoints;


   // So we have all the clusters that are sufficiently dark
   // Accept the largest island
   int * attributes = (int *)malloc(NUM_ATTRIBUTES*sizeof(int));
   memset(attributes, 0, NUM_ATTRIBUTES*sizeof(int));
   attributes[NUM_ATTRIBUTES-3] = darkestPoint;
   attributes[6] = numWhitePixels;
   attributes[7] = numBlackPixels;


   Cluster * bestIsland = NULL;
   int numIslands = 0;
   for (unsigned int n = 0; n < clusters.size(); ++n) {
      Cluster * c = clusters[n];
      if (c == NULL) {
         //cout << "We got a null cluster" << endl;
         continue;
         exit(1);
      }
      if (c->id == UNDEFINED) continue;
      if (c->id == (signed int)n+1) {
         // If any of the corners are too far form the centre, rejet this cluster
         // Accept the first island
         Point tl = c->tl;
         Point tr = Point(c->tl.y(), c->br.x());
         Point br = c->br;
         Point bl = Point(c->tl.x(), c->br.y());

         if (squaredDist(tl, centre) < (SIZE/2)*(SIZE/2) &&
             squaredDist(tr, centre) < (SIZE/2)*(SIZE/2) &&
             squaredDist(bl, centre) < (SIZE/2)*(SIZE/2) &&
             squaredDist(br, centre) < (SIZE/2)*(SIZE/2)) {

            numIslands ++;

            if (bestIsland == NULL) {
               bestIsland = c;
            } else if (c->size > bestIsland->size) {
               bestIsland = c;
            }
         }

         // Free the cluster
         c->id = UNDEFINED;
         free(c);
         clusters[n] = NULL;
      }
   }

   attributes[NUM_ATTRIBUTES-2] = numIslands;
   attributes[NUM_ATTRIBUTES-1] = numBrightPixels;

   if (bestIsland != NULL) {
      // We got an island - process it, add it to the list of attributes

      int width = bestIsland->br.x() - bestIsland->tl.x() + 1;
      int height = bestIsland->br.y() - bestIsland->tl.y() + 1;

      // If any of the corners are too far from the centre,
      // reject this cluster. Accept 

      int area = width*height;
      int ratio = 100*bestIsland->size / area;

      attributes[0] = bestIsland->size;
      attributes[1] = width;
      attributes[2] = height;
      attributes[3] = bestIsland->centre.x();
      attributes[4] = bestIsland->centre.y();
      attributes[5] = ratio;

      // The distance in from each side
      // How well it fits the square, by a really naive measure
      int i = 0;
      for (i = bestIsland->tl.y(); i < bestIsland->br.y(); ++i) {
         if (bits[i*SIZE + bestIsland->centre.x()] == bestIsland->id) {
            break;
         }
      }
      attributes[8] = i-bestIsland->tl.y();
      for (i = bestIsland->br.y(); i > bestIsland->tl.y(); --i) {
         if (bits[i*SIZE + bestIsland->centre.x()] == bestIsland->id) {
            break;
         }
      }
      attributes[9] = bestIsland->br.y()-i;
      for (i = bestIsland->tl.x(); i < bestIsland->br.x(); ++i) {
         if (bits[bestIsland->centre.y()*SIZE + i] == bestIsland->id) {
            break;
         }
      }
      attributes[10] = i-bestIsland->tl.x();
      for (i = bestIsland->br.x(); i > bestIsland->tl.x(); --i) {
         if (bits[bestIsland->centre.y()*SIZE + i] == bestIsland->id) {
            break;
         }
      }
      attributes[11] = bestIsland->br.x()-i;
   }



   return attributes;
}


// Create a ball and add it to the list of balls in the frame
// This was build from Carl Chatfield's ball construction in ransacBall3P or whatever
BallInfo BallDetection::createFieldBall(
      VisionFrame &frame,
      const Fovea &fovea,
      BallInfo ball)
{
      if (ball.visionVar < -0.1) {
         llog(DEBUG1) << "tossing ball due to var = " << ball.visionVar << std::endl;
         ball.visionVar = topImageHeight;
         return ball;
      }

      /* Compensate for sparse sampling
       * Diameter is increased by density / 2, Radius by 4
       * Centre is increased by density * 1
       * 0.5 * sampling + 0.5 * edge image offset
       */
      // Check bottom of ball is below field edge
      int baseY = ball.imageCoords.y() + ball.radius;
      int fieldEdgeY = frame.topStartScanCoords[ball.imageCoords.x()];
      if (!fovea.top) {
         fieldEdgeY = frame.botStartScanCoords[ball.imageCoords.x()];
         if (fieldEdgeY == botImageHeight)
            fieldEdgeY = 0;
      }
      if (baseY < fieldEdgeY) {
         llog(DEBUG1)  << "tossing ball because above field edge at " << baseY << std::endl;
         ball.imageCoords = Point(-1,-1);
         ball.visionVar = topImageHeight; // far too high a var for the filter to accept it
         return ball;
      }

      // lifetime info
      ball.lifetime = 0;
      ball.lastSeen = 0;

      Point b = frame.cameraToRR.pose.imageToRobotXY(
            ball.imageCoords,
            ball.radius);

      float diff = 190*tan(angleXDelay[0]);

      b.y() -= diff;
      RRCoord rr;
      rr.distance() = hypotf(b.y(), b.x());
      rr.heading() = atan2f(b.y(), b.x());
      ball.rr = rr;

      float robot_height = 500;
      float error = 30 * ball.rr.distance() / robot_height;
      ball.rr.distance() -= error;

      ball.neckRelative = frame.cameraToRR.pose.robotRelativeToNeckCoord(ball.rr, ball.radius);

      llog(DEBUG1) << "Ball at (" << ball.imageCoords.x() << ",";
      llog(DEBUG1) << ball.imageCoords.y() << ") radius = " << ball.radius;
      llog(DEBUG1) << std::endl;

      return ball;
}



/* ######################################## */
/* Ball Detection for pre-2016 orange ball  */
/* Activate with //#define WHITE_BALL above */
/* ######################################## */

void BallDetection::findBallsR(
      VisionFrame &frame,
      const Fovea &fovea,
      unsigned int *seed,
      bool terminal)
{

   std::vector<ball_seed_t> seeds;
   populateSeedLocations(frame, fovea, seeds);

   std::vector<ball_seed_t>::reverse_iterator i, j;
   for (i = seeds.rbegin(); i != seeds.rend(); ++ i) {
      for (j = i + 1; j != seeds.rend(); ++ j) {
         if (doSeedsOverlap(*i, *j)) {
            seeds.erase(i.base() - 1) + 1;
            break;
         }
      }
   }

   std::vector<ball_seed_t>::iterator s;
   for (s = seeds.begin(); s != seeds.end(); ++ s) {
      findBall(frame, fovea, *s, seed);
   }

   /* If no balls where found, attempt to track the last known ball,
    * This function recurses on findBallR, so if terminal is specified
    * dont bother
    */
   if (! terminal && frame.balls.empty() && frame.last) {
      if (! frame.last->balls.empty()) {

         /* Only bother trying to track distant balls. If the ball is close
          * and we missed it we will probably get it next frame any way
          */
         if (frame.last->balls[0].radius < maxTrackBallRadius) {
            llog(DEBUG1) << "findBallsR: using tracking fovea" << std::endl;
            trackLastBall(frame, fovea, frame.last->balls[0], seed);
         } 
      }
   }

   // If we still can't find anything, try looking at the localisation ball
   // This is populated by other teams members even if we can't see the ball
   // Only look if the ball is far away, otherwise its probably occluded
   const int min_teamball_dist = 4000;
   if (! terminal && frame.balls.empty() &&
       localisationBall.distance() > min_teamball_dist) {
      float x = localisationBall.distance() * cos (localisationBall.heading());
      float y = localisationBall.distance() * sin (localisationBall.heading());

      Point lBall = frame.cameraToRR.convertToImageXY(Point(x,y));
      if ((lBall.x() > 0) && (lBall.x() < TOP_IMAGE_COLS) &&
          (lBall.y() > 0) && (lBall.y() < TOP_IMAGE_ROWS)) {
         BallInfo ball;
         ball.imageCoords = lBall;
         trackLastBall(frame, fovea, ball, seed);
      }
   }

   std::vector<BallInfo>::iterator b;
   for (b = frame.balls.begin(); b != frame.balls.end(); ++ b) {
      if (isBallRadiusTooBig(frame, *b)) {
         llog(DEBUG1) << "findBallsR: isBallRadiusTooBig (failed)(";
         llog(DEBUG1) << b->imageCoords.x() << ","  << b->imageCoords.y();
         llog(DEBUG1) << ")" << std::endl;
         b = frame.balls.erase(b) - 1;
      }
   }

   std::sort(frame.balls.begin(), frame.balls.end(), BallInfoCmp());


   /* Throw out excess balls */
   if (frame.balls.size() > MAX_BALLS) {
      frame.balls.resize(MAX_BALLS);
   }
}

void BallDetection::findBall(
      VisionFrame &frame, 
      const Fovea &fovea,
      ball_seed_t &ball,
      unsigned int *seed)
{
   const int desiredBallRadius = 12;

   int density;
   if (ball.radius * fovea.density / 1 < desiredBallRadius) {
      density = 1;
   } else if (ball.radius * fovea.density / 2 < desiredBallRadius) {
      density = 2;
   } else if (ball.radius * fovea.density / 4 < desiredBallRadius) {
      density = 4;
   } else if (ball.radius * fovea.density / 8 < desiredBallRadius) {
      density = 8;
   } else {
      /* Ball is too big for saliency based methods. Use large ball
       * detector instead
       */
      density = 8; /* -1 */
   }

   llog(DEBUG1) << "findBall: seed at (" << ball.centre.x() << ",";
   llog(DEBUG1) << ball.centre.y() << ") radius = " << ball.radius;
   llog(DEBUG1) << std::endl;

   //points.push_back(fovea.mapFoveaToImage(ball.centre));

   // Previous calculations were for 640x480 image
   // Now that we use 1280x960, need to double density for top camera
   if (fovea.top) density *= 2;

   std::vector<Point> edges;
   if (density == -1) {
      /* use large ball detector */
      /* TODO(carl) write a large ball detector */
   } else {
      /* Create a zoomed in ball fovea */
      int size = ball.radius + (8 / fovea.density) + (density / 4);
      int extraW = 0; //2*size; part of refereeing code
      int extraH = 0; //size;
      Point tl(std::max(ball.centre.x() - (size + extraW), 0),
               std::max(ball.centre.y() - (size + extraH), 0));

      Point br(std::min(ball.centre.x() + (size + extraW), fovea.bb.width ()),
               std::min(ball.centre.y() + (size + extraH), fovea.bb.height()));

      tl = (tl + fovea.bb.a) * fovea.density / density;
      br = (br + fovea.bb.a) * fovea.density / density;

      boost::shared_ptr<FoveaT<hBall, eBall> > ballFovea(
            new FoveaT<hBall, eBall>(BBox(tl, br), density, 0, fovea.top));

      ballFovea->actuate(frame);
      ballFovea->xhistogram.applyWindowFilter(HIST_AVE_WINDOW_SIZE, hBall);
      ballFovea->yhistogram.applyWindowFilter(HIST_AVE_WINDOW_SIZE, hBall);

      /* Update ball seed. */
      ball.centre = fovea.mapFoveaToImage(ball.centre);
      ball.centre = ballFovea->mapImageToFovea(ball.centre);

      if (density < (int)fovea.density) {
         /* Refine the radius guestimate */
         ball.radius = ballRadiusFromHistogram(ballFovea->asFovea(),
                                               ballFovea->xhistogram,
                                               ballFovea->yhistogram,
                                               ball);
      }

      //ballFoveas.push_back(ballFovea);

      findBallEdges(ballFovea->asFovea(), edges);

      std::vector<Point>::iterator p;
      for (p = edges.begin(); p != edges.end(); ++ p) {
         //points.push_back(ballFovea->mapFoveaToImage(*p));
      }
      ransacBall(frame, ballFovea->asFovea(), edges, ball.radius, seed);
   }
}

void BallDetection::trackLastBall(
      VisionFrame &frame, 
      const Fovea &fovea, 
      const BallInfo &ball,
      unsigned int *seed)
{
   /* Make a new fovea somewhere around where we think the ball will be. Use a
    * relatively high resolution.
    *
    * Note: cost of creating a new fovea is determined by two factors.
    * 1. Area
    * 2. Are edge values calculated.
    *
    * For the tracking, we have two options, create one big fovea around
    * where the ball was last seen, or create an intermediate fovea
    * at half full resolution, and then create a full resolution fovea.
    * As we are only tracking distant balls, it really doesnt make sense
    * to attempt to find the ball in a subsampled fovea. Therefore the
    * costs of our two options are:
    *
    * Area is the area of the tracking fovea
    * 30x30 is an observation that zoom foveas tend to be of this size
    * density is the density of the intermediate fovea
    *
    * 1. Using an intermediate fovea:
    *    Area * colour_cost / density**2 + 30x30 * (colour_cost + edge_cost)
    * 2. Using full fovea
    *    Area * (colour_cost + edge_cost)
    *
    * It should be clear that creating the intermediate fovea will save us
    * processing time unless we can get the tracking fovea to be small.
    */

   const int density = 2;
   const int sizex = trackingSizeX / 2;
   const int sizey = trackingSizeY / 2;

   const int W = (fovea.top) ? TOP_IMAGE_COLS : BOT_IMAGE_COLS;
   const int H = (fovea.top) ? TOP_IMAGE_ROWS : BOT_IMAGE_ROWS;

   const Point centre = ball.imageCoords;
   if (centre.y() > TOP_IMAGE_ROWS && fovea.top) return;
   if (centre.y() < TOP_IMAGE_ROWS && !fovea.top) return;
   
   Point tl(std::max(centre.x() - sizex, 0),
            std::max(centre.y() - sizey, 0));

   Point br(std::min(centre.x() + sizex, W),
            std::min(centre.y() + sizey, H));

   /* If part of the tracking fovea is above the field edge
    * remove it
    */
   int fieldTopL = frame.topStartScanCoords[tl.x()] - maxPixelsAboveFieldEdge;
   int fieldTopR = frame.topStartScanCoords[br.x()] - maxPixelsAboveFieldEdge;
   if (!fovea.top) {
      fieldTopL = frame.botStartScanCoords[tl.x()] - maxPixelsAboveFieldEdge;
      fieldTopR = frame.botStartScanCoords[br.x()] - maxPixelsAboveFieldEdge;
   }
  
   int fieldTop  = std::min(fieldTopR, fieldTopL);


   tl.y() = std::max(fieldTop, tl.y());

   tl /= density;
   br /= density;

   if (tl.y() >= br.y()) {
      return;
   }

   boost::shared_ptr<FoveaT<hNone, eNone> > trackingFovea(
         new FoveaT<hNone, eNone>(BBox(tl, br), density, 0, fovea.top));

   trackingFovea->actuate(frame);

   trackingFoveas.push_back(trackingFovea);

   findBallsR(frame, trackingFovea->asFovea(), seed, true);
}

void BallDetection::ransacBall(
      VisionFrame &frame,
      const Fovea &fovea,
      const std::vector<Point> &ballEdges,
      int radius,
      unsigned int *seed)
{
   static std::vector<bool> buf[2];

   /* Dynamically resize buffers whenever needed, after a few iterations,
    * this will not be needed any more
    */
   if (buf[0].size() < ballEdges.size()) {
      buf[0].resize(ballEdges.size());
      buf[1].resize(ballEdges.size());
   }

   std::vector<bool> *cons;
   RANSACCircle result;

   const float ballError   = 1.5;//1.5;
   const float radiusError = 3; //2 * ballError;

   if (RANSAC::findCircleOfRadius3P(ballEdges, radius, radiusError,
            &cons, result, 50, ballError,
            ballEdges.size() / 2, buf, seed))
   {
      BallInfo best;
      best.imageCoords = result.centre.cast<int>();
      best.radius      = result.radius * fovea.density;
      best.visionVar   = result.var / best.radius;
      best.imageCoords = fovea.mapFoveaToImage(best.imageCoords);
      best.topCamera   = fovea.top;

      if (best.visionVar < -0.1) {
         llog(DEBUG1) << "tossing ball due to var = " << best.visionVar << std::endl;
         return;
      }

      /* Compensate for sparse sampling
       * Diameter is increased by density / 2, Radius by 4
       * Centre is increased by density * 1
       * 0.5 * sampling + 0.5 * edge image offset
       */
      int compensation = fovea.density;
      best.imageCoords += Point(compensation, compensation);
      best.radius += fovea.density / 4;

      // Check centre of ball isn't green
      Point p = fovea.mapImageToFovea(best.imageCoords);
      if (fovea.bb.validIndex(p)) {
         Colour c = fovea.colour(fovea.mapImageToFovea(best.imageCoords));
         if (c == cFIELD_GREEN) {
            llog(DEBUG1) << "centre of the ball is green" << std::endl;
            return;
         }
      }

      // Check bottom of ball is below field edge
      int baseY = best.imageCoords.y() + best.radius;
      int fieldEdgeY = frame.topStartScanCoords[best.imageCoords.x()];
      if (!fovea.top) fieldEdgeY = frame.botStartScanCoords[best.imageCoords.x()];
      if (baseY < fieldEdgeY) {
         llog(DEBUG1) << "tossing ball because above field edge" << std::endl;
         return;
      }

      Point b = frame.cameraToRR.pose.imageToRobotXY(
            best.imageCoords,
            BALL_RADIUS);

      float diff = 190*tan(angleXDelay[0]);

//      std::cout << "d=" << diff << std::endl;
//      std::cout << "r=" << b.y() << std::endl;

      b.y() -= diff;
      RRCoord rr;
      rr.distance() = hypotf(b.y(), b.x());
      rr.heading() = atan2f(b.y(), b.x());
      best.rr = rr;

//      std::cout << "f=" << b.y() << std::endl;

      //best.rr = frame.cameraToRR.pose.imageToRobotRelative(
      //      best.imageCoords,
      //      BALL_RADIUS);

      float robot_height = 500;
      float error = 30 * best.rr.distance() / robot_height;
      best.rr.distance() -= error;
      
      best.neckRelative = frame.cameraToRR.pose.robotRelativeToNeckCoord(best.rr, BALL_RADIUS);

      frame.balls.push_back(best);

      /******************************************/
////      static int xOffset = 330;
//      static TorsoStateFilter kf;
//      static int yOffset = 395;//370;
//      static float prev = DEG2RAD(-(best.imageCoords.y() - yOffset)*34.8/480.0);
//      static Timer t;
//      float dt = t.elapsed_us()/1000000.0;
//      kf.init(dt, 0.0008, 0.024, false);
////      cout << dt << endl;
////      float dt = 1/30.0;
//      static matrix<float> curObs(2, 1);
//      curObs(0, 0) = DEG2RAD(-(best.imageCoords.y() - yOffset)*34.8/480.0);
//      curObs(1, 0) = (curObs(0, 0) - prev)/dt;
//      float footPos[2][3];
//      float CoP[2][2];
//      for(int i = 0; i < 3; i++){
//         footPos[0][i] = FilteredTouch::lf(i, 0);
//         footPos[1][i] = FilteredTouch::rf(i, 0);
//      }
//      CoP[0][0] = FilteredTouch::state.sensors[Sensors::LFoot_FSR_CenterOfPressure_X] * 1000;
//      CoP[0][1] = FilteredTouch::state.sensors[Sensors::LFoot_FSR_CenterOfPressure_Y] * 1000;
//      CoP[1][0] = FilteredTouch::state.sensors[Sensors::RFoot_FSR_CenterOfPressure_X] * 1000;
//      CoP[1][1] = FilteredTouch::state.sensors[Sensors::RFoot_FSR_CenterOfPressure_Y] * 1000;
//      matrix<float> est = kf.update(curObs, CoP, footPos);
//
//      FilteredTouch::ang = est(0, 0);
//      FilteredTouch::vel = est(1, 0);
//      FilteredTouch::ang2 = curObs(0, 0);
//      FilteredTouch::vel2 = curObs(1, 0);
//      prev = curObs(0, 0);
//      t.restart();
      /**************************************/

      llog(DEBUG1) << "ransacBall: ball at (" << best.imageCoords.x() << ",";
      llog(DEBUG1) << best.imageCoords.y() << ") radius = " << best.radius;
      llog(DEBUG1) << std::endl;
   }
   llog(DEBUG1) << "ransacBall: failed" << std::endl;
}

void BallDetection::populateSeedLocations(
      VisionFrame &frame,
      const Fovea &fovea,
      std::vector<ball_seed_t> &seeds)
{
   const int min_edge_area = 12;

   Histogram<int, cNUM_COLOURS> xhist(fovea.bb.width());
   Histogram<int, cNUM_COLOURS> yhist(fovea.bb.height());

   makeBallHistsBelowFieldEdge(frame, fovea, xhist, yhist);

   typedef std::pair<int, int> cut;
   std::vector<cut> horz, vert;

   int e, s = -1;
   for (e = 0; e < xhist.size; ++ e) {
      if (xhist.counts(e, cBALL) > 0) {
         if (s == -1) {
            s = e;
         }
      } else {
         if (s != -1) {
            horz.push_back(cut(s, e));
            s = -1;
         }
      }
   }
   if (s != -1) {
            horz.push_back(cut(s, e));
   }

   for (e = 0; e < yhist.size; ++ e) {
      if (yhist.counts(e, cBALL) > 0) {
         if (s == -1) {
            s = e;
         }
      } else {
         if (s != -1) {
            vert.push_back(cut(s, e));
            s = -1;
         }
      }
   }
   if (s != -1) {
      vert.push_back(cut(s, e));
   }

   std::vector<cut>::iterator i, j;
   for (j = vert.begin(); j != vert.end(); ++ j) {
      for (i = horz.begin(); i != horz.end(); ++ i) {
         ball_seed_t ball;

         int w = i->second - i->first;
         int h = j->second - j->first;

         /* If on an edge, dont bother looking for small balls */
         if (i->first == 0 || i->second == fovea.bb.width () - 1
          || j->first == 0 || j->second == fovea.bb.height() - 1)
         {
            if (w * h < min_edge_area) {
               continue;
            }
         }

         ball.centre.x() = (i->second + i->first) / 2;
         ball.centre.y() = (j->second + j->first) / 2;

         /* Round radius up */
         ball.radius = std::max((w + 1) / 2, (h + 1) / 2);
         if (checkSeed(fovea, ball)) {
            seeds.push_back(ball);
         }
      }
   }

   /*
   xhist.applyWindowFilter(HIST_AVE_WINDOW_SIZE, hBall);
   yhist.applyWindowFilter(HIST_AVE_WINDOW_SIZE, hBall);

   int x, y;
   for (y = 0; yhist.peaks(y, cBALL) != -1; ++ y) {
      for (x = 0; xhist.peaks(x, cBALL) != -1; ++ x) {
         ball_seed_t ball;
         ball.centre = Point(xhist.peaks(x, cBALL),
                             yhist.peaks(y, cBALL));

         if (guessRadiusAndValidateSeed(frame, fovea, xhist, yhist, ball)) {
            seeds.push_back(ball);
         }
      }
   }
   */
}

void BallDetection::findBallEdges(
      const Fovea &fovea,
      std::vector<Point> &edges)
{
   const int t2 = ballEdgeThreshold * ballEdgeThreshold;

   const int x_ends[16] = {4, 4, 4, 3, 2, 1, 0, 0, 0, 0, 0, 1, 2, 3, 4, 4};
   const int y_ends[16] = {2, 1, 0, 0, 0, 0, 0, 1, 2, 3, 4, 4, 4, 4, 4, 3};

   /* Fovea data is stored in columns, not rows. To use the BresenhamPtr's
    * to iterate across the fovea buffers, invert x and y
    */
   Point start(fovea.xhistogram.maxPeak(cBALL),
               fovea.yhistogram.maxPeak(cBALL));

   /* If start.[xy]() == -1, then there are no peaks, likely because the
    * ball is on the edge of the image. Work out which one
    */
   enum
   {
      edge_top    = 1 << 0,
      edge_bottom = 1 << 1,
      edge_left   = 1 << 2,
      edge_right  = 1 << 3
   } edge;


   if (start.x() == -1) {
      if (fovea.xhistogram.counts(0                   , cBALL) >
          fovea.xhistogram.counts(fovea.bb.width() - 1, cBALL))
      {
         start.x() = 0;
         edge = edge_left;
      } else {
         start.x() = fovea.bb.width() - 1;
         edge = edge_right;
      }
   }

   if (start.y() == -1) {
      if (fovea.yhistogram.counts(0                    , cBALL) >
          fovea.yhistogram.counts(fovea.bb.height() - 1, cBALL))
      {
         start.y() = 0;
         edge = edge_top;
      } else {
         start.y() = fovea.bb.height() - 1;
         edge = edge_bottom;
      }
   }

   int n_ball, n_total, n, green_white_count;;

   int i;
   for (i = 0; i < 16; ++ i) {
      Point end(fovea.bb.height() * y_ends[i] / 4,
                fovea.bb.width()  * x_ends[i] / 4);

      BresenhamPtr<const Colour> cbp(fovea._colour, start, end,
                                     fovea.bb.height());
      BresenhamPtr<const Colour>::iterator cp = cbp.begin();

      BresenhamPtr<const Point> ebp(fovea._edge, start, end,
                                    fovea.bb.height());
      BresenhamPtr<const Point>::iterator ep = ebp.begin();

      Point bestPoint = Point(-1,-1);
      int best = std::numeric_limits<int>::min();

      n_ball = n_total = n = green_white_count = 0;
      while (fovea.bb.validIndex(cp.point())) {
         if (*cp == cFIELD_GREEN || *cp == cWHITE) {
            ++ green_white_count;
         }
         if (green_white_count > 2) {
            break;
         }

         if (*cp == cBALL) {
            ++ n_ball;
         }
         ++ n;

         // Uncomment this to show the scan lines
         /*Colour *c = const_cast<Colour *>(cp.get());
         *c = cBLACK;*/

         if (best < ep->squaredNorm() && ep->squaredNorm() >= t2) {
            best = ep->squaredNorm();
            bestPoint = ep.point();
            n_total = n;
         }
      
         ++ cp, ++ ep;
      }

      if ((n_ball * 1024 > n_total * ballColourRatio) &&
          (bestPoint.x() != -1)) {
         edges.push_back(bestPoint);
      }
   }
}

void BallDetection::findSmallBallEdges(
      const Fovea &fovea,
      std::vector<Point> &edges)
{
   /* Scan 8 times in all directions, rotating pi/4 at each iteration.
    * Scan grid looks something like this
    *
    *     +------+
    *    /|\    /|\
    *   / | \  / | \
    * +<--+--><--+-->+
    * | \ | /  \ | / |
    * |  \|/    \|/  |
    * |  /|\    /|\  |
    * | / | \  / | \ |
    * +<--+--><--+-->+
    *   \ | /  \ | /
    *    \|/    \|/
    *     +------+
    */

   Point start;
   /* Length of edges of square boxes */
   int s_size = fovea.bb.width() / 3;
   /* Length of edges of diamond boxes */
   int d_size = fovea.bb.width() / 4;

   (void)d_size;
   /* Square scans */
   start = Point(fovea.bb.width(), s_size);
   scanBoxForEdges(fovea, start, s_size, 1, 0, edges);
   
}


void BallDetection::scanBoxForEdges(
      const Fovea &fovea,
      const Point start,
      const int size,
      const int dx,
      const int dy,
      std::vector<Point> &edges)
{
   const int minEdge2 = ballEdgeThreshold * ballEdgeThreshold;

   Point s = start, best(0, 0);

   int edge_mag2, best_mag2;
   int x, y;
   do {
      x = s.x();
      y = s.y();

      best_mag2 = 0;
      do {
         const Point &edge = fovea.edge(x, y);
         edge_mag2 = edge[0] * edge[0] + edge[1] * edge[1];

         if (edge_mag2 >= minEdge2 && edge_mag2 >= best_mag2) {
            best_mag2 = edge_mag2;
            best = Point(x, y);
         }

         x += dx;
         y += dy;
      } while (x != s.x() + size * dx || y != s.y() + size * dy);

      if (best_mag2 != 0) {
         edges.push_back(best);
      }

      s.x() -= dy;
      s.y() += dx;
   } while (s.x() != start.x() + size * (-dy) || s.y() != start.y() + size * dx);
}

bool BallDetection::guessRadiusAndValidateSeed(
      VisionFrame &frame,
      const Fovea &fovea,
      const Histogram<int, cNUM_COLOURS> &xhistogram,
      const Histogram<int, cNUM_COLOURS> &yhistogram,
      ball_seed_t &ball)
{
   /*
   const int ballRadiusAtMaxTrustDist =
      ballRadiusFromDistance(dontTrustKinematicsBeyond) / fovea.density;
   */

   const Pose &pose = frame.cameraToRR.pose;
   int distRadius, histRadius;

   RRCoord toBall = pose.imageToRobotRelative(ball.centre, BALL_RADIUS);

   /* TODO(carl) use a variable scheme instead of a hard limit */
   if (false && toBall.distance() < dontTrustKinematicsBeyond) {
      /* Trust the kinematic radius calculation as the word of god */
      // NOT USED SO TOP JUST MADE FALSE TO COMPILE
      bool top = false; //(ball.imageCoords.y <= TOP_IMAGE_ROWS)
      distRadius = ballRadiusFromDistance(toBall.distance(), top);
      ball.radius = distRadius / fovea.density;
   } else {
      /* if there is a ball beyond the trust distance, use the radius
       * as guessed from the histogram. If the histogram guess is larger
       * than the ball radius at the trust limit + a tolerance, invalidate
       * the seed
       */
      histRadius = ballRadiusFromHistogram(fovea, xhistogram, yhistogram, ball);
      /*
      if (histRadius <= ballRadiusAtMaxTrustDist * 1.1) {
      */
         ball.radius = histRadius;
      /*
      } else {
         return false;
      }
      */
   }

   const int C = (fovea.top) ? TOP_IMAGE_COLS : BOT_IMAGE_COLS;
   const int R = (fovea.top) ? TOP_IMAGE_ROWS : BOT_IMAGE_ROWS;

   static const int orangeTestRadius = 1;
   Point orangeTestPoint;
   orangeTestPoint.x() = crop(ball.centre.x(),
                              orangeTestRadius,
                              C - orangeTestRadius - 1
                             );
   orangeTestPoint.y() = crop(ball.centre.x(),
                              orangeTestRadius,
                              R - orangeTestRadius - 1
                             );

   if (countOrangeAround(fovea, ball, orangeTestRadius) < 1) {
      return false;
   }

   return true;
}

int BallDetection::ballRadiusFromDistance(int distance, bool top)
{
   const float PIXEL = (top) ? TOP_PIXEL_SIZE : BOT_PIXEL_SIZE;
   return (1 / PIXEL) * (FOCAL_LENGTH * BALL_RADIUS) / distance;
}

int BallDetection::ballRadiusFromHistogram(
      const Fovea &fovea,
      const Histogram<int, cNUM_COLOURS> &xhistogram,
      const Histogram<int, cNUM_COLOURS> &yhistogram,
      const ball_seed_t &ball)
{
   const int x = ball.centre.x();
   const int y = ball.centre.y();

   const int x_peak  = xhistogram.window(x);
   const int x_start = HIST_AVE_WINDOW_SIZE / 2;
   const int x_end   = xhistogram.size - (HIST_AVE_WINDOW_SIZE - 1) / 2;
   const int x_min   = x_peak < 2 ? 1 : 2;

   const int y_peak  = yhistogram.window(y);
   const int y_start = HIST_AVE_WINDOW_SIZE / 2;
   const int y_end   = yhistogram.size - (HIST_AVE_WINDOW_SIZE - 1) / 2;
   const int y_min   = y_peak < 2 ? 1 : 2;

   /* Initial size is 1 pixel.
    *
    * The sliding window fattens input by 'HIST_AVE_WINDOW_SIZE - 1',
    * so compensate by removing this ammount from the initial count
    *
    * Add 1 such that radius is rounded up at final /2 step
    */
   int x_size = 1 - (HIST_AVE_WINDOW_SIZE - 1) + 1;
   int y_size = 1 - (HIST_AVE_WINDOW_SIZE - 1) + 1;
   
   int i;
   for (i = x + 1; i < x_end && xhistogram.window(i) >= x_min; ++ i) {
      ++ x_size;
   }
   for (i = x - 1; i >= x_start && xhistogram.window(i) >= x_min; -- i) {
      ++ x_size;
   }
   for (i = y + 1; i < y_end && yhistogram.window(i) >= y_min; ++ i) {
      ++ y_size;
   }
   for (i = y - 1; i >= y_start && yhistogram.window(i) >= y_min; -- i) {
      ++ y_size;
   }

   return std::max(x_size / 2, y_size / 2);
}

int BallDetection::countOrangeAround(
      const Fovea &fovea,
      const ball_seed_t &ball,
      const int radius)
{
   int x, y;
   int count = 0;
   for (y = ball.centre.y() - radius; y <= ball.centre.y() + radius; ++ y) {
      for (x = ball.centre.x() - radius; x <= ball.centre.x() + radius; ++ x) {
         if (fovea.colour(x, y) == cBALL) {
            ++ count;
         }
      }
   }

   return count;
}

void BallDetection::makeBallHistsBelowFieldEdge(
      VisionFrame &frame,
      const Fovea &fovea,
      Histogram<int, cNUM_COLOURS> &xhist,
      Histogram<int, cNUM_COLOURS> &yhist)
{
   typedef int (*histogram_ptr_t)[cNUM_COLOURS];

   Point start, end;
   int x, y;
   int fieldTop;

   const Colour *saliencyPixel, *saliencyEnd;

   for (y = 0; y < fovea.bb.height(); ++ y) {
      yhist._counts[y][cBALL] = 0;
   }
   for (x = 0; x < fovea.bb.width(); ++ x) {
      xhist._counts[x][cBALL] = 0;
   }

   histogram_ptr_t yhist_ptr, xhist_ptr = xhist._counts;

   for (x = 0; x < fovea.bb.width(); ++ x) {

      /* Take advantage of sequencial vertical access */
      start = fovea.mapFoveaToImage(Point(x, 0));
      fieldTop = frame.topStartScanCoords[start.x()] - maxPixelsAboveFieldEdge;
      if (!fovea.top) {
         fieldTop =frame.botStartScanCoords[start.x()]-maxPixelsAboveFieldEdge;
      }
      start.y() = std::max(start.y(), fieldTop);
      start = fovea.mapImageToFovea(start);

      end = fovea.mapFoveaToImage(Point(x, fovea.bb.height()));
      if (fovea.top) {
         end.y() =std::min(end.y(),frame.cameraToRR.topEndScanCoords[end.x()]);
      } else {
         end.y() = TOP_IMAGE_ROWS +
               std::min(end.y(),frame.cameraToRR.botEndScanCoords[end.x()]);
      }
      end = fovea.mapImageToFovea(end);

      end.y() = std::max(end.y(), 0);
      end.y() = std::min(end.y(), fovea.bb.height());

      start.y() = std::max(start.y(), 0);
      start.y() = std::min(start.y(), end.y());

      yhist_ptr = yhist._counts + start.y();

      saliencyPixel = fovea._colour + start.x() * fovea.bb.height() + start.y();
      saliencyEnd   = fovea._colour + end.x()   * fovea.bb.height() + end.y();

      while (saliencyPixel != saliencyEnd) {
         if (*saliencyPixel == cBALL) {
            ++ (*xhist_ptr)[cBALL];
            ++ (*yhist_ptr)[cBALL];
         }

         ++ saliencyPixel;
         ++ yhist_ptr;
      }
      ++ xhist_ptr;
   }
}

bool BallDetection::doSeedsOverlap(
      const ball_seed_t &a,
      const ball_seed_t &b)
{
   int rads = a.radius + b.radius;
   return (a.centre - b.centre).squaredNorm() < rads * rads;
}

bool BallDetection::isBallRadiusTooBig(
      const VisionFrame &frame,
      const BallInfo &ball)
{
   const Pose &pose = frame.cameraToRR.pose;

   RRCoord toBall = pose.imageToRobotRelative(ball.imageCoords, BALL_RADIUS);
   bool top = (ball.imageCoords.y() <= TOP_IMAGE_ROWS);
   int distRadius = ballRadiusFromDistance(toBall.distance(), top);
   if (distRadius * 2 < ball.radius) {
      return true;
   }

   // Also check ball isn't too small
   if (distRadius / 3 > ball.radius) {
      return true;
   }
   return false;
}

int BallDetection::checkSeed(
      const Fovea       &fovea,
      const ball_seed_t &seed)
{
   int numOrange, numGreen;
   int density;

   BBox roi(seed.centre - Point(1, 1),
            seed.centre + Point(2, 2));

   numOrange = countColourInBox(fovea, roi, 1, cBALL);
   if (numOrange < 1) {
      return false;
   }

   /* For one pixel balls that are potentially close to the field edge,
    * search a 5x2 box on and below the ball
    *
    * ..o..
    * .....
    *
    * Otherwise scan around it at radius width
    *
    * ....
    * .oo.
    * .oo.
    * ....
    */
   
   if (seed.radius == 1) {
      roi = BBox(seed.centre - Point(2, 0), seed.centre + Point(3, 2));
      numGreen = countColourInBox(fovea, roi, 1, cFIELD_GREEN);
   } else {
      density = seed.radius / 2;
      if (density == 0) {
         density = 1;
      }

      roi = BBox(seed.centre - Point(seed.radius, seed.radius),
                 seed.centre + Point(seed.radius + 2, seed.radius + 2));
      numGreen = countColourAroundBox(fovea, roi, density, cFIELD_GREEN);
   }


   if (numGreen < 1) {
      return false;
   }

   return true;
}

int BallDetection::countColourInBox(
      const Fovea &fovea,
      BBox         roi,
      int          density,
      Colour       colour)
{
   int x, y;
   int count;

   roi.a.x() = std::max(0, roi.a.x());
   roi.a.y() = std::max(0, roi.a.y());

   roi.b.x() = std::min(fovea.bb.width() , roi.b.x());
   roi.b.y() = std::min(fovea.bb.height(), roi.b.y());

   count = 0;
   for (y = roi.a.y(); y < roi.b.y(); y += density) {
      for (x = roi.a.x(); x < roi.b.x(); x += density) {
         if (fovea.colour(x, y) == colour) {
            ++ count;
         }
      }
   }

   return count;
}

int BallDetection::countColourAroundBox(
      const Fovea &fovea,
      BBox         roi,
      int          density,
      Colour       colour)
{
   int x, y;
   int count;

   bool count_top   = true;
   bool count_left  = true;
   bool count_bot   = true;
   bool count_right = true;

   if (roi.a.x() < 0) {
      count_left = false;
      roi.a.x() = 0;
   }

   if (roi.a.y() < 0) {
      count_top = false;
      roi.a.y() = 0;
   }

   if (roi.b.x() > fovea.bb.width()) {
      count_right = false;
      roi.b.x() = fovea.bb.width();
   }

   if (roi.b.x() > fovea.bb.height()) {
      count_bot = false;
      roi.b.y() = fovea.bb.height();
   }

   count = 0;

   if (count_top) {
      for (x = roi.a.x(); x < roi.b.x(); x += density) {
         if (fovea.colour(x, roi.a.y()) == colour) {
            ++ count;
         }
      }
   }

   if (count_left) {
      for (y = roi.a.y(); y < roi.b.y(); y += density) {
         if (fovea.colour(roi.a.x(), y) == colour) {
            ++ count;
         }
      }
   }

   if (count_bot) {
      for (x = roi.a.x(); x < roi.b.x(); x += density) {
         if (fovea.colour(x, roi.b.y() - 1) == colour) {
            ++ count;
         }
      }
   }

   if (count_right) {
      for (y = roi.a.y(); y < roi.b.y(); y += density) {
         if (fovea.colour(roi.b.x() - 1, y) == colour) {
            ++ count;
         }
      }
   }

   return count;
}
   

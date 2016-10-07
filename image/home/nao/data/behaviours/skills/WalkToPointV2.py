from math import cos, sin, radians, copysign

import Constants
import Global
import actioncommand
import robot

from Task import BehaviourTask, TaskState
from util import (
   FieldGeometry,
   MathUtil,
   PotentialField,
   Sonar,
   Vector2D,
)
from util.Hysteresis import Hysteresis
from util.MathUtil import clamp


# limits set in generator instead
def limitWalk(x, y, turn):
   # rotate target x, y
   forward = x * cos(-turn) - y * sin(-turn)
   left = x * sin(-turn) + y * cos(-turn)
   
   # avoid setting all parameters 0 because the generator thinks it should stand still
   if abs(forward) == 0 and abs(left) == 0 and abs(turn) ==0:
      forward = 3
   return forward, left, turn

def logObstacle(dist, heading):
#    Log.debug("obstacle distance %s, heading %s", dist, heading*56)
   return

def logTarget(vectorToTarget, headingDiff):
#    Log.debug("target distance %s, current heading %s change %s", vectorToTarget.length(), vectorToTarget.heading()*56, headingDiff*56)
   return

def isNearPathToTarget(targetHeading, rotatedVectorToTarget, obs):
    obs = obs.rotated(-targetHeading)
    # within 1m to path
    if abs(obs.y) < 1000 :
        obsHeadingFromTarget = obs.minus(rotatedVectorToTarget).heading()
        # within frontal cone of target to self
        if abs(obsHeadingFromTarget) > radians(135) :
            return True
    return False

class WalkToPointV2(BehaviourTask):

   NEARBY_SWITCHING_DISTANCE = 400
   FARAWAY_SWITCHING_DISTANCE = 600
   FIXED_OBSTACLES = [
      # Temporarily (I hope) turning off all fixed obstacles because our localisation isn't yet stable enough to avoid
      # these.
      ( Constants.GOAL_POST_ABS_X, Constants.GOAL_POST_ABS_Y, Constants.GOAL_POST_DIAMETER),
      ( Constants.GOAL_POST_ABS_X,-Constants.GOAL_POST_ABS_Y, Constants.GOAL_POST_DIAMETER),
      (-Constants.GOAL_POST_ABS_X, Constants.GOAL_POST_ABS_Y, Constants.GOAL_POST_DIAMETER),
      (-Constants.GOAL_POST_ABS_X,-Constants.GOAL_POST_ABS_Y, Constants.GOAL_POST_DIAMETER),
      # # Obstacles behind the posts so that goalie doesn't walk through the triangles.
      # (-Constants.GOAL_POST_ABS_X - 200, Constants.GOAL_POST_ABS_Y, 400),
      # (-Constants.GOAL_POST_ABS_X - 200,-Constants.GOAL_POST_ABS_Y, 400),
      # ( -500, 0, 100),
      # (-1500, 0, 100),
      # (-2500, 0, 100),
      # (-3500, 0, 100),
   ]

   def init(self, keepFacing=False, relative=False, useAvoidance=True):
      self.keepFacing = keepFacing
      self.relativeTarget=relative
      self.useAvoidance = useAvoidance
      self.currentState = Initial(self)
      self.visionPostHys = Hysteresis(0, 10)

   def transition(self):
      prevState = self.currentState
      if self.keepFacing:
         if not isinstance(self.currentState, Facing):
            self.currentState = Facing(self)
      else:
         if isinstance(self.currentState, FarAway) and self.currentTarget.isShorterThan(WalkToPointV2.NEARBY_SWITCHING_DISTANCE):
            self.currentState = Nearby(self)
         elif isinstance(self.currentState, Nearby) and self.currentTarget.isLongerThan(WalkToPointV2.FARAWAY_SWITCHING_DISTANCE):
            self.currentState = FarAway(self)
         elif not self.keepFacing and (isinstance(self.currentState, Facing) or isinstance(self.currentState, Initial)):
            self.currentState = FarAway(self)

      #self.printStateChange(prevState, self.currentState)

   def getGoalPostsFromVision(self):
       """Add Goalposts From Vision.
 
       This takes vision goalposts and marks them as obstacles, so that hopefully we have one more point where we avoid
       them (beyond just sonar). The main issue is that these are unfiltered.
       """
 
       # If you see a goalpost -> Trigger a hysteresis item.
       # Else decay it.
       # If there is some hysteresis from a goalpost.
       # If you see it: Avoid the vision goalpost.
       # If you don't: Avoid the localisation one.
       posts = Global.getVisionPosts()
       posts_with_distance = []
       obstacles = []
       for post in posts:
          if not 0 < post.rr.distance < (Constants.GOAL_POST_ABS_Y * 2):
             # Sometimes posts are -1 in distance, I think when we don't trust our calculation. Don't try to avoid these.
             # Also don't include posts we see across the field / far away, byt ignoring any over 1 goal width away.
             continue
          dist = max(0, post.rr.distance-Constants.GOAL_POST_DIAMETER/2)
          posts_with_distance.append(Vector2D.makeVector2DFromDistHeading(dist, post.rr.heading))
       if posts_with_distance:
          self.visionPostHys.resetMax()
       else:
          self.visionPostHys.down()
       #print "HysVal: %3d, PostsLen: %2d, DistPostsLen: %2d" % (self.visionPostHys.value, len(posts), len(posts_with_distance))
       if self.visionPostHys.value > 0:
          if posts_with_distance:
             # If we can see the posts, avoid the posts we're seeing.
             #print "Avoiding from vision"
             obstacles.extend(posts_with_distance)
             # print "adding posts from vision"
          else:
             # If we can't see a goalpost in vision, avoid the one in localisation (as an approximation so we don't
             # stop avoiding as we turn our head).
             robotPose = Global.myPose()
             for (obs_x, obs_y, obs_diam) in WalkToPointV2.FIXED_OBSTACLES:
                # Calculate distance to the obstacle.
                distance, heading = MathUtil.absToRr((robotPose.x, robotPose.y, robotPose.theta), (obs_x, obs_y))
                # print "Adding post from localisation: (%5d < %4.0f)" % (distance, degrees(heading))
                distance = max(0, distance-obs_diam/2)
                obstacles.append(Vector2D.makeVector2DFromDistHeading(distance, heading))
             #print "Avoiding from localisation"
       #return obstacles
       return []
             
   def calculateDestinationViaObstacles(self, vectorToTarget, extraObstacles):
      isSupporter = len(extraObstacles) > 0;
      "attractive field of target"
      Uatt = PotentialField.getAttractiveField(vectorToTarget)

      # let x axis be path
      targetHeading = vectorToTarget.heading()
      rotatedVectorToTarget = vectorToTarget.rotated(-targetHeading)
      # dont avoid obstacles within 100mm to target
      rotatedVectorToTarget.x -= copysign(100, rotatedVectorToTarget.x)
         
      "repulsive field of each obstacle"
      Urep = Vector2D.Vector2D(0,0)  
      obstacles = Global.robotObstaclesList()
      for i in range(0, len(obstacles)) :
#          LedOverride.override(LedOverride.leftEye, Constants.LEDColour.magenta)
         logObstacle(obstacles[i].rr.distance, obstacles[i].rr.heading)
         obsPos = Vector2D.makeVector2DFromDistHeading(obstacles[i].rr.distance, obstacles[i].rr.heading)
         if isSupporter or isNearPathToTarget(targetHeading, rotatedVectorToTarget, obsPos) :
             Urep = Urep.plus(PotentialField.getRepulsiveField(obsPos))
 
      # add sonar and goal posts
      obstacles = self.getSonarObstacles()
      obstacles.extend(self.getGoalPostsFromVision())
      obstacles.extend(extraObstacles)
      for i in range(0, len(obstacles)) :
         logObstacle(obstacles[i].length(), obstacles[i].heading())
         if isSupporter or isNearPathToTarget(targetHeading, rotatedVectorToTarget, obstacles[i]) :
             Urep = Urep.plus(PotentialField.getRepulsiveField(obstacles[i]))
       
      # add closest points to field borders
      myPos = Global.myPos()
      myHeading = Global.myHeading() 
      BORDER_PADDING = 800
      obstacles = [
             Vector2D.Vector2D(Constants.FIELD_LENGTH/2 + BORDER_PADDING - myPos.x, 0),
             Vector2D.Vector2D(-(Constants.FIELD_LENGTH/2 + BORDER_PADDING) - myPos.x, 0),
             Vector2D.Vector2D(0, Constants.FIELD_WIDTH/2 + BORDER_PADDING - myPos.y),
             Vector2D.Vector2D(0, -(Constants.FIELD_WIDTH/2 + BORDER_PADDING) - myPos.y)
       ]
      for i in range(0, len(obstacles)) :
         obstacle = obstacles[i].rotate(-myHeading)
         logObstacle(obstacle.length(), obstacle.heading())
         if isSupporter or isNearPathToTarget(targetHeading, rotatedVectorToTarget, obstacle) :
             Urep = Urep.plus(PotentialField.getRepulsiveField(obstacle))
      
      #sum the fields, we only need the heading change
      U = Uatt.plus(Urep)
      # avoid losing too much forward momentum
      headingDiff = clamp(U.heading()-vectorToTarget.heading(), radians(-70), radians(70))
      vectorToTarget.rotate(headingDiff)
      logTarget(vectorToTarget, headingDiff)
      return vectorToTarget
     
   @staticmethod
   def getSonarObstacles():
      obstacles = []
      LEFT, RIGHT = 0, 1
      nearbyObject = [Sonar.hasNearbySonarObject(LEFT), Sonar.hasNearbySonarObject(RIGHT)]
      sonarDebug = ""
      if nearbyObject[LEFT] :
         sonarDebug += " left"
         obstacles.append(Vector2D.makeVector2DFromDistHeading(Constants.ROBOT_DIAM, radians(30)))
      if nearbyObject[RIGHT] :
         sonarDebug += " right"
         obstacles.append(Vector2D.makeVector2DFromDistHeading(Constants.ROBOT_DIAM, -radians(30)))
      if sonarDebug :
         robot.say("sonar" + sonarDebug)
      return obstacles
     
   def _tick(self, x, y, theta, urgency=0, keepFacing=False, relative=False, useAvoidance=True, extraObstacles=[]):
#       LedOverride.override(LedOverride.leftEye, Constants.LEDColour.off)
#       LedOverride.override(LedOverride.rightEye, Constants.LEDColour.off)
      self.keepFacing = keepFacing

      # Save destination for walkingTo
      self.world.b_request.walkingToX = int(x)
      self.world.b_request.walkingToY = int(y)
      if relative:
         new = FieldGeometry.addRrToRobot(Global.myPose(), x, y)
         self.world.b_request.walkingToX = int(new[0])
         self.world.b_request.walkingToY = int(new[1])

      # Convert everything to relative.
      if relative: 
         vectorToTarget = Vector2D.Vector2D(x, y)
         facingTurn = theta
      else:
         for i in range(0, len(extraObstacles)) :
            extraObstacles[i] = FieldGeometry.globalPointToRobotRelativePoint(extraObstacles[i])
         vectorToTarget, facingTurn = FieldGeometry.globalPoseToRobotRelativePose(Vector2D.Vector2D(x, y), theta)

      # always keep heading the same after avoidance
      facingTurn = MathUtil.normalisedTheta(facingTurn)
      targetHeading = MathUtil.normalisedTheta(vectorToTarget.heading())

      if useAvoidance:
         vectorToTarget = self.calculateDestinationViaObstacles(vectorToTarget, extraObstacles)
       
      # avoidance doesn't change which way robot's facing  
      self.currentTarget = vectorToTarget
      forward = vectorToTarget.x
      left = vectorToTarget.y
       
      self.currentState.urgency = urgency
      # myPose = Global.myPose()
      # print "Pos: (%5.0f, %5.0f < %2.2f) - CurrTarget: (%5.0f, %5.0f < %2.2f) fac?%s, rel?%s, flt: (%4.0f, %4.0f < T%2.2f, F%2.2f)" % (
      #    myPose.x, myPose.y, degrees(myPose.theta), x, y, degrees(theta), "Y" if keepFacing else "N", "Y" if relative else "N", forward, left, degrees(targetHeading), degrees(facingTurn)
      # )
      self.currentState.tick(forward, left, targetHeading, facingTurn)


# Helper function to print the forward, left and turn if need be
def applyWalk(forward, left, turn, urgency):
    # print("forward: %.0f left: %.0f turn: %.2f" %(forward, left, turn))
  if urgency == 100 : # this is for circle strafe to not get affected by hardcoded urgency
    urgency = 1
  else :
    urgency = min(0.5, urgency)  # use 0 speed in lab environment, comment out for competition
  return actioncommand.walk(forward, left, turn, speed=urgency)

class Nearby(TaskState):

   def tick(self, forward, left, _, facingTurn):

      forward, left, facingTurn = limitWalk(forward, left, facingTurn)
      # print "Nearby walking towards: %.0f %.0f %.2f" % (forward, left, facingTurn)
      self.world.b_request.actions.body = applyWalk(forward, left, facingTurn, self.urgency)

class FarAway(TaskState):

   def tick(self, forward, left, heading, _):

      forward, left, heading = limitWalk(forward, left, heading)
      # print "Walking towards: %.0f %.0f %.2f" % (forward, left, heading)
      self.world.b_request.actions.body = applyWalk(forward, left, heading, self.urgency)

class Facing(TaskState):

   def tick(self, forward, left, _, facingTurn):

      forward, left, facingTurn = limitWalk(forward, left, facingTurn)
      # print "Turning towards: %.0f %.0f %.2f" % (forward, left, facingTurn)
      self.world.b_request.actions.body = applyWalk(forward, left, facingTurn, self.urgency)


class Initial(TaskState):

   def tick(self, *args, **kwargs):

      raise NotImplementedError("Should have changed states!")

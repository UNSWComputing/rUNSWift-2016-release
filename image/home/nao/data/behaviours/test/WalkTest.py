import actioncommand
from Task import BehaviourTask, TaskState
from skills.WalkToPointV2 import WalkToPointV2
from util.Timer import WallTimer

HALF_TURN = 0.44

# tests the limits and parameter combinations
class WalkTest(BehaviourTask):
    
   def init(self):
      self.stateTimer = WallTimer(10 * 1000000)
      self.w2p = WalkToPointV2(self.world)
      self.current_state = Stand(self, timeToRun=10, initMessage="Crouched and ready to pounce.")


class TimedTask(TaskState):

   def init(self, timeToRun=0, initMessage=""):
      self.timer = WallTimer(timeToRun*1000000)
      #robot.say(initMessage)
      self.w2p = WalkToPointV2(self.world)

   def transition(self):
      return self


class Stand(TimedTask):

   def tick(self):
      self.world.b_request.actions.body = actioncommand.crouch()

   def transition(self):
      if self.timer.finished():
         return WalkForwardFast(self.parent, timeToRun=5, initMessage="Roadrunner")
      else:
         return self

class WalkForwardFast(TimedTask):

   def tick(self):
      self.world.b_request.actions.body = actioncommand.walk(500, 0, 0)

   def transition(self):
      if self.timer.finished():
         return Back(self, timeToRun=5, initMessage="You spin my world right round.")
      else:
         return self

class Back(TimedTask):
   
   def tick(self):
      self.world.b_request.actions.body = actioncommand.walk(-500, 0, 0)

   def transition(self):
      if self.timer.finished():
         return Left(self, timeToRun=5, initMessage="You spin my world right round.")
      else:
         return self
         
class Left(TimedTask):
   
   def tick(self):
      self.world.b_request.actions.body = actioncommand.walk(0, 500, 0)

   def transition(self):
      if self.timer.finished():
         return Right(self, timeToRun=5, initMessage="You spin my world right round.")
      else:
         return self
      
class Right(TimedTask):
      
   def tick(self):
      self.world.b_request.actions.body = actioncommand.walk(0, -500, 0)

   def transition(self):
      if self.timer.finished():
         return Turn(self, timeToRun=5, initMessage="You spin my world right round.")
      else:
         return self
         
class Turn(TimedTask):
   
   def tick(self):
      self.world.b_request.actions.body = actioncommand.walk(0, 0, 100)

   def transition(self):
      if self.timer.finished():
         return TurnRight(self, timeToRun=5, initMessage="You spin my world right round.")
      else:
         return self
   
class TurnRight(TimedTask):
   
   def tick(self):
      self.world.b_request.actions.body = actioncommand.walk(0, 0, -100)

   def transition(self):
      if self.timer.finished():
         return ForwardLeft(self, timeToRun=5, initMessage="You spin my world right round.")
      else:
         return self
      
class ForwardLeft(TimedTask):
   
   def tick(self):
      self.world.b_request.actions.body = actioncommand.walk(500, 500, 0)

   def transition(self):
      if self.timer.finished():
         return BackLeft(self, timeToRun=5, initMessage="You spin my world right round.")
      else:
         return self

class BackLeft(TimedTask):
   
   def tick(self):
      self.world.b_request.actions.body = actioncommand.walk(-500, 500, 0)

   def transition(self):
      if self.timer.finished():
         return TurnForward(self, timeToRun=5, initMessage="You spin my world right round.")
      else:
         return self
      
class TurnForward(TimedTask):
   
   def tick(self):
      self.world.b_request.actions.body = actioncommand.walk(500, 0, HALF_TURN)

   def transition(self):
      if self.timer.finished():
         return TurnBack(self, timeToRun=5, initMessage="You spin my world right round.")
      else:
         return self
      
class TurnBack(TimedTask):
   
   def tick(self):
      self.world.b_request.actions.body = actioncommand.walk(-500, 0, HALF_TURN)

   def transition(self):
      if self.timer.finished():
         return TurnLeft(self, timeToRun=5, initMessage="You spin my world right round.")
      else:
         return self
      
class TurnLeft(TimedTask):
   
   def tick(self):
      self.world.b_request.actions.body = actioncommand.walk(0, 500, HALF_TURN)

   def transition(self):
      if self.timer.finished():
         return TurnForwardLeft(self, timeToRun=5, initMessage="You spin my world right round.")
      else:
         return self
      
class TurnForwardLeft(TimedTask):
   
   def tick(self):
      self.world.b_request.actions.body = actioncommand.walk(500, 500, HALF_TURN)

   def transition(self):
      if self.timer.finished():
         return TurnBackLeft(self, timeToRun=5, initMessage="You spin my world right round.")
      else:
         return self   

class TurnBackLeft(TimedTask):
   
   def tick(self):
      self.world.b_request.actions.body = actioncommand.walk(-500, 500, HALF_TURN)

   def transition(self):
      if self.timer.finished():
         return Stand(self, timeToRun=5, initMessage="You spin my world right round.")
      else:
         return self   

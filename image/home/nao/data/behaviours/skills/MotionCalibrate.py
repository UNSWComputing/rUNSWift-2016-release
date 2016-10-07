import actioncommand
import robot

class MotionCalibrate(object):
   def __init__(self, blackboard, parent):
      self.parent = parent
   def tick(self, blackboard):
      body = actioncommand.motionCalibrate()
      head = actioncommand.head(0, 0, False, 1, 1)
      leds = actioncommand.leds()
      req = robot.BehaviourRequest()
      req.actions = actioncommand.compose(head, body, leds)
      return req

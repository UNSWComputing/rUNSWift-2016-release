import actioncommand
import robot

class StandStraight(object):
   def __init__(self, blackboard, parent):
      self.parent = parent
   def tick(self, blackboard):
      body = actioncommand.standStraight()
      head = actioncommand.head(0, 0, False, 1, 1)
      leds = actioncommand.leds()
      req = robot.BehaviourRequest()
      req.actions = actioncommand.compose(head, body, leds)
      return req

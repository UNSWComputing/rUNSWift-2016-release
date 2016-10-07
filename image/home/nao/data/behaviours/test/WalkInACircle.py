import Global
from Task import BehaviourTask

from skills.WalkToPointV2 import WalkToPointV2

class WalkInACircle(BehaviourTask):
   def init(self):
      self.walk2Point = WalkToPointV2(self.world)

   def tick(self):

      self.walk2Point.tick(Global.ballDistance() - 50, -130, Global.ballHeading(), relative=True)

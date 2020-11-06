#!/usr/bin/env python

from __future__ import print_function

import rospy
import sys
from ObservationService.srv import Observation


def getObservation(state, action):
  rospy.wait_for_service('Observations')
  try:
    obs = rospy.ServiceProxy('Observations', Observation)
    print("reached here")
    response = obs(state,action)
    return response.observation
  except Exception as e:
    print(e)

if __name__ == '__main__':
  if (len(sys.argv) == 3):
    state = int(sys.argv[1])
    action = int(sys.argv[2])
  else:
    print("wrong params")
    sys.exit()
  print("Observation", getObservation(state, action))
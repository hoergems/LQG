import numpy as np
import random
from ompl import base as ob
from ompl import geometric as og

class GoalRegion(ob.GoalSampleableRegion):
    def __init__(self, si, goal_states):
        super(GoalRegion, self).__init__(si)
        self.si = si
        self.goal_states = goal_states        
        
    def sampleGoal(self, state):
        random_goal_state = random.choice(self.goal_states)
        for i in xrange(self.si.getStateSpace().getDimension()):
            state[i] = random_goal_state[i]
        
    def maxSampleCount(self):
        return len(self.goal_states)
    
    def canSample(self):
        return True
    
    def couldSample(self):
        return True
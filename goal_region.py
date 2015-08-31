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
            
    def getRandomGoalState(self):
        return random.choice(self.goal_states) 
        
    def maxSampleCount(self):
        return len(self.goal_states)
    
    def canSample(self):
        if len(self.goal_states) > 0:
            return True
        return False
    
    def couldSample(self):
        return True
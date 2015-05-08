import numpy as np
from kinematics import Kinematics
from ompl import base as ob
from ompl import geometric as og

class GoalRegion(ob.GoalSampleableRegion):
    def __init__(self, si):
        super(GoalRegion, self).__init__(si)
        self.si = si
        self.kinematics = Kinematics()
        
    def set_goal_radius(self, radius):
        self.goal_radius = radius
        
    def set_goal_position(self, cartesian_coords):
        """
        The goal position of the end effector
        """
        self.goal_position = cartesian_coords
        
    def set_bounds(self, bounds):
        self.bounds = bounds
        
    def sampleGoal(self, state):
        while True:
            sampled_state = self._sample_joint_state()            
            end_effector_position = self.kinematics.get_end_effector_position(sampled_state)
            vec = end_effector_position - self.goal_position
            vec_norm = vec / np.linalg.norm(vec)
            vec_radius = vec_norm * self.goal_radius            
            if np.linalg.norm(vec) <= np.linalg.norm(vec_radius):                          
                for i in xrange(self.si.getStateSpace().getDimension()):
                    state[i] = sampled_state[i]                
                return sampled_state        
        
    def _sample_joint_state(self):        
        while True:
            state = ob.State(self.si.getStateSpace())
            for i in xrange(self.si.getStateSpace().getDimension()):
                state[i] = np.random.uniform(-np.pi, np.pi)
            if self._satisfies_bounds(state):
                return state
            
    def _satisfies_bounds(self, state):
        for i in xrange(self.si.getStateSpace().getDimension()):
            if state[i] < self.bounds[i][0] or state[i] > self.bounds[i][1]:
                return False
        return True       
        
        
    def maxSampleCount(self):
        return 1000
    
    def canSample(self):
        return True
    
    def couldSample(self):
        return True
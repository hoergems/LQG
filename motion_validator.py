import numpy as np
from ompl import base as ob
from ompl import geometric as og

class MotionValidator(ob.MotionValidator):
    def __init__(self, si=None): 
        self.max_dist = 2.0 / 30.0       
        if not si == None:   
            self.si = si         
            super(MotionValidator, self).__init__(si)
            
    def set_max_distance(self, max_velocity, delta_t):
        self.max_dist = delta_t *  max_velocity
            
    def checkMotion(self, s1, s2):
        """
        Checks if a motion is valid
        """        
        if not self._is_valid(s2):
            return False
        dist = self._dist(s1, s2)
        if  dist > self.max_dist:
            return False        
        return True
    
    def _is_valid(self, state):
        """
        Checks if a state is valid
        """
        return self.si.getStateSpace().satisfiesBounds(state)        
        
    def _dist(self, s1, s2):
        """
        Computes the distance between two states
        """
        return self.si.distance(s1, s2)
        
        
        
if __name__ == "__main__":
    MotionValidator()
import numpy as np
import time
from ompl import base as ob
from ompl import geometric as og
from kinematics import Kinematics
from obstacle import Obstacle


class MotionValidator(ob.MotionValidator):
    def __init__(self, si=None): 
        #self.max_dist = 2.0 / 30.0        
        self.kinematics = Kinematics(si.getStateSpace().getDimension())       
        if not si == None:   
            self.si = si         
            super(MotionValidator, self).__init__(si)
            
    def set_max_distance(self, max_velocity, delta_t):
        self.max_dist = delta_t *  max_velocity
        
    def set_obstacles(self, obstacles):
        obstacle_list = []
        if not obstacles == None:
            for o in obstacles:
                obstacle_list.append(Obstacle(o[0], o[1], o[2], o[3]))
        
        self.obstacles = obstacle_list                
            
    def checkMotion(self, s1, s2):
        """
        Checks if a motion is valid
        """        
        if not self._is_valid(s2):
            return False
        '''dist = self._dist(s1, s2)
        if  dist > self.max_dist:
            return False'''
        if self._in_collision(s2):
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
    
    def _in_collision(self, state):
        """
        Checks if a state collides with the obstacles
        """
        collides = False
        point = self.kinematics.get_end_effector_position(state)        
        for obstacle in self.obstacles:            
            if obstacle.collides(point):
                return True
        return False
        
        
        
if __name__ == "__main__":
    MotionValidator()
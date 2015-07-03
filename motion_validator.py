import numpy as np
import time
from ompl import base as ob
from ompl import geometric as og
from kinematics import Kinematics
from obstacle import Obstacle
from threading import Lock


class MotionValidator(ob.MotionValidator):
    def __init__(self, si=None): 
        #self.max_dist = 2.0 / 30.0   
        self.mutex = Lock()     
        self.kinematics = Kinematics(si.getStateSpace().getDimension())       
        if not si == None:   
            self.si = si         
            super(MotionValidator, self).__init__(si)
            
    def set_max_distance(self, max_velocity, delta_t):
        self.max_dist = np.sqrt(self.si.getStateSpace().getDimension() * np.square(delta_t * max_velocity))
        
    def set_obstacles(self, obstacles):
        '''obstacle_list = []
        if not obstacles == None:
            for o in obstacles:
                obstacle_list.append(Obstacle(o[0], o[1], o[2], o[3]))
        
        self.obstacles = obstacle_list'''
        self.obstacles = obstacles                
            
    def checkMotion(self, s1, s2):
        """
        Checks if a motion is valid
        """        
        with self.mutex:
            if not self._is_valid(s1) or not self._is_valid(s2):
                return False        
            return not self._in_collision(s1, s2)        
    
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
    
    def _in_collision(self, state1, state2):
        """
        Checks if a state collides with the obstacles
        """ 
        #state1 = [0.0, 0.0, 0.0]
        #state2 = [0.009645623885753837, -0.13843772823710768, -0.10163138662043647]
        
        for state in [state2]:
            p1 = self.kinematics.get_link_n_position(state, 1)            
            p2 = self.kinematics.get_link_n_position(state, 2)            
            p3 = self.kinematics.get_link_n_position(state, 3)                     
            for obstacle in self.obstacles:
                if obstacle.manipulator_collides([[np.array([0, 0]), p1], [p1, p2], [p2, p3]]):
                    return True        
        return False
        
        
        collides = False
        point1 = self.kinematics.get_end_effector_position(state1) 
        point2 = self.kinematics.get_end_effector_position(state2)        
        for obstacle in self.obstacles:            
            if obstacle.in_collision(point1, point2):
                return True
        return False
        
        
        
if __name__ == "__main__":
    MotionValidator()
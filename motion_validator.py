import numpy as np
import time
from ompl import base as ob
from ompl import geometric as og
from libkinematics import *
from libutil import *
from libobstacle import Obstacle
from threading import Lock


class MotionValidator(ob.MotionValidator):
    def __init__(self, si=None): 
        self.utils = Utils()  
        self.mutex = Lock()
        if not si == None:   
            self.si = si         
            super(MotionValidator, self).__init__(si)
            
    def set_workspace_dimension(self, workspace_dimension):
        axis = v2_int()
        ax1 = v_int()
        ax2 = v_int()
        ax1[:] = [0, 0, 1]
        if workspace_dimension == 2:
            ax2[:] = [0, 0, 1]            
        elif workspace_dimension == 3:
            ax2[:] = [0, 1, 0]
            
        axis[:] = [ax1, ax2, ax1]        
        self.kinematics = Kinematics()
        self.kinematics.setLinksAndAxis(self.link_dimensions, axis)
        
    def set_link_dimensions(self, link_dimensions):        
        self.link_dimensions = link_dimensions
            
    def set_max_distance(self, max_velocity, delta_t):
        self.max_dist = np.sqrt(self.si.getStateSpace().getDimension() * np.square(delta_t * max_velocity))
        
    def set_obstacles(self, obstacles):        
        self.obstacles = obstacles                
            
    def checkMotion(self, s1, s2):
        """
        Checks if a motion is valid
        """        
        with self.mutex:                        
            if not self._satisfies_bounds(s2):                              
                return False                 
            return not self._in_collision(s1, s2)    
    
    def isValid(self, state):
        """
        Checks if a state is valid
        """
        if not self._satisfies_bounds(state):
            return False
        if self._in_collision(None, state):
            return False
        return True  
                
        
    def _dist(self, s1, s2):
        """
        Computes the distance between two states
        """
        return self.si.distance(s1, s2)
    
    def _satisfies_bounds(self, state):
        bounds = self.si.getStateSpace().getBounds()
        for i in xrange(self.si.getStateSpace().getDimension()):            
            if (state[i] < bounds.low[0] or
                state[i] > bounds.high[0]):                              
                return False
        return True
    
    def _in_collision(self, state1, state2):
        """
        Checks if a state collides with the obstacles
        """ 
        joint_angles2 = v_double()
        joint_angles2[:] = [state2[i] for i in xrange(self.si.getStateSpace().getDimension())]
        collision_structures_goal = self.utils.createManipulatorCollisionStructures(joint_angles2, 
                                                                                    self.link_dimensions, 
                                                                                    self.kinematics)
                
        for obstacle in self.obstacles:
            if obstacle.inCollisionDiscrete(collision_structures_goal):
                return True            
        if state1 == None:
            return False
            
        joint_angles1 = v_double()
        joint_angles1[:] = [state1[i] for i in xrange(self.si.getStateSpace().getDimension())]        
        
        manipulator_collision_objects_start = self.utils.createManipulatorCollisionObjects(joint_angles1,
                                                                                           self.link_dimensions, 
                                                                                           self.kinematics)
        manipulator_collision_objects_goal = self.utils.createManipulatorCollisionObjects(joint_angles2,
                                                                                          self.link_dimensions, 
                                                                                          self.kinematics)
        for obstacle in self.obstacles:
            for i in xrange(len(manipulator_collision_objects_start)):
                if obstacle.inCollisionContinuous([manipulator_collision_objects_start[i],
                                                   manipulator_collision_objects_goal[i]]):
                    return True            
        return False
        
        
if __name__ == "__main__":
    MotionValidator()
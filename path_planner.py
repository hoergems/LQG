import numpy as np
import plot as Plot
import copy
import time
import logging
from ompl import base as ob
from ompl.util import noOutputHandler, restorePreviousOutputHandler
from ompl import geometric as og
from motion_validator import MotionValidator
from goal_region import GoalRegion

class PathPlanner:
    def __init__(self):
        pass     
    
    def set_params(self, 
                   space_dimension, 
                   workspace_dimension,
                   max_velocity, 
                   delta_t, 
                   use_linear_path, 
                   sim_run, 
                   joint_constraints,                   
                   goal_states=[]):        
        self.space_dimension = space_dimension
        self.workspace_dimension = workspace_dimension        
        self.max_velocity = max_velocity
        self.delta_t = delta_t 
        self.use_linear_path = use_linear_path
        self.sim_run = sim_run
        self.joint_constraints = joint_constraints
        self.goal_states = goal_states              
        if logging.getLogger().getEffectiveLevel() == logging.WARN:
            #noOutputHandler()
            pass
        
    def set_start_state(self, start_state):
        self.start_state = ob.State(self.si.getStateSpace())
        for i in xrange(self.si.getStateSpace().getDimension()):
            self.start_state[i] = start_state[i]
            
    def plan_path(self, goal_state=None):
        self.problem_definition.clearSolutionPaths()
        path_collides = True 
        if self.use_linear_path and not goal_state == None:       
            path = self.linear_path(self.start_state, goal_state)
            path_collides = self.path_collides(path)
        if path_collides:
            if not self.motion_validator.isValid(self.start_state):
                logging.warn("PathPlanner: Start or goal state not valid. Skipping")
                return [], [], []            
            if len(self.goal_states) == 0:                
                gs = ob.State(self.si.getStateSpace())
                for i in xrange(self.si.getStateSpace().getDimension()):
                    gs[i] = goal_state[i]
                self.problem_definition.setStartAndGoalStates(self.start_state, gs)
            else:
                gs = []
                for goal_state in self.goal_states:
                    if self.motion_validator.isValid(goal_state):
                        gs.append(goal_state)
                if len(gs) == 0:
                    return [], [], []       
                self.problem_definition.addStartState(self.start_state)
                self.problem_definition.setGoal(GoalRegion(self.si, gs))
            self.planner = og.RRTConnect(self.si)            
            self.planner.setRange(np.sqrt(self.si.getStateSpace().getDimension() * np.square(self.delta_t * self.max_velocity)))        
            self.planner.setProblemDefinition(self.problem_definition)
            self.planner.setup()
            
            
            start_time = time.time()            
            while not self.problem_definition.hasSolution():
                print "range " + str(self.planner.getRange())                
                self.planner.solve(10.0)
                delta = time.time() - start_time
                '''if delta > 2.0:
                    logging.warn("PathPlanner: Maximum allowed planning time exceeded. Aborting")
                    xs = []
                    us = []
                    zs = []
                    return xs, us, zs''' 
            logging.info("PathPlanner: Finished planning. Planning time was " + 
                         str(time.time() - start_time) +
                         " seconds")        
            path = []
                
            if self.problem_definition.hasSolution():                                
                solution_path = self.problem_definition.getSolutionPath()
                states = solution_path.getStates()                          
                path = [np.array([state[i] for i in xrange(self.space.getDimension())]) for state in states]       
        return self._augment_path(path)
    
    
    
    def path_collides(self, path):
        for i in xrange(1, len(path)):
            if self.motion_validator._in_collision(path[i-1], path[i]):
                return True
        return False
    
    def linear_path(self, start, goal):
        path = []        
        max_dist = np.sqrt(self.si.getStateSpace().getDimension() * np.square(self.delta_t * self.max_velocity))           
        s = []
        g = []
        for i in xrange(self.space.getDimension()):
            s.append(start[i])
            g.append(goal[i])        
        start = np.array(s)
        path.append(start)
        goal = np.array(g)
        vec = goal - start        
        vec_length = np.linalg.norm(vec)
        vec_norm = vec / vec_length
        steps = vec_length / max_dist
        steps_full = np.floor(steps)
        steps_half = steps - steps_full        
        
        for i in xrange(int(steps_full)):
            new_state = path[-1] + max_dist * vec_norm
            path.append(new_state)
        if steps_half > 1.0e-10:           
            path.append(goal)
        return path
        
    def setup_ompl(self):
        self.space = ob.RealVectorStateSpace(dim=self.space_dimension)
        bounds = ob.RealVectorBounds(self.space_dimension)        
        for i in xrange(self.space_dimension):
            bounds.setLow(i, self.joint_constraints[0])
            bounds.setHigh(i, self.joint_constraints[1])
        self.space.setBounds(bounds)
        self.si = ob.SpaceInformation(self.space)
        self.motion_validator = MotionValidator(self.si)
        self.motion_validator.set_workspace_dimension(self.workspace_dimension) 
        self.motion_validator.set_max_distance(self.max_velocity, self.delta_t) 
        #self.si.setStateValidityChecker(self.motion_validator.isValid)      
        self.si.setMotionValidator(self.motion_validator)
        self.si.setup()
        self.problem_definition = ob.ProblemDefinition(self.si)        
        
    def set_obstacles(self, obstacles):        
        self.motion_validator.set_obstacles(obstacles)        

    def _augment_path(self, path):
        """
        Augments the path with controls and observations
        """    
        new_path = []             
        for i in xrange(len(path) - 1):
            u = (np.array(path[i + 1]) - np.array(path[i])) / self.delta_t            
            #new_path.append([path[i], [u[j] for j in xrange(len(u))], path[i]])            
            new_path.append([path[i], u, path[i]])
        new_path.append([path[-1], np.array([0.0 for i in xrange(self.si.getStateSpace().getDimension())]), path[-1]])
        xs = [new_path[i][0] for i in xrange(len(path))]
        us = [new_path[i][1] for i in xrange(len(path))]
        zs = [new_path[i][2] for i in xrange(len(path))]
        #print "xs" + str(xs)
        return xs, us, zs
               
        
        
        
if __name__ == "__main__":
    PathPlanner()
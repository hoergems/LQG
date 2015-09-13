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
                   link_dimensions, 
                   workspace_dimension,
                   max_velocity, 
                   delta_t, 
                   use_linear_path, 
                   sim_run, 
                   joint_constraints):
        self.link_dimensions = link_dimensions        
        self.space_dimension = len(link_dimensions)
        self.workspace_dimension = workspace_dimension        
        self.max_velocity = max_velocity
        self.delta_t = delta_t 
        self.use_linear_path = use_linear_path
        self.sim_run = sim_run
        self.joint_constraints = joint_constraints                     
        if logging.getLogger().getEffectiveLevel() == logging.WARN:
            noOutputHandler()
            
        
    def set_start_state(self, start_state):
        self.start_state = ob.State(self.si.getStateSpace())
        for i in xrange(self.si.getStateSpace().getDimension()):
            self.start_state[i] = start_state[i]
            
    def set_goal_state(self, goal):
        self.problem_definition.clearGoal()
        try:
            iterator = iter(goal)
        except TypeError:
            """
            Only one goal state
            """            
            if self.motion_validator.isValid(goal):
                self.problem_definition.setGoal(GoalRegion(self.si, goal))
        else:
            """
            A goal area defined by several goal states
            """    
            gs = []
            for g in goal:
                if self.motion_validator.isValid(g):
                    gs.append(g)
            self.problem_definition.setGoal(GoalRegion(self.si, gs))
        
            
    def plan_path(self):
        self.problem_definition.clearSolutionPaths()
        path_collides = True        
        if (not self.motion_validator.isValid(self.start_state) or 
            not self.problem_definition.getGoal().canSample()):
            return [], [], []
                     
        if self.use_linear_path:       
            path = self.linear_path(self.start_state, self.problem_definition.getGoal().getRandomGoalState())
            path_collides = self.path_collides(path)
        if path_collides:            
            self.problem_definition.addStartState(self.start_state)
            self.planner = og.RRTConnect(self.si)            
            self.planner.setRange(np.sqrt(self.si.getStateSpace().getDimension() * np.square(self.delta_t * self.max_velocity)))        
            self.planner.setProblemDefinition(self.problem_definition)
            self.planner.setup()           
            
            start_time = time.time()            
            while not self.problem_definition.hasSolution():                             
                self.planner.solve(1.0)
                delta = time.time() - start_time
                if delta > 3.0:
                    #logging.warn("PathPlanner: Maximum allowed planning time exceeded. Aborting")
                    xs = []
                    us = []
                    zs = []
                    return xs, us, zs
            path = []
            if self.problem_definition.hasSolution():
                logging.info("PathPlanner: Solution found after " +str(time.time() - start_time) + " s")                             
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
        print "set space"          
        self.space = ob.RealVectorStateSpace(dim=self.space_dimension)
        bounds = ob.RealVectorBounds(self.space_dimension) 
        print "set bounds"              
        for i in xrange(self.space_dimension):
            bounds.setLow(i, self.joint_constraints[0])
            bounds.setHigh(i, self.joint_constraints[1])
        print "set bounds 2"
        self.space.setBounds(bounds)
        print "set si"
        self.si = ob.SpaceInformation(self.space)
        print "set motion validator"
        self.motion_validator = MotionValidator(self.si)
        self.motion_validator.set_link_dimensions(self.link_dimensions)
        self.motion_validator.set_workspace_dimension(self.workspace_dimension) 
        self.motion_validator.set_max_distance(self.max_velocity, self.delta_t) 
        print "set state validiy checker"
        self.si.setStateValidityChecker(ob.StateValidityCheckerFn(self.motion_validator.isValid))
        print "set motion validator"      
        self.si.setMotionValidator(self.motion_validator)
        print "si.setup"
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
        return xs, us, zs
               
        
        
        
if __name__ == "__main__":
    PathPlanner()
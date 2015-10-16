from path_planner import PathPlanner
from path_evaluator import *
from multiprocessing import Process, cpu_count, Lock, Queue

import libpath_planner
import util
import time
import kin
import collections
import numpy as np


class PathPlanningInterface:
    def __init__(self):
        self.path_evaluator = PathEvaluator()
        self.mutex = Lock()
        pass
    
    def setup_path_evaluator(self, A, B, C, D, H, M, N, V, W, 
                             link_dimensions, 
                             workspace_dimension, 
                             sample_size, 
                             obstacles,
                             joint_constraints,
                             enforce_constraints,
                             w1, 
                             w2):
        self.path_evaluator.setup(A, B, C, D, H, M, N, V, W, 
                                  link_dimensions, 
                                  workspace_dimension, 
                                  sample_size, 
                                  obstacles,
                                  joint_constraints,
                                  enforce_constraints,
                                  w1, 
                                  w2)
    
    def setup(self, 
              link_dimensions, 
              workspace_dimension,
              obstacles, 
              max_velocity, 
              delta_t, 
              use_linear_path, 
              joint_constraints,
              enforce_constraints,
              planning_algorithm):
        self.link_dimensions = link_dimensions
        self.workspace_dimension = workspace_dimension        
        self.num_cores = cpu_count()
        #self.num_cores = 2         
        self.obstacles = obstacles        
        self.max_velocity = max_velocity
        self.delta_t = delta_t
        self.use_linear_path = use_linear_path
        self.joint_constraints = joint_constraints
        self.enforce_constraints = enforce_constraints
        
        self.joint_constraints_vec = v_double()
        self.joint_constraints_vec[:] = self.joint_constraints
        
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
        self.verbose = False 
        if(logging.getLogger().isEnabledFor(logging.INFO)):
            self.verbose = True
        self.planning_algorithm = planning_algorithm
               
        
    def set_start_and_goal(self, start_state, goal_states):        
        self.start_state = start_state
        self.goal_states = goal_states
        return True
        
    def plan_and_evaluate_paths(self, num, sim_run, horizon, P_t, timeout):
        jobs = collections.deque()
        path_queue = Queue()
        evaluated_paths = []
        num_paths = 0
        timeout_reached = False
        t0 = time.time()        
        while True: 
            while len(jobs) != self.num_cores - 1:           
                p = Process(target=self.construct_and_evaluate_path, args=(self.obstacles, path_queue, self.joint_constraints, horizon, P_t,))
                #p.daemon = True
                p.start()                       
                jobs.append(p)
                p.join()
            if len(jobs) == self.num_cores - 1:                
                while not path_queue.qsize() == self.num_cores - 1:                                        
                    elapsed = time.time() - t0                                       
                    if timeout > 0.0 and elapsed > timeout:
                        timeout_reached = True
                        for job in jobs:
                            job.terminate()
                        break                                             
                jobs.clear()
                print "qsize " + str(path_queue.qsize())
                while path_queue.qsize():
                    try: 
                        print "getting " + str(path_queue.qsize())                                          
                        p_e = path_queue.get(True, 0.01)
                        print "got"                                                                    
                        if not len(p_e[0]) == 0:
                            if num > 0:
                                if not len(evaluated_paths) == num:                         
                                    evaluated_paths.append(([[p_e[0][k].tolist() for k in xrange(len(p_e[0]))], 
                                                             [p_e[1][k].tolist() for k in xrange(len(p_e[0]))], 
                                                             [p_e[2][k].tolist() for k in xrange(len(p_e[0]))]], p_e[3]))
                            else:
                                evaluated_paths.append(([[p_e[0][k].tolist() for k in xrange(len(p_e[0]))], 
                                                         [p_e[1][k].tolist() for k in xrange(len(p_e[0]))], 
                                                         [p_e[2][k].tolist() for k in xrange(len(p_e[0]))]], p_e[3]))
                        #path_queue.task_done()                        
                    except Exception as e:
                        logging.warning("PathPlanningInterface: Error while getting element from path queue. Cancelling.") 
                        print e                         
                        path_queue = Queue()                        
                        break
                
                if timeout_reached:
                    break
                if num > 0 and len(evaluated_paths) == num:
                    break
                
        if len(evaluated_paths) == 0:
            logging.error("PathPlanningInterface: Couldn't generate and evaluate any paths within the given planning time")
            return [], [], [], 0.0     
        best_val = evaluated_paths[0][1]
        best_path = evaluated_paths[0][0]        
        for i in xrange(1, len(evaluated_paths)):
            if evaluated_paths[i][1] < best_val:
                best_val = evaluated_paths[i][1]
                best_path = evaluated_paths[i][0]                              
        return best_path[0], best_path[1], best_path[2], len(evaluated_paths), best_val
            
        
    def plan_paths(self, num, sim_run, timeout=0.0):          
        jobs = collections.deque()        
        path_queue = Queue()
        paths = []  
        #return self.construct_path(self.obstacles, path_queue, sim_run, self.joint_constraints)      
        for i in xrange(num):            
            p = Process(target=self.construct_path, args=(self.obstacles, path_queue, self.joint_constraints,))
            p.start()            
            jobs.append(p)
            if len(jobs) == self.num_cores - 1 or i == num - 1:
                if i == num - 1 and not len(jobs) == self.num_cores - 1:
                    while not path_queue.qsize() == num % (self.num_cores - 1):
                        time.sleep(0.00001)
                else:
                    while not path_queue.qsize() == self.num_cores - 1:
                        time.sleep(0.00001)
                jobs.clear()
                q_size = path_queue.qsize()
                for j in xrange(q_size):
                    p_e = path_queue.get()                    
                    if not len(p_e[0]) == 0:                         
                        paths.append([[p_e[0][i].tolist() for i in xrange(len(p_e[0]))], 
                                      [p_e[1][i].tolist() for i in xrange(len(p_e[0]))], 
                                      [p_e[2][i].tolist() for i in xrange(len(p_e[0]))]]) 
               
        return paths
    
    def construct_and_evaluate_path(self, obstacles, queue, joint_constraints, horizon, P_t):
        print "constructingggg"              
        xs, us, zs = self._construct(obstacles, joint_constraints)
        print "constructed"
        eval_result = self.path_evaluator.evaluate_path([xs, us, zs], P_t, horizon)
        print "evaluated"  
        queue.put((xs, us, zs, eval_result[1]), True, 0.01)
        print "put"
    
    def construct_path(self, obstacles, queue, joint_constraints,):
        xs, us, zs = self._construct(obstacles, joint_constraints) 
        queue.put((xs, us, zs))        
        
    def _construct(self, obstacles, joint_constraints):        
        path_planner2 = libpath_planner.PathPlanner(self.kinematics,
                                                    len(self.link_dimensions),
                                                    self.delta_t,
                                                    True,
                                                    self.max_velocity,
                                                    self.joint_constraints_vec,
                                                    self.enforce_constraints,                                                    
                                                    1.0,
                                                    False,
                                                    self.use_linear_path,
                                                    self.verbose,
                                                    self.planning_algorithm)
        path_planner2.setup()
        path_planner2.setObstacles(obstacles)
        link_dimensions = util.v2_double()
        ld = []       
        for i in xrange(len(self.link_dimensions)):
            link_dim = util.v_double()            
            link_dim[:] = [self.link_dimensions[i][j] for j in xrange(len(self.link_dimensions[i]))]
            ld.append(link_dim)
        link_dimensions[:] = ld        
        path_planner2.setLinkDimensions(link_dimensions)         
        goal_states = util.v2_double()
        gs = []               
        for i in xrange(len(self.goal_states)):
            goal_state = util.v_double()                  
            goal_state[:] = [self.goal_states[i][j] for j in xrange(len(self.goal_states[i]))]            
            if path_planner2.isValid(goal_state):                
                gs.append(goal_state)            
        if len(gs) == 0:
            return [], [], []               
        goal_states[:] = gs        
        path_planner2.setGoalStates(goal_states)        
        
        start_state = util.v_double()
        v = [self.start_state[i] for i in xrange(len(self.start_state))]
        start_state[:] = v  
        xs_temp = path_planner2.solve(start_state)
        state_path = []
        for i in xrange(len(xs_temp)):
            xs_elem = []
            for j in xrange(len(xs_temp[i])):
                xs_elem.append(xs_temp[i][j])
            state_path.append(xs_elem)    
        xs, us, zs = self._augment_path(state_path)
        return xs, us, zs
        
    
    def _augment_path(self, path):
        """
        Augments the path with controls and observations
        """    
        
        new_path = []                   
        for i in xrange(len(path) - 1):
            u = (np.array(path[i + 1]) - np.array(path[i])) / self.delta_t
            new_path.append([path[i], u, path[i]])
        new_path.append([path[-1], np.array([0.0 for i in xrange(len(self.link_dimensions))]), path[-1]])
        xs = [np.array(new_path[i][0]) for i in xrange(len(path))]
        us = [np.array(new_path[i][1]) for i in xrange(len(path))]
        zs = [np.array(new_path[i][2]) for i in xrange(len(path))]        
        return xs, us, zs  
    
if __name__ == "__main__":
    PathPlanningInterface()
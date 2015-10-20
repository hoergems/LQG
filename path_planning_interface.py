from path_planner import PathPlanner
from path_evaluator import *
from multiprocessing import Process, cpu_count, Lock, Queue
from Queue import Empty, Full
import libpath_planner
import util
import time
import kin
import collections
import numpy as np
import scipy.stats


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
        path_queue = Queue()
        evaluated_paths = []
        gen_times = []
        eval_times = []
        res_paths = collections.deque()
        processes = [Process(target=self.construct_and_evaluate_path, 
                             args=(self.obstacles, path_queue, 
                                   self.joint_constraints, 
                                   horizon, 
                                   P_t,)) for i in xrange(self.num_cores - 1)]
        t0 = time.time()
        print "Path planning interface: " + str(len(processes)) + " processes started"
        for i in xrange(len(processes)):
            processes[i].daemon = True
            processes[i].start()
        while True:
            try:
                res_paths.append(path_queue.get_nowait())
            except:
                pass
            elapsed = time.time() - t0
            if num != 0 and len(res_paths) == num:
                break
            if timeout > 0.0 and elapsed > timeout:
                break
            time.sleep(0.000001)
        for i in xrange(len(processes)):
            processes[i].terminate()
        for i in xrange(len(res_paths)):
            p_e = res_paths.pop()            
            if not len(p_e[0]) == 0:
                gen_times.append(p_e[4])
                eval_times.append(p_e[5])
                evaluated_paths.append(([[p_e[0][k].tolist() for k in xrange(len(p_e[0]))], 
                                         [p_e[1][k].tolist() for k in xrange(len(p_e[0]))], 
                                         [p_e[2][k].tolist() for k in xrange(len(p_e[0]))]], p_e[3]))
        if len(evaluated_paths) == 0:
            logging.error("PathPlanningInterface: Couldn't generate and evaluate any paths within the given planning time")
            return [], [], [], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0    
        best_val = evaluated_paths[0][1]
        best_path = evaluated_paths[0][0]        
        for i in xrange(1, len(evaluated_paths)):
            if evaluated_paths[i][1] < best_val:
                best_val = evaluated_paths[i][1]
                best_path = evaluated_paths[i][0]
        n, min_max, mean_gen_times, var, skew, kurt = scipy.stats.describe(np.array(gen_times))
        n, min_max, mean_eval_times, var, skew, kurt = scipy.stats.describe(np.array(eval_times))
        total_gen_times = sum(gen_times)
        total_eval_times = sum(eval_times)                              
        return (best_path[0], 
                best_path[1], 
                best_path[2], 
                len(evaluated_paths), 
                best_val, 
                mean_gen_times, 
                mean_eval_times,
                total_gen_times,
                total_eval_times)
        
    def plan_paths(self, num, sim_run, timeout=0.0):        
        path_queue = Queue()
        paths = []
        res_paths = collections.deque()
        processes = [Process(target=self.construct_path, 
                             args=(self.obstacles, 
                                   path_queue, 
                                   self.joint_constraints,)) for i in xrange(self.num_cores - 1)]
        t0 = time.time()
        for i in xrange(len(processes)):
            processes[i].daemon = True
            processes[i].start()
        while True:
            try:
                res_paths.append(path_queue.get_nowait())
            except:
                pass
            elapsed = time.time() - t0
            if len(res_paths) == num:
                break
            if timeout > 0.0:
                if elapsed > timeout:
                    break
            time.sleep(0.0001)
        for i in xrange(len(processes)):
            processes[i].terminate()        
        for i in xrange(len(res_paths)):
            p_e = res_paths.pop()                                
            if not len(p_e[0]) == 0:                         
                paths.append([[p_e[0][i].tolist() for i in xrange(len(p_e[0]))], 
                              [p_e[1][i].tolist() for i in xrange(len(p_e[0]))], 
                              [p_e[2][i].tolist() for i in xrange(len(p_e[0]))]])        
        return paths
    
    def construct_and_evaluate_path(self, obstacles, queue, joint_constraints, horizon, P_t):
        while True:
            t0 = time.time()               
            xs, us, zs = self._construct(obstacles, joint_constraints)
            gen_time = time.time() - t0
            if len(xs) > 1:  
                t0 = time.time()                      
                eval_result = self.path_evaluator.evaluate_path([xs, us, zs], P_t, horizon)
                eval_time = time.time() - t0        
                queue.put((xs, us, zs, eval_result[1], gen_time, eval_time))        
    
    def construct_path(self, obstacles, queue, joint_constraints,):
        while True:
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
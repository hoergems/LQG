from path_planner import PathPlanner
from path_evaluator import *
from multiprocessing import Process, Queue, JoinableQueue, cpu_count
import time
import collections
import numpy as np


class PathPlanningInterface:
    def __init__(self):
        self.path_evaluator = PathEvaluator()
        pass
    
    def setup_path_evaluator(self, A, B, C, D, H, M, N, V, W, 
                             link_dimensions, 
                             workspace_dimension, 
                             sample_size, 
                             obstacles,
                             w1, 
                             w2):
        self.path_evaluator.setup(A, B, C, D, H, M, N, V, W, 
                                  link_dimensions, 
                                  workspace_dimension, 
                                  sample_size, 
                                  obstacles,
                                  w1, 
                                  w2)
    
    def setup(self, 
              link_dimensions, 
              workspace_dimension,
              obstacles, 
              max_velocity, 
              delta_t, 
              use_linear_path, 
              joint_constraints):
        self.link_dimensions = link_dimensions
        self.workspace_dimension = workspace_dimension        
        self.num_cores = cpu_count()
        #self.num_cores = 2         
        self.obstacles = obstacles        
        self.max_velocity = max_velocity
        self.delta_t = delta_t
        self.use_linear_path = use_linear_path
        self.joint_constraints = joint_constraints
        
    def set_start_and_goal(self, start_state, goal_states):        
        self.start_state = start_state
        self.goal_states = goal_states
        
    def plan_and_evaluate_paths(self, num, sim_run, horizon, timeout):
        jobs = collections.deque()
        path_queue = Queue()
        evaluated_paths = []
        num_paths = 0
        timeout_reached = False
        t0 = time.time()
        i = 0
        while True:
            p = Process(target=self.construct_and_evaluate_path, args=(self.obstacles, path_queue, sim_run, self.joint_constraints, horizon,))
            p.daemon = True
            p.start()                       
            jobs.append(p)            
            i += 1            
            if len(jobs) == self.num_cores - 1:
                while not path_queue.qsize() == self.num_cores - 1:
                    elapsed = time.time() - t0                    
                    if timeout > 0.0 and elapsed > timeout:
                        timeout_reached = True
                        for job in jobs:
                            job.terminate()
                        break
                jobs.clear()
                while path_queue.qsize():
                    try:                        
                        p_e = path_queue.get_nowait()
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
                        #logging.warning("PathPlanningInterface: Error while getting element from path queue. Canceling.")                                              
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
        return best_path[0], best_path[1], best_path[2], len(evaluated_paths)
            
        
    def plan_paths(self, num, sim_run, timeout=0.0):          
        jobs = collections.deque()        
        path_queue = Queue()
        paths = []        
        for i in xrange(num):            
            p = Process(target=self.construct_path, args=(self.obstacles, path_queue, sim_run, self.joint_constraints,))
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
    
    def construct_path2(self, path_planner, queue):
        xs, us, zs = path_planner.plan_path()
        if len(xs) == 0:
            return        
        queue.put((xs, us, zs))
        return
    
    def construct_and_evaluate_path(self, obstacles, queue, sim_run, joint_constraints, horizon):
        path_planner = PathPlanner()        
        path_planner.set_params(self.link_dimensions,
                                self.workspace_dimension,                                 
                                self.max_velocity, 
                                self.delta_t, 
                                self.use_linear_path,
                                sim_run, 
                                joint_constraints)
        path_planner.setup_ompl()
        path_planner.set_obstacles(obstacles)
        path_planner.set_start_state(self.start_state) 
        path_planner.set_goal_state(self.goal_states)
        xs, us, zs = path_planner.plan_path()
        eval_result = self.path_evaluator.evaluate_path([xs, us, zs], horizon)
                           
        queue.put((xs, us, zs, eval_result[1]), block=False)
    
    def construct_path(self, obstacles, queue, sim_run, joint_constraints,):                
        path_planner = PathPlanner()        
        path_planner.set_params(self.link_dimensions,
                                self.workspace_dimension,                                 
                                self.max_velocity, 
                                self.delta_t, 
                                self.use_linear_path,
                                sim_run, 
                                joint_constraints)
        path_planner.setup_ompl()
        path_planner.set_obstacles(obstacles)
        path_planner.set_start_state(self.start_state) 
        path_planner.set_goal_state(self.goal_states)
        xs, us, zs = path_planner.plan_path()
        queue.put((xs, us, zs))        
        return   
    
if __name__ == "__main__":
    PathPlanningInterface()
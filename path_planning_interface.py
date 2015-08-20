from path_planner import PathPlanner
from multiprocessing import Process, Queue, cpu_count
import time
import collections
import numpy as np


class PathPlanningInterface:
    def __init__(self):
        pass
    
    def setup(self, 
              num_links, 
              workspace_dimension,
              obstacles, 
              max_velocity, 
              delta_t, 
              use_linear_path, 
              joint_constraints, 
              verbose):
        self.num_links = num_links
        self.workspace_dimension = workspace_dimension        
        self.num_cores = cpu_count()
        #self.num_cores = 2       
        self.obstacles = obstacles
        
        self.max_velocity = max_velocity
        self.delta_t = delta_t
        self.use_linear_path = use_linear_path
        self.joint_constraints = joint_constraints        
        self.verbose = verbose
        
    def set_start_and_goal(self, start_state, goal_states):        
        self.start_state = start_state
        self.goal_states = goal_states
        
    def plan_paths(self, num, sim_run, verbose):        
        jobs = collections.deque()        
        path_queue = Queue()
        paths = [] 
        print "Generating paths..."
        for i in xrange(num): 
            if self.verbose: 
                print "generating path " + str(i)
            p = Process(target=self.construct_path, args=(self.obstacles, path_queue, sim_run, self.joint_constraints, verbose,))
            #p = Process(target=self.construct_path2, args=(path_planner, path_queue,))
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
                    if not len(p_e) == 0:                         
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
    
    def construct_path(self, obstacles, queue, sim_run, joint_constraints, verbose):                
        path_planner = PathPlanner()
        path_planner.set_params(self.num_links,
                                self.workspace_dimension,                                 
                                self.max_velocity, 
                                self.delta_t, 
                                self.use_linear_path,
                                sim_run, 
                                joint_constraints,                               
                                verbose,
                                goal_states=self.goal_states)
        path_planner.setup_ompl()
        path_planner.set_start_state(self.start_state)        
        path_planner.set_obstacles(obstacles)             
        xs, us, zs = path_planner.plan_path()
        if len(xs) == 0:
            return        
        queue.put((xs, us, zs))
        return   
    
if __name__ == "__main__":
    PathPlanningInterface()
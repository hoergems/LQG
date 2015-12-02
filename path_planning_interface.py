from path_planner import PathPlanner
from path_evaluator import *
from multiprocessing import Process, cpu_count, Lock, Queue, Pool
import multiprocessing.queues
from Queue import Empty, Full
from librobot import v_string
import libpath_planner
import libdynamic_path_planner
import libutil
import time
import libkinematics
import collections
import numpy as np
import scipy.stats

class XQueue(multiprocessing.queues.Queue):
    def __init__(self, maxsize=0):
        self.objects = collections.deque()
        self.mutex = Lock()
        self._maxsize = maxsize        
        
    def empty(self):        
        return len(self.objects) == 0
    
    def put(self, obj, block=True, timeout=None):
        with self.mutex:
            self.objects.append(obj)
            print "Object put. size is now " + str(len(self.objects))
            
    def get(self, block=True, timeout=None):        
        if len(self.objects):
            raise Empty
        with self.mutex():
            return self.objects.popleft()
        
    def qsize(self):
        return len(self.objects)
        


class PathPlanningInterface:
    def __init__(self):
        self.path_evaluator = PathEvaluator()
    
    def setup_path_evaluator(self, A, B, C, D, H, M, N, V, W, 
                             link_dimensions, 
                             robot, 
                             sample_size, 
                             obstacles,
                             joint_constraints,
                             enforce_constraints,
                             goal_position,
                             goal_radius,
                             w1, 
                             w2):
        self.path_evaluator.setup(A, B, C, D, H, M, N, V, W, 
                                  link_dimensions, 
                                  robot, 
                                  sample_size, 
                                  obstacles,
                                  joint_constraints,
                                  enforce_constraints,
                                  goal_position,
                                  goal_radius,
                                  w1, 
                                  w2)
        
    def setup_reward_function(self, 
                              step_penalty, 
                              exit_reward, 
                              illegal_move_penalty, 
                              discount_factor):
        self.path_evaluator.setup_reward_function(step_penalty, illegal_move_penalty, exit_reward, discount_factor)
    
    def setup(self,              
              robot,
              obstacles, 
              max_velocity, 
              delta_t, 
              use_linear_path, 
              joint_constraints,
              enforce_constraints,
              planning_algorithm,
              path_timeout):
        
        self.link_dimensions = v2_double()
        robot.getActiveLinkDimensions(self.link_dimensions)
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
        
        self.kinematics = libkinematics.Kinematics()
        self.kinematics.setParams([robot]) 
              
        #self.kinematics.setLinksAndAxis(self.link_dimensions, axis)        
        self.verbose = False 
        if(logging.getLogger().isEnabledFor(logging.INFO)):
            self.verbose = True        
        self.planning_algorithm = planning_algorithm
        self.dynamic_problem = False
        self.path_timeout = path_timeout
        
    def setup_dynamic_problem(self, 
                              urdf_model,
                              environment_model, 
                              simulation_step_size,
                              coulomb, 
                              viscous,
                              continuous_collision):
        self.dynamic_problem = True
        self.model_file = urdf_model
        self.environment_file = environment_model
        self.simulation_step_size = simulation_step_size
        self.coulomb = coulomb
        self.viscous = viscous
        self.continuous_collision = continuous_collision
        
    def set_start_and_goal(self, start_state, goal_states, ee_goal_position, ee_goal_threshold):                
        self.start_state = start_state
        self.goal_states = goal_states
        self.ee_goal_position = ee_goal_position
        self.ee_goal_threshold = ee_goal_threshold        
        return True
        
    def plan_and_evaluate_paths(self, num, sim_run, current_step, horizon, P_t, timeout):        
        path_queue = Queue()
        evaluated_paths = []
        gen_times = []
        eval_times = []
        res_paths = collections.deque()
        processes = [Process(target=self.construct_and_evaluate_path, 
                             args=(self.obstacles, path_queue, 
                                   self.joint_constraints,
                                   current_step,
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
            time.sleep(0.001)
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
        
    def plan_paths2(self, num, sim_run, timeout=0.0):
        path_queue = Queue()
        pool = Pool(processes=self.num_cores - 1)
        result = pool.apply_async(self.construct_path(obstacles, queue, joint_constraints), args=(self.obstacles,
                                                                                                  path_queue,
                                                                                                  self.joint_constraints))
        
    def plan_paths(self, num, sim_run, timeout=0.0):
        queues = [Queue() for i in xrange(self.num_cores - 1)]               
        path_queue = Queue()
        paths = []
        res_paths = collections.deque()
        processes = [Process(target=self.construct_path, 
                             args=(self.obstacles, 
                                   queues[i], 
                                   self.joint_constraints)) for i in xrange(self.num_cores - 1)]
        t0 = time.time() 
        for i in xrange(len(processes)):
            processes[i].daemon = True
            processes[i].start()
        curr_len = 0
        while True:
            breaking = False
            for queue in queues:
                elapsed = time.time() - t0
                if queue.qsize() > 0:
                    try:
                        res_paths.append(queue.get(timeout=0.1))
                        print "GOT A PATH " + str(len(res_paths))
                    except:
                        pass
                    if len(res_paths) == num:
                        breaking = True
                        break
                    if timeout > 0.0:
                        if elapsed > timeout:
                            breaking = True
                            break
            if breaking:
                break
            '''try:  
                if path_queue.qsize() > 0 and not path_queue.empty():                              
                    t0 = time.time()                    
                    res_paths.append(path_queue.get(timeout=0.1))                    
                    te = time.time()
                    print str(len(res_paths)) + " paths generated " + str(te - t0)            
            except Empty:                
                #print "FUCK"                                       
                pass
            elapsed = time.time() - t0 
            if len(res_paths) != curr_len:
                curr_len = len(res_paths)                                   
            if len(res_paths) == num:
                break
            if timeout > 0.0:
                if elapsed > timeout:
                    break
            time.sleep(0.001)'''
        
        #path_queue.close()         
        for i in xrange(len(processes)):
            processes[i].terminate()
        '''for i in xrange(num):
            print "Getting " + str(i + 1)
            try:
                res_paths.append(path_queue.get(timeout=1.0))
            except Empty:
                print "what: " + str(len(res_paths))
                print "what2: " + str(path_queue.qsize()) '''       
        for i in xrange(len(res_paths)):
            p_e = res_paths.pop()                                
            if not len(p_e[0]) == 0:                                      
                paths.append([[p_e[0][i] for i in xrange(len(p_e[0]))], 
                              [p_e[1][i] for i in xrange(len(p_e[0]))], 
                              [p_e[2][i] for i in xrange(len(p_e[0]))]])           
        return paths
    
    def construct_and_evaluate_path(self, 
                                    obstacles, 
                                    queue, 
                                    joint_constraints,
                                    current_step, 
                                    horizon, 
                                    P_t):
        sleep
        while True:
            t0 = time.time()               
            xs, us, zs = self._construct(obstacles, joint_constraints)
            gen_time = time.time() - t0
            if len(xs) > 1:  
                t0 = time.time()                      
                eval_result = self.path_evaluator.evaluate_path([xs, us, zs], P_t, current_step, horizon)
                eval_time = time.time() - t0        
                queue.put((xs, us, zs, eval_result[1], gen_time, eval_time))        
    
    def construct_path(self, obstacles, queue, joint_constraints):        
        while True:
            xs, us, zs = self._construct(obstacles, joint_constraints)
            if not len(xs) == 0:                             
                queue.put((xs, us, zs))
                print "put. New size is " + str(queue.qsize())             
            else:
                print "PATH HAS SIZE 0!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
        
    def _construct(self, obstacles, joint_constraints):
        path_planner2 = None        
        if not self.dynamic_problem:
            path_planner2 = libpath_planner.PathPlanner(len(self.link_dimensions),
                                                        self.delta_t,
                                                        True,
                                                        self.max_velocity,
                                                        self.joint_constraints_vec,
                                                        self.enforce_constraints,                                                    
                                                        1.0,                                                    
                                                        self.use_linear_path,
                                                        self.verbose,
                                                        self.planning_algorithm)            
            path_planner2.setKinematics(self.kinematics)
            path_planner2.setup()
        else:            
            path_planner2 = libdynamic_path_planner.DynamicPathPlanner(len(self.link_dimensions) * 2,
                                                                       self.verbose)
            path_planner2.setupMotionValidator(self.continuous_collision)
            logging.info("PathPlanningInterface: Set up motion validator. Setting kinematics...")
            
            
            path_planner2.setKinematics(self.kinematics)
            logging.info("PathPlanningInterface: Kinematics set. Running setup...")
            path_planner2.setup(self.model_file,
                                self.environment_file,
                                self.simulation_step_size,
                                False,
                                self.coulomb,
                                self.viscous,
                                self.delta_t)
            logging.info("PathPlanningInterface: Path planner setup")        
        path_planner2.setObstacles(obstacles)
        
        link_dimensions = libutil.v2_double()
        ld = []       
        for i in xrange(len(self.link_dimensions)):
            link_dim = libutil.v_double()            
            link_dim[:] = [self.link_dimensions[i][j] for j in xrange(len(self.link_dimensions[i]))]
            ld.append(link_dim)
        link_dimensions[:] = ld               
        path_planner2.setLinkDimensions(link_dimensions)              
        goal_states = libutil.v2_double()
        gs = []       
        for i in xrange(len(self.goal_states)):
            goal_state = libutil.v_double()                  
            goal_state[:] = [self.goal_states[i][j] for j in xrange(len(self.goal_states[i]))]            
            if path_planner2.isValid(goal_state):                
                gs.append(goal_state)                    
        if len(gs) == 0:
            return [], [], []               
        goal_states[:] = gs 
        
        ee_goal_position = libutil.v_double()
        ee_goal_position[:] = self.ee_goal_position
               
        path_planner2.setGoalStates(goal_states, ee_goal_position, self.ee_goal_threshold)
        start_state = libutil.v_double()
        v = [self.start_state[i] for i in xrange(len(self.start_state))]
        start_state[:] = v 
        logging.info("PathPlanningInterface: Solve...") 
        xs_temp = path_planner2.solve(start_state, self.path_timeout)
        xs = []
        us = []
        zs = []        
        for i in xrange(len(xs_temp)):
            xs.append([xs_temp[i][j] for j in xrange(0, 2 * len(self.link_dimensions))])
            us.append([xs_temp[i][j] for j in xrange(2 * len(self.link_dimensions), 4 * len(self.link_dimensions))])
            zs.append([xs_temp[i][j] for j in xrange(4 * len(self.link_dimensions), 6 * len(self.link_dimensions))])
        return xs, us, zs
    
if __name__ == "__main__":
    PathPlanningInterface()

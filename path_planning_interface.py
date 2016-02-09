from path_evaluator import *
from multiprocessing import Process, cpu_count, Lock, Queue, Pool
import multiprocessing.queues
from Queue import Empty, Full
from librobot import v_string, v_double, v2_double
import libpath_planner
import libdynamic_path_planner
import time
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
                             robot, 
                             sample_size, 
                             obstacles,                             
                             goal_position,
                             goal_radius,
                             model_file,
                             env_file):
        self.path_evaluator.setup(A, B, C, D, H, M, N, V, W, 
                                  robot,
                                  sample_size, 
                                  obstacles,                                  
                                  goal_position,
                                  goal_radius,
                                  False,
                                  model_file,
                                  env_file,
                                  self.num_cores)
        
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
              planning_algorithm,
              path_timeout,
              continuous_collision,
              num_cores):
        self.robot = robot
        self.robot_dof = self.robot.getDOF()
        self.num_cores = num_cores + 1
        #self.num_cores = cpu_count() - 1
        #self.num_cores = 2       
        self.obstacles = obstacles        
        self.max_velocity = max_velocity
        self.delta_t = delta_t
        self.use_linear_path = use_linear_path         
           
        self.verbose = False 
        if(logging.getLogger().isEnabledFor(logging.INFO)):
            self.verbose = True        
        self.planning_algorithm = planning_algorithm
        self.dynamic_problem = False
        self.path_timeout = path_timeout 
        self.continuous_collision = continuous_collision       
        
    def setup_dynamic_problem(self, 
                              urdf_model,
                              environment_model, 
                              simulation_step_size,
                              num_control_samples,
                              min_control_duration,
                              max_control_duration,
                              add_intermediate_states,
                              rrt_goal_bias,
                              control_sampler):
        self.dynamic_problem = True
        self.model_file = urdf_model
        self.environment_file = environment_model
        self.simulation_step_size = simulation_step_size
        self.num_control_samples = num_control_samples
        self.min_control_duration = min_control_duration
        self.max_control_duration = max_control_duration
        self.add_intermediate_states = add_intermediate_states
        self.rrt_goal_bias = rrt_goal_bias
        self.control_sampler = control_sampler
        
    def set_start_and_goal(self, start_state, goal_states, ee_goal_position, ee_goal_threshold):                
        self.start_state = start_state
        self.goal_states = goal_states
        self.ee_goal_position = ee_goal_position
        self.ee_goal_threshold = ee_goal_threshold        
        return True
        
    def plan_and_evaluate_paths(self, 
                                num, 
                                sim_run, 
                                current_step, 
                                horizon, 
                                P_t,
                                deviation_covariance,
                                estimated_deviation_covariance,
                                timeout):        
        path_queue = Queue()
        evaluated_paths = []
        state_covariances = []
        deviation_covariances = []
        estimated_deviation_covariances = []
        gen_times = []
        eval_times = []        
        res_paths = collections.deque()
        processes = [Process(target=self.construct_and_evaluate_path, 
                             args=(self.robot,
                                   self.obstacles, 
                                   path_queue,                                   
                                   current_step,
                                   horizon, 
                                   P_t,
                                   deviation_covariance,
                                   estimated_deviation_covariance)) for i in xrange(self.num_cores - 1)]
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
                print "TIMEOUT"
                break
            time.sleep(0.001)
        for i in xrange(len(processes)):
            processes[i].terminate()
        for i in xrange(len(res_paths)):
            p_e = res_paths.pop()            
            if not len(p_e[0]) == 0:
                state_covariances.append(p_e[5])
                deviation_covariances.append(p_e[6])
                estimated_deviation_covariances.append(p_e[7])
                gen_times.append(p_e[8])
                eval_times.append(p_e[9])                               
                evaluated_paths.append(([[p_e[0][k] for k in xrange(len(p_e[0]))], 
                                         [p_e[1][k] for k in xrange(len(p_e[0]))], 
                                         [p_e[2][k] for k in xrange(len(p_e[0]))],
                                         [p_e[3][k] for k in xrange(len(p_e[0]))]], p_e[4]))
        if len(evaluated_paths) == 0:
            logging.error("PathPlanningInterface: Couldn't generate and evaluate any paths within the given planning time")
            return [], [], [], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0    
        best_val = evaluated_paths[0][1]
        best_path = evaluated_paths[0][0]        
        best_estimated_state_covariances = state_covariances[0]
        best_deviation_covariances = deviation_covariances[0]
        best_estimated_deviation_covariances = estimated_deviation_covariances[0]        
        for i in xrange(1, len(evaluated_paths)):
            if evaluated_paths[i][1] < best_val:                
                best_val = evaluated_paths[i][1]
                best_path = evaluated_paths[i][0]
                best_estimated_state_covariances = state_covariances[i]                
                best_deviation_covariances = deviation_covariances[i]
                best_estimated_deviation_covariances = estimated_deviation_covariances[i]
                
        n, min_max, mean_gen_times, var, skew, kurt = scipy.stats.describe(np.array(gen_times))
        n, min_max, mean_eval_times, var, skew, kurt = scipy.stats.describe(np.array(eval_times))
        total_gen_times = sum(gen_times)
        total_eval_times = sum(eval_times)                                    
        return (best_path[0], 
                best_path[1], 
                best_path[2],
                best_path[3], 
                len(evaluated_paths), 
                best_val,
                best_estimated_state_covariances,
                best_deviation_covariances,
                best_estimated_deviation_covariances, 
                mean_gen_times, 
                mean_eval_times,
                total_gen_times,
                total_eval_times)
        
    def plan_paths(self, num, sim_run, timeout=0.0):
        queue = Queue() 
        paths = []        
        res_paths = collections.deque()
        processes = [Process(target=self.construct_path, 
                             args=(self.robot,
                                   self.obstacles, 
                                   queue,
                                   i)) for i in xrange(self.num_cores - 1)]
        t0 = time.time() 
        for i in xrange(len(processes)):
            processes[i].daemon = True
            processes[i].start()
        curr_len = 0
        while True:
            breaking = False            
            for p in processes:
                if p.is_alive():
                    break
                breaking = True                
            try:                
                if queue.empty() == False:                                            
                    res_paths.append(queue.get(timeout=0.1))
                    print "PathPlanningInterface: Got " + str(len(res_paths)) + " so far"                    
                    if len(res_paths) == num:                        
                        breaking = True
                        break
                    if timeout > 0.0:                        
                        if elapsed > timeout:
                            breaking = True
                            break 
            except:
                pass           
            if breaking:
                break 
            time.sleep(0.0001)                 
        for i in xrange(len(processes)):
            processes[i].terminate()              
        for i in xrange(len(res_paths)):
            p_e = res_paths.pop()                                
            if not len(p_e[0]) == 0:                                      
                paths.append([[p_e[0][i] for i in xrange(len(p_e[0]))], 
                              [p_e[1][i] for i in xrange(len(p_e[0]))], 
                              [p_e[2][i] for i in xrange(len(p_e[0]))],
                              [p_e[3][i] for i in xrange(len(p_e[0]))]])                  
        return paths
    
    def construct_and_evaluate_path(self,
                                    robot, 
                                    obstacles, 
                                    queue,                                    
                                    current_step, 
                                    horizon, 
                                    P_t,
                                    deviation_covariance,
                                    estimated_deviation_covariance):        
        while True:
            t0 = time.time()               
            xs, us, zs, control_durations, success = self._construct(robot, obstacles)
            gen_time = time.time() - t0
            if len(xs) > 1:  
                t0 = time.time()                      
                eval_result = self.path_evaluator.evaluate_path([xs, us, zs, control_durations], 
                                                                P_t,
                                                                deviation_covariance,
                                                                estimated_deviation_covariance, 
                                                                current_step, 
                                                                horizon)
                eval_time = time.time() - t0        
                queue.put((xs, 
                           us, 
                           zs, 
                           control_durations, 
                           eval_result[1], 
                           eval_result[2],
                           eval_result[3],
                           eval_result[4],
                           gen_time, 
                           eval_time))        
    
    def construct_path(self, robot, obstacles, queue, process_num):
        while True:
            #xs = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0] for i in xrange(10)]
            #us = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0] for i in xrange(10)]
            #zs = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0] for i in xrange(10)]                     
            xs, us, zs, control_durations, success = self._construct(robot, obstacles)
            if (not success):
                break;
            if not len(xs) == 0:                              
                queue.put((xs, us, zs, control_durations))
                time.sleep(0.1)
        
    def _construct(self, robot, obstacles):        
        path_planner2 = None        
        if not self.dynamic_problem:
            path_planner2 = libpath_planner.PathPlanner(robot,                                                        
                                                        self.delta_t,
                                                        self.continuous_collision,
                                                        self.max_velocity,                                 
                                                        1.0,                                                    
                                                        self.use_linear_path,
                                                        self.verbose,
                                                        self.planning_algorithm)
            path_planner2.setup()            
        else:            
            path_planner2 = libdynamic_path_planner.DynamicPathPlanner(robot,                                                                       
                                                                       self.verbose)            
            path_planner2.setupMotionValidator(self.continuous_collision)
            logging.info("PathPlanningInterface: Set up motion validator. Setting kinematics...")            
            
            
            logging.info("PathPlanningInterface: Kinematics set. Running setup...")            
            path_planner2.setup(self.simulation_step_size,                                
                                self.delta_t)
            path_planner2.setControlSampler(self.control_sampler)            
            num_control_samples = libdynamic_path_planner.v_int()
            num_control_samples[:] = [self.num_control_samples]
            path_planner2.setNumControlSamples(num_control_samples)
            path_planner2.setRRTGoalBias(self.rrt_goal_bias)            
            
            min_max_control_duration = libdynamic_path_planner.v_int()
            min_max_control_duration[:] = [self.min_control_duration,
                                           self.max_control_duration]
            path_planner2.setMinMaxControlDuration(min_max_control_duration)
            path_planner2.addIntermediateStates(self.add_intermediate_states)
            logging.info("PathPlanningInterface: Path planner setup")              
        path_planner2.setObstacles(obstacles)
        goal_states = v2_double()
        gs = [] 
        
        '''
        Check if the start state is valid
        '''
        ss_vec = v_double()
        ss_vec[:] = self.start_state
        if not path_planner2.isValid(ss_vec):
            logging.warn("Path planning interface: Start state not valid")            
            return [], [], [], [], False
              
        '''
        Check of the goal states are valid
        '''
        for i in xrange(len(self.goal_states)):            
            goal_state = v_double()                  
            goal_state[:] = [self.goal_states[i][j] for j in xrange(len(self.goal_states[i]))]                    
            if path_planner2.isValid(goal_state):                                             
                gs.append(goal_state)            
                              
        if len(gs) == 0:
            logging.warn("PathPlanningInterface: No valid goal states")                        
            return [], [], [], [], False               
        goal_states[:] = gs 
        
        ee_goal_position = v_double()
        ee_goal_position[:] = self.ee_goal_position        
        path_planner2.setGoalStates(goal_states, ee_goal_position, self.ee_goal_threshold)        
        start_state = v_double()
        v = [self.start_state[i] for i in xrange(len(self.start_state))]
        start_state[:] = v 
        logging.info("PathPlanningInterface: Solve...") 
        xs_temp = path_planner2.solve(start_state, self.path_timeout)        
        xs = []
        us = []
        zs = [] 
        control_durations = []       
        for i in xrange(len(xs_temp)):                    
            xs.append([xs_temp[i][j] for j in xrange(0, 2 * self.robot_dof)])
            us.append([xs_temp[i][j] for j in xrange(2 * self.robot_dof, 3 * self.robot_dof)])
            zs.append([xs_temp[i][j] for j in xrange(3 * self.robot_dof, 5 * self.robot_dof)])
            
            control_durations.append(xs_temp[i][5 * self.robot_dof])        
        '''if self.dynamic_problem:
            all_states = v2_double()
            print "getting all states"
            path_planner2.getAllStates(all_states)
            print "got all states " + str(len(all_states))
            plot_states_states = [np.array(all_states[i][0:3]) for i in xrange(len(all_states))]
            plot_states_velocities = [np.array(all_states[i][3:6]) for i in xrange(len(all_states))]
            from plot import plot_3d_points
            scale = [-np.pi - 0.01, np.pi + 0.01]
            scale2 = [-11, 11]
            plot_3d_points(np.array(plot_states_states),
                           scale,
                           scale,
                           scale)
            plot_3d_points(np.array(plot_states_velocities),
                           scale2,
                           scale2,
                           scale2)'''      
        return xs, us, zs, control_durations, True
    
if __name__ == "__main__":
    PathPlanningInterface()

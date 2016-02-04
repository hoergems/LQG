import sys
import argparse
import time
import numpy as np
import os
import glob
import scipy
from serializer import Serializer
from libutil import *
import logging
from librobot import v_string, Robot
from util_py import check_positive_definite, get_goal_states, copyToTmp
from simulator import Simulator
from path_evaluator import PathEvaluator
from path_planning_interface import PathPlanningInterface
from libobstacle import Obstacle, Terrain
from libcollision_checker import CollisionChecker
import plot as plt
import warnings

class MPC:
    def __init__(self, plot):
        """ Reading the config """
        warnings.filterwarnings("ignore")
        self.init_serializer()
        config = self.serializer.read_config("config_mpc.yaml")
        self.set_params(config)
        logging_level = logging.WARN
        if config['verbose']:
            logging_level = logging.DEBUG
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging_level)        
        np.set_printoptions(precision=16)
        dir = "stats/lqg"        
        self.utils = Utils()
        if not self.init_robot(self.robot_file):
            logging.error("LQG: Couldn't initialize robot")
            return               
        if not self.setup_scene(self.environment_file, self.robot):
            return        
        self.clear_stats(dir)
        logging.info("Start up simulator")
        sim = Simulator()
        path_evaluator = PathEvaluator()
        path_planner = PathPlanningInterface()
        logging.info("MPC: Generating goal states...")        
        goal_states = get_goal_states("mpc",
                                      self.serializer, 
                                      self.obstacles,                                                                           
                                      self.robot,                                    
                                      self.max_velocity,
                                      self.delta_t,
                                      self.start_state,
                                      self.goal_position,
                                      self.goal_radius,
                                      self.planning_algortihm,
                                      self.path_timeout,
                                      self.num_generated_goal_states,
                                      self.continuous_collision,
                                      self.environment_file)
        
        if len(goal_states) == 0:
            logging.error("LQG: Couldn't generate any goal states. Problem seems to be infeasible")
            return
        logging.info("LQG: Generated " + str(len(goal_states)) + " goal states")
        sim.setup_reward_function(self.discount_factor, self.step_penalty, self.illegal_move_penalty, self.exit_reward)  
        path_planner.setup(self.robot,                         
                           self.obstacles,  
                           self.max_velocity, 
                           self.delta_t, 
                           self.use_linear_path,
                           self.planning_algortihm,
                           self.path_timeout,
                           self.continuous_collision)
        if self.dynamic_problem:
            path_planner.setup_dynamic_problem(self.robot_file,
                                               self.environment_file,
                                               self.simulation_step_size,
                                               self.num_control_samples,
                                               self.min_control_duration,
                                               self.max_control_duration,
                                               self.add_intermediate_states,
                                               self.rrt_goal_bias,
                                               self.control_sampler)
             
        A, H, B, V, W, C, D, M_base, N_base = self.problem_setup(self.delta_t, self.robot_dof)
        time_to_generate_paths = 0.0
        if check_positive_definite([C, D]):
            m_covs = None
            if self.inc_covariance == "process":
                m_covs = np.linspace(self.min_process_covariance, 
                                     self.max_process_covariance, 
                                     self.covariance_steps)                 
            elif self.inc_covariance == "observation":          
                m_covs = np.linspace(self.min_observation_covariance, 
                                     self.max_observation_covariance,
                                     self.covariance_steps)
        for j in xrange(len(m_covs)):
            M = None
            N = None
            if self.inc_covariance == "process":
                """ The process noise covariance matrix """
                M = self.calc_covariance_value(self.robot, m_covs[j], M_base)
                #M = m_covs[j] * M_base
                    
                """ The observation error covariance matrix """
                N = self.calc_covariance_value(self.robot, 
                                               self.min_observation_covariance, 
                                               N_base, 
                                               covariance_type='observation')                    
            elif self.inc_covariance == "observation":
                M = self.calc_covariance_value(self.robot, 
                                               self.min_process_covariance,
                                               M_base)
                N = self.calc_covariance_value(self.robot, 
                                               m_covs[j],
                                               N_base, 
                                               covariance_type='observation')                    
            P_t = np.array([[0.0 for k in xrange(2 * self.robot_dof)] for l in xrange(2 * self.robot_dof)])
            
            path_planner.setup_path_evaluator(A, B, C, D, H, M, N, V, W,                                     
                                              self.robot, 
                                              self.sample_size, 
                                              self.obstacles,
                                              self.goal_position,
                                              self.goal_radius,                                              
                                              self.robot_file,
                                              self.environment_file)
            if self.dynamic_problem:
                path_evaluator.setup_dynamic_problem()
            path_planner.setup_reward_function(self.step_penalty, self.exit_reward, self.illegal_move_penalty, self.discount_factor)
            for k in xrange(self.num_simulation_runs):
                self.serializer.write_line("log.log", "tmp/mpc", "RUN #" + str(k + 1) + " \n")
                current_step = 0
                x_true = self.start_state
                x_estimate = self.start_state
                P_t = np.array([[0.0 for i in xrange(2 * self.robot_dof)] for i in xrange(2 * self.robot_dof)])                
                total_reward = 0.0
                terminal = False
                while current_step < self.max_num_steps and not terminal:
                    path_planner.set_start_and_goal(self.start_state, goal_states, self.goal_position, self.goal_radius)
                    (xs, 
                     us, 
                     zs, 
                     num_generated_paths, 
                     best_val, 
                     mean_gen_times, 
                     mean_eval_times,
                     total_gen_times,
                     total_eval_times) = path_planner.plan_and_evaluate_paths(self.num_paths, 
                                                                              0, 
                                                                              current_step, 
                                                                              self.evaluation_horizon, 
                                                                              P_t, 
                                                                              self.timeout)
                     
                
        
                
    def init_robot(self, urdf_model_file):
        self.robot = Robot(urdf_model_file)
        self.robot.enforceConstraints(self.enforce_constraints)
        self.robot.setGravityConstant(self.gravity_constant)
        """ Setup operations """
        self.robot_dof = self.robot.getDOF()        
        if len(self.start_state) != 2 * self.robot_dof:
            logging.error("LQG: Start state dimension doesn't fit to the robot state space dimension")
            return False
        return True
    
    def setup_scene(self,                    
                    environment_file,
                    robot):
        """ Load the obstacles """         
        self.obstacles = self.utils.loadObstaclesXML(environment_file)      
        
        """ Load the goal area """
        goal_area = v_double()
        self.utils.loadGoalArea(environment_file, goal_area)
        if len(goal_area) == 0:
            print "ERROR: Your environment file doesn't define a goal area"
            return False
        self.goal_position = [goal_area[i] for i in xrange(0, 3)]
        self.goal_radius = goal_area[3]   
        self.collision_checker = CollisionChecker() 
        self.collision_checker.setObstacles(self.obstacles)    
        return True
                
    def init_serializer(self):
        print "INIT S"
        self.serializer = Serializer()
        self.serializer.create_temp_dir("mpc") 
                
    def clear_stats(self, dir):
        if os.path.isdir(dir):
            cmd = "rm -rf " + dir + "/*"            
            os.system(cmd)
        else:
            os.makedirs(dir)
                
    def get_average_distance_to_goal_area(self, goal_position, goal_radius, cartesian_coords):        
        avg_dist = 0.0
        goal_pos = np.array(goal_position)        
        for i in xrange(len(cartesian_coords)):            
            cart = np.array(cartesian_coords[i])            
            dist = np.linalg.norm(goal_pos - cart)            
            if dist < goal_radius:
                dist = 0.0
            avg_dist += dist
        if avg_dist == 0.0:
            return avg_dist        
        return np.asscalar(avg_dist) / len(cartesian_coords) 
            
    def mpc(self, initial_belief, m_covs, horizon, obstacles, serializer):
        A, H, B, V, W, C, D = self.problem_setup(self.delta_t, self.link_dimensions)
        cart_coords = []
        emds = []
        for j in xrange(len(m_covs)):                       
            
            logging.info("MPC: Evaluating covariance " + str(m_covs[j]))
            
            """
            The process noise covariance matrix
            """
            M = m_covs[j] * np.identity(len(self.link_dimensions))
                
            """
            The observation noise covariance matrix
            """
            N = self.observation_covariance * np.identity(len(self.link_dimensions))
            P_t = np.array([[0.0 for k in xrange(len(self.link_dimensions))] for l in xrange(len(self.link_dimensions))])
                
            self.path_planning_interface.setup_path_evaluator(A, B, C, D, H, M, N, V, W, 
                                                              self.link_dimensions,
                                                              self.workspace_dimension, 
                                                              self.sample_size, 
                                                              obstacles,
                                                              self.joint_constraints,
                                                              self.enforce_constraints, 
                                                              self.goal_position,
                                                              self.goal_radius,                                                             
                                                              self.w1,
                                                              self.w2)
            self.path_planning_interface.setup_reward_function(self.step_penalty, 
                                                               self.exit_reward, 
                                                               self.illegal_move_penalty, 
                                                               self.discount_factor)
            self.sim.setup_problem(A, B, C, D, H, V, W, M, N, 
                                   obstacles, 
                                   self.goal_position, 
                                   self.goal_radius, 
                                   self.link_dimensions,
                                   self.workspace_dimension, 
                                   self.joint_constraints,
                                   self.enforce_constraints,
                                   self.max_velocity)            
            self.sim.setup_simulator(self.num_simulation_runs, self.stop_when_terminal)            
            
            cartesian_coords = []            
            all_estimated_states = []
            all_estimated_covariances = []           
            
            mean_planning_time_per_run = 0.0
            
            successful_runs = 0
            num_generated_paths_run = 0.0
            mean_number_planning_steps = 0.0 
            mean_number_of_steps = 0.0 
            num_collisions = 0.0
            rewards_cov = []          
            for k in xrange(self.num_simulation_runs):                  
                serializer.write_line("log.log", 
                                      "tmp/mpc" + str(self.num_execution_steps), 
                                      "RUN #" + str(k + 1) + " \n")           
                print "MPC: Joint covariance: " + str(m_covs[j])
                print "MPC: simulation run " + str(k + 1)
                x_true = initial_belief
                x_estimate = initial_belief
                P_t = np.array([[0.0 for i in xrange(len(self.link_dimensions))] for i in xrange(len(self.link_dimensions))])                
                total_reward = 0.0
                
                current_step = 0                
                terminal = False                         
                while current_step < self.max_num_steps and not terminal:                    
                    mean_number_planning_steps += 1.0
                    t0 = time.time()                    
                    self.path_planning_interface.set_start_and_goal(x_estimate, self.goal_states)
                    
                    logging.info("MPC: Constructing paths")
                    (xs, 
                     us, 
                     zs, 
                     num_generated_paths, 
                     best_val, 
                     mean_gen_times, 
                     mean_eval_times,
                     total_gen_times,
                     total_eval_times) = self.path_planning_interface.plan_and_evaluate_paths(self.num_paths, 
                                                                                              0, 
                                                                                              current_step, 
                                                                                              horizon, 
                                                                                              P_t, 
                                                                                              self.timeout)
                    
                    t_e = time.time() - t0
                    mean_planning_time_per_run += t_e  
                    num_generated_paths_run += num_generated_paths                   
                    if len(xs) == 0:
                        logging.error("MPC: Couldn't find any paths from start state" + 
                                      str(x_estimate) + 
                                      " to goal states") 
                                      
                        total_reward = np.array([-self.illegal_move_penalty])[0]
                        current_step += 1                                             
                        break                         
                    logging.warn("MPC: Generated " + str(num_generated_paths) + " paths in " + str(t_e) + " seconds")
                    logging.warn("MPC: Mean time to generate paths: " + str(mean_gen_times) + " seconds")
                    logging.warn("MPC: Mean time to evaluate_paths: " + str(mean_eval_times) + " seconds")
                    logging.warn("MPC: Total time to generate paths: " + str(total_gen_times) + " seconds")
                    logging.warn("MPC: Total time to evaluate_paths: " + str(total_eval_times) + " seconds")
                    logging.warn("MPC: Length of best path is " + str(len(xs)))
                    logging.warn("MPC: Expected reward of best path: " + str(best_val))
                    x_tilde = np.array([0.0 for i in xrange(len(self.link_dimensions))])
                    n_steps = self.num_execution_steps
                    if n_steps > len(xs) - 1:
                       n_steps = len(xs) - 1
                    if current_step + n_steps > self.max_num_steps:
                        n_steps = self.max_num_steps - current_step
                                          
                    logging.info("MPC: Execute best path for " + str(n_steps) + " steps")
                    (x_true, 
                     x_tilde, 
                     x_estimate, 
                     P_t, 
                     current_step, 
                     total_reward, 
                     success, 
                     terminal,
                     estimated_s,
                     estimated_c,
                     history_entries) = self.sim.simulate_n_steps(xs, us, zs,
                                                                  x_true,                                                              
                                                                  x_tilde,
                                                                  x_estimate,
                                                                  P_t,
                                                                  total_reward,                                                                 
                                                                  current_step,
                                                                  n_steps)
                    if (success):
                        successful_runs += 1
                        
                    for history_entry in history_entries:                        
                        history_entry.serialize("tmp/mpc" + str(self.num_execution_steps), "log.log")
                        if history_entry.collided:
                            num_collisions += 1         
                    logging.warn("MPC: Execution finished. True state is " + str(x_true))
                    logging.warn("MPC: Estimated state is " + str(x_estimate))
                    logging.warn("MPC: Executed " + str(current_step) + " steps") 
                    logging.warn("MPC: terminal " + str(terminal))
                    if terminal:
                        print "MPC: Final state: " + str(x_true)
                rewards_cov.append(total_reward)
                serializer.write_line("log.log", 
                                      "tmp/mpc" + str(self.num_execution_steps), 
                                      "Reward: " + str(total_reward) + " \n") 
                serializer.write_line("log.log", 
                                      "tmp/mpc"  + str(self.num_execution_steps), 
                                      "\n")
                               
                mean_number_of_steps += current_step                
                x_true_vec = v_double()
                x_true_vec[:] = x_true
                ee_position_vec = self.kinematics.getEndEffectorPosition(x_true_vec)
                ee_position = np.array([ee_position_vec[i] for i in xrange(len(ee_position_vec))])
                cartesian_coords.append(ee_position.tolist())
                logging.info("MPC: Done. total_reward is " + str(total_reward))
            logging.info("MPC: Finished simulations for covariance value  " + 
                         str(m_covs[j]))
            
            #serializer.save_stats([np.asscalar(m_covs[i]) for i in xrange(len(m_covs))], "stats/mpc" + str(self.num_execution_steps))
            
            n, min_max, mean, var, skew, kurt = scipy.stats.describe(np.array(rewards_cov))
            
            serializer.write_line("log.log",
                                  "tmp/mpc" + str(self.num_execution_steps),
                                  "################################ \n")            
            
            serializer.write_line("log.log",
                                  "tmp/mpc" + str(self.num_execution_steps),
                                  "Process covariance: " + str(m_covs[j]) + " \n")
            
            serializer.write_line("log.log", 
                                  "tmp/mpc" + str(self.num_execution_steps), 
                                  "Mean rewards: " + str(mean) + " \n")
            serializer.write_line("log.log",
                                  "tmp/mpc" + str(self.num_execution_steps), 
                                  "Reward variance: " + str(var) + " \n")
            serializer.write_line("log.log", 
                                  "tmp/mpc" + str(self.num_execution_steps), 
                                  "Reward standard deviation: " + str(np.sqrt(var)) + " \n")
            
            serializer.write_line("log.log", 
                                  "tmp/mpc"  + str(self.num_execution_steps), 
                                  "Num successes: " + str(successful_runs) + " \n")
            
            serializer.write_line("log.log", 
                                  "tmp/mpc"  + str(self.num_execution_steps), 
                                  "Mean num collisions per run: " + str(float(num_collisions) / float(self.num_simulation_runs)) + " \n")
            
            mean_number_planning_steps /= self.num_simulation_runs            
            serializer.write_line("log.log", 
                                  "tmp/mpc"  + str(self.num_execution_steps), 
                                  "Mean number of planning steps: " + str(mean_number_planning_steps) + " \n")
            mean_number_of_steps /= self.num_simulation_runs
            serializer.write_line("log.log", 
                                  "tmp/mpc"  + str(self.num_execution_steps), 
                                  "Mean number of steps per run: " + str(mean_number_of_steps) + " \n")
            
            mean_planning_time_per_run /= self.num_simulation_runs
            serializer.write_line("log.log", 
                                  "tmp/mpc"  + str(self.num_execution_steps), 
                                  "Mean planning time per run: " + str(mean_planning_time_per_run) + " \n")
            
            mean_planning_time_per_step = mean_planning_time_per_run / mean_number_planning_steps
            serializer.write_line("log.log", 
                                  "tmp/mpc"  + str(self.num_execution_steps), 
                                  "Mean planning time per step: " + str(mean_planning_time_per_step) + " \n")            
            
            num_generated_paths_run /= self.num_simulation_runs
            serializer.write_line("log.log", 
                                  "tmp/mpc"  + str(self.num_execution_steps), 
                                  "Mean number of generated paths per run: " + str(num_generated_paths_run) + " \n")  
            
            num_generated_paths_step = num_generated_paths_run / mean_number_planning_steps
            serializer.write_line("log.log", 
                                  "tmp/mpc"  + str(self.num_execution_steps), 
                                  "Mean number of generated paths per step: " + str(num_generated_paths_step) + " \n")
            prob = "mpc" + str(self.num_execution_steps)
            d = "stats/mpc" + str(self.num_execution_steps)
            cmd = "mv tmp/" + prob + "/log.log " + d + "/log_" + prob + "_" + str(m_covs[j]) + ".log"
            os.system(cmd)           
            
            
            emds.append(calc_EMD(cartesian_coords, 
                                 self.num_bins, 
                                 self.goal_position, 
                                 self.link_dimensions))
            cart_coords.append([cartesian_coords[i] for i in xrange(len(cartesian_coords))])
            #estimated_states_cov.append(all_estimated_states)
            #estimated_covariances_cov.append(all_estimated_covariances)
            
    def calc_covariance_value(self, robot, error, covariance_matrix, covariance_type='process'):
        active_joints = v_string()
        robot.getActiveJoints(active_joints)        
        if covariance_type == 'process':
            if self.dynamic_problem:            
                torque_limits = v_double()
                robot.getJointTorqueLimits(active_joints, torque_limits)
                torque_limits = [torque_limits[i] for i in xrange(len(torque_limits))]
                for i in xrange(len(torque_limits)):
                    torque_range = 2.0 * torque_limits[i]
                    covariance_matrix[i, i] = np.square((torque_range / 100.0) * error)
            else:
                for i in xrange(self.robot_dof):
                    covariance_matrix[i, i] = np.square((self.max_velocity / 100.0) * error)
        else:
            lower_position_limits = v_double()
            upper_position_limits = v_double()
            velocity_limits = v_double()            
            robot.getJointLowerPositionLimits(active_joints, lower_position_limits)
            robot.getJointUpperPositionLimits(active_joints, upper_position_limits)
            robot.getJointVelocityLimits(active_joints, velocity_limits)            
            for i in xrange(self.robot_dof):
                position_range = upper_position_limits[i] - lower_position_limits[i]
                covariance_matrix[i, i] = np.square((position_range / 100.0) * error)            
                velocity_range = 2.0 * velocity_limits[i]
                covariance_matrix[i + self.robot_dof, i + self.robot_dof] = np.square((velocity_range / 100.0) * error)          
        return covariance_matrix            
        
            
    def problem_setup(self, delta_t, num_links):
        A = np.identity(num_links * 2)
        H = np.identity(num_links * 2)
        W = np.identity(num_links * 2)
        C = self.path_deviation_cost * np.identity(num_links * 2)
        
        '''B = delta_t * np.identity(num_links * 2)                
        V = np.identity(num_links * 2)
        D = self.control_deviation_cost * np.identity(num_links * 2)        
        M_base = np.identity(2 * self.robot_dof)
        N_base = np.identity(2 * self.robot_dof)'''
        
        B = delta_t * np.vstack((np.identity(num_links),
                                 np.zeros((num_links, num_links))))
        V = np.vstack((np.identity(num_links),
                       np.zeros((num_links, num_links))))
        D = self.control_deviation_cost * np.identity(num_links)
        M_base = np.identity(self.robot_dof)
        N_base = np.identity(2 * self.robot_dof)
        
        
        return A, H, B, V, W, C, D, M_base, N_base
        
    def set_params(self, config):
        self.num_paths = config['num_generated_paths']
        self.use_linear_path = config['use_linear_path']        
        self.max_velocity = config['max_velocity']
        self.delta_t = 1.0 / config['control_rate']
        self.start_state = config['start_state']        
        self.num_simulation_runs = config['num_simulation_runs']
        self.num_bins = config['num_bins']
        self.min_process_covariance = config['min_process_covariance']
        self.max_process_covariance = config['max_process_covariance']
        self.covariance_steps = config['covariance_steps']
        self.min_observation_covariance = config['min_observation_covariance']
        self.max_observation_covariance = config['max_observation_covariance']       
        self.discount_factor = config['discount_factor']
        self.illegal_move_penalty = config['illegal_move_penalty']
        self.step_penalty = config['step_penalty']
        self.exit_reward = config['exit_reward']
        self.stop_when_terminal = config['stop_when_terminal']        
        self.enforce_constraints = config['enforce_constraints']
        self.enforce_control_constraints = config['enforce_control_constraints']
        self.sample_size = config['sample_size']
        self.plot_paths = config['plot_paths']
        self.planning_algortihm = config['planning_algorithm']
        self.dynamic_problem = config['dynamic_problem'] 
        self.simulation_step_size = config['simulation_step_size']        
        self.path_timeout = config['path_timeout'] 
        self.continuous_collision = config['continuous_collision_check']
        self.show_viewer_evaluation = config['show_viewer_evaluation']
        self.show_viewer_simulation = config['show_viewer_simulation']   
        self.path_deviation_cost = config['path_deviation_cost'] 
        self.control_deviation_cost = config['control_deviation_cost']
        self.num_control_samples = config['num_control_samples'] 
        self.min_control_duration = config['min_control_duration']
        self.max_control_duration = config['max_control_duration']   
        self.inc_covariance = config['inc_covariance'] 
        self.add_intermediate_states = config['add_intermediate_states']
        self.gravity_constant = config['gravity']
        self.num_generated_goal_states = config['num_generated_goal_states']
        self.robot_file = config['robot_file']
        self.environment_file = config['environment_file']
        self.rrt_goal_bias = config['rrt_goal_bias']
        self.control_sampler = config['control_sampler']
        self.max_num_steps = config['max_num_steps']
        self.evaluation_horizon = config['horizon']
        self.timeout = config['timeout']
        
        
        
        """
        The number of steps a path is being executed before a new path gets calculated
        """
        self.num_execution_steps = config['num_execution_steps']
    
if __name__ == "__main__":
    if len(sys.argv) > 1:
        if "plot" in sys.argv[1]:
            MPC(True)
            sys.exit()
        logging.error("Unrecognized command line argument: " + str(sys.argv[1])) 
        sys.exit()   
    MPC(False)
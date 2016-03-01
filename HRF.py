import sys
import argparse
import time
import numpy as np
import os
import glob
import scipy
import kalman as kalman
from plan_adjuster import PlanAdjuster
from serializer import Serializer
from libutil import *
import logging
from librobot import v_string, Robot
from util_py import check_positive_definite, get_goal_states, copyToTmp
from simulator import Simulator
from path_evaluator import PathEvaluator
from path_planning_interface import PathPlanningInterface
from libobstacle import Obstacle, Terrain
import warnings

class HRF:
    def __init__(self):
        self.abs_path = os.path.dirname(os.path.abspath(__file__))
        """ Reading the config """
        warnings.filterwarnings("ignore")
        self.init_serializer()
        config = self.serializer.read_config("config_mpc.yaml", path=self.abs_path)
        self.set_params(config)
        if self.seed < 0:
            """
            Generate a random seed that will be stored
            """
            self.seed = np.random.randint(0, 42949672)
        np.random.seed(self.seed)
        
        logging_level = logging.WARN
        if config['verbose']:
            logging_level = logging.DEBUG
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging_level)        
        np.set_printoptions(precision=16)
        dir = self.abs_path + "/stats/hrf"
        tmp_dir = self.abs_path + "/tmp/hrf"
        self.utils = Utils()
        if not self.init_robot(self.robot_file):
            logging.error("MPC: Couldn't initialize robot")
            return               
        if not self.setup_scene(self.environment_file, self.robot):
            return   
        #self.run_viewer(self.robot_file, self.environment_file)     
        self.clear_stats(dir)
        logging.info("Start up simulator")
        sim = Simulator() 
        plan_adjuster = PlanAdjuster()       
        path_evaluator = PathEvaluator()
        path_planner = PathPlanningInterface()
        logging.info("MPC: Generating goal states...") 
        goal_states = get_goal_states("hrf",
                                      self.abs_path,
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
                                      self.environment_file,
                                      self.num_cores)
        
        if len(goal_states) == 0:
            logging.error("MPC: Couldn't generate any goal states. Problem seems to be infeasible")
            return
        logging.info("MPC: Generated " + str(len(goal_states)) + " goal states")
        sim.setup_reward_function(self.discount_factor, self.step_penalty, self.illegal_move_penalty, self.exit_reward)
        path_planner.setup(self.robot,                         
                           self.obstacles,  
                           self.max_velocity, 
                           self.delta_t, 
                           self.use_linear_path,
                           self.planning_algortihm,
                           self.path_timeout,
                           self.continuous_collision,
                           self.num_cores)
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
            sim.setup_problem(A, B, C, D, H, V, W, M, N,
                              self.robot, 
                              self.enforce_control_constraints,
                              self.obstacles, 
                              self.goal_position, 
                              self.goal_radius,
                              self.max_velocity,                                  
                              self.show_viewer_simulation,
                              self.robot_file,
                              self.environment_file)
            sim.set_stop_when_colliding(self.replan_when_colliding)
            if self.dynamic_problem:
                path_evaluator.setup_dynamic_problem()
                sim.setup_dynamic_problem(self.simulation_step_size)
            path_planner.setup_reward_function(self.step_penalty, self.exit_reward, self.illegal_move_penalty, self.discount_factor)
            mean_number_planning_steps = 0.0
            number_of_steps = 0.0
            mean_planning_time = 0.0
            num_generated_paths_run = 0.0
            successful_runs = 0
            num_collisions = 0.0
            linearization_error = 0.0
            final_states= []
            rewards_cov = []
            for k in xrange(self.num_simulation_runs):
                print "MPC: Run " + str(k + 1)                                
                self.serializer.write_line("log.log", tmp_dir, "RUN #" + str(k + 1) + " \n")
                current_step = 0
                x_true = self.start_state
                x_estimate = self.start_state
                x_tilde_linear = np.array([0.0 for i in xrange(2 * self.robot_dof)])                
                P_t = np.array([[0.0 for i in xrange(2 * self.robot_dof)] for i in xrange(2 * self.robot_dof)]) 
                deviation_covariance = np.array([[0.0 for i in xrange(2 * self.robot_dof)] for i in xrange(2 * self.robot_dof)])
                estimated_deviation_covariance = np.array([[0.0 for i in xrange(2 * self.robot_dof)] for i in xrange(2 * self.robot_dof)])              
                total_reward = 0.0
                terminal = False
                
                """
                Obtain a nominal path
                """
                path_planner.set_start_and_goal(x_estimate, goal_states, self.goal_position, self.goal_radius)
                t0 = time.time()
                print "plan"
                (xs, 
                 us, 
                 zs,
                 control_durations, 
                 num_generated_paths, 
                 best_val,
                 state_covariances,
                 deviation_covariances,
                 estimated_deviation_covariances, 
                 mean_gen_times, 
                 mean_eval_times,
                 total_gen_times,
                 total_eval_times) = path_planner.plan_and_evaluate_paths(self.num_paths, 
                                                                          0, 
                                                                          current_step, 
                                                                          self.evaluation_horizon, 
                                                                          P_t,
                                                                          deviation_covariance,
                                                                          estimated_deviation_covariance, 
                                                                          self.timeout)
                print "planned"
                
                while True:
                    """
                    Predict system state at t+1 using nominal path
                    """
                    
                    """ Get state matrices """
                    As, Bs, Vs, Ms, Hs, Ws, Ns = sim.get_linear_model_matrices(xs, us, control_durations)
                    
                    """ Predict using EKF """
                    (x_predicted, P_predicted) = kalman.predict_state(x_tilde, 
                                                                      xs[0], 
                                                                      xs[1], 
                                                                      u_dash, 
                                                                      u[0], 
                                                                      control_durations[0], 
                                                                      As[0],
                                                                      Bs[0],
                                                                      Vs[0],
                                                                      Ms[0],
                                                                      P_t)                    
                    
                    """
                    Execute path for 1 time step
                    """
                    (x_true,                     
                     x_tilde,
                     x_tilde_linear, 
                     x_estimate, 
                     P_t, 
                     current_step, 
                     total_reward,
                     terminal,
                     estimated_s,
                     estimated_c,
                     history_entries) = sim.simulate_n_steps(xs, us, zs,
                                                             control_durations,
                                                             x_true,                                                                                                                          
                                                             x_tilde,
                                                             x_tilde_linear,
                                                             x_estimate,
                                                             P_t,
                                                             total_reward,                                                                 
                                                             current_step,
                                                             1,
                                                             0.0,
                                                             0.0,
                                                             max_num_steps=self.max_num_steps)
                    
                    """
                    Plan new trajectories from predicted state
                    """
                    path_planner.set_start_and_goal(x_predicted, goal_states, self.goal_position, self.goal_radius)
                    (xs_new, 
                     us_new, 
                     zs_new,
                     control_durations_new, 
                     num_generated_paths, 
                     best_val_new,
                     state_covariances,
                     deviation_covariances,
                     estimated_deviation_covariances, 
                     mean_gen_times, 
                     mean_eval_times,
                     total_gen_times,
                     total_eval_times) = path_planner.plan_and_evaluate_paths(self.num_paths, 
                                                                              0, 
                                                                              current_step, 
                                                                              self.evaluation_horizon, 
                                                                              P_predicted,
                                                                              deviation_covariance,
                                                                              estimated_deviation_covariance, 
                                                                              self.timeout)
                    
                    """
                    Filter update
                    """
                    x_estimate
                    P_t
                    
                    """
                    Adjust plan
                    """
                    
                    """
                    Evaluate the adjusted plan
                    """
                    
                    if best_val_new > best_val_adjusted:
                        """
                        We found a better trajectory
                        """
                        xs = xs_new
                        us = us_new
                        zs = zs_new
                        control_durations = control_durations_new
                        x_tilde = np.array([0.0 for i in xrange(2 * self.robot_dof)])
                    else:
                        """
                        The adjusted trajectory is the better one
                        """
                    
                 
        
    def init_serializer(self):        
        self.serializer = Serializer()
        self.serializer.create_temp_dir(self.abs_path, "hrf")
        
    def init_robot(self, urdf_model_file):
        self.robot = Robot(self.abs_path + "/" + urdf_model_file)
        self.robot.enforceConstraints(self.enforce_constraints)
        self.robot.setGravityConstant(self.gravity_constant)
        self.robot.setAccelerationLimit(self.acceleration_limit)
        """ Setup operations """
        self.robot_dof = self.robot.getDOF()        
        if len(self.start_state) != 2 * self.robot_dof:
            logging.error("MPC: Start state dimension doesn't fit to the robot state space dimension")
            return False
        return True 
    
    def setup_scene(self,                    
                    environment_file,
                    robot):
        """ Load the obstacles """         
        self.obstacles = self.utils.loadObstaclesXML(self.abs_path + "/" + environment_file)      
        
        """ Load the goal area """
        goal_area = v_double()
        self.utils.loadGoalArea(self.abs_path + "/" + environment_file, goal_area)
        if len(goal_area) == 0:
            print "ERROR: Your environment file doesn't define a goal area"
            return False
        self.goal_position = [goal_area[i] for i in xrange(0, 3)]
        self.goal_radius = goal_area[3]
        return True
    
    def clear_stats(self, dir):
        if os.path.isdir(dir):
            cmd = "rm -rf " + dir + "/*"            
            os.system(cmd)
        else:
            os.makedirs(dir)
            
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
        self.seed = config['seed']
        self.num_cores = config['num_cores']
        self.replan_when_colliding = config['replan_when_colliding']
        self.acceleration_limit = config['acceleration_limit']

if __name__ == "__main__":
    HRF()
from path_evaluator import PathEvaluator
from simulator import Simulator
from path_planning_interface import PathPlanningInterface
from serializer import Serializer
from obstacle import Obstacle
from kin import *
from util import *
from util_py import *
from obstacle import *
from EMD import *
from plot_stats import PlotStats
import numpy as np
import sys
import os
import time
import scipy
import logging
from xml.dom import minidom

class MPC:
    def __init__(self, plot):
        """ Reading the config """
        serializer = self.init_serializer()
        self.utils = Utils()
        config = serializer.read_config("config_mpc.yaml")
        self.set_params(config)
        
        logging_level = logging.WARN
        if self.verbose:
            logging_level = logging.DEBUG
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging_level)        
        
        dir = "stats/mpc" + str(self.num_execution_steps)
        self.clear_stats(dir)
        serializer.create_temp_dir("mpc" + str(self.num_execution_steps))
        cmd = "cp config_mpc.yaml " + dir            
        os.system(cmd)
        
        self.sim = Simulator()
        self.path_evaluator = PathEvaluator()
        self.path_planning_interface = PathPlanningInterface()
        
        """ Load the environment """ 
        environment = serializer.load_environment(file="env.xml", path="environment")       
        obstacles = []
        terrain = Terrain("default", 0.0, 1.0, False)
        logging.info("Loading obstacles")
        for obstacle in environment:            
            obstacles.append(Obstacle(obstacle[0][0], obstacle[0][1], obstacle[0][2], obstacle[1][0], obstacle[1][1], obstacle[1][2], terrain)) 
        
        """ Setup operations """
        model_file = os.getcwd() + "/model/model.xml"
        if self.workspace_dimension == 3:
            model_file = os.getcwd() + "/model/model3D.xml"
        self.link_dimensions = self.utils.getLinkDimensions(model_file) 
        self.sim.setup_reward_function(self.discount_factor, 
                                       self.step_penalty, 
                                       self.illegal_move_penalty, 
                                       self.exit_reward)        
        self.path_planning_interface.setup(self.link_dimensions, 
                                           self.workspace_dimension, 
                                           obstacles, 
                                           self.max_velocity, 
                                           self.delta_t, 
                                           self.use_linear_path, 
                                           self.joint_constraints,
                                           self.enforce_constraints,                                           
                                           self.planning_algorithm)
        
        A, H, B, V, W, C, D = self.problem_setup(self.delta_t, self.link_dimensions)
        self.goal_states = get_goal_states("mpc", 
                                           serializer, 
                                           obstacles, 
                                           self.link_dimensions, 
                                           self.workspace_dimension,
                                           self.max_velocity,
                                           self.delta_t,
                                           self.joint_constraints,
                                           self.enforce_constraints,                                           
                                           self.theta_0,
                                           self.goal_position,
                                           self.planning_algorithm)
        if self.goal_states == None:
            return         
        
        if check_positive_definite([C, D]):            
            m_covs = np.linspace(self.min_covariance, self.max_covariance, self.covariance_steps)
            emds = []
            self.mpc(self.theta_0, m_covs, self.horizon, obstacles, serializer)
                       
            #stats = dict(m_cov = m_covs.tolist(), emd = emds)
            #serializer.save_paths(best_paths, 'best_paths.yaml', True, path="stats")
            #serializer.save_cartesian_coords(cartesian_coords, path=dir, filename="cartesian_coords_mpc" + str(self.num_execution_steps) + ".yaml")
            
            #serializer.save_stats(stats, path=dir)
            '''average_distances_to_goal_area = []            
            for i in xrange(len(cartesian_coords)):
                average_distances_to_goal_area.append(self.get_average_distance_to_goal_area(self.goal_position, 
                                                                                             self.goal_radius, 
                                                                                             cartesian_coords[i]))'''
            
            if not os.path.exists(dir + "/environment"):
                os.makedirs(dir + "/environment") 
                       
            cmd = "cp environment/env.xml " + dir + "/environment"
            os.system(cmd)
            if not os.path.exists(dir + "/model"):
                os.makedirs(dir + "/model")
                
            cmd = "cp " + model_file + " " + dir + "/model"
            os.system(cmd)
            if plot:
                PlotStats(True, "mpc")
                
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
    
    def init_serializer(self):
        serializer = Serializer()        
        return serializer    
            
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
                                                              self.w1,
                                                              self.w2)
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
                     total_eval_times) = self.path_planning_interface.plan_and_evaluate_paths(self.num_paths, 0, horizon, P_t, self.timeout)
                    
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
                    logging.warn("MPC: Success probability of best path: " + str(best_val))
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
        
            
    def problem_setup(self, delta_t, link_dimensions):        
        axis = v2_int()
        ax1 = v_int()
        ax2 = v_int()
        ax1[:] = [0, 0, 1]
        if self.workspace_dimension == 2:
            ax2[:] = [0, 0, 1]            
        elif self.workspace_dimension == 3:
            ax2[:] = [0, 1, 0]
        axis[:] = [ax1, ax2, ax1]        
        self.kinematics = Kinematics()
        self.kinematics.setLinksAndAxis(link_dimensions, axis)
        
        A = np.identity(len(link_dimensions))
        H = np.identity(len(link_dimensions))
        B = delta_t * np.identity(len(link_dimensions))
        V = np.identity(len(link_dimensions))
        W = np.identity(len(link_dimensions))
        C = 2.0 * np.identity(len(link_dimensions))
        D = 1.0 * np.identity(len(link_dimensions))
        return A, H, B, V, W, C, D
        
    def set_params(self, config):
        self.num_paths = config['num_generated_paths']
        self.use_linear_path = config['use_linear_path']        
        self.max_velocity = config['max_velocity']
        self.delta_t = 1.0 / config['control_rate']
        self.theta_0 = config['init_joint_angles']
        self.goal_position = config['goal_position']
        self.goal_state = np.array([-np.pi / 2.0, 0.0, 0.0])
        self.goal_radius = config['goal_radius']
        self.num_simulation_runs = config['num_simulation_runs']
        self.num_bins = config['num_bins']
        self.min_covariance = config['min_covariance']
        self.max_covariance = config['max_covariance']
        self.covariance_steps = config['covariance_steps']
        self.observation_covariance = config['observation_covariance']        
        self.discount_factor = config['discount_factor']
        self.illegal_move_penalty = config['illegal_move_penalty']
        self.step_penalty = config['step_penalty']
        self.exit_reward = config['exit_reward']
        self.stop_when_terminal = config['stop_when_terminal']
        self.horizon = config['horizon']
        self.max_num_steps = config['max_num_steps']
        self.verbose = config['verbose']
        self.joint_constraints = config['joint_constraints']
        self.enforce_constraints = config['enforce_constraints']
        self.planning_algorithm = config['planning_algorithm']
        self.sample_size = config['sample_size']  
        self.workspace_dimension = config['workspace_dimension']
        self.w1 = config['w1']
        self.w2 = config['w2']
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
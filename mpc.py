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
        serializer = Serializer()
        self.utils = Utils()
        config = serializer.read_config("config_mpc.yaml")
        self.set_params(config)
        
        logging_level = logging.WARN
        if self.verbose:
            logging_level = logging.DEBUG
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging_level)        
        
        dir = "stats/mpc" + str(self.num_execution_steps)
        if not os.path.exists(dir):
            os.makedirs(dir)
        cmd = "cp config_mpc.yaml " + dir            
        os.system(cmd)
        
        self.sim = Simulator()
        self.path_evaluator = PathEvaluator()
        self.path_planning_interface = PathPlanningInterface()
        
        """ Load the environment """ 
        environment = serializer.load_environment(file="env.xml", path="environment")       
        obstacles = []
        terrain = Terrain("default", 0.0, 1.0, True)
        logging.info("Loading obstacles")
        for obstacle in environment:            
            obstacles.append(Obstacle(obstacle[0][0], obstacle[0][1], obstacle[0][2], obstacle[1][0], obstacle[1][1], obstacle[1][2], terrain)) 
        
        """ Setup operations """
        self.sim.setup_reward_function(self.discount_factor, self.step_penalty, self.illegal_move_penalty, self.exit_reward)        
        self.path_planning_interface.setup(self.num_links, 
                                           self.workspace_dimension, 
                                           obstacles, 
                                           self.max_velocity, 
                                           self.delta_t, 
                                           self.use_linear_path, 
                                           self.joint_constraints)
        
        A, H, B, V, W, C, D = self.problem_setup(self.delta_t, self.num_links)
        self.goal_states = get_goal_states("mpc", 
                                           serializer, 
                                           obstacles, 
                                           self.num_links, 
                                           self.workspace_dimension,
                                           self.max_velocity,
                                           self.delta_t,
                                           self.joint_constraints,
                                           self.theta_0,
                                           self.goal_position)
        
        if check_positive_definite([C, D]):            
            m_covs = np.linspace(self.min_covariance, self.max_covariance, self.covariance_steps)
            emds = []
            (cartesian_coords, 
             rewards, 
             emds, 
             mean_planning_times_per_step, 
             mean_planning_times_per_run, 
             all_succesful_runs, 
             mean_number_generated_paths_per_step, 
             mean_number_generated_paths_per_run,
             mean_number_planning_steps_cov,
             mean_number_of_steps_per_run) = self.mpc(self.theta_0, m_covs, self.horizon, obstacles)
                       
            stats = dict(m_cov = m_covs.tolist(), emd = emds)
            #serializer.save_paths(best_paths, 'best_paths.yaml', True, path="stats")
            serializer.save_cartesian_coords(cartesian_coords, path=dir, filename="cartesian_coords_mpc" + str(self.num_execution_steps) + ".yaml")
            serializer.save_num_successes(all_succesful_runs, path=dir, filename="num_successes_mpc" + str(self.num_execution_steps) + ".yaml")            
            serializer.save_stats(stats, path=dir)
            serializer.save_mean_planning_times(mean_planning_times_per_step, path=dir, filename="mean_planning_times_per_step_mpc" + str(self.num_execution_steps) + ".yaml")
            serializer.save_mean_planning_times(mean_planning_times_per_run, path=dir, filename="mean_planning_times_per_run_mpc" + str(self.num_execution_steps) + ".yaml")
            serializer.save_mean_num_generated_paths(mean_number_generated_paths_per_step, 
                                                     path=dir,
                                                     filename="mean_num_generated_paths_per_step_mpc" + str(self.num_execution_steps) + ".yaml")
            serializer.save_mean_num_generated_paths(mean_number_generated_paths_per_run, 
                                                     path=dir,
                                                     filename="mean_num_generated_paths_per_run_mpc" + str(self.num_execution_steps) + ".yaml")
            serializer.save_mean_num_planning_steps(mean_number_planning_steps_cov,
                                                    path=dir,
                                                    filename="mean_num_planning_steps_per_run_mpc" + str(self.num_execution_steps) + ".yaml")
            serializer.save_mean_num_planning_steps(mean_number_of_steps_per_run,
                                                    path=dir,
                                                    filename="mean_num_steps_per_run_mpc" + str(self.num_execution_steps) + ".yaml")
            
            reward_variances = []
            mean_rewards = []
            for i in xrange(len(m_covs)):
                n, min_max, mean, var, skew, kurt = scipy.stats.describe(np.array(rewards[i]))                            
                mean_rewards.append(np.asscalar(mean))
                if np.isnan(np.asscalar(var)):
                    var = 0.0
                else:
                    var = np.asscalar(var)               
                reward_variances.append(var)
            
            serializer.save_rewards(rewards, path=dir, filename="rewards_mpc" + str(self.num_execution_steps) + ".yaml")    
            serializer.save_rewards(mean_rewards, path=dir, filename="mean_rewards_mpc" + str(self.num_execution_steps) + ".yaml")
            serializer.save_rewards(reward_variances, path=dir, filename="sample_variances_mpc" + str(self.num_execution_steps) + ".yaml")
            serializer.save_rewards([np.asscalar(np.sqrt(reward_variances[i])) for i in xrange(len(reward_variances))],
                                    path=dir,
                                    filename="sample_standard_deviations_mpc" + str(self.num_execution_steps) + ".yaml")
            
            if not os.path.exists(dir + "/environment"):
                os.makedirs(dir + "/environment") 
                       
            cmd = "cp environment/env.xml " + dir + "/environment"
            os.system(cmd)
            if plot:
                PlotStats(True, "mpc")     
            
    def mpc(self, initial_belief, m_covs, horizon, obstacles):
        A, H, B, V, W, C, D = self.problem_setup(self.delta_t, self.num_links)
        cart_coords = []
        mean_rewards = []
        total_rewards = []
        sample_variances = []
        mean_planning_times_per_step = []
        mean_planning_times_per_run = []
        all_successful_runs = []
        mean_number_generated_paths_per_step = []
        mean_number_generated_paths_per_run = []
        mean_number_planning_steps_cov = []
        mean_number_of_steps_per_run = []
        emds = []
        for j in xrange(len(m_covs)):            
            
            logging.info("MPC: Evaluating covariance " + str(m_covs[j]))
            
            """
            The process noise covariance matrix
            """
            M = m_covs[j] * np.identity(self.num_links)
                
            """
            The observation noise covariance matrix
            """
            N = self.observation_covariance * np.identity(self.num_links)
            P_t = np.array([[0.0 for k in xrange(self.num_links)] for l in xrange(self.num_links)])
                
            self.path_planning_interface.setup_path_evaluator(A, B, C, D, H, M, N, V, W, 
                                                              self.num_links,
                                                              self.workspace_dimension, 
                                                              self.sample_size, 
                                                              obstacles,
                                                              self.w1,
                                                              self.w2)
            self.sim.setup_problem(A, B, C, D, H, V, W, M, N, 
                                   obstacles, 
                                   self.goal_position, 
                                   self.goal_radius, 
                                   self.num_links,
                                   self.workspace_dimension, 
                                   self.joint_constraints)            
            self.sim.setup_simulator(self.num_simulation_runs, self.stop_when_terminal)
            
            mean_reward = 0.0
            cartesian_coords = []
            total_reward_cov = []            
            
            mean_planning_time_per_run = 0.0
            
            successful_runs = 0
            num_generated_paths_run = 0
            mean_number_planning_steps = 0 
            mean_number_of_steps = 0           
            for k in xrange(self.num_simulation_runs):
                #x_tilde = initial_belief               
                print "MPC: Joint covariance: " + str(m_covs[j])
                print "MPC: simulation run " + str(k + 1)
                x_true = initial_belief
                x_estimate = initial_belief
                total_reward = 0.0
                
                current_step = 0
                terminal = False                      
                              
                while current_step < self.max_num_steps and not terminal:                    
                    mean_number_planning_steps += 1
                    t0 = time.time()                    
                    self.path_planning_interface.set_start_and_goal(x_estimate, self.goal_states)
                    logging.info("MPC: Constructing paths")
                    xs, us, zs, num_generated_paths = self.path_planning_interface.plan_and_evaluate_paths(self.num_paths, 0, horizon, self.timeout)
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
                    #logging.info("MPC: " + str(len(paths)) + " Paths constructed. Evaluating them according the planning objective")
                    #xs, us, zs = self.path_evaluator.evaluate_paths(paths, horizon=horizon)                                     
                    logging.info("MPC: Generated " + str(num_generated_paths) + " paths in " + str(t_e) + " seconds")
                    x_tilde = np.array([0.0 for i in xrange(self.num_links)])
                    n_steps = self.num_execution_steps
                    if n_steps > len(xs) - 1:
                       n_steps = len(xs) - 1
                    if current_step + n_steps > self.max_num_steps:
                        n_steps = self.max_num_steps - current_step
                                          
                    logging.info("MPC: Execute best path for " + str(n_steps) + " steps")
                    x_true, x_tilde, x_estimate, P_t, current_step, total_reward, successful_runs, terminal = self.sim.simulate_n_steps(xs, us, zs, 
                                                                                                                                        x_true, 
                                                                                                                                        x_tilde,
                                                                                                                                        x_estimate,
                                                                                                                                        P_t,
                                                                                                                                        total_reward,
                                                                                                                                        successful_runs,
                                                                                                                                        current_step,
                                                                                                                                        n_steps) 
                                        
                    logging.info("MPC: Execution finished. True state is " + str(x_true))
                    logging.info("MPC: Estimated state is " + str(x_estimate))                
                total_reward_cov.append(np.asscalar(total_reward))
                mean_reward += total_reward                
                mean_number_of_steps += current_step                
                x_true_vec = v_double()
                x_true_vec[:] = x_true
                ee_position_vec = self.kinematics.getEndEffectorPosition(x_true_vec)
                ee_position = np.array([ee_position_vec[i] for i in xrange(len(ee_position_vec))])
                cartesian_coords.append(ee_position.tolist())                
                logging.info("MPC: Done. total_reward is " + str(total_reward))
            logging.info("MPC: Finished simulations for covariance value  " + 
                         str(m_covs[j]) +
                         ". Mean reward is " + 
                         str(mean_reward))
            total_rewards.append(total_reward_cov)
            mean_reward /= self.num_simulation_runs            
            mean_rewards.append(np.asscalar(mean_reward))
            
            mean_number_planning_steps /= self.num_simulation_runs
            mean_number_planning_steps_cov.append(mean_number_planning_steps)
            
            mean_number_of_steps_per_run.append(mean_number_of_steps / self.num_simulation_runs)
            
            mean_planning_times_per_run.append(mean_planning_time_per_run / self.num_simulation_runs)
            mean_planning_times_per_step.append(mean_planning_times_per_run[-1] / mean_number_planning_steps)
            
            mean_number_generated_paths_per_run.append(num_generated_paths_run / self.num_simulation_runs)
            mean_number_generated_paths_per_step.append(mean_number_generated_paths_per_run[-1] / mean_number_planning_steps)            
            
            all_successful_runs.append((100.0 / self.num_simulation_runs) * successful_runs)
            emds.append(calc_EMD(cartesian_coords, self.num_bins))
            cart_coords.append([cartesian_coords[i] for i in xrange(len(cartesian_coords))])            
        return (cart_coords, 
                total_rewards, 
                emds, 
                mean_planning_times_per_step, 
                mean_planning_times_per_run, 
                all_successful_runs, 
                mean_number_generated_paths_per_step, 
                mean_number_generated_paths_per_run,
                mean_number_planning_steps_cov,
                mean_number_of_steps_per_run)
            
    def problem_setup(self, delta_t, num_links):
        links = v2_double()
        axis = v2_int()
        
        link = v_double()
        ax1 = v_int()
        ax2 = v_int()
        link[:] = [1.0, 0.0, 0.0]
        links[:] = [link for i in xrange(num_links)]
        
        ax1[:] = [0, 0, 1]
        if self.workspace_dimension == 2:
            ax2[:] = [0, 0, 1]            
        elif self.workspace_dimension == 3:
            ax2[:] = [0, 1, 0]
            
        axis[:] = [ax1, ax2, ax1]
        
        self.utils = Utils()
                
        self.kinematics = Kinematics()
        self.kinematics.setLinksAndAxis(links, axis)
        
        A = np.identity(num_links)
        H = np.identity(num_links)
        B = delta_t * np.identity(num_links)
        V = np.identity(num_links)
        W = np.identity(num_links)
        C = 2.0 * np.identity(num_links)
        D = 2.0 * np.identity(num_links)
        return A, H, B, V, W, C, D
        
    def set_params(self, config):
        self.num_paths = config['num_generated_paths']
        self.use_linear_path = config['use_linear_path']
        self.num_links = config['num_links']
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
        self.joint_constraints = [-config['joint_constraint'], config['joint_constraint']]
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
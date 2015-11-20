import numpy as np
import plot as Plot
import os
import glob
import sys
#import libutil
import logging
import scipy
from libutil import *
from serializer import Serializer
from libobstacle import *
from util_py import *
import kalman as kalman
from path_evaluator import PathEvaluator
from simulator import Simulator
from path_planning_interface import PathPlanningInterface
from EMD import *
from xml.dom import minidom
from gen_ik_solution import *
from history_entry import *


class LQG:
    def __init__(self, plot):
        np.set_printoptions(precision=16)
        dir = "stats/lqg"
        self.clear_stats(dir)
        sim = Simulator()
        path_evaluator = PathEvaluator()
        path_planner = PathPlanningInterface()
        self.init_serializer()        
        
        """ Reading the config """
        config = self.serializer.read_config("config_lqg.yaml")
        self.set_params(config) 
        
        logging_level = logging.WARN
        if config['verbose']:
            logging_level = logging.DEBUG
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging_level)
        
        self.utils = Utils()
        
        model_file = os.getcwd() + "/model/model.xml"
        urdf_model_file = "test.urdf"
        if self.workspace_dimension == 3:
            model_file = os.getcwd() + "/model/model3D.xml"
        self.setup_scene("environment", "env.xml", model_file)
                        
        print "LQG: Generating goal states..."
        goal_states = get_goal_states("lqg",
                                      self.serializer, 
                                      self.obstacles,                                      
                                      self.link_dimensions,
                                      self.workspace_dimension,
                                      self.max_velocity,
                                      self.delta_t,
                                      self.joint_constraints,
                                      self.enforce_constraints,
                                      self.start_state,
                                      self.goal_position,
                                      self.goal_radius,
                                      self.planning_algortihm)
        
        if len(goal_states) == 0:
            logging.error("LQG: Couldn't generate any goal states. Problem seems to be infeasible")
        print "LQG: " + str(len(goal_states)) + " goal states generated"         
        sim.setup_reward_function(self.discount_factor, self.step_penalty, self.illegal_move_penalty, self.exit_reward)  
        path_planner.setup(self.link_dimensions,
                           self.workspace_dimension,
                           self.obstacles,  
                           self.max_velocity, 
                           self.delta_t, 
                           self.use_linear_path, 
                           self.joint_constraints,
                           self.enforce_constraints,
                           self.planning_algortihm,
                           self.path_timeout)
        if self.dynamic_problem:
            path_planner.setup_dynamic_problem(urdf_model_file,
                                               self.simulation_step_size,
                                               self.coulomb,
                                               self.viscous)       
        path_planner.set_start_and_goal(self.start_state, goal_states, self.goal_position, self.goal_radius)         
        A, H, B, V, W, C, D = self.problem_setup(self.delta_t, len(self.link_dimensions))
        
        if check_positive_definite([C, D]):            
            m_covs = np.linspace(self.min_covariance, self.max_covariance, self.covariance_steps)            
            #m_covs = np.array([np.around(m_covs[i], 5) for i in xrange(len(m_covs))])            
            emds = []
            mean_planning_times = []
            time_to_generate_paths = 0.0
            if self.use_paths_from_file and len(glob.glob(os.path.join(dir, "paths.yaml"))) == 1:
                logging.info("LQG: Loading paths from file")
                in_paths = self.serializer.load_paths("paths.yaml", path=dir) 
                paths = []               
                for path in in_paths:
                    xs = []
                    us = []
                    zs = []
                    for j in xrange(len(path)):
                        xs.append([path[j][i] for i in xrange(len(self.link_dimensions))])
                        us.append([path[j][len(self.link_dimensions) + i] for i in xrange(len(self.link_dimensions))])
                        zs.append([path[j][2 * len(self.link_dimensions) + i] for i in xrange(len(self.link_dimensions))])
                    paths.append([xs, us, zs])
            else:
                print "LQG: Generating " + str(self.num_paths) + " paths from the inital state to the goal position..."
                t0 = time.time()
                paths = path_planner.plan_paths(self.num_paths, 0)
                if len(paths) == 0:
                    logging.error("LQG: Couldn't create any paths within the given time.")
                    return
                time_to_generate_paths = time.time() - t0 
                print "LQG: Time to generate paths: " + str(time_to_generate_paths) + " seconds"
                if self.plot_paths:                
                    self.serializer.save_paths(paths, "paths.yaml", self.overwrite_paths_file, path=dir)               
            
            
            """ Determine average path length """
            avg_path_length = self.get_avg_path_length(paths)            
            self.serializer.save_avg_path_lengths(avg_path_length, path=dir)
                                       
            cart_coords = []  
            best_paths = []
            all_rewards = []
            successes = []                   
            for j in xrange(len(m_covs)):
                print "LQG: Evaluating paths for covariance value " + str(m_covs[j]) + "..."
                """
                The process noise covariance matrix
                """
                M = m_covs[j] * np.identity(2 * len(self.link_dimensions))
                
                P_t = np.array([[0.0 for k in xrange(2 * len(self.link_dimensions))] for l in xrange(2 * len(self.link_dimensions))]) 
                
                """
                The observation noise covariance matrix
                """
                N = self.observation_covariance * np.identity(2 * len(self.link_dimensions))
                
                path_evaluator.setup(A, B, C, D, H, M, N, V, W, 
                                     self.link_dimensions,
                                     config['workspace_dimension'], 
                                     self.sample_size, 
                                     self.obstacles,
                                     self.joint_constraints,
                                     self.enforce_constraints,
                                     self.goal_position,
                                     self.goal_radius,
                                     self.w1,
                                     self.w2)
                if self.dynamic_problem:
                    path_evaluator.setup_dynamic_problem(self.delta_t)
                path_evaluator.setup_reward_function(self.step_penalty, self.illegal_move_penalty, self.exit_reward, self.discount_factor)
                t0 = time.time() 
                xs, us, zs, objective = path_evaluator.evaluate_paths(paths, P_t, 0)
                te = time.time() - t0
                print "LQG: Time to evaluate " + str(len(paths)) + " paths: " + str(te) + "s"
                mean_planning_time = time_to_generate_paths + te
                #mean_planning_times.append(time_to_generate_paths + (time.time() - t0))
                print "LQG: Best objective value: " + str(objective)
                print "LQG: Length of best path: " + str(len(xs))  
                best_paths.append([[xs[i] for i in xrange(len(xs))], 
                                   [us[i] for i in xrange(len(us))],
                                   [zs[i] for i in xrange(len(zs))]])
                
                sim.setup_problem(A, B, C, D, H, V, W, M, N, 
                                  self.obstacles, 
                                  self.goal_position, 
                                  self.goal_radius, 
                                  self.link_dimensions, 
                                  config['workspace_dimension'], 
                                  self.joint_constraints,
                                  self.enforce_constraints,
                                  self.max_velocity)
                sim.setup_simulator(self.num_simulation_runs, self.stop_when_terminal)
                if self.dynamic_problem:
                    sim.setup_dynamic_problem(urdf_model_file,
                                              self.coulomb,
                                              self.viscous,
                                              self.delta_t,
                                              self.simulation_step_size,
                                              True)
                
                successes = 0
                num_collisions = 0 
                rewards_cov = []
                print "LQG: Running " + str(self.num_simulation_runs) + " simulations..."              
                for k in xrange(self.num_simulation_runs):
                    self.serializer.write_line("log.log", "tmp/lqg", "RUN #" + str(k + 1) + " \n")
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
                     history_entries) = sim.simulate_n_steps(xs, us, zs,
                                                             xs[0],
                                                             np.array([0.0 for i in xrange(2 * len(self.link_dimensions))]),
                                                             xs[0],
                                                             np.array([[0.0 for k in xrange(2 * len(self.link_dimensions))] for l in xrange(2 * len(self.link_dimensions))]),
                                                             0.0,                                                           
                                                             0,
                                                             len(xs) - 1)
                    if success:
                        successes += 1
                    rewards_cov.append(total_reward)
                    for history_entry in history_entries:                        
                        history_entry.serialize("tmp/lqg", "log.log")
                        if history_entry.collided:
                            num_collisions += 1                                          
                    self.serializer.write_line("log.log", "tmp/lqg", "Reward: " + str(total_reward) + " \n") 
                    self.serializer.write_line("log.log", "tmp/lqg", "\n")             
                                          
                '''(cartesian_coords, 
                 rewards, 
                 successful_runs, 
                 all_collisions,
                 state_covariances,
                 estimated_states) = sim.simulate(xs, us, zs, m_covs[j])                
                                
                cart_coords.append([cartesian_coords[i] for i in xrange(len(cartesian_coords))])
                try:                             
                    emds.append(calc_EMD(cartesian_coords, 
                                         self.num_bins, 
                                         self.goal_position, 
                                         self.link_dimensions))
                except:
                    pass
                '''
                self.serializer.write_line("log.log", "tmp/lqg", "################################# \n")
                self.serializer.write_line("log.log",
                                      "tmp/lqg",
                                      "Process covariance: " + str(m_covs[j]) + " \n")
                self.serializer.write_line("log.log", "tmp/lqg", "Objective value of best path: " + str(objective) + " \n")                
                self.serializer.write_line("log.log", "tmp/lqg", "Mean num collisions per run: " + str(float(num_collisions) / float(self.num_simulation_runs)) + " \n")
                print "total num collisions " + str(num_collisions)    
                print "mean num collisions " + str(float(num_collisions) / float(self.num_simulation_runs))
                self.serializer.write_line("log.log", "tmp/lqg", "Length best path: " + str(len(xs)) + " \n")
                self.serializer.write_line("log.log", 
                                      "tmp/lqg", 
                                      "Average distance to goal area: 0 \n")
                self.serializer.write_line("log.log", "tmp/lqg", "Num successes: " + str(successes) + " \n")
                print "succ " + str((100.0 / self.num_simulation_runs) * successes)
                self.serializer.write_line("log.log", "tmp/lqg", "Percentage of successful runs: " + str((100.0 / self.num_simulation_runs) * successes) + " \n")
                self.serializer.write_line("log.log", "tmp/lqg", "Mean planning time: " + str(mean_planning_time) + " \n")
                
                n, min_max, mean, var, skew, kurt = scipy.stats.describe(np.array(rewards_cov))
                self.serializer.write_line("log.log", "tmp/lqg", "Mean rewards: " + str(mean) + " \n")
                self.serializer.write_line("log.log", "tmp/lqg", "Reward variance: " + str(var) + " \n")
                self.serializer.write_line("log.log", 
                                      "tmp/lqg", 
                                      "Reward standard deviation: " + str(np.sqrt(var)) + " \n")
                cmd = "mv tmp//lqg/log.log " + dir + "/log_lqg_" + str(m_covs[j]) + ".log"
                os.system(cmd)
                
            
            if self.plot_paths:
                self.serializer.save_paths(best_paths, 'best_paths.yaml', True, path=dir)                      
            #self.serializer.save_stats(stats, path=dir)
            
            cmd = "cp config_lqg.yaml " + dir           
            os.system(cmd)
            
            if not os.path.exists(dir + "/environment"):
                os.makedirs(dir + "/environment") 
                       
            cmd = "cp environment/env.xml " + dir + "/environment"
            os.system(cmd) 
            
            if not os.path.exists(dir + "/model"):
                os.makedirs(dir + "/model")
                
            cmd = "cp " + model_file + " " + dir + "/model"
            os.system(cmd)
        print "Done"        
        
    def setup_scene(self, 
                    environment_path, 
                    environment_file, 
                    model_file):
        """ Load the obstacles """ 
        environment = self.serializer.load_environment(file="env.xml", path="environment")       
        self.obstacles = []
        terrain = Terrain("default", 0.0, 1.0, False)                
        for obstacle in environment:                       
            self.obstacles.append(Obstacle(obstacle[0][0], obstacle[0][1], obstacle[0][2], obstacle[1][0], obstacle[1][1], obstacle[1][2], terrain))                    
        
        """ Setup operations """        
        self.link_dimensions = self.utils.getLinkDimensions(model_file)
            
    def init_serializer(self):
        self.serializer = Serializer()
        self.serializer.create_temp_dir("lqg")
        
            
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
            
    def problem_setup(self, delta_t, num_links):
        A = np.identity(num_links * 2)
        H = np.identity(num_links * 2)
        B = delta_t * np.identity(num_links * 2)
        #B = np.vstack((B, np.zeros((num_links, num_links))))        
        V = np.identity(num_links * 2)
        W = np.identity(num_links * 2)
        C = 5000.0 * np.identity(num_links * 2)
        
        D = 1.0 * np.identity(num_links * 2)
        #D  = np.vstack((D, np.zeros((num_links, num_links))))
        
        #D = 1.0 * np.identity(num_links * 2)
        return A, H, B, V, W, C, D
            
    def get_avg_path_length(self, paths):
        avg_length = 0.0
        for path in paths:            
            avg_length += len(path[0])
        return avg_length / len(paths)
        
    def set_params(self, config):        
        self.num_paths = config['num_generated_paths']
        self.use_linear_path = config['use_linear_path']        
        self.max_velocity = config['max_velocity']
        self.delta_t = 1.0 / config['control_rate']
        self.start_state = config['start_state']
        self.goal_position = config['goal_position']
        self.goal_state = np.array([-np.pi / 2.0, 0.0, 0.0])
        self.goal_radius = config['goal_radius']
        self.num_simulation_runs = config['num_simulation_runs']
        self.num_bins = config['num_bins']
        self.min_covariance = config['min_covariance']
        self.max_covariance = config['max_covariance']
        self.covariance_steps = config['covariance_steps']
        self.observation_covariance = config['observation_covariance']        
        self.use_paths_from_file = config['use_paths_from_file']        
        self.overwrite_paths_file = config['overwrite_paths_file']
        self.discount_factor = config['discount_factor']
        self.illegal_move_penalty = config['illegal_move_penalty']
        self.step_penalty = config['step_penalty']
        self.exit_reward = config['exit_reward']
        self.stop_when_terminal = config['stop_when_terminal']
        self.joint_constraints = config['joint_constraints']
        self.enforce_constraints = config['enforce_constraints']
        self.sample_size = config['sample_size']  
        self.workspace_dimension = config['workspace_dimension'] 
        self.w1 = config['w1']
        self.w2 = config['w2']
        self.plot_paths = config['plot_paths']
        self.planning_algortihm = config['planning_algorithm']
        self.dynamic_problem = config['dynamic_problem'] 
        self.simulation_step_size = config['simulation_step_size']
        self.coulomb = config['coulomb']
        self.viscous = config['viscous']
        self.path_timeout = config['path_timeout']          

if __name__ == "__main__":
    if len(sys.argv) > 1:
        if "plot" in sys.argv[1]:
            LQG(True)
            sys.exit()
        print "Unrecognized command line argument: " + str(sys.argv[1]) 
        sys.exit()   
    LQG(False)    
    
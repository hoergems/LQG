import numpy as np
import plot as Plot
import os
import glob
import sys
from serializer import Serializer
from obstacle import Obstacle
import kalman as kalman
from path_evaluator import PathEvaluator
from simulator import Simulator
from path_planning_interface import PathPlanningInterface
from EMD import *
from plot_stats import PlotStats
from xml.dom import minidom

class LQG:
    def __init__(self, plot):
        dir = "stats/LQG"
        sim = Simulator()
        path_evaluator = PathEvaluator()
        path_planner = PathPlanningInterface()    
        serializer = Serializer()        
        
        """ Reading the config """
        config = serializer.read_config("config.yaml")
        self.set_params(config) 
        
        """ Load the obstacles """        
        obstacles = serializer.load_environment("env.xml", path="environment")           
        
        """ Setup operations """
        sim.setup_reward_function(self.discount_factor, self.step_penalty, self.illegal_move_penalty, self.exit_reward)  
        path_planner.setup(obstacles, self.num_links, self.max_velocity, self.delta_t, self.use_linear_path, self.joint_constraints, config['verbose'])
        #self.path_planning_interface.setup(obstacles, self.num_links, self.max_velocity, self.delta_t, self.use_linear_path, self.joint_constraints, config['verbose'])
        path_planner.set_start_and_goal_state(self.theta_0, self.goal_state, self.goal_radius)      
        A, H, B, V, W, C, D = self.problem_setup(self.delta_t, self.num_links)
        
        if self.check_positive_definite([C, D]):            
            m_covs = np.linspace(self.min_covariance, self.max_covariance, self.covariance_steps)
            emds = []
            print "use from file " + str(self.use_paths_from_file)
            
            if self.use_paths_from_file and len(glob.glob(os.path.join(dir, "paths.yaml"))) == 1:
                print "Loading paths from file"
                in_paths = serializer.load_paths("paths.yaml", path=dir) 
                paths = []               
                for path in in_paths:
                    xs = []
                    us = []
                    zs = []
                    for j in xrange(len(path)):
                        xs.append([path[j][i] for i in xrange(self.num_links)])
                        us.append([path[j][self.num_links + i] for i in xrange(self.num_links)])
                        zs.append([path[j][2 * self.num_links + i] for i in xrange(self.num_links)])
                    paths.append([xs, us, zs])
            else:
                paths = path_planner.plan_paths(self.num_paths, 0, False)                   
                serializer.save_paths(paths, "paths.yaml", self.overwrite_paths_file, path=dir)               
            
            
            """ Determine average path length """
            avg_path_length = self.get_avg_path_length(paths)            
            serializer.save_avg_path_lengths(avg_path_length, path=dir)
                                       
            cart_coords = []  
            best_paths = []
            mean_rewards = []          
            for j in xrange(len(m_covs)):
                print "Evaluating covariance " + str(m_covs[j])
                """
                The process noise covariance matrix
                """
                M = m_covs[j] * np.identity(self.num_links)
                
                """
                The observation noise covariance matrix
                """
                N = self.observation_covariance * np.identity(self.num_links)
                
                path_evaluator.setup(A, B, C, D, H, M, N, V, W, self.num_links, self.sample_size, obstacles, config['verbose'])  
                xs, us, zs = path_evaluator.evaluate_paths(paths)
                                
                best_paths.append([[xs[i] for i in xrange(len(xs))], 
                                   [us[i] for i in xrange(len(us))],
                                   [zs[i] for i in xrange(len(zs))]])
                
                sim.setup_problem(A, B, C, D, H, V, W, M, N, obstacles, self.goal_position, self.goal_radius, self.num_links, self.joint_constraints)
                sim.setup_simulator(self.num_simulation_runs, self.stop_when_terminal, config['verbose'])           
                cartesian_coords, mean_reward = sim.simulate(xs, us, zs, j)
                                
                cart_coords.append([cartesian_coords[i] for i in xrange(len(cartesian_coords))])                
                emds.append(calc_EMD(cartesian_coords, self.num_bins))
                mean_rewards.append(mean_reward)            
            stats = dict(m_cov = m_covs.tolist(), emd = emds)
            serializer.save_paths(best_paths, 'best_paths.yaml', True, path=dir)
            serializer.save_cartesian_coords(cart_coords, path=dir, filename="cartesian_coords_LQG.yaml")            
            serializer.save_stats(stats, path=dir)            
            serializer.save_rewards(mean_rewards, path=dir)
            cmd = "cp config.yaml " + dir           
            os.system(cmd)
            
            if not os.path.exists(dir + "/environment"):
                os.makedirs(dir + "/environment") 
                       
            cmd = "cp environment/env.xml " + dir + "/environment"
            os.system(cmd)
            
            PlotStats(True, "LQG")
            
    def problem_setup(self, delta_t, num_links):
        A = np.identity(num_links)
        H = np.identity(num_links)
        B = delta_t * np.identity(num_links)
        V = np.identity(num_links)
        W = np.identity(num_links)
        C = 2.0 * np.identity(num_links)
        D = 2.0 * np.identity(num_links)
        return A, H, B, V, W, C, D
            
    def get_avg_path_length(self, paths):
        avg_length = 0.0
        for path in paths:            
            avg_length += len(path[0])
        return avg_length / len(paths)
        
    def check_positive_definite(self, matrices):
        for m in matrices:
            try:
                np.linalg.cholesky(m)
            except:
                print "Matrices are not positive definite. Fix that!"
                return False
        return True
        
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
        self.use_paths_from_file = config['use_paths_from_file']        
        self.overwrite_paths_file = config['overwrite_paths_file']
        self.discount_factor = config['discount_factor']
        self.illegal_move_penalty = config['illegal_move_penalty']
        self.step_penalty = config['step_penalty']
        self.exit_reward = config['exit_reward']
        self.stop_when_terminal = config['stop_when_terminal']
        self.joint_constraints = [-config['joint_constraint'], config['joint_constraint']]
        self.sample_size = config['sample_size']        

if __name__ == "__main__":
    if len(sys.argv) > 1:
        if "plot" in sys.argv[1]:
            LQG(True)
            sys.exit()
        print "Unrecognized command line argument: " + str(sys.argv[1]) 
        sys.exit()   
    LQG(False)
    
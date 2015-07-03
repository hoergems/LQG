import numpy as np
import plot as Plot
import os
import glob
from serializer import Serializer
from obstacle import Obstacle
import kalman as kalman
from path_evaluator import PathEvaluator
from simulator import Simulator
from path_planning_interface import PathPlanningInterface
from EMD import *

class LQG:
    def __init__(self):
        sim = Simulator()
        path_evaluator = PathEvaluator()
        path_planner = PathPlanningInterface()    
        serializer = Serializer()
        
        """ Reading the config """
        config = serializer.read_config("config.yaml")
        self.set_params(config) 
        
        """ Load the obstacles """
        obstacle_params = serializer.load_obstacles("obstacles.yaml", path="obstacles")
        obstacles = self.construct_obstacles(obstacle_params)                      
        
        """ Setup operations """
        sim.setup_reward_function(self.discount_factor, self.step_penalty, self.illegal_move_penalty, self.exit_reward)  
        path_planner.setup(obstacles, self.num_links, self.max_velocity, self.delta_t, self.use_linear_path)
        path_planner.set_start_and_goal_state(self.theta_0, self.goal_state, self.goal_radius)      
        A, H, B, V, W, C, D = self.problem_setup(self.delta_t, self.num_links)
        
        if self.check_positive_definite([C, D]):            
            m_covs = np.linspace(self.min_covariance, self.max_covariance, self.covariance_steps)
            emds = []
            if self.use_paths_from_file and len(glob.glob(os.path.join("stats", "paths.yaml"))) == 1:
                print "Loading paths from file"
                in_paths = serializer.load_paths("paths.yaml", path="stats") 
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
                paths = path_planner.plan_paths(self.num_paths, 0)                   
                serializer.save_paths(paths, "paths.yaml", self.overwrite_paths_file, path="stats")               
            
            
            """ Determine average path length """
            avg_path_length = self.get_avg_path_length(paths)            
            serializer.save_avg_path_lengths(avg_path_length, path="stats")
                                       
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
                
                path_evaluator.setup(A, B, C, D, H, M, N, V, W, self.num_links, obstacles)  
                xs, us, zs = path_evaluator.evaluate_paths(paths)
                                
                best_paths.append([[xs[i] for i in xrange(len(xs))], 
                                   [us[i] for i in xrange(len(us))],
                                   [zs[i] for i in xrange(len(zs))]])
                
                sim.setup_problem(A, B, C, D, H, V, W, M, N, obstacles, self.goal_position, self.goal_radius, self.num_links)
                sim.setup_simulator(self.num_simulation_runs, self.stop_when_terminal)           
                cartesian_coords, mean_reward = sim.simulate(xs, us, zs, j)
                                
                cart_coords.append([cartesian_coords[i] for i in xrange(len(cartesian_coords))])                
                emds.append(calc_EMD(cartesian_coords, self.num_bins))
                mean_rewards.append(mean_reward)            
            stats = dict(m_cov = m_covs.tolist(), emd = emds)
            serializer.save_paths(best_paths, 'best_paths.yaml', True, path="stats")
            serializer.save_cartesian_coords(cart_coords, path="stats")            
            serializer.save_stats(stats, path="stats")            
            serializer.save_rewards(mean_rewards, path="stats")
            cmd = "cp config.yaml stats/"            
            os.system(cmd)
            cmd = "cp obstacles/obstacles.yaml stats/"
            os.system(cmd)
            
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
            
    def construct_obstacles(self, obstacle_params):
        obstacle_list = []
        if not obstacle_params == None:
            for o in obstacle_params:
                obstacle_list.append(Obstacle(o[0], o[1], o[2], o[3]))
        return obstacle_list        
        
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

if __name__ == "__main__":
    LQG()
    
from path_evaluator import PathEvaluator
from simulator import Simulator
from path_planning_interface import PathPlanningInterface
from serializer import Serializer
from obstacle import Obstacle
from kinematics import Kinematics
from EMD import *
from plot_stats import PlotStats
import numpy as np
import os
import sys
import time
from xml.dom import minidom

class MPC:
    def __init__(self, plot):
        """ Reading the config """
        serializer = Serializer() 
        config = serializer.read_config("config_mpc.yaml")
        self.set_params(config)        
        
        dir = "stats/mpc" + str(self.num_execution_steps)
        if not os.path.exists(dir):
            os.makedirs(dir)
        cmd = "cp config_mpc.yaml " + dir            
        os.system(cmd)
        
        self.sim = Simulator()
        self.path_evaluator = PathEvaluator()
        self.path_planning_interface = PathPlanningInterface()
        
        """ Load the environment """        
        obstacles = serializer.load_environment("env.xml", path="environment")
        
        """ Setup operations """
        self.sim.setup_reward_function(self.discount_factor, self.step_penalty, self.illegal_move_penalty, self.exit_reward)        
        self.path_planning_interface.setup(obstacles, self.num_links, self.max_velocity, self.delta_t, self.use_linear_path, self.joint_constraints, config['verbose'])
        
        A, H, B, V, W, C, D = self.problem_setup(self.delta_t, self.num_links)
        
        if self.check_positive_definite([C, D]):            
            m_covs = np.linspace(self.min_covariance, self.max_covariance, self.covariance_steps)
            emds = []
            cartesian_coords, mean_rewards, emds, mean_planning_times = self.mpc(self.theta_0, m_covs, self.horizon, obstacles, config['verbose'])
                       
            stats = dict(m_cov = m_covs.tolist(), emd = emds)
            #serializer.save_paths(best_paths, 'best_paths.yaml', True, path="stats")
            serializer.save_cartesian_coords(cartesian_coords, path=dir)            
            serializer.save_stats(stats, path=dir)            
            serializer.save_rewards(mean_rewards, path=dir)
            serializer.save_mean_planning_times(mean_planning_times, path=dir)
            
            if not os.path.exists(dir + "/environment"):
                os.makedirs(dir + "/environment") 
                       
            cmd = "cp environment/env.xml " + dir + "/environment"
            os.system(cmd)
            if plot:
                PlotStats(True, "mpc")
                
    def load_environment(self):
        xmldoc = minidom.parse('environment/env.xml') 
        obstacle_translations = xmldoc.getElementsByTagName('Translation')
        obstacle_dimensions = xmldoc.getElementsByTagName('extents')
        obstacles = []
        for i in xrange(len(obstacle_translations)):
            trans = [float(k) for k in obstacle_translations[i].childNodes[0].nodeValue.split(" ")]
            dim =  [float(k) for k in obstacle_dimensions[i].childNodes[0].nodeValue.split(" ")] 
            obstacles.append(Obstacle(trans[0], trans[1], 2.0 * dim[0], 2.0 * dim[1]))        
        return obstacles                   
            
    def mpc(self, initial_belief, m_covs, horizon, obstacles, verbose):
        A, H, B, V, W, C, D = self.problem_setup(self.delta_t, self.num_links)
        cart_coords = []
        mean_rewards = []
        mean_planning_times = []
        emds = []
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
            P_t = np.array([[0.0 for k in xrange(self.num_links)] for l in xrange(self.num_links)])
                
            self.path_evaluator.setup(A, B, C, D, H, M, N, V, W, self.num_links, self.sample_size, obstacles, verbose)
            self.sim.setup_problem(A, B, C, D, H, V, W, M, N, obstacles, self.goal_position, self.goal_radius, self.num_links, self.joint_constraints)            
            self.sim.setup_simulator(self.num_simulation_runs, self.stop_when_terminal, verbose)
            
            mean_reward = 0.0
            cartesian_coords = []
            mean_planning_time = 0.0
            for k in xrange(self.num_simulation_runs):
                #x_tilde = initial_belief               
                
                x_true = initial_belief
                x_estimate = initial_belief
                total_reward = 0.0
                
                current_step = 0
                terminal = False
                planning_time = 0.0                
                while current_step < self.max_num_steps and not terminal:
                    t0 = time.time()
                    self.path_planning_interface.set_start_and_goal_state(x_estimate, self.goal_state, self.goal_radius)
                    paths = self.path_planning_interface.plan_paths(self.num_paths, 0, self.verbose_rrt)
                    if len(paths) == 0:
                        print "unsolvable situation"
                        total_reward = np.array([-self.illegal_move_penalty])[0]
                        current_step += 1
                        print "break"                       
                        break
                    print "evaluate paths..."
                    xs, us, zs = self.path_evaluator.evaluate_paths(paths, horizon=horizon)
                    planning_time += time.time() - t0
                    print "planning time: " + str(time.time() - t0)
                    print "evaluation done. Average planning time: " + str(planning_time / (current_step + 1.0))
                    x_tilde = np.array([0.0 for i in xrange(self.num_links)])
                    
                    n_steps = self.num_execution_steps
                    if n_steps > len(xs) - 1:
                       n_steps = len(xs) - 1                    
                    print "run for " + str(n_steps) + " steps"                    
                    x_true, x_tilde, x_estimate, P_t, current_step, total_reward, terminal = self.sim.simulate_n_steps(xs, us, zs, 
                                                                                                                       x_true, 
                                                                                                                       x_tilde,
                                                                                                                       x_estimate,
                                                                                                                       P_t,
                                                                                                                       total_reward,
                                                                                                                       current_step,
                                                                                                                       n_steps) 
                    #x_estimate = x_tilde + xs[n_steps]
                    print "m_cov: " + str(m_covs[j])
                    print "run: " + str(k)
                    print "step: " + str(current_step)
                    if verbose:
                        print "x_true " + str(x_true)
                        print "x_estimate " + str(x_estimate)
                mean_reward += total_reward
                mean_planning_time += (planning_time / current_step)
                ee_position = self.kinematics.get_end_effector_position(x_true)
                cartesian_coords.append(ee_position.tolist())
                print "total_reward " + str(total_reward)
            print "mean_reward " + str(mean_reward)
            mean_reward /= self.num_simulation_runs            
            mean_rewards.append(np.asscalar(mean_reward))            
            mean_planning_time /= self.num_simulation_runs
            mean_planning_times.append(mean_planning_time)
            emds.append(calc_EMD(cartesian_coords, self.num_bins))
            cart_coords.append([cartesian_coords[i] for i in xrange(len(cartesian_coords))])
        print "mean_rewards " + str(mean_rewards)        
        return cart_coords, mean_rewards, emds, mean_planning_times
            
    def problem_setup(self, delta_t, num_links):
        self.kinematics = Kinematics(num_links)
        A = np.identity(num_links)
        H = np.identity(num_links)
        B = delta_t * np.identity(num_links)
        V = np.identity(num_links)
        W = np.identity(num_links)
        C = 2.0 * np.identity(num_links)
        D = 2.0 * np.identity(num_links)
        return A, H, B, V, W, C, D
        
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
        self.horizon = config['horizon']
        self.max_num_steps = config['max_num_steps']
        self.verbose_rrt = config['verbose_rrt']
        self.joint_constraints = [-config['joint_constraint'], config['joint_constraint']]
        self.sample_size = config['sample_size']  
        
        """
        The number of steps a path is being executed before a new path gets calculated
        """
        self.num_execution_steps = config['num_execution_steps']
    
if __name__ == "__main__":
    if len(sys.argv) > 1:
        if "plot" in sys.argv[1]:
            MPC(True)
            sys.exit()
        print "Unrecognized command line argument: " + str(sys.argv[1]) 
        sys.exit()   
    MPC(False)
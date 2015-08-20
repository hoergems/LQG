from path_evaluator import PathEvaluator
from simulator import Simulator
from path_planning_interface import PathPlanningInterface
from serializer import Serializer
from obstacle import Obstacle
from kin import *
from util import *
from obstacle import *
from EMD import *
from plot_stats import PlotStats
import numpy as np
import os
import sys
import time
from xml.dom import minidom
from gen_ik_solution import *

class MPC:
    def __init__(self, plot):
        """ Reading the config """
        serializer = Serializer()
        self.utils = Utils()
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
        environment = serializer.load_environment(file="env.xml", path="environment")       
        obstacles = []
        terrain = Terrain("default", 0.0, 1.0, True)
        for obstacle in environment:
            print obstacle
            obstacles.append(Obstacle(obstacle[0][0], obstacle[0][1], obstacle[0][2], obstacle[1][0], obstacle[1][1], obstacle[1][2], terrain)) 
        
        """ Setup operations """
        self.sim.setup_reward_function(self.discount_factor, self.step_penalty, self.illegal_move_penalty, self.exit_reward)        
        self.path_planning_interface.setup(self.num_links, 
                                           self.workspace_dimension, 
                                           obstacles, 
                                           self.max_velocity, 
                                           self.delta_t, 
                                           self.use_linear_path, 
                                           self.joint_constraints, 
                                           config['verbose'])
        
        A, H, B, V, W, C, D = self.problem_setup(self.delta_t, self.num_links)
        self.goal_states = self.get_goal_states(serializer, obstacles)
        
        if self.check_positive_definite([C, D]):            
            m_covs = np.linspace(self.min_covariance, self.max_covariance, self.covariance_steps)
            emds = []
            cartesian_coords, mean_rewards, total_rewards, sample_variances, emds, mean_planning_times = self.mpc(self.theta_0, m_covs, self.horizon, obstacles, config['verbose'])
                       
            stats = dict(m_cov = m_covs.tolist(), emd = emds)
            #serializer.save_paths(best_paths, 'best_paths.yaml', True, path="stats")
            serializer.save_cartesian_coords(cartesian_coords, path=dir, filename="cartesian_coords_mpc" + str(self.num_execution_steps) + ".yaml")            
            serializer.save_stats(stats, path=dir)            
            serializer.save_rewards(mean_rewards, path=dir)
            serializer.save_mean_planning_times(mean_planning_times, path=dir)
            serializer.save_total_rewards(total_rewards, path=dir)
            serializer.save_sample_variances(sample_variances, path=dir)
            
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
        total_rewards = []
        sample_variances = []
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
                
            self.path_evaluator.setup(A, B, C, D, H, M, N, V, W, 
                                      self.num_links,
                                      self.workspace_dimension, 
                                      self.sample_size, 
                                      obstacles, 
                                      verbose)
            self.sim.setup_problem(A, B, C, D, H, V, W, M, N, 
                                   obstacles, 
                                   self.goal_position, 
                                   self.goal_radius, 
                                   self.num_links,
                                   self.workspace_dimension, 
                                   self.joint_constraints)            
            self.sim.setup_simulator(self.num_simulation_runs, self.stop_when_terminal, verbose)
            
            mean_reward = 0.0
            cartesian_coords = []
            total_reward_cov = []
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
                    print "gs " + str(self.goal_states)
                    self.path_planning_interface.set_start_and_goal(x_estimate, self.goal_states)
                    paths = self.path_planning_interface.plan_paths(self.num_paths, 0, self.verbose)
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
                    if current_step + n_steps > self.max_num_steps:
                        n_steps = self.max_num_steps - current_step
                                          
                    print "run for " + str(n_steps) + " steps" 
                    print "current step " + str(current_step)                   
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
                total_reward_cov.append(np.asscalar(total_reward))
                mean_reward += total_reward
                mean_planning_time += (planning_time / current_step)
                
                x_true_vec = v_double()
                x_true_vec[:] = x_true
                ee_position_vec = self.kinematics.getEndEffectorPosition(x_true_vec)
                ee_position = np.array([ee_position_vec[i] for i in xrange(len(ee_position_vec))])
                cartesian_coords.append(ee_position.tolist())
                print "total_reward " + str(total_reward)
            print "mean_reward " + str(mean_reward)
            
            """Calculate sample variance """
            mean = sum(total_reward_cov) / len(total_reward_cov)
            print "MEAN " + str(mean)            
            variance = sum([np.square(total_reward_cov[i] - mean) for i in xrange(len(total_reward_cov))]) / len(total_reward_cov)
            print "VARIANCE " + str(variance)
            sample_variances.append(np.asscalar(variance))
            total_rewards.append(total_reward_cov)
            mean_reward /= self.num_simulation_runs            
            mean_rewards.append(np.asscalar(mean_reward))            
            mean_planning_time /= self.num_simulation_runs
            mean_planning_times.append(mean_planning_time)
            emds.append(calc_EMD(cartesian_coords, self.num_bins))
            cart_coords.append([cartesian_coords[i] for i in xrange(len(cartesian_coords))])
        print "mean_rewards " + str(mean_rewards)        
        return cart_coords, mean_rewards, total_rewards, sample_variances, emds, mean_planning_times
            
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
    
    def get_goal_states(self, serializer, obstacles):
        #goal_states = [np.array(gs) for gs in serializer.load_goal_states("goal_states.yaml")]
        #return goal_states
        ik_solution_generator = IKSolutionGenerator()
        model_file = "model/model.xml"
        if self.workspace_dimension == 3:
            model_file = "model/model3D.xml"
        ik_solution_generator.setup(self.num_links,
                                    self.workspace_dimension,
                                    obstacles,
                                    self.max_velocity,
                                    self.delta_t,
                                    self.joint_constraints,
                                    model_file,
                                    "environment/env.xml",
                                    self.verbose)
        solutions = ik_solution_generator.generate(self.theta_0, self.goal_position, self.workspace_dimension)
        '''print "goal position " + str(self.goal_position)
        print "solutions " + str(solutions)
        vecs = [v_double() for i in xrange(len(solutions))]
        for i in xrange(len(vecs)):
            vecs[i][:] = solutions[i]
            print "EE " + str([k for k in self.kinematics.getEndEffectorPosition(vecs[i])])
        sleep'''
        serializer.save_goal_states([solutions[i] for i in xrange(len(solutions))])
        
        return solutions
        
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
        self.discount_factor = config['discount_factor']
        self.illegal_move_penalty = config['illegal_move_penalty']
        self.step_penalty = config['step_penalty']
        self.exit_reward = config['exit_reward']
        self.stop_when_terminal = config['stop_when_terminal']
        self.horizon = config['horizon']
        self.max_num_steps = config['max_num_steps']
        self.verbose = config['verbose_rrt']
        self.joint_constraints = [-config['joint_constraint'], config['joint_constraint']]
        self.sample_size = config['sample_size']  
        self.workspace_dimension = config['workspace_dimension']
        
        
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
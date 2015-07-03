from path_evaluator import PathEvaluator
from simulator import Simulator
from path_planning_interface import PathPlanningInterface
from serializer import Serializer
from obstacle import Obstacle
import numpy as np

class MPC:
    def __init__(self):
        self.sim = Simulator()
        self.path_evaluator = PathEvaluator()
        self.path_planner = PathPlanningInterface()    
        serializer = Serializer()
        
        """ Reading the config """
        config = serializer.read_config("config_mpc.yaml")
        self.set_params(config) 
        
        """ Load the obstacles """
        obstacle_params = serializer.load_obstacles("obstacles.yaml", path="obstacles")
        obstacles = self.construct_obstacles(obstacle_params)
        
        """ Setup operations """
        self.sim.setup_reward_function(self.discount_factor, self.step_penalty, self.illegal_move_penalty, self.exit_reward)        
        self.path_planner.setup(obstacles, self.num_links, self.max_velocity, self.delta_t, self.use_linear_path)
        
        A, H, B, V, W, C, D = self.problem_setup(self.delta_t, self.num_links)
        
        if self.check_positive_definite([C, D]):            
            m_covs = np.linspace(self.min_covariance, self.max_covariance, self.covariance_steps)
            emds = []
            self.mpc(self.theta_0, m_covs, self.horizon, obstacles)
            
    def mpc(self, initial_belief, m_covs, horizon, obstacles):
        A, H, B, V, W, C, D = self.problem_setup(self.delta_t, self.num_links)
        x_tilde = initial_belief
        x_true = initial_belief
        total_reward = 0.0
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
                
            self.path_evaluator.setup(A, B, C, D, H, M, N, V, W, self.num_links, obstacles)
            self.sim.setup_problem(A, B, C, D, H, V, W, M, N, obstacles, self.goal_position, self.goal_radius, self.num_links)            
            self.sim.setup_simulator(self.num_simulation_runs, self.stop_when_terminal)
            
            current_step = 0
            terminal = False
            
            while current_step < self.max_num_steps and not terminal:
                self.path_planner.set_start_and_goal_state(x_true, self.goal_state, self.goal_radius)
                paths = self.path_planner.plan_paths(self.num_paths, 0)
                print "evaluate paths..."
                xs, us, zs = self.path_evaluator.evaluate_paths(paths, horizon=horizon)
                print "evaluation done"
                
                x_true, x_tilde, P_t, total_reward, terminal = self.sim.simulate_step(xs, us, zs, 
                                                                                      x_true, 
                                                                                      x_tilde,
                                                                                      P_t,
                                                                                      total_reward,
                                                                                      current_step)                
                current_step += 1
            print "total_reward " + str(total_reward)
            
    def problem_setup(self, delta_t, num_links):
        A = np.identity(num_links)
        H = np.identity(num_links)
        B = delta_t * np.identity(num_links)
        V = np.identity(num_links)
        W = np.identity(num_links)
        C = 2.0 * np.identity(num_links)
        D = 2.0 * np.identity(num_links)
        return A, H, B, V, W, C, D   
            
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
        self.horizon = config['horizon']
        self.max_num_steps = config['max_num_steps'] 
    
if __name__ == "__main__":
    MPC()
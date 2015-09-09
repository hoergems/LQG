import numpy as np
import kalman as kalman
import logging
from scipy.stats import multivariate_normal
from kin import *
from util import *

class Simulator:
    def __init__(self):
        pass
    
    def setup_problem(self,
                      A,
                      B,
                      C,
                      D,
                      H,
                      V,
                      W,
                      M,
                      N,
                      obstacles,
                      goal_position,
                      goal_radius,
                      link_dimensions,                      
                      workspace_dimension,
                      joint_constraints):
        self.A = A
        self.B = B
        self.C = C
        self.D = D
        self.H = H
        self.V = V
        self.W = W
        self.M = M
        self.N = N
        self.obstacles = obstacles
        self.goal_position = goal_position
        self.goal_radius = goal_radius
        self.link_dimensions = link_dimensions 
        self.workspace_dimension = workspace_dimension   
        self.joint_constraints = joint_constraints        
        
    def setup_reward_function(self, discount_factor, step_penalty, illegal_move_penalty, exit_reward):
        self.discount_factor = discount_factor
        self.step_penalty = step_penalty
        self.illegal_move_penalty = illegal_move_penalty
        self.exit_reward = exit_reward
    
    def setup_simulator(self, num_simulation_runs, stop_when_terminal):
        axis = v2_int()
        ax1 = v_int()
        ax2 = v_int()
        ax1[:] = [0, 0, 1]
        if self.workspace_dimension == 2:
            ax2[:] = [0, 0, 1]            
        elif self.workspace_dimension == 3:
            ax2[:] = [0, 1, 0]            
        axis[:] = [ax1, ax2, ax1]        
        self.utils = Utils()                
        self.kinematics = Kinematics()        
        self.kinematics.setLinksAndAxis(self.link_dimensions, axis)               
        self.num_simulation_runs = num_simulation_runs        
        self.stop_when_terminal = stop_when_terminal
        
    
    
    def simulate_n_steps(self,
                         xs, us, zs,
                         x_true,                         
                         x_tilde,
                         x_estimate,
                         P_t,
                         total_reward,
                         successes,
                         current_step,
                         n_steps):
        terminal_state_reached = False
        Ls = kalman.compute_gain(self.A, self.B, self.C, self.D, len(xs))
        logging.info("Simulator: Executing for " + str(n_steps) + " steps")        
        for i in xrange(n_steps):                        
            if not (terminal_state_reached and self.stop_when_terminal):
                u_dash = np.dot(Ls[i + 1], x_tilde)        
                x_true, collided = self.apply_control(x_true, np.add(u_dash, us[i]), self.A, self.B, self.V, self.M)
                discount = np.power(self.discount_factor, current_step)
                if collided:
                    total_reward += discount * (-1.0 * self.illegal_move_penalty)
                else:
                    total_reward += discount * (-1.0 * self.step_penalty)                
                
                state = v_double()
                state[:] = x_true   
                ee_position_arr = self.kinematics.getEndEffectorPosition(state)                
                ee_position = np.array([ee_position_arr[j] for j in xrange(len(ee_position_arr))])
                logging.info("Simulator: Current end-effector position is " + str(ee_position))
                if self.is_terminal(ee_position):
                    terminal_state_reached = True                        
                    total_reward += discount * self.exit_reward 
                    successes += 1                       
                    logging.info("Terminal state reached: reward = " + str(total_reward))                                  
                    
                z_t = self.get_observation(x_true, self.H, self.N, self.W)
                z_dash_t = z_t - zs[i]
                x_tilde_dash_t, P_dash = kalman.kalman_predict(x_tilde, u_dash, self.A, self.B, P_t, self.V, self.M)
                x_tilde, P_t = kalman.kalman_update(x_tilde_dash_t, 
                                                    z_dash_t, 
                                                    self.H, 
                                                    P_dash, 
                                                    self.W, 
                                                    self.N, 
                                                    len(self.link_dimensions))               
                x_estimate_new = self.check_constraints(x_tilde + xs[i + 1])
                
                if not self.is_in_collision(x_estimate_new):
                    x_estimate = x_estimate_new 
                                    
                #print "x_true " + str(x_true)
                #print "x_estimate " + str(x_estimate_new)            
        return x_true, x_tilde, x_estimate, P_t, current_step + n_steps, total_reward, successes, terminal_state_reached
    
    def check_constraints(self, state):        
        for i in xrange(len(state)):                          
            if state[i] < self.joint_constraints[0]:
                state[i] = self.joint_constraints[0] + 0.00001
            if state[i] > self.joint_constraints[1]:
                state[i] = self.joint_constraints[1] - 0.00001        
        return state
        
    def simulate(self, xs, us, zs, cov_value):
        Ls = kalman.compute_gain(self.A, self.B, self.C, self.D, len(xs) - 1)
        cart_coords = []
        rewards = []
        successes = 0
        
        for j in xrange(self.num_simulation_runs):
            print "Simulator: Execute simulation run " + str(j) + " for covariance value " + str(cov_value)
            x_true = xs[0]
            #x_tilde = xs[0]
            x_tilde = np.array([0.0 for i in xrange(len(self.link_dimensions))])        
            u_dash = np.array([0.0 for j in xrange(len(self.link_dimensions))])        
            P_t = np.array([[0.0 for k in xrange(len(self.link_dimensions))] for l in xrange(len(self.link_dimensions))])
            reward = 0.0
            terminal_state_reached = False          
            for i in xrange(0, len(xs) - 1):
                if not (terminal_state_reached and self.stop_when_terminal):                                
                    """
                    Generate u_dash using LQG
                    """                
                    u_dash = np.dot(Ls[i], x_tilde)
                                
                    """
                    Generate a true state and check for collision and terminal state
                    """                            
                    x_true, collided = self.apply_control(x_true, np.add(u_dash, us[i]), self.A, self.B, self.V, self.M)
                    discount = np.power(self.discount_factor, i)
                    if collided:
                        reward += discount * (-1.0 * self.illegal_move_penalty)
                    else:
                        reward += discount * (-1.0 * self.step_penalty)
                    state = v_double()
                    state[:] = x_true   
                    ee_position_arr = self.kinematics.getEndEffectorPosition(state)                
                    ee_position = np.array([ee_position_arr[k] for k in xrange(len(ee_position_arr))])
                    if self.is_terminal(ee_position):
                        terminal_state_reached = True
                        successes += 1                       
                        reward += discount * self.exit_reward                       
                    
                    """
                    Obtain an observation
                    """
                    z_t = self.get_observation(x_true, self.H, self.N, self.W)
                    z_dash_t = z_t - zs[i]
                                
                    """
                    Kalman prediction and update
                    """
                    x_tilde_dash_t, P_dash = kalman.kalman_predict(x_tilde, u_dash, self.A, self.B, P_t, self.V, self.M)
                    x_tilde, P_t = kalman.kalman_update(x_tilde_dash_t, 
                                                        z_dash_t, 
                                                        self.H, 
                                                        P_dash, 
                                                        self.W, 
                                                        self.N, 
                                                        len(self.link_dimensions))                   
            state = v_double()
            state[:] = x_true   
            ee_position_arr = self.kinematics.getEndEffectorPosition(state)                
            ee_position = np.array([ee_position_arr[i] for i in xrange(len(ee_position_arr))])
            cart_coords.append(ee_position.tolist())
            rewards.append(reward)
        return cart_coords, rewards, successes
    
    def is_in_collision(self, state):
        """
        Is the given end effector position in collision with an obstacle?
        """              
        joint_angles = v_double()
        joint_angles[:] = state
        collision_structures = self.utils.createManipulatorCollisionStructures(joint_angles,
                                                                               self.link_dimensions, 
                                                                               self.kinematics)
        for obstacle in self.obstacles:
            if obstacle.inCollisionDiscrete(collision_structures):                               
                return True
        return False
    
    def is_terminal(self, ee_position):        
        if np.linalg.norm(ee_position - self.goal_position) < self.goal_radius:                       
            return True
        return False
    
    def apply_control(self, x_dash, u_dash, A, B, V, M):        
        m = self.get_random_joint_angles([0.0 for i in xrange(len(self.link_dimensions))], M)        
        x_new = np.add(np.add(np.dot(A, x_dash), np.dot(B, u_dash)), np.dot(V, m))
        x_new = self.check_constraints(x_new)
        if self.is_in_collision(x_new):
            logging.info("Simulator: Collision detected. Setting state estimate to the previous state")
            return x_dash, True 
        return x_new, False
    
    def get_random_joint_angles(self, mu, cov):        
        #with self.lock:
        """
        Random numbers need to be generated by using a lock and creating a new random seed in order to avoid
        correlations between different processes
        """
        #np.random.seed()
        return np.random.multivariate_normal(mu, cov)
    
    def get_observation(self, true_theta, H, N, W):
        return np.add(np.dot(H, true_theta), 
                      np.dot(W, self.get_random_joint_angles([0.0 for i in xrange(len(self.link_dimensions))], N)))
    
if __name__ == "__main__":
    Simulator()
import numpy as np
import kalman as kalman
import logging
import time
from scipy.stats import multivariate_normal
from kin import *
from util import *
from history_entry import *

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
                      joint_constraints,
                      enforce_constraints):
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
        self.enforce_constraints = enforce_constraints       
        
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
                         current_step,
                         n_steps):
        history_entries = []        
        terminal_state_reached = False
        success = False
        Ls = kalman.compute_gain(self.A, self.B, self.C, self.D, len(xs) - 1)
        logging.info("Simulator: Executing for " + str(n_steps) + " steps") 
        estimated_states = []
        estimated_covariances = []        
        for i in xrange(n_steps):                        
            if not (terminal_state_reached and self.stop_when_terminal):                
                history_entries.append(HistoryEntry(current_step + i,
                                                    x_true, 
                                                    x_estimate, 
                                                    None,
                                                    None,
                                                    P_t,
                                                    False,
                                                    False,
                                                    False,
                                                    0.0))
                                
                u_dash = np.dot(Ls[i], x_tilde)                
                u = np.add(u_dash, us[i])
                
                history_entries[-1].set_action(u)
                        
                x_true_temp = self.apply_control(x_true, 
                                                 u, 
                                                 self.A, 
                                                 self.B, 
                                                 self.V, 
                                                 self.M)                
                discount = np.power(self.discount_factor, current_step + i)
                collided = False        
                if self.is_in_collision(x_true, x_true_temp):
                    logging.info("Simulator: Collision detected. Setting state estimate to the previous state")
                    total_reward += discount * (-1.0 * self.illegal_move_penalty)
                    history_entries[-1].set_reward(-1.0 * self.illegal_move_penalty)
                    collided = True
                else:
                    total_reward += discount * (-1.0 * self.step_penalty)
                    history_entries[-1].set_reward(-1.0 * self.step_penalty)
                    x_true = x_true_temp               
                x_dash = np.subtract(x_true, xs[i + 1])
                state = v_double()
                state[:] = x_true   
                ee_position_arr = self.kinematics.getEndEffectorPosition(state)                
                ee_position = np.array([ee_position_arr[j] for j in xrange(len(ee_position_arr))])
                logging.info("Simulator: Current end-effector position is " + str(ee_position))                                                 
                    
                z = self.get_observation(x_true, self.H, self.N, self.W)
                z_dash = np.subtract(z, zs[i+1])
                history_entries[-1].set_observation(z)
                
                """
                Kalman prediction and update
                """ 
                #if not collided:               
                x_tilde_dash, P_dash = kalman.kalman_predict(x_tilde, u_dash, self.A, self.B, P_t, self.V, self.M)
                x_tilde, P_t = kalman.kalman_update(x_tilde_dash, 
                                                    z_dash, 
                                                    self.H, 
                                                    P_dash, 
                                                    self.W, 
                                                    self.N, 
                                                    len(self.link_dimensions))
                x_estimate_new = x_tilde + xs[i + 1]
                
                if self.enforce_constraints:                                 
                    x_estimate_new = self.check_constraints(x_estimate_new) 
                estimate_collided = True                                       
                if not self.is_in_collision([], x_estimate_new):                                                                                
                    x_estimate = x_estimate_new
                    estimate_collided = False
                        
                            
                estimated_states.append(x_estimate)
                estimated_covariances.append(P_t)
                
                if self.is_terminal(ee_position):
                    history_entries.append(HistoryEntry(current_step + i + 1,
                                                        x_true, 
                                                        x_estimate, 
                                                        None,
                                                        None,
                                                        P_t,
                                                        False,
                                                        False,
                                                        True,
                                                        0.0))                    
                    terminal_state_reached = True                        
                    total_reward += discount * self.exit_reward
                    history_entries[-1].set_reward(self.exit_reward)
                    success = True                      
                    logging.info("Terminal state reached: reward = " + str(total_reward)) 
                    
                history_entries[-1].set_collided(collided)
                history_entries[-1].set_estimate_collided(estimate_collided)
                history_entries[-1].set_terminal(terminal_state_reached)
        return (x_true, 
                x_tilde, 
                x_estimate, 
                P_t, 
                current_step + n_steps, 
                total_reward, 
                success, 
                terminal_state_reached,
                estimated_states,
                estimated_covariances,                
                history_entries)
    
    def check_constraints(self, state):
        for i in xrange(len(state)):                          
            if state[i] < -self.joint_constraints[i]:
                state[i] = -self.joint_constraints[i] + 0.00001
            if state[i] > self.joint_constraints[i]:
                state[i] = self.joint_constraints[i] - 0.00001        
        return state
    
    
    def is_in_collision(self, previous_state, state, p=False):
        """
        Is the given end effector position in collision with an obstacle?
        """
        #previous_state = []
                    
        joint_angles_goal = v_double()
        joint_angles_goal[:] = state
        collision_structures = self.utils.createManipulatorCollisionStructures(joint_angles_goal,
                                                                               self.link_dimensions, 
                                                                               self.kinematics)
        for obstacle in self.obstacles:
            if obstacle.inCollisionDiscrete(collision_structures):                               
                return True
        return False  
        
        if len(previous_state) > 0:
            """
            Perform continuous collision checking if previous state is provided
            """
            joint_angles_start = v_double()        
            joint_angles_start[:] = previous_state        
            
            collision_objects_start = self.utils.createManipulatorCollisionObjects(joint_angles_start,
                                                                                   self.link_dimensions,
                                                                                   self.kinematics)
            collision_objects_goal = self.utils.createManipulatorCollisionObjects(joint_angles_goal,
                                                                                  self.link_dimensions,
                                                                                  self.kinematics)
            for obstacle in self.obstacles:
                for i in xrange(len(collision_objects_start)):
                    if obstacle.inCollisionContinuous([collision_objects_start[i], collision_objects_goal[i]]):
                        return True
            return False
    
    def is_terminal(self, ee_position):        
        if np.linalg.norm(ee_position - self.goal_position) < self.goal_radius:                       
            return True
        return False
    
    def apply_control(self, x_dash, u_dash, A, B, V, M):               
        m = self.get_random_joint_angles([0.0 for i in xrange(len(self.link_dimensions))], M)
        x_new = np.add(np.add(np.dot(A, x_dash), np.dot(B, u_dash)), np.dot(V, m))
        if self.enforce_constraints:            
            x_new = self.check_constraints(x_new)
        return x_new
    
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
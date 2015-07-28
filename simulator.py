import numpy as np
import kalman as kalman
from scipy.stats import multivariate_normal
from kinematics import Kinematics

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
                      num_links,
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
        self.num_links = num_links
        self.joint_constraints = joint_constraints
        
    def setup_reward_function(self, discount_factor, step_penalty, illegal_move_penalty, exit_reward):
        self.discount_factor = discount_factor
        self.step_penalty = step_penalty
        self.illegal_move_penalty = illegal_move_penalty
        self.exit_reward = exit_reward
    
    def setup_simulator(self, num_simulation_runs, stop_when_terminal, verbose): 
        self.kinematics = Kinematics(self.num_links)
        self.num_simulation_runs = num_simulation_runs        
        self.stop_when_terminal = stop_when_terminal
        self.verbose = verbose
        
    def simulate_step(self, 
                      xs, us, zs, 
                      x_true,
                      x_tilde,
                      P_t,
                      total_reward,
                      current_step):        
        terminal_state_reached = False
        Ls = kalman.compute_gain(self.A, self.B, self.C, self.D, len(xs))
        u_dash = np.dot(Ls[1], x_tilde)        
        x_true = self.apply_control(x_true, np.add(u_dash, us[0]), self.A, self.B, self.V, self.M)
        
        ee_position = self.kinematics.get_end_effector_position(x_true) 
        print "ee_position " + str(ee_position)               
        discount = np.power(self.discount_factor, current_step)
        if self.is_terminal(ee_position):
            terminal_state_reached = True                        
            total_reward += discount * self.exit_reward                        
            print "Terminal state reached: reward = " + str(total_reward)                        
        else:                        
            if self.check_collision(x_true, self.obstacles, self.kinematics): 
                print "COLLISION"                           
                total_reward += discount * (-1.0 * self.illegal_move_penalty)                            
            else:
                total_reward += discount * (-1.0 * self.step_penalty)
        
        z_t = self.get_observation(x_true, self.H, self.N, self.W)
        z_dash_t = z_t - zs[0]
        x_tilde_dash_t, P_dash = kalman.kalman_predict(x_tilde, u_dash, self.A, self.B, P_t, self.V, self.M)
        x_tilde, P_t = kalman.kalman_update(x_tilde_dash_t, z_dash_t, self.H, P_dash, self.W, self.N, self.num_links)
        return x_true, x_tilde, P_t, total_reward, terminal_state_reached
    
    def simulate_n_steps(self,
                         xs, us, zs,
                         x_true,                         
                         x_tilde,
                         x_estimate,
                         P_t,
                         total_reward,
                         current_step,
                         n_steps):
        terminal_state_reached = False
        Ls = kalman.compute_gain(self.A, self.B, self.C, self.D, len(xs))       
        if self.verbose:
            print "Executing for " + str(n_steps) + " steps"
        for i in xrange(n_steps):
            if not (terminal_state_reached and self.stop_when_terminal):
                u_dash = np.dot(Ls[i + 1], x_tilde)        
                x_true = self.apply_control(x_true, np.add(u_dash, us[i]), self.A, self.B, self.V, self.M)
                
                ee_position = self.kinematics.get_end_effector_position(x_true) 
                print "ee_position " + str(ee_position)               
                discount = np.power(self.discount_factor, current_step)
                if self.is_terminal(ee_position):
                    terminal_state_reached = True                        
                    total_reward += discount * self.exit_reward                        
                    print "Terminal state reached: reward = " + str(total_reward)
                else:                        
                    if self.check_collision(x_true, self.obstacles, self.kinematics): 
                        print "COLLISION"                           
                        total_reward += discount * (-1.0 * self.illegal_move_penalty)                            
                    else:
                        total_reward += discount * (-1.0 * self.step_penalty)                   
                    
                z_t = self.get_observation(x_true, self.H, self.N, self.W)
                z_dash_t = z_t - zs[0]
                x_tilde_dash_t, P_dash = kalman.kalman_predict(x_tilde, u_dash, self.A, self.B, P_t, self.V, self.M)
                x_tilde, P_t = kalman.kalman_update(x_tilde_dash_t, z_dash_t, self.H, P_dash, self.W, self.N, self.num_links)
                x_estimate_new = self.check_constraints(x_tilde + xs[i + 1])
                
                if self.check_collision(x_estimate_new, self.obstacles, self.kinematics):
                    x_estimate_new = x_estimate
                else:                    
                    x_estimate = x_estimate_new 
                #print "x_true " + str(x_true)
                #print "x_estimate " + str(x_estimate_new) 
        return x_true, x_tilde, x_estimate, P_t, current_step + n_steps, total_reward, terminal_state_reached
    
    def check_constraints(self, state):        
        for i in xrange(len(state)):                          
            if state[i] < self.joint_constraints[0]:
                state[i] = self.joint_constraints[0] + 0.00001
            if state[i] > self.joint_constraints[1]:
                state[i] = self.joint_constraints[1] - 0.00001        
        return state
        
    def simulate(self, xs, us, zs, run):
        Ls = kalman.compute_gain(self.A, self.B, self.C, self.D, len(xs) - 1)
        cart_coords = []
        mean_reward = 0.0
        for j in xrange(self.num_simulation_runs):
            print "simulation run " + str(j) + " for run " + str(run) 
            x_true = xs[0]
            #x_tilde = xs[0]
            x_tilde = np.array([0.0 for i in xrange(self.num_links)])        
            u_dash = np.array([0.0 for j in xrange(self.num_links)])        
            P_t = np.array([[0.0 for k in xrange(self.num_links)] for l in xrange(self.num_links)])
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
                    x_true = self.apply_control(x_true, np.add(u_dash, us[i]), self.A, self.B, self.V, self.M)
                    ee_position = self.kinematics.get_end_effector_position(x_true)                
                    discount = np.power(self.discount_factor, i)
                    if self.is_terminal(ee_position):
                        terminal_state_reached = True                        
                        reward += discount * self.exit_reward                        
                        print "Terminal state reached: reward = " + str(reward)                        
                    else:                        
                        if self.check_collision(x_true, self.obstacles, self.kinematics):                            
                            reward += discount * (-1.0 * self.illegal_move_penalty)                            
                        else:
                            reward += discount * (-1.0 * self.step_penalty)
                    """
                    Obtain an observation
                    """
                    z_t = self.get_observation(x_true, self.H, self.N, self.W)
                    z_dash_t = z_t - zs[i]
                                
                    """
                    Kalman prediction and update
                    """
                    x_tilde_dash_t, P_dash = kalman.kalman_predict(x_tilde, u_dash, self.A, self.B, P_t, self.V, self.M)
                    x_tilde, P_t = kalman.kalman_update(x_tilde_dash_t, z_dash_t, self.H, P_dash, self.W, self.N, self.num_links)                   
            ee_position = self.kinematics.get_end_effector_position(x_true)
            cart_coords.append(ee_position.tolist())
            mean_reward += reward            
        mean_reward /= self.num_simulation_runs                        
        return cart_coords, np.asscalar(mean_reward)
    
    def check_collision(self, state, obstacles, kinematics):
        """
        Is the given end effector position in collision with an obstacle?
        """              
        p1 = self.kinematics.get_link_n_position(state, 1)            
        p2 = self.kinematics.get_link_n_position(state, 2)            
        p3 = self.kinematics.get_link_n_position(state, 3)
        for obstacle in self.obstacles:
            if obstacle.manipulator_collides([[np.array([0, 0]), p1], [p1, p2], [p2, p3]]):                                    
                return True
        return False
    
    def is_terminal(self, ee_position):        
        if np.linalg.norm(ee_position - self.goal_position) < self.goal_radius:            
            return True
        return False
    
    def apply_control(self, x_dash, u_dash, A, B, V, M):        
        m = self.get_random_joint_angles([0.0 for i in xrange(self.num_links)], M)        
        x_new = np.add(np.add(np.dot(A, x_dash), np.dot(B, u_dash)), np.dot(V, m))
        x_new = self.check_constraints(x_new)
        p1 = self.kinematics.get_link_n_position(x_new, 1)            
        p2 = self.kinematics.get_link_n_position(x_new, 2)            
        p3 = self.kinematics.get_link_n_position(x_new, 3)
        for obstacle in self.obstacles:
            if obstacle.manipulator_collides([[np.array([0, 0]), p1], [p1, p2], [p2, p3]]):
                print "COLLISION DETECTED"
                return x_dash                    
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
        return np.add(np.dot(H, true_theta), np.dot(W, self.get_random_joint_angles([0.0 for i in xrange(self.num_links)], N)))
    
if __name__ == "__main__":
    Simulator()
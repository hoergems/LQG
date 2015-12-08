import numpy as np
import scipy
import kalman as kalman
import logging
import time
from scipy.stats import multivariate_normal
from libintegrate import *
from librobot import v_string
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
                      robot,
                      obstacles,
                      goal_position,
                      goal_radius,                      
                      enforce_constraints,
                      joint_velocity_limit,
                      control_duration,
                      show_viewer,
                      model_file,
                      env_file):
        self.A = A
        self.B = B
        self.C = C
        self.D = D
        self.H = H
        self.V = V
        self.W = W
        self.M = M
        self.N = N
        self.robot = robot
        self.obstacles = obstacles
        self.goal_position = goal_position
        self.goal_radius = goal_radius        
        
        active_joints = v_string()
        robot.getActiveJoints(active_joints)
        lower_position_constraints = v_double()
        upper_position_constraints = v_double()
        robot.getJointLowerPositionLimits(active_joints, lower_position_constraints)
        robot.getJointUpperPositionLimits(active_joints, upper_position_constraints)
        
        self.lower_position_constraints = [lower_position_constraints[i] for i in xrange(len(lower_position_constraints))]
        self.upper_position_constraints = [upper_position_constraints[i] for i in xrange(len(upper_position_constraints))]
        
        self.enforce_constraints = enforce_constraints
        self.dynamic_problem = False
        self.integrate = Integrate()
        self.show_viewer = show_viewer        
        self.control_duration = control_duration
        active_joints = v_string()
        self.robot.getActiveJoints(active_joints)
        self.robot_dof = len(active_joints)
        if show_viewer:
            self.robot.setupViewer(model_file, env_file)
        
    def setup_dynamic_problem(self,                           
                              simulation_step_size):
        self.dynamic_problem = True        
        self.simulation_step_size = simulation_step_size        
        
    def setup_reward_function(self, discount_factor, step_penalty, illegal_move_penalty, exit_reward):
        self.discount_factor = discount_factor
        self.step_penalty = step_penalty
        self.illegal_move_penalty = illegal_move_penalty
        self.exit_reward = exit_reward
    
    def setup_simulator(self, num_simulation_runs, stop_when_terminal):
        self.num_simulation_runs = num_simulation_runs        
        self.stop_when_terminal = stop_when_terminal
    
    def get_linear_model_matrices(self, state_path, control_path):
        As = []
        Bs = []
        Vs = []
        Ms = []
        Hs = []
        Ws = []
        Ns = []
        if self.dynamic_problem:
            for i in xrange(len(state_path)):
                state = v_double()
                control = v_double()
                state[:] = state_path[i]
                control[:] = control_path[i]
                
                t0 = time.time()
                A = self.integrate.getProcessMatrices(state, control, self.control_duration)                
                Matr_list = [A[i] for i in xrange(len(A))]
                
                A_list = np.array([Matr_list[i] for i in xrange(len(state)**2)])
                           
                B_list = np.array([Matr_list[i] for i in xrange(len(state)**2, 2 * len(state)**2)])
                              
                V_list = np.array([Matr_list[i] for i in xrange(2 * len(state)**2, 
                                                                3 * len(state)**2)])
               
                A_Matr = A_list.reshape(len(state), len(state)).T                
                V_Matr = V_list.reshape(len(state), len(state)).T
                B_Matr = B_list.reshape(len(state), len(state)).T 
                
                As.append(A_Matr)
                Bs.append(B_Matr)
                Vs.append(V_Matr)
                
                Ms.append(self.M)
                Hs.append(self.H)
                Ws.append(self.W)
                Ns.append(self.N)
        else:
            for i in xrange(len(state_path)):
                As.append(self.A)
                Bs.append(self.B)
                Vs.append(self.V)
                Ms.append(self.M)
                Hs.append(self.H)
                Ws.append(self.W)
                Ns.append(self.N)
            
        return As, Bs, Vs, Ms, Hs, Ws, Ns
    
    def simulate_n_steps(self,
                         xs, us, zs,
                         x_true,                         
                         x_tilde,
                         x_tilde_linear,
                         x_estimate,
                         P_t,
                         total_reward,                         
                         current_step,
                         n_steps):
        history_entries = []        
        terminal_state_reached = False
        success = False
        x_dash = np.copy(x_tilde)
        x_dash_linear = np.copy(x_tilde_linear)
        
        As, Bs, Vs, Ms, Hs, Ws, Ns = self.get_linear_model_matrices(xs, us)
        Ls = kalman.compute_gain(As, Bs, self.C, self.D, len(xs) - 1)
        logging.info("Simulator: Executing for " + str(n_steps) + " steps") 
        estimated_states = []
        estimated_covariances = []
        for i in xrange(n_steps):                        
            if not (terminal_state_reached and self.stop_when_terminal):
                linearization_error = self.calc_linearization_error(x_dash, x_dash_linear)                
                history_entries.append(HistoryEntry(current_step + i,
                                                    x_true, 
                                                    x_estimate,
                                                    x_dash,
                                                    x_dash_linear,
                                                    linearization_error, 
                                                    None,
                                                    None,
                                                    P_t,
                                                    False,
                                                    False,
                                                    False,
                                                    0.0))
                #print "Ls[i] " + str(Ls[i])                
                u_dash = np.dot(Ls[i], x_tilde) 
                u = u_dash + us[i]                           
                history_entries[-1].set_action(u)
                t0 = time.time()
                x_true_temp, ce = self.apply_control(x_true, 
                                                     u, 
                                                     As[i], 
                                                     Bs[i], 
                                                     Vs[i], 
                                                     Ms[i],
                                                     False)
                x_dash_linear, x_true_linear = self.get_linearized_next_state(x_dash, u_dash, ce, xs[i+1], As[i], Bs[i], Vs[i])
                t_e = time.time() - t0
                
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
                state[:] = [x_true[j] for j in xrange(len(x_true) / 2)]
                ee_position_arr = v_double()
                self.robot.getEndEffectorPosition(state, ee_position_arr)
                ee_position = np.array([ee_position_arr[j] for j in xrange(len(ee_position_arr))])
                logging.info("Simulator: Current end-effector position is " + str(ee_position))                                                
                
                z = self.get_observation(x_true, Hs[i + 1], Ns[i + 1], Ws[i + 1])
                z_dash = np.subtract(z, zs[i+1])
                history_entries[-1].set_observation(z)
                
                """
                Kalman prediction and update
                """ 
                #if not collided:               
                x_tilde_dash, P_dash = kalman.kalman_predict(x_tilde, u_dash, As[i], Bs[i], P_t, Vs[i], Ms[i])
                x_tilde, P_t = kalman.kalman_update(x_tilde_dash, 
                                                    z_dash, 
                                                    Hs[i], 
                                                    P_dash, 
                                                    Ws[i], 
                                                    Ns[i], 
                                                    2 * self.robot_dof)                            
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
                                                        x_dash,
                                                        x_dash_linear,
                                                        0.0,
                                                        None,
                                                        z,
                                                        P_t,
                                                        False,
                                                        False,
                                                        True,
                                                        0.0))                    
                    terminal_state_reached = True                        
                    total_reward += discount * self.exit_reward
                    history_entries[-1].set_reward(self.exit_reward)
                    history_entries[-1].set_linearization_error(self.calc_linearization_error(x_dash, x_dash_linear))
                    success = True                      
                    logging.info("Terminal state reached: reward = " + str(total_reward)) 
                    
                history_entries[-1].set_collided(collided)
                history_entries[-1].set_estimate_collided(estimate_collided)
                history_entries[-1].set_terminal(terminal_state_reached)
                #time.sleep(1)
        #print "========================================"
        #print "======= Simulation done"
        #print "========================================"
        
        return (x_true, 
                x_tilde,
                x_dash_linear, 
                x_estimate, 
                P_t, 
                current_step + n_steps, 
                total_reward, 
                success, 
                terminal_state_reached,
                estimated_states,
                estimated_covariances,                
                history_entries)
        
    def calc_linearization_error(self, x_dash, x_dash_linear):
        linearization_error = 0.0
        for i in xrange(len(x_dash)):
            linearization_error += np.square(x_dash[i] - x_dash_linear[i])
        return np.sqrt(linearization_error)
        
    
    def check_constraints(self, state):
        for i in xrange(len(state) / 2):                          
            if state[i] < self.lower_position_constraints[i]:
                state[i] = self.lower_position_constraints[i] + 0.00001
            if state[i] > self.upper_position_constraints[i]:
                state[i] = self.upper_position_constraints[i] - 0.00001        
        return state
    
    
    def is_in_collision(self, previous_state, state, p=False):
        """
        Is the given end effector position in collision with an obstacle?
        """
        #previous_state = []
                    
        joint_angles_goal = v_double()
        joint_angles_goal[:] = [state[i] for i in xrange(self.robot_dof)]
        collision_structures = self.robot.createRobotCollisionStructures(joint_angles_goal)
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
            
            collision_objects_start = self.robot.createRobotCollisionObjects(joint_angles_start)
            collision_objects_goal = self.robot.createRobotCollisionObjects(joint_angles_goal)
            for obstacle in self.obstacles:
                for i in xrange(len(collision_objects_start)):
                    if obstacle.inCollisionContinuous([collision_objects_start[i], collision_objects_goal[i]]):
                        return True
            return False
    
    def is_terminal(self, ee_position):
        norm = np.linalg.norm(ee_position - self.goal_position)               
        if norm - 0.01 <= self.goal_radius:                       
            return True
        return False
    
    def get_linearized_next_state(self, x_dash, u_dash, control_error, x_star_next, A, B, V):
        delta_x_new = np.dot(A, x_dash) + np.dot(B, u_dash) + np.dot(V, control_error)        
        x_new2 = delta_x_new + x_star_next
        return delta_x_new, x_new2
       
    def apply_control(self, x, u, A, B, V, M, apply_zero_torque=False):
        x_new = None
        ce = None
        if apply_zero_torque:        
            while True:
                u = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                current_state = v_double()
                current_state[:] = x
                control = v_double()
                control[:] = u            
                control_error = v_double()
                ce = self.sample_control_error(M)
                control_error[:] = ce
                result = v_double() 
                self.robot.propagate(current_state,
                                     control,
                                     control_error,
                                     self.simulation_step_size,
                                     self.control_duration,
                                     result)                               
                x = np.array([result[i] for i in xrange(len(result))])
                cjvals = v_double()
                cjvels = v_double()
                cjvals_arr = [x[i] for i in xrange(len(x) / 2)]
                cjvels_arr = [x[i] for i in xrange(len(x) / 2, len(x))]
                cjvals[:] = cjvals_arr
                cjvels[:] = cjvels_arr
                self.robot.updateViewerValues(cjvals, cjvels)
                time.sleep(self.control_duration)
            
        if self.dynamic_problem:
            current_state = v_double()
            current_state[:] = x
            control = v_double()
            control[:] = u
            control_error = v_double()
            ce = self.sample_control_error(M)
            control_error[:] = ce
            result = v_double()            
            self.robot.propagate(current_state,
                                 control,
                                 control_error,
                                 self.simulation_step_size,
                                 self.control_duration,
                                 result)                               
            x_new = np.array([result[i] for i in xrange(len(result))])
        else:               
            ce = self.get_random_joint_angles([0.0 for i in xrange(2 * self.robot_dof)], M)
            x_new = np.dot(A, x) + np.dot(B, u) + np.dot(V, ce)
            if self.enforce_constraints:            
                x_new = self.check_constraints(x_new)
        if self.show_viewer:
            cjvals = v_double()
            cjvels = v_double()
            cjvals_arr = [x_new[i] for i in xrange(len(x_new) / 2)]
            cjvels_arr = [x_new[i] for i in xrange(len(x_new) / 2, len(x_new))]
            cjvals[:] = cjvals_arr
            cjvels[:] = cjvels_arr
            self.robot.updateViewerValues(cjvals, cjvels)
            time.sleep(self.control_duration) 
        return x_new, ce
    
    def sample_control_error(self, M):
        mu = np.array([0.0 for i in xrange(2 * self.robot_dof)])
        return np.random.multivariate_normal(mu, M)
    
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
                      np.dot(W, self.get_random_joint_angles([0.0 for i in xrange(2 * self.robot_dof)], N)))
    
if __name__ == "__main__":
    Simulator()
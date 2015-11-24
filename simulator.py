import numpy as np
import scipy
import kalman as kalman
import logging
import time
from scipy.stats import multivariate_normal
from libkinematics import *
from libpropagator import *
from libintegrate import *
from libutil import *
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
                      enforce_constraints,
                      joint_velocity_limit):
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
        self.joint_velocity_limit = joint_velocity_limit
        self.dynamic_problem = False
        self.integrate = Integrate()
        
    def setup_dynamic_problem(self, 
                              model_file,
                              environment_file,
                              coulomb,
                              viscous,
                              control_duration,
                              simulation_step_size,
                              show_viewer):
        self.dynamic_problem = True
        self.show_viewer = show_viewer
        self.control_duration = control_duration
        
        self.propagator = Propagator()        
        self.propagator.setup(model_file,
                              environment_file,
                              coulomb,
                              viscous,
                              show_viewer)
        self.simulation_step_size = simulation_step_size        
        
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
        
    def enforce_velocity_limit(self, u):        
        for i in xrange(len(u)):
            if u[i] < -self.joint_velocity_limit:
                u[i] = -self.joint_velocity_limit
            elif u[i] > self.joint_velocity_limit:
                u[i] = self.joint_velocity_limit
        return u
    
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
                         x_estimate,
                         P_t,
                         total_reward,                         
                         current_step,
                         n_steps):
        history_entries = []        
        terminal_state_reached = False
        success = False
        x_dash = np.copy(x_tilde)
        
        '''t_state1 = [2.5682342444639845, -1.5329856012320175, 0.0696156786090998]
        t_state2 = [2.568234769257803, -1.5330883128644996, 0.06953418224489541]
        tsv1 = v_double()
        tsv2 = v_double()
        
        tsv1[:] = t_state1
        tsv2[:] = t_state2
        
                
        ee_position_arr1 = self.kinematics.getEndEffectorPosition(tsv1)
        ee_position_arr2 = self.kinematics.getEndEffectorPosition(tsv2)                
        ee_position1 = np.array([ee_position_arr1[j] for j in xrange(len(ee_position_arr1))])
        ee_position2 = np.array([ee_position_arr2[j] for j in xrange(len(ee_position_arr2))])
        
        print "terminal1 " + str(self.is_terminal(ee_position1))
        print "terminal2 " + str(self.is_terminal(ee_position2))
        sleep'''
        
        As, Bs, Vs, Ms, Hs, Ws, Ns = self.get_linear_model_matrices(xs, us)
        Ls = kalman.compute_gain(As, Bs, self.C, self.D, len(xs) - 1)
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
                u = u_dash + us[i]                           
                history_entries[-1].set_action(u)
                t0 = time.time()
                x_true_temp = self.apply_control(x_true, 
                                                 u, 
                                                 As[i], 
                                                 Bs[i], 
                                                 Vs[i], 
                                                 Ms[i])
                #self.apply_control2(x_dash, u_dash, xs[i], xs[i+1], us[i], As[i], Bs[i], Vs[i], Ms[i])
                t_e = time.time() - t0
                print "u " + str(u)
                print "us[i] " + str(us[i])
                print "xs[i] " + str(xs[i + 1])
                print "x_true_temp " + str(x_true_temp)
                #print "integrating took " + str(t_e) + " seconds"
                print "===================================== "
                
                
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
                state[:] = [x_true[i] for i in xrange(len(x_true) / 2)]                  
                ee_position_arr = self.kinematics.getEndEffectorPosition(state)                
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
                                                    2 * len(self.link_dimensions))                
                x_estimate_new = x_tilde + xs[i + 1]
                #print "x_estimate " + str(x_estimate_new)
                #sleep
                
                if self.enforce_constraints:     
                    x_estimate_new = self.check_constraints(x_estimate_new) 
                estimate_collided = True                                       
                if not self.is_in_collision([], x_estimate_new):                                                                                
                    x_estimate = x_estimate_new
                    estimate_collided = False
                        
                            
                estimated_states.append(x_estimate)
                estimated_covariances.append(P_t)
                
                if self.is_terminal(ee_position):
                    print "IS TERMINAL!!!!!!!!!!!!!!!!!"
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
                #time.sleep(1)
        print "========================================"
        print "======= Simulation done"
        print "========================================"
        
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
        for i in xrange(len(state) / 2):                          
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
        joint_angles_goal[:] = [state[i] for i in xrange(len(self.link_dimensions))]
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
        norm = np.linalg.norm(ee_position - self.goal_position)
        print "dist " + str(norm)        
        if norm - 0.01 <= self.goal_radius:                       
            return True
        return False
    
    def apply_control2(self, x_dash, u_dash, x_star, x_star2, u_star, A, B, V, M):
        x = x_dash + x_star 
        u = u_dash + u_star
        
        state = v_double()
        control = v_double()
        state[:] = [x[i] for i in xrange(len(x))]
        control[:] = [u[i] for i in xrange(len(u))]
                
        t0 = time.time()
        A = self.integrate.getProcessMatrices(state, control, self.control_duration)                
        Matr_list = [A[i] for i in xrange(len(A))]
                
        A_list = np.array([Matr_list[i] for i in xrange(len(state)**2)])
                           
        B_list = np.array([Matr_list[i] for i in xrange(len(state)**2, 2 * len(state)**2)])
                              
        V_list = np.array([Matr_list[i] for i in xrange(2 * len(state)**2, 
                                                                3 * len(state)**2)])
               
        A = A_list.reshape(len(state), len(state)).T                
        V = V_list.reshape(len(state), len(state)).T
        B = B_list.reshape(len(state), len(state)).T
        print "Got linear matrices in " + str(time.time() - t0) + " seconds"
        
        current_joint_values = v_double()
        current_joint_velocities = v_double()
        
        current_joint_values[:] = [x[i] for i in xrange(len(self.link_dimensions))]
        current_joint_velocities[:] = [x[i + len(self.link_dimensions)] for i in xrange(len(self.link_dimensions))]        
        control_error = v_double()
        ce = self.sample_control_error(M)
            
        control_error[:] = ce 
        t0 = time.time()
        result = v_double()
        vec = []
        num_prop_runs = 1
        #self.control_duration = 0.00001
        for i in xrange(num_prop_runs):          
            result = v_double()
            cjv = [current_joint_values[j] for j in xrange(len(current_joint_values))]
            cjv.extend([current_joint_velocities[j] for j in xrange(len(current_joint_velocities))])
            self.propagator.propagate(current_joint_values,
                                      current_joint_velocities,
                                      control,
                                      control_error,
                                      self.simulation_step_size,
                                      self.control_duration,
                                      result)                    
            x_new = [result[i] for i in xrange(len(result))]                
            vec.append(np.array(x_new))                
            n, min_max, mean, var, skew, kurt = scipy.stats.describe(np.array(vec))            
            res = [np.asscalar(mean[i]) for i in xrange(len(mean))]
            
        inte_res = v_double()
        int_times = v_double()
        int_times[:] = [0.0, self.control_duration, 0.000001]
        self.integrate.doIntegration(state, control, int_times, inte_res) 
        inte_ress = [inte_res[i] for i in xrange(len(inte_res))]   
        
        print "Propagated in " + str(time.time() - t0) + " seconds"
        delta_x_new = np.dot(A, x_dash) + np.dot(B, u_dash) + np.dot(V, ce)
        x_new_2 = delta_x_new + x_star2
        
        result_lin = v_double()
        self.propagator.propagateLinear(state,
                                        control,
                                        control_error,
                                        self.control_duration,
                                        result_lin)
        res_lin = [result_lin[i] for i in xrange(len(result_lin))]
        
        print ""
        print "x_dash " + str(x_dash)
        print ""
        print "u_dash " + str(u_dash)
        print ""
        print "x_new1 " + str(res)
        print ""
        print "delta_x_new " + str(delta_x_new)
        print ""
        print "x_new2 " + str(x_new_2)
        print ""
        print "x_new3 " + str(inte_ress)
        print " "
        sum = 0.0
        for i in xrange(len(res)):
            sum += np.square(res[i] - x_new_2[i])
        print "dist " + str(np.sqrt(sum))
        time.sleep(0.5)
        
       
    def apply_control(self, x_dash, u_dash, A, B, V, M):
        x_new = None
        if self.dynamic_problem:
            current_joint_values = v_double()
            current_joint_velocities = v_double()
            
            current_joint_values[:] = [x_dash[i] for i in xrange(len(self.link_dimensions))]
            current_joint_velocities[:] = [x_dash[i + len(self.link_dimensions)] for i in xrange(len(self.link_dimensions))]
            
            control = v_double()
            control[:] = u_dash
            
            control_error = v_double()
            ce = self.sample_control_error(M)
            
            control_error[:] = ce 
            result = v_double()
            vec = []
            num_prop_runs = 1
            if self.show_viewer:
                num_prop_runs = 1
            for i in xrange(num_prop_runs):          
                result = v_double()
                cjv = [current_joint_values[j] for j in xrange(len(current_joint_values))]
                cjv.extend([current_joint_velocities[j] for j in xrange(len(current_joint_velocities))])
                self.propagator.propagate(current_joint_values,
                                          current_joint_velocities,
                                          control,
                                          control_error,
                                          self.simulation_step_size,
                                          self.control_duration,
                                          result)
                time.sleep(self.control_duration)                    
                x_new = [result[i] for i in xrange(len(result))]                
                vec.append(np.array(x_new))                
            n, min_max, mean, var, skew, kurt = scipy.stats.describe(np.array(vec))            
            res = [np.asscalar(mean[i]) for i in xrange(len(mean))]            
            return res   
        else:               
            m = self.get_random_joint_angles([0.0 for i in xrange(2 * len(self.link_dimensions))], M)
            x_new = np.dot(A, x_dash) + np.dot(B, u_dash) + np.dot(V, m)
            if self.enforce_constraints:            
                x_new = self.check_constraints(x_new)
        return x_new
    
    def sample_control_error(self, M):
        mu = np.array([0.0 for i in xrange(2 * len(self.link_dimensions))])
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
                      np.dot(W, self.get_random_joint_angles([0.0 for i in xrange(2 * len(self.link_dimensions))], N)))
    
if __name__ == "__main__":
    Simulator()
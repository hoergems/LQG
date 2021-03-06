import numpy as np
import scipy
import kalman as kalman
import logging
import time
from scipy.stats import multivariate_normal
from librobot import v_string, v_double, v2_double
from history_entry import *
import util_py as utils


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
                      enforce_control_constraints,
                      obstacles,
                      goal_position,
                      goal_radius,
                      joint_velocity_limit,                      
                      show_viewer,
                      model_file,
                      env_file,
                      knows_collision):
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
        velocity_constraints = v_double()
        robot.getJointVelocityLimits(active_joints, velocity_constraints)
        self.velocity_constraints = [velocity_constraints[i] for i in xrange(len(velocity_constraints))]
        
        self.max_joint_velocities = [joint_velocity_limit for i in xrange(2 * len(active_joints))]
        torque_limits = v_double()
        robot.getJointTorqueLimits(active_joints, torque_limits)
        torque_limits = [torque_limits[i] for i in xrange(len(torque_limits))]
        torque_limits.extend([0.0 for i in xrange(len(active_joints))])
        self.torque_limits = [torque_limits[i] for i in xrange(len(torque_limits))]
        
        self.enforce_constraints = robot.constraintsEnforced()
        self.control_constraints_enforced = enforce_control_constraints
        self.dynamic_problem = False
        self.stop_when_colliding = False
        self.show_viewer = show_viewer
        active_joints = v_string()
        self.robot.getActiveJoints(active_joints)
        self.robot_dof = self.robot.getDOF()
        self.knows_collision = knows_collision      
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
    
    def set_stop_when_colliding(self, stop_when_colliding):
        self.stop_when_colliding = stop_when_colliding
    
    def get_linear_model_matrices(self, state_path, control_path, control_durations):
        """ Get the linearized model matrices along a given nominal path
        """
        
        if self.dynamic_problem:           
            return kalman.get_linear_model_matrices(self.robot, 
                                                    state_path, 
                                                    control_path, 
                                                    control_durations,
                                                    self.dynamic_problem,                                                    
                                                    self.M,
                                                    self.H,
                                                    self.W,
                                                    self.N)            
        else:
            As = []
            Bs = []
            Vs = []
            Ms = []
            Hs = []
            Ws = []
            Ns = []
            for i in xrange(len(state_path) + 1):
                As.append(self.A)
                Bs.append(self.B)
                Vs.append(self.V)
                Ms.append(self.M)
                Hs.append(self.H)
                Ws.append(self.W)
                Ns.append(self.N)
            return As, Bs, Vs, Ms, Hs, Ws, Ns
    
    def enforce_control_constraints(self, u, us):
        """ Enforces the control constraints on control 'u' and return
        the enforced control and enforced control deviation 'u_dash'
        """
        u_dash = None
        if self.dynamic_problem:
            for i in xrange(len(u)):
                if u[i] > self.torque_limits[i]:
                    u[i] = self.torque_limits[i]
                elif u[i] < -self.torque_limits[i]:
                    u[i] = -self.torque_limits[i]
            u_dash = u - us            
        else:
            for i in xrange(len(u)):
                if u[i] > self.max_joint_velocities[i]:
                    u[i] = self.max_joint_velocities[i]
                elif u[i] < -self.max_joint_velocities[i]:
                    u[i] = -self.max_joint_velocities[i]
            u_dash = u - us
        return u, u_dash                
    
    def simulate_n_steps(self,
                         xs, us, zs,                         
                         control_durations,
                         x_true,
                         x_estimate,
                         P_t,
                         total_reward,                         
                         current_step,
                         n_steps,
                         deviation_covariances,
                         estimated_deviation_covariances,
                         max_num_steps=None):        
        history_entries = []        
        terminal_state_reached = False
        success = False
        collided = False
        z = None
        x_dash = np.subtract(np.array(x_true), np.array(xs[0]))
        x_dash_linear = np.copy(x_dash)
        x_true_linear = x_true        
        x_tilde = np.array(x_estimate) - np.array(xs[0])
        As, Bs, Vs, Ms, Hs, Ws, Ns = self.get_linear_model_matrices(xs, 
                                                                    us, 
                                                                    control_durations)
        Ls = kalman.compute_gain(As, Bs, self.C, self.D, len(xs) - 1)
        logging.info("Simulator: Executing for " + str(n_steps) + " steps") 
        estimated_states = []
        estimated_covariances = []
        self.show_nominal_path(xs)        
        """ Execute the nominal path for n_steps """        
        for i in xrange(n_steps):                        
            if not terminal_state_reached:                             
                history_entries.append(HistoryEntry(current_step,
                                                    x_true, 
                                                    xs[i],
                                                    x_true_linear,
                                                    x_estimate,
                                                    x_dash,
                                                    x_dash_linear,
                                                    0.0, 
                                                    None,
                                                    None,
                                                    None,
                                                    P_t,
                                                    False,
                                                    False,
                                                    False,
                                                    0.0)) 
                x_t_n = np.linalg.norm(np.array([x_true[k] for k in xrange(len(x_true) / 2)]))
                x_e_n = np.linalg.norm(np.array([x_estimate[k] for k in xrange(len(x_estimate) / 2)]))
                x_true_norm = np.array([0.0 for k in xrange(len(x_true) / 2)])
                x_estimate_norm = np.array([0.0 for k in xrange(len(x_estimate) / 2)])
                if x_t_n != 0:
                    x_true_norm = np.array([x_true[k] for k in xrange(len(x_true) / 2)]) / x_t_n   
                if x_e_n != 0:
                    x_estimate_norm = np.array([x_estimate[k] for k in xrange(len(x_estimate) / 2)]) / x_e_n         
                
                history_entries[-1].set_estimation_error(np.linalg.norm(np.array(x_true) - np.array(x_estimate)))
                history_entries[-1].set_estimation_error_normalized(np.linalg.norm(np.array(x_true_norm) - np.array(x_estimate_norm)))
                
                current_step += 1
                history_entries[-1].set_s_dash_estimated(x_tilde)
                
                """ Calc u_dash from the estimated deviation of the state from the nominal path 'x_tilde'
                using the optimal gain L
                """
                try:                    
                    u_dash = np.dot(Ls[i], x_tilde)
                except Exception as e:
                    print e
                    print "i " + str(i)
                    print "len(LS) " + str(len(Ls))
                    print "len(xs) " + str(len(xs))                
                history_entries[-1].set_u_dash(u_dash)
                
                """ Calc the actual control input """ 
                u = u_dash + us[i] 
                
                """ Enforce the control constraints on 'u' and 'u_dash' """
                if self.control_constraints_enforced:                    
                    u, u_dash = self.enforce_control_constraints(u, us[i])                          
                history_entries[-1].set_action(u)
                
                history_entries[-1].set_nominal_action(us[i])
                
                
                
                """ Apply the control 'u' and propagate the state 'x_true' """
                x_true_temp, ce = self.apply_control(x_true, 
                                                     u,
                                                     control_durations[i], 
                                                     As[i], 
                                                     Bs[i], 
                                                     Vs[i], 
                                                     Ms[i],
                                                     xs[i],
                                                     us[i])
                
                x_dash_linear_temp = self.get_linearized_next_state(x_dash, u_dash, ce, As[i], Bs[i], Vs[i])
                x_true_linear_temp = np.add(x_dash_linear_temp, xs[i+1]) 
                if self.enforce_constraints:
                    x_true_temp = self.check_constraints(x_true_temp)
                    x_true_linear_temp = self.check_constraints(x_true_linear_temp)
                
                """ Calc the linearized next state (used to compare with x_true) """
                discount = np.power(self.discount_factor, current_step - 1)
                
                """ Check if the propagated state collides with an obstacle
                If yes, the true state is set to the previous state with 0 velocity.
                If not, set the true state to the propagated state
                """
                collided = False
                in_collision_true_state, colliding_obstacle = self.is_in_collision(x_true, x_true_temp)
                self.set_colliding_obstacle(colliding_obstacle)    
                                                                
                if in_collision_true_state: #or current_step == 7:
                    print "in collision!!!!"                                                                    
                    for j in xrange(len(x_true) / 2, len(x_true)):                        
                        x_true[j] = 0.0
                        x_true_linear[j] = 0.0                   
                    logging.info("Simulator: Collision detected. Setting state estimate to the previous state")
                    
                    total_reward += discount * (-1.0 * self.illegal_move_penalty)
                    history_entries[-1].set_reward(discount * (-1.0 * self.illegal_move_penalty))
                    try:
                        history_entries[-1].set_colliding_obstacle(colliding_obstacle.getName())
                    except:
                        pass
                    history_entries[-1].set_colliding_state(x_true_temp)
                    collided = True
                    """ Calculate the state deviation from the nominal path """                           
                    x_dash = np.subtract(x_true, xs[i])
                    x_dash_linear = np.subtract(x_true_linear , xs[i])
                else:
                    total_reward += discount * (-1.0 * self.step_penalty)
                    history_entries[-1].set_reward(discount * (-1.0 * self.step_penalty))
                    x_true = x_true_temp 
                    x_true_linear = x_true_linear_temp
                    x_dash = np.subtract(x_true, xs[i + 1])
                    x_dash_linear = np.subtract(x_true_linear , xs[i + 1])                              
                
                """ Do the same for the linearized true state """ 
                '''if self.is_in_collision(x_true_linear, x_true_linear_temp)[0]:
                    for j in xrange(len(x_true_linear) / 2, len(x_true_linear)):
                        x_true_linear[j] = 0.0                  
                else:'''
                    
                linearization_error = utils.dist(x_dash, x_dash_linear)
                history_entries[-1].set_linearization_error(linearization_error)
                
                
                
                """ Get the end effector position for the true state """
                state = v_double()
                state[:] = [x_true[j] for j in xrange(len(x_true) / 2)]
                ee_position_arr = v_double()
                self.robot.getEndEffectorPosition(state, ee_position_arr)
                ee_position = np.array([ee_position_arr[j] for j in xrange(len(ee_position_arr))])
                logging.info("Simulator: Current end-effector position is " + str(ee_position))                                                
                
                """ Generate an observation for the true state """
                z = self.get_observation(x_true, Hs[i + 1], Ns[i + 1], Ws[i + 1])
                try:
                    z_dash = np.subtract(z, zs[i+1])
                    history_entries[-1].set_observation(z)
                except:
                    print "len(xs) " + str(len(xs))
                    print "len(zs) " + str(len(zs))
                
                """ Kalman prediction and update """                   
                x_tilde_dash, P_dash = kalman.kalman_predict(x_tilde, u_dash, As[i], Bs[i], P_t, Vs[i], Ms[i])
                x_tilde, P_t = kalman.kalman_update(x_tilde_dash, 
                                                    z_dash, 
                                                    Hs[i], 
                                                    P_dash, 
                                                    Ws[i], 
                                                    Ns[i])
                
                """ x_estimate_new is the estimated state """                                    
                x_estimate_new = x_tilde + xs[i + 1]
                
                """ Enforce the constraints to the estimated state """
                if self.enforce_constraints:     
                    x_estimate_new = self.check_constraints(x_estimate_new) 
                
                """ Check if the estimated state collides.
                If yes, set it to the previous estimate (with velicity 0).
                If no, set the true estimate to this estimate 
                """
                estimate_collided = True
                if self.is_in_collision([], x_estimate_new)[0]:
                    for l in xrange(len(x_estimate) / 2, len(x_estimate)):                        
                        x_estimate[l] = 0
                elif in_collision_true_state and self.knows_collision:
                    for l in xrange(len(x_estimate) / 2, len(x_estimate)):                        
                        x_estimate[l] = 0
                else:
                    x_estimate = x_estimate_new                   
                    estimate_collided = False
                    
                                      
                '''if not self.is_in_collision([], x_estimate_new)[0]:                                                                                                    
                    x_estimate = x_estimate_new                   
                    estimate_collided = False
                else:                    
                    for l in xrange(len(x_estimate) / 2, len(x_estimate)):
                        x_estimate[l] = 0'''
                                                          
                estimated_states.append(x_estimate)
                estimated_covariances.append(P_t)
                
                """ Update the viewer to display the true state """                       
                '''self.update_viewer(x_true,
                                   x_estimate, 
                                   z, 
                                   control_durations[i], 
                                   colliding_obstacle)'''                
                
                """ Check if the end-effector position is terminal. If yes, we're done """
                if self.is_terminal(ee_position):
                    discount = np.power(self.discount_factor, current_step)
                    history_entries.append(HistoryEntry(current_step,
                                                        x_true,
                                                        xs[i + 1],
                                                        x_true_linear, 
                                                        x_estimate, 
                                                        x_dash,
                                                        x_dash_linear,
                                                        0.0,
                                                        None,
                                                        None,
                                                        z,
                                                        P_t,
                                                        False,
                                                        False,
                                                        True,
                                                        discount * self.exit_reward))                                      
                    terminal_state_reached = True                                           
                    total_reward += discount * self.exit_reward                    
                    history_entries[-1].set_linearization_error(utils.dist(x_dash, x_dash_linear))                                         
                    logging.info("Terminal state reached: reward = " + str(total_reward))                    
                history_entries[-1].set_collided(collided)
                history_entries[-1].set_estimate_collided(estimate_collided)
                
                
                if current_step == max_num_steps and not terminal_state_reached: 
                    """ This was the last step -> append final history entry"""
                    history_entries.append(HistoryEntry(current_step,
                                                        x_true,
                                                        xs[i + 1],
                                                        x_true_linear,
                                                        x_estimate,
                                                        x_dash,
                                                        x_dash_linear,
                                                        0.0,
                                                        None,
                                                        None,
                                                        z,
                                                        P_t,
                                                        False,
                                                        False,
                                                        False,
                                                        0.0))
                    history_entries[-1].set_linearization_error(utils.dist(x_dash, x_dash_linear))                        
                    #total_reward += discount * self.exit_reward
                if (collided and self.stop_when_colliding):                                                                               
                    break                
        #print "========================================"
        #print "======= Simulation done"
        #print "========================================"              
        return (x_true,                 
                x_tilde,
                x_dash_linear, 
                x_estimate,
                z, 
                P_t, 
                current_step, 
                total_reward,                
                terminal_state_reached,
                estimated_states,
                estimated_covariances,                
                history_entries) 
        
    def set_colliding_obstacle(self, colliding_obstacle):
        self.colliding_obstacle = colliding_obstacle
    
    def check_constraints(self, state):
        """ Checks if a state satisfies the system constraints.
        Furthermore, enforce the constraints if they are violated
        """
        for i in xrange(len(state) / 2):
            if state[i] < self.lower_position_constraints[i] - 0.0000001:
                state[i] = self.lower_position_constraints[i]
                state[i + len(state) / 2] = 0.0
            elif state[i] > self.upper_position_constraints[i] - 0.0000001:
                state[i] = self.upper_position_constraints[i]
                state[i + len(state) / 2] = 0.0
            else:
                if state[i + len(state) / 2] < -self.velocity_constraints[i] - 0.0000001:
                    state[i + len(state) / 2] = -self.velocity_constraints[i]
                elif state[i + len(state) / 2] > self.velocity_constraints[i] - 0.0000001:
                    state[i + len(state) / 2] = self.velocity_constraints[i]
        return state
    
    def is_in_collision(self, previous_state, state):
        """
        Is the given end effector position in collision with an obstacle?
        """
        collidable_obstacles = [o for o in self.obstacles if not o.isTraversable()]
        joint_angles_goal = v_double()
        joint_angles_goal[:] = [state[i] for i in xrange(self.robot_dof)]
        collision_objects_goal = self.robot.createRobotCollisionObjects(joint_angles_goal)        
        if len(previous_state) > 0:
            """
            Perform continuous collision checking if previous state is provided
            """            
            joint_angles_start = v_double()        
            joint_angles_start[:] = [previous_state[i] for i in xrange(self.robot_dof)]
            collision_objects_start = self.robot.createRobotCollisionObjects(joint_angles_start)                      
            for obstacle in collidable_obstacles:
                for i in xrange(len(collision_objects_start)):                    
                    if obstacle.inCollisionContinuous([collision_objects_start[i], collision_objects_goal[i]]):
                        return True, obstacle
            return False, None
        else:            
            for obstacle in collidable_obstacles:
                if obstacle.inCollisionDiscrete(collision_objects_goal):                                                   
                    return True, obstacle
            return False, None 
    
    def is_terminal(self, ee_position):
        """ Determines if the end-effector position is terminal """
        norm = np.linalg.norm(ee_position - self.goal_position)               
        if norm - 0.01 <= self.goal_radius:                       
            return True
        return False
    
    def get_linearized_next_state_prop(self, 
                                       x, 
                                       x_nominal, 
                                       u, 
                                       u_nominal, 
                                       control_error,
                                       control_duration):
        current_state = v_double()
        current_state[:] = x
        nominal_state = v_double()
        nominal_state[:] = x_nominal
        control = v_double()
        control[:] = u
        nominal_control = v_double()
        nominal_control[:] = u_nominal
        control_e = v_double()
        control_e[:] = control_error
        
        result = v_double()
        result2 = v_double()
        self.robot.propagate_first_order(current_state,
                                         control,
                                         control_e,
                                         nominal_state,                                         
                                         nominal_control,                                         
                                         0.001,
                                         control_duration,
                                         result)
        self.robot.propagate_second_order(current_state,
                                          control,
                                          control_e,
                                          nominal_state,                                         
                                          nominal_control,                                         
                                          0.001,
                                          control_duration,
                                          result2)
        r = np.array([result[i] for i in xrange(len(result))])
        r2 = np.array([result2[i] for i in xrange(len(result2))])
        print "dev first order " + str(r)
        print "dev second order " + str(r2)
        return (r, r2)
    
    def get_linearized_next_state(self, x_dash, u_dash, control_error, A, B, V):
        """ Apply the linear(ized) model matrices to calcuate
        delta_x_new (the linearized deviation from the nominal path)
        """
        delta_x_new = np.dot(A, x_dash) + np.dot(B, u_dash) + np.dot(V, control_error)        
        return delta_x_new
    
    def get_linearized_next_state_second_order(self, x_dash, u_dash, control_error):
        pass
       
    def apply_control(self, 
                      x, 
                      u,
                      control_duration, 
                      A, 
                      B, 
                      V, 
                      M,
                      x_nominal=None,
                      u_nominal=None):
        """ Applies a control input 'u' to the system which
        is in state 'x' for the duration of 'control_duration'.
        A, B, and V are used for the linear problem. M is used
        to sample a control (or state) error
        """
        x_new = None
        ce = None    
        if self.dynamic_problem:
            current_state = v_double()
            current_state[:] = x
            control = v_double()
            control[:] = u
            control_error = v_double()
            ce = self.sample_control_error(M)
            control_error[:] = ce            
            result = v_double()
            result_first_order = v_double()
            result_second_order = v_double()            
            self.robot.propagate(current_state,
                                 control,
                                 control_error,
                                 self.simulation_step_size,
                                 control_duration,
                                 result)
            x_new = np.array([result[i] for i in xrange(len(result))])
            
            """
            Determine the second order terms
            """
            '''nominal_state = v_double()
            nominal_state[:] = x_nominal
            nominal_control = v_double()
            nominal_control[:] = u_nominal
            nominal_error = v_double()
            
            self.robot.propagate_first_order(current_state,
                                             nominal_state,
                                             nominal_control,                                              
                                             control,
                                             control_error,
                                             0.0001,
                                             control_duration,
                                             result_first_order)
            
            self.robot.propagate_second_order(current_state,
                                              nominal_state,
                                              nominal_control,                                              
                                              control,
                                              control_error,
                                              self.simulation_step_size,
                                              control_duration,
                                              result_second_order)
            second_order_result = np.array([result_second_order[i] for i in xrange(len(result_second_order))])
            first_order_result = np.array([result_first_order[i] for i in xrange(len(result_first_order))])
            x_new_first_order = np.dot(A, x) + np.dot(B, u) + np.dot(V, ce)
            print "current_state " + str(x)
            #print "result first_order matr: " + str(x_new_first_order)
            print "result first_order: " + str(first_order_result)
            #print "result second order: " + str(second_order_result)
            #print "sum: " + str(x_new_first_order + second_order_result)
            print "real: " + str(x_new)'''
                                      
            
        else:               
            ce = self.get_random_joint_angles([0.0 for i in xrange(M.shape[0])], M)            
            x_new = np.dot(A, x) + np.dot(B, u) + np.dot(V, ce)
            #x_new = self.normalize_state(x_new)             
            if self.enforce_constraints:            
                x_new = self.check_constraints(x_new)
            
        return x_new, ce
    
    def normalize_state(self, state):
        normalized_angles = []
        for i in xrange(len(angles) / 2):
            if angles[i] < -np.pi:
                normalized_angles.push_back(angles[i] + 2.0 * np.pi)
            elif angles[i] > np.pi:
                normalized_angles.push_back(angles[i] - 2.0 * np.pi)
            else:
                normalized_angles.push_back(angles[i])
        normalized_angles.extend([state[i] for i in xrange(self.robot_dof, 2 * self.robot_dof)])
        return np.array(normalized_angles)
                
    
    def show_nominal_path(self, path):
        """ Shows the nominal path in the viewer """        
        if self.show_viewer:
            self.robot.removePermanentViewerParticles()
            particle_joint_values = v2_double()
            particle_joint_colors = v2_double()
            pjvs = []
            for p in path:
                particle = v_double()
                particle[:] = [p[i] for i in xrange(len(p) / 2)]
                pjvs.append(particle)
                
                color = v_double()
                color[:] = [0.0, 0.0, 0.0, 0.7]
                particle_joint_colors.append(color)            
            particle_joint_values[:] = pjvs
            self.robot.addPermanentViewerParticles(particle_joint_values,
                                                   particle_joint_colors)            
    
    def update_viewer(self, 
                      x,
                      x_estimate, 
                      z, 
                      control_duration=None, 
                      colliding_obstacle=None):
        """ Update the viewer to display the state 'x' of the robot 
        Optionally sleep for 'control_duration'
        """
        if self.show_viewer:            
            cjvals = v_double()
            cjvels = v_double()
            cjvals_arr = [x[i] for i in xrange(len(x) / 2)]
            cjvels_arr = [x[i] for i in xrange(len(x) / 2, len(x))]
            cjvals[:] = cjvals_arr
            cjvels[:] = cjvels_arr
            particle_joint_values = v2_double()
            pjv = v_double()
            pjv[:] = [z[i] for i in xrange(len(z))]
            pjv_estimate = v_double()
            pjv_estimate[:] = [x_estimate[i] for i in xrange(len(x_estimate))]
            particle_joint_values.append(pjv)
            particle_joint_values.append(pjv_estimate)
            particle_joint_colors = v2_double()
            pja = v_double()            
            pja[:] = [0.5, 0.5, 0.9, 0.0]
            pja_estimate = v_double()
            pja_estimate[:] = [0.2, 0.8, 0.2, 0.0]
            particle_joint_colors.append(pja)
            particle_joint_colors.append(pja_estimate)
            self.robot.updateViewerValues(cjvals, 
                                          cjvels, 
                                          particle_joint_values,
                                          particle_joint_colors)
            for o in self.obstacles:
                self.robot.setObstacleColor(o.getName(), 
                                            o.getStandardDiffuseColor(),
                                            o.getStandardAmbientColor())
            if colliding_obstacle != None:                
                diffuse_col = v_double()
                ambient_col = v_double()
                diffuse_col[:] = [0.5, 0.0, 0.0, 0.0]
                ambient_col[:] = [0.8, 0.0, 0.0, 0.0]
                self.robot.setObstacleColor(colliding_obstacle.getName(), 
                                            diffuse_col, 
                                            ambient_col)
            time.sleep(control_duration)
            time.sleep(1.0)
    
    def sample_control_error(self, M):
        """ Samples a zero mean control error from a multivariate Gaussian
        distirbution with covariance matrix 'M'
        """
        mu = np.array([0.0 for i in xrange(M.shape[0])])        
        r = np.random.multivariate_normal(mu, M)        
        return r
    
    def get_random_joint_angles(self, mu, cov):        
        #with self.lock:
        """
        Random numbers need to be generated by using a lock and creating a new random seed in order to avoid
        correlations between different processes
        """
        return np.random.multivariate_normal(mu, cov)        
        #return self.normalize_state(np.random.multivariate_normal(mu, cov))
    
    def get_observation(self, x, H, N, W):
        """ Gets an observation for the state 'x' using observation model matrices 'H' and 'W'
        which is disturbed by Gaussian distributed noise. 'N' is the covariance matrix for the
        Gaussian distribution
        """
        res = np.dot(H, x) + np.dot(W, self.get_random_joint_angles([0.0 for i in xrange(N.shape[0])], N))        
        return res        
    
if __name__ == "__main__":
    Simulator()

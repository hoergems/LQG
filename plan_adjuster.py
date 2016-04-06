import numpy as np
import kalman as kalman
from librobot import v_string, v_double, v2_double

class PlanAdjuster:
    def __init__(self):
        pass     
        
    def setup(self,
              robot,
              M, 
              H, 
              W, 
              N, 
              C, 
              D,
              dynamic_problem, 
              enforce_control_constraints):
        self.M = M
        self.H = H
        self.W = W
        self.N = N
        self.C = C
        self.D = D
        self.dynamic_problem = dynamic_problem
        self.control_constraints_enforced = enforce_control_constraints
        
        active_joints = v_string()
        robot.getActiveJoints(active_joints)
        lower_position_constraints = v_double()
        upper_position_constraints = v_double()
        robot.getJointLowerPositionLimits(active_joints, lower_position_constraints)
        robot.getJointUpperPositionLimits(active_joints, upper_position_constraints)
        
        torque_limits = v_double()
        robot.getJointTorqueLimits(active_joints, torque_limits)
        torque_limits = [torque_limits[i] for i in xrange(len(torque_limits))]
        torque_limits.extend([0.0 for i in xrange(len(active_joints))])
        self.torque_limits = [torque_limits[i] for i in xrange(len(torque_limits))]
        
    def set_simulation_step_size(self, simulation_step_size):
        self.simulation_step_size = simulation_step_size
        
    def set_max_joint_velocities_linear_problem(self, max_joint_velocities):
        self.max_joint_velocities = max_joint_velocities
        
    def set_model_matrices(self, A, B, V):
        self.A = A
        self.B = B
        self.V = V
        
    def enforce_control_constraints(self, u):
        """ Enforces the control constraints on control 'u' and return
        the enforced control and enforced control deviation 'u_dash'
        """        
        if self.dynamic_problem:
            for i in xrange(len(u)):
                if u[i] > self.torque_limits[i]:
                    u[i] = self.torque_limits[i]
                elif u[i] < -self.torque_limits[i]:
                    u[i] = -self.torque_limits[i]                       
        else:
            for i in xrange(len(u)):
                if u[i] > self.max_joint_velocities[i]:
                    u[i] = self.max_joint_velocities[i]
                elif u[i] < -self.max_joint_velocities[i]:
                    u[i] = -self.max_joint_velocities[i]
            
        return u
    
    def get_linear_model_matrices(self, robot, xs, us, control_durations):
        if self.dynamic_problem:
            return kalman.get_linear_model_matrices(robot, 
                                                    xs, 
                                                    us, 
                                                    control_durations, 
                                                    True,
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
            for i in xrange(len(xs) + 1):
                As.append(self.A)
                Bs.append(self.B)
                Vs.append(self.V)
                Ms.append(self.M)
                Hs.append(self.H)
                Ws.append(self.W)
                Ns.append(self.N)
            return As, Bs, Vs, Ms, Hs, Ws, Ns
        
    
    def adjust_plan(self, 
                    robot,                    
                    plan,                                      
                    x_estimated,
                    P_t):
        xs = [plan[0][i] for i in xrange(1, len(plan[0]))]
        us = [plan[1][i] for i in xrange(1, len(plan[1]))]
        zs = [plan[2][i] for i in xrange(1, len(plan[2]))]
        control_durations = [plan[3][i] for i in xrange(1, len(plan[3]))]
        As, Bs, Vs, Ms, Hs, Ws, Ns = self.get_linear_model_matrices(robot, 
                                                                    xs, 
                                                                    us,                                                                    
                                                                    control_durations)
        Ls = kalman.compute_gain(As, Bs, self.C, self.D, len(xs) - 1) 
        
        xs_adjusted = []
        us_adjusted = []
        zs_adjusted = []
        
        xs_adjusted.append(xs[0])
        zs_adjusted.append(zs[0])
        try:
            x_tilde = x_estimated - xs[0] 
        except Exception as e:
            print e
            print "==================="
            print "x_estimated " + str(x_estimated)
            print "xs[0] " + str(xs[0])
            print "xs " + str(xs)
            raise ValueError("Raised")       
        for i in xrange(len(xs) - 1):
            x_predicted = np.array([xs[i][k] for k in xrange(len(xs[i]))])
            u = np.dot(Ls[i], x_estimated - x_predicted) + us[i]            
            if self.control_constraints_enforced:                
                u = self.enforce_control_constraints(u) 
            if self.dynamic_problem:
                current_state = v_double()
                current_state[:] = [xs_adjusted[i][j] for j in xrange(len(xs_adjusted[i]))]
                control = v_double()
                control[:] = u
                
                control_error = v_double()
                control_error[:] = [0.0 for k in xrange(len(u))]
                result = v_double()
                robot.propagate(current_state,
                                control,
                                control_error,
                                self.simulation_step_size,
                                control_durations[i],
                                result)
                xs_adjusted.append(np.array([result[k] for k in xrange(len(result))]))            
                us_adjusted.append(np.array([u[k] for k in xrange(len(u))]))
                zs_adjusted.append(np.array([result[k] for k in xrange(len(result))]))
                #print "xs_adjusted: " + str(np.array([result[k] for k in xrange(len(result))]))
                #print "xs_prior: " + str(xs[i + 1])
            else:
                current_state = np.array([xs_adjusted[i][j] for j in xrange(len(xs_adjusted[i]))])
                result = np.dot(As[i], current_state) + np.dot(Bs[i], u)
                xs_adjusted.append(np.array([result[k] for k in xrange(len(result))]))            
                us_adjusted.append(np.array([u[k] for k in xrange(len(u))]))
                zs_adjusted.append(np.array([result[k] for k in xrange(len(result))]))
                
            
            u_dash = u - us[i]
            z_dash = np.array([0.0 for k in xrange(len(zs_adjusted[-1]))])
            
            """
            Maximum likelikhood observation
            """
            """ Kalman prediction and update """                   
            x_tilde_dash, P_dash = kalman.kalman_predict(x_tilde, u_dash, As[i], Bs[i], P_t, Vs[i], Ms[i])
            x_tilde, P_t = kalman.kalman_update(x_tilde_dash, 
                                                z_dash, 
                                                Hs[i], 
                                                P_dash, 
                                                Ws[i], 
                                                Ns[i])
            
            x_estimated = x_tilde + xs[i + 1]
            #x_estimated = x_tilde + xs_adjusted[-1]
        us_adjusted.append(np.array([0.0 for i in xrange(len(us[0]))]))
        control_durations_adjusted = [control_durations[i] for i in xrange(len(control_durations))]
        control_durations_adjusted.append(0.0)
        return (xs_adjusted, us_adjusted, zs_adjusted, control_durations_adjusted, True)


if __name__ == "__main__":
    PlanAdjuster()
import numpy as np
import kalman as kalman

class PlanAdjuster:
    def __init__(self, robot):
        self.robot = robot
    
    def setC(self, C):
        self.C = C
        
    def sedD(self, D):
        self.D = D
        
    def set_simulation_step_size(self, simulation_step_size):
        self.simulation_step_size = simulation_step_size
    
    def adjust_plan(self, 
                    robot,
                    plan, 
                    step,
                    x_predicted,
                    x_estimates):
        xs = [plan[0][i] for i in xrange(step, len(plan[0]))]
        us = [plan[1][i] for i in xrange(step, len(plan[1]))]
        zs = [plan[2][i] for i in xrange(step, len(plan[3]))]
        control_durations = [plan[3][i] for i in xrange(step, len(plan[4]))]
        As, Bs, Vs, Ms, Hs, Ws, Ns = self.get_linear_model_matrices(xs, us, control_durations)
        Ls = kalman.compute_gain(As, Bs, self.C, self.D, horizon_L - 1) 
        
        xs_adjusted = []
        us_adjusted = []
        zs_adjusted = []
        
        for i in xrange(len(xs)):
            u = np.dot(Ls[i], x_predicted - x_estimated) + us[i]            
            current_state = v_double()
            current_state = xs[i]
            control = v_double()
            control[:] = u
            
            control_error = v_double()
            control_error[:] = [0.0 for k in xrange(len(u))]
            
            robot.propagate(current_state,
                            control,
                            control_error,
                            self.simulation_step_size,
                            control_durations[i],
                            result)
            xs_adjusted.append(np.array([result[k] for k in xrange(len(result)))])
            us_adjusted.append(np.array([u[k] for k in xrange(len(u))]))
            
            """
            Maximum likelikhood observation
            """
            
            
        


if __name__ == "__main__":
    PlanAdjuster()
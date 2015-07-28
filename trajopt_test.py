import openravepy
import trajoptpy
import json
import time
import numpy as np
from trajoptpy.check_traj import traj_is_safe

class TrajoptTest:
    def __init__(self, interactive):
        env = openravepy.Environment()
        env.StopSimulation()
        print "Loading robot model"
        env.Load("model/model_test.xml")
        print "loaded robot model"
        print "Loading environment"
        env.Load("environment/env.xml")
        print "Environment loaded"
        
        trajoptpy.SetInteractive(True) # pause every iteration, until you press 'p'. Press escape to disable further plotting
        robot = env.GetRobots()[0]
        print robot.GetManipulator("arm")
        print robot.GetManipulator('arm').GetArmIndices()
        
        joint_start = [0.0, 0.0]
        
        robot.SetDOFValues(joint_start, robot.GetManipulator('arm').GetArmIndices())
        
        joint_target = [-np.pi / 2.0, 0.0]
        n_steps = 100
        control_rate = 30.0
        delta_t = 1.0 / control_rate
        max_velocity = 2.0
        
        request = {
          "basic_info" : {
            "n_steps" : n_steps,
            "manip" : "arm", # see below for valid values
            "start_fixed" : True # i.e., DOF values at first timestep are fixed based on current robot state
          },
          "costs" : [
          {
            "type" : "joint_vel", # joint-space velocity cost
            "params": {"coeffs" : [1]} # a list of length one is automatically expanded to a list of length n_dofs
            # also valid: [1.9, 2, 3, 4, 5, 5, 4, 3, 2, 1]
          },
          {
            "type" : "collision",
            "params" : {
              "coeffs" : [20], # penalty coefficients. list of length one is automatically expanded to a list of length n_timesteps
              "dist_pen" : [0.25] # robot-obstacle distance that penalty kicks in. expands to length n_timesteps
            },    
          }
          ],
          "constraints" : [
          {
            "type" : "joint", # joint-space target
            "params" : {"vals" : joint_target } # length of vals = # dofs of manip
          },
          {
            "type" : "joint_vel_limits",
            "name" : "joint_vel_limits",
            "params" : {
              "vals" : [delta_t * max_velocity, delta_t * max_velocity],
              "first_step" : 0,
              "last_step" : n_steps-1, #inclusive              
             }
          }
          ],
          "init_info" : {
              "type" : "straight_line", # straight line in joint space.
              "endpoint" : joint_target
          }
        }
        s = json.dumps(request)
        prob = trajoptpy.ConstructProblem(s, env) # create object that stores optimization problem
        t_start = time.time()
        result = trajoptpy.OptimizeProblem(prob) # do optimization
        t_elapsed = time.time() - t_start
        print result
        print "optimization took %.3f seconds"%t_elapsed
        prob.SetRobotActiveDOFs() # set robot DOFs to DOFs in optimization problem
        print traj_is_safe(result.GetTraj(), robot) # Check that trajectory is collision free
        
        
if __name__ == "__main__":
    TrajoptTest(True)
import sys
import argparse
import time
import numpy as np
import os
import glob
import scipy
from serializer import Serializer
from libutil import *
import logging
from librobot import v_string, Robot
from util_py import check_positive_definite, get_goal_states, copyToTmp
from simulator import Simulator
from path_evaluator import PathEvaluator
from path_planning_interface import PathPlanningInterface
from libobstacle import Obstacle, Terrain

class LQG:
    def __init__(self, show_scene, deserialize, append_paths):
        """ Reading the config """
        self.init_serializer()
        config = self.serializer.read_config("config_lqg.yaml")
        self.set_params(config)
        logging_level = logging.WARN
        if config['verbose']:
            logging_level = logging.DEBUG
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging_level)        
        np.set_printoptions(precision=16)
        dir = "stats/lqg"        
        self.utils = Utils()
        model_file = "model/test.xml"
        urdf_model_file = "model/test.urdf"
        self.init_robot(urdf_model_file)
        environment_file = os.path.join("environment", "env.xml")        
        if not self.setup_scene("environment", "env.xml", self.robot):
            return
        #self.show_state_distribution(urdf_model_file, environment_file)
        if show_scene:
            self.run_viewer(urdf_model_file, environment_file)
        self.clear_stats(dir)
        logging.info("Start up simulator")
        sim = Simulator()
        path_evaluator = PathEvaluator()
        path_planner = PathPlanningInterface()
        
                
        logging.info("LQG: Generating goal states...")
        goal_states = get_goal_states("lqg",
                                      self.serializer, 
                                      self.obstacles,
                                      model_file,                                      
                                      self.robot,                                    
                                      self.max_velocity,
                                      self.delta_t,
                                      self.start_state,
                                      self.goal_position,
                                      self.goal_radius,
                                      self.planning_algortihm,
                                      self.path_timeout)
          
        if len(goal_states) == 0:
            logging.error("LQG: Couldn't generate any goal states. Problem seems to be infeasible")
            return
        logging.info("LQG: Generated " + str(len(goal_states)) + " goal states")         
        sim.setup_reward_function(self.discount_factor, self.step_penalty, self.illegal_move_penalty, self.exit_reward)  
        path_planner.setup(self.robot,                           
                           self.obstacles,  
                           self.max_velocity, 
                           self.delta_t, 
                           self.use_linear_path,
                           self.planning_algortihm,
                           self.path_timeout)
        
        if self.dynamic_problem:
            path_planner.setup_dynamic_problem(urdf_model_file,
                                               environment_file,
                                               self.simulation_step_size,                                               
                                               self.continuous_collision,
                                               self.num_control_samples,
                                               self.min_control_duration,
                                               self.max_control_duration,
                                               self.add_intermediate_states)       
        path_planner.set_start_and_goal(self.start_state, goal_states, self.goal_position, self.goal_radius)         
        A, H, B, V, W, C, D, M_base, N_base = self.problem_setup(self.delta_t, self.robot_dof)
        
        if check_positive_definite([C, D]):
            m_covs = None
            if self.inc_covariance == "process":
                m_covs = np.linspace(self.min_process_covariance, 
                                     self.max_process_covariance, 
                                     self.covariance_steps)                 
            elif self.inc_covariance == "observation":          
                m_covs = np.linspace(self.min_observation_covariance, 
                                     self.max_observation_covariance,
                                     self.covariance_steps)            
            emds = []
            mean_planning_times = []
            time_to_generate_paths = 0.0            
            paths = []
            if ((not append_paths) and deserialize):
                paths = self.serializer.deserialize_paths("paths.txt", self.robot_dof)                
                #paths = [paths[236], paths[386]]
            if len(paths) == 0:
                print "LQG: Generating " + str(self.num_paths) + " paths from the inital state to the goal position..."
                t0 = time.time()
                paths = path_planner.plan_paths(self.num_paths, 0)
                if len(paths) == 0:
                    logging.error("LQG: Couldn't create any paths within the given time.")
                    return
                time_to_generate_paths = time.time() - t0 
                print "LQG: Time to generate paths: " + str(time_to_generate_paths) + " seconds"
                if self.plot_paths:                
                    self.serializer.save_paths(paths, "paths.yaml", self.overwrite_paths_file, path=dir)                
                if deserialize or append_paths:                    
                    self.serializer.serialize_paths("paths.txt", paths, append_paths, self.robot_dof)
                    paths = self.serializer.deserialize_paths("paths.txt", self.robot_dof)
            
            """ Determine average path length """
            avg_path_length = self.get_avg_path_length(paths)            
            self.serializer.save_avg_path_lengths(avg_path_length, path=dir)                                       
            cart_coords = []  
            best_paths = []
            all_rewards = []
            successes = []            
                                           
            for j in xrange(len(m_covs)):
                print "LQG: Evaluating " + str(len(paths)) + " paths for covariance value " + str(m_covs[j]) + "..."
                M = None
                N = None
                if self.inc_covariance == "process":
                    """ The process noise covariance matrix """
                    M = m_covs[j] * M_base
                    
                    """ The observation error covariance matrix """
                    N = self.min_observation_covariance * N_base
                elif self.inc_covariance == "observation":
                    M = self.min_process_covariance * M_base
                    N = m_covs[j] * N_base
                P_t = np.array([[0.0 for k in xrange(2 * self.robot_dof)] for l in xrange(2 * self.robot_dof)]) 
                path_evaluator.setup(A, B, C, D, H, M, N, V, W,                                     
                                     self.robot, 
                                     self.sample_size, 
                                     self.obstacles,
                                     self.goal_position,
                                     self.goal_radius,
                                     self.show_viewer,
                                     urdf_model_file,
                                     environment_file)
                if self.dynamic_problem:
                    path_evaluator.setup_dynamic_problem()
                path_evaluator.setup_reward_function(self.step_penalty, self.illegal_move_penalty, self.exit_reward, self.discount_factor)
                t0 = time.time()
                 
                (path_index,
                 xs, 
                 us, 
                 zs, 
                 control_durations, 
                 objective, 
                 state_covariances) = path_evaluator.evaluate_paths(paths, P_t, 0)
                 
                te = time.time() - t0
                print "LQG: Time to evaluate " + str(len(paths)) + " paths: " + str(te) + "s"
                mean_planning_time = time_to_generate_paths + te
                print "LQG: Best objective value: " + str(objective)
                print "LQG: Length of best path: " + str(len(xs))
                print "LQG: Best path has index " + str(path_index)  
                best_paths.append([[xs[i] for i in xrange(len(xs))], 
                                   [us[i] for i in xrange(len(us))],
                                   [zs[i] for i in xrange(len(zs))],
                                   [control_durations[i] for i in xrange(len(control_durations))]])
                
                sim.setup_problem(A, B, C, D, H, V, W, M, N,
                                  self.robot, 
                                  self.enforce_control_constraints,
                                  self.obstacles, 
                                  self.goal_position, 
                                  self.goal_radius,
                                  self.max_velocity,                                  
                                  self.show_viewer,
                                  urdf_model_file,
                                  environment_file)                              
                sim.setup_simulator(self.num_simulation_runs, self.stop_when_terminal)
                if self.dynamic_problem:
                    sim.setup_dynamic_problem(self.simulation_step_size)                
                successes = 0
                num_collisions = 0 
                rewards_cov = []
                num_steps = 0
                print "LQG: Running " + str(self.num_simulation_runs) + " simulations..."              
                for k in xrange(self.num_simulation_runs):
                    self.serializer.write_line("log.log", "tmp/lqg", "RUN #" + str(k + 1) + " \n")
                    print "simulation run: " + str(k)
                    (x_true, 
                     x_tilde,
                     x_tilde_linear, 
                     x_estimate, 
                     P_t, 
                     current_step, 
                     total_reward, 
                     success, 
                     terminal,
                     estimated_s,
                     estimated_c,                     
                     history_entries) = sim.simulate_n_steps(xs, us, zs,
                                                             state_covariances, 
                                                             control_durations,
                                                             xs[0],
                                                             np.array([0.0 for i in xrange(2 * self.robot_dof)]),
                                                             np.array([0.0 for i in xrange(2 * self.robot_dof)]),
                                                             xs[0],
                                                             np.array([[0.0 for k in xrange(2 * self.robot_dof)] for l in xrange(2 * self.robot_dof)]),
                                                             0.0,                                                           
                                                             0,
                                                             len(xs) - 1)
                    if success:
                        successes += 1
                    rewards_cov.append(total_reward)
                    for history_entry in history_entries:                        
                        history_entry.serialize("tmp/lqg", "log.log")
                        if history_entry.collided:
                            num_collisions += 1 
                    num_steps += history_entries[-1].t                                         
                    self.serializer.write_line("log.log", "tmp/lqg", "Reward: " + str(total_reward) + " \n") 
                    self.serializer.write_line("log.log", "tmp/lqg", "\n")
                
                self.serializer.write_line("log.log", "tmp/lqg", "################################# \n")
                self.serializer.write_line("log.log",
                                           "tmp/lqg",
                                           "inc_covariance: " + str(self.inc_covariance) + " \n")
                if self.inc_covariance == "process":                  
                    self.serializer.write_line("log.log",
                                               "tmp/lqg",
                                               "Process covariance: " + str(m_covs[j]) + " \n")
                    self.serializer.write_line("log.log",
                                               "tmp/lqg",
                                               "Observation covariance: " + str(self.min_observation_covariance) + " \n")
                    
                elif self.inc_covariance == "observation":                    
                    self.serializer.write_line("log.log",
                                               "tmp/lqg",
                                               "Process covariance: " + str(self.min_process_covariance) + " \n")
                    self.serializer.write_line("log.log",
                                               "tmp/lqg",
                                               "Observation covariance: " + str(m_covs[j]) + " \n")
                    
                num_steps /= self.num_simulation_runs
                self.serializer.write_line("log.log", "tmp/lqg", "Mean number of steps: " + str(num_steps) + " \n")
                self.serializer.write_line("log.log", "tmp/lqg", "Objective value of best path: " + str(objective) + " \n")                
                self.serializer.write_line("log.log", "tmp/lqg", "Mean num collisions per run: " + str(float(num_collisions) / float(self.num_simulation_runs)) + " \n")
                print "total num collisions " + str(num_collisions)    
                print "mean num collisions " + str(float(num_collisions) / float(self.num_simulation_runs))
                self.serializer.write_line("log.log", "tmp/lqg", "Length best path: " + str(len(xs)) + " \n")
                self.serializer.write_line("log.log", 
                                      "tmp/lqg", 
                                      "Average distance to goal area: 0 \n")
                self.serializer.write_line("log.log", "tmp/lqg", "Num successes: " + str(successes) + " \n")
                print "succ " + str((100.0 / self.num_simulation_runs) * successes)
                self.serializer.write_line("log.log", "tmp/lqg", "Percentage of successful runs: " + str((100.0 / self.num_simulation_runs) * successes) + " \n")
                self.serializer.write_line("log.log", "tmp/lqg", "Mean planning time: " + str(mean_planning_time) + " \n")
                
                n, min_max, mean, var, skew, kurt = scipy.stats.describe(np.array(rewards_cov))
                self.serializer.write_line("log.log", "tmp/lqg", "Mean rewards: " + str(mean) + " \n")
                self.serializer.write_line("log.log", "tmp/lqg", "Reward variance: " + str(var) + " \n")
                self.serializer.write_line("log.log", 
                                      "tmp/lqg", 
                                      "Reward standard deviation: " + str(np.sqrt(var)) + " \n")
                cmd = "mv tmp/lqg/log.log " + dir + "/log_lqg_" + str(m_covs[j]) + ".log"
                os.system(cmd)
                
            
            if self.plot_paths:
                self.serializer.save_paths(best_paths, 'best_paths.yaml', True, path=dir)                      
            #self.serializer.save_stats(stats, path=dir)
            
            cmd = "cp config_lqg.yaml " + dir           
            os.system(cmd)
            
            if not os.path.exists(dir + "/environment"):
                os.makedirs(dir + "/environment") 
                       
            cmd = "cp environment/env.xml " + dir + "/environment"
            os.system(cmd) 
            
            if not os.path.exists(dir + "/model"):
                os.makedirs(dir + "/model")
                
            cmd = "cp " + urdf_model_file + " " + dir + "/model"
            os.system(cmd)
            cmd = "cp " + model_file + " " + dir + "/model"
            os.system(cmd)         
        print "Done"
        
    def init_robot(self, urdf_model_file):
        self.robot = Robot(urdf_model_file)
        self.robot.enforceConstraints(self.enforce_constraints)
        self.robot.setGravityConstant(self.gravity_constant)
        
    """
    Analyzing functions (will be removed later)
    =====================================================================
    """
        
    def sample_control_error(self, M):
        mu = np.array([0.0 for i in xrange(2 * self.robot_dof)])
        return np.random.multivariate_normal(mu, M)
    
    def show_state_distribution(self, model_file, env_file):
        self.robot.setupViewer(model_file, env_file)       
        M = 30.0 * np.identity(2 * self.robot_dof)
        active_joints = v_string()
        self.robot.getActiveJoints(active_joints)
        self.robot_dof = len(active_joints)
        x = [0.0, -np.pi / 2.0, 0.0, 0.0, 0.0, 0.0]
        states = []
        for z in xrange(2000):
            u = [70.0, 70.0, 70.0, 0.0, 0.0, 0.0]
            current_state = v_double()
            current_state[:] = x
            control = v_double()
            control[:] = u            
            control_error = v_double()
            ce = self.sample_control_error(M)
            #ce = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            control_error[:] = ce
            x = None
            for k in xrange(1):
                result = v_double()
                self.robot.propagate(current_state,
                                     control,
                                     control_error,
                                     self.simulation_step_size,
                                     self.delta_t,
                                     result)                               
                x = [result[i] for i in xrange(len(result))]
                
                current_state[:] = x
                print x
            states.append(np.array(x[3:6]))
            x = [0.0, -np.pi/ 2.0, 0.0, 0.0, 0.0, 0.0]
            
            cjvals = v_double()
            cjvels = v_double()
            cjvals_arr = [x[i] for i in xrange(len(x) / 2)]
            cjvels_arr = [x[i] for i in xrange(len(x) / 2, len(x))]
            cjvals[:] = cjvals_arr
            cjvels[:] = cjvels_arr
            particle_joint_values = v2_double()
            particle_joint_colors = v2_double()
            self.robot.updateViewerValues(cjvals, 
                                          cjvels,
                                          particle_joint_values,
                                          particle_joint_values)
        from plot import plot_3d_points
        mins = []
        maxs = []
        
        x_min = min([states[i][0] for i in xrange(len(states))])
        x_max = max([states[i][0] for i in xrange(len(states))])
        y_min = min([states[i][1] for i in xrange(len(states))])
        y_max = max([states[i][1] for i in xrange(len(states))])
        z_min = min([states[i][2] for i in xrange(len(states))])
        z_max = max([states[i][2] for i in xrange(len(states))])
        
        scale = [-0.2, 0.2]
        plot_3d_points(np.array(states), 
                       x_scale = [x_min, x_max], 
                       y_scale = [y_min, y_max], 
                       z_scale=  [z_min, z_max])
        sleep
        
    def run_viewer(self, model_file, env_file):        
        self.robot.setupViewer(model_file, env_file)        
        fx = 0.0
        fy = 0.0
        fz = 0.0
        f_roll = 0.0
        f_pitch = 0.0
        f_yaw = 0.0        
        x = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        while True:            
            #u_in = [3.0, 1.5, 0.0, 0.0, 0.0, 0.0]
            u_in = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            current_state = v_double()
            current_state[:] = x
            control = v_double()            
            control[:] = u_in
                       
            control_error = v_double()
            ce = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            control_error[:] = ce
            result = v_double()
            ja_start = v_double()
            ja_start[:] = [current_state[i] for i in xrange(len(current_state) / 2)]
            collision_objects_start = self.robot.createRobotCollisionObjects(ja_start)             
            self.robot.propagate(current_state,
                                 control,
                                 control_error,
                                 self.simulation_step_size,
                                 0.03,
                                 result)
            joint_angles = v_double()
            joint_angles[:] = [result[i] for i in xrange(len(result) / 2)]
            collision_objects_goal = self.robot.createRobotCollisionObjects(joint_angles)
            #print "ee_velocity " + str([ee_velocity[i] for i in xrange(len(ee_velocity))])
            
            '''
            Get the end effector position
            '''
            ee_position = v_double()            
            self.robot.getEndEffectorPosition(joint_angles, ee_position)            
            ee_collision_objects = self.robot.createEndEffectorCollisionObject(joint_angles)
            #collision_objects = self.robot.createRobotCollisionObjects(joint_angles)
            
            
            in_collision = False
            
            for o in self.obstacles:
                #in_collision = o.inCollisionDiscrete(collision_objects)
                if o.inCollisionDiscrete(ee_collision_objects):
                    sleep                    
                    in_collision = True                    
                    ###Get the end effector velocity vector                                       
                    ee_velocity = v_double()
                    self.robot.getEndEffectorVelocity(result, ee_velocity)                   
                    ee_velocity_vec = np.array([ee_velocity[i] for i in xrange(len(ee_velocity))])
                    #ee_velocity_vec /= np.linalg.norm(ee_velocity_vec)
                    ext_force = o.getExternalForce()
                    force_vector = -ext_force * ee_velocity_vec
                    print "ee_velocity " + str(ee_velocity_vec)
                    print "force vector: " + str(force_vector)
                    self.robot.setExternalForce(fx + force_vector[0], 
                                                fy + force_vector[1], 
                                                fz + force_vector[2],
                                                f_roll + force_vector[3],
                                                f_pitch + force_vector[4],
                                                f_yaw + force_vector[5])
                    print "in collision!!!"           
                for i in xrange(len(collision_objects_start)):                        
                    if o.inCollisionContinuous([collision_objects_start[i], collision_objects_goal[i]]):
                        pass          
            if not in_collision:
                self.robot.setExternalForce(fx, fy, fz, f_roll, f_pitch, f_yaw)                                                              
            x = np.array([result[i] for i in xrange(len(result))])
            
            cjvals = v_double()
            cjvels = v_double()
            cjvals_arr = [x[i] for i in xrange(len(x) / 2)]
            cjvels_arr = [x[i] for i in xrange(len(x) / 2, len(x))]
            cjvals[:] = cjvals_arr
            cjvels[:] = cjvels_arr
            particle_joint_values = v2_double()
            self.robot.updateViewerValues(cjvals, 
                                          cjvels,
                                          particle_joint_values,
                                          particle_joint_values)
            time.sleep(0.03) 
            
    """
    ================================================================
    """
        
        
    def setup_scene(self, 
                    environment_path, 
                    environment_file,
                    robot):
        """ Load the obstacles """         
        self.obstacles = self.utils.loadObstaclesXML("environment/env.xml")      
        
        """ Load the goal area """
        goal_area = v_double()
        self.utils.loadGoalArea("environment/env.xml", goal_area)
        if len(goal_area) == 0:
            print "ERROR: Your environment file doesn't define a goal area"
            return False
        self.goal_position = [goal_area[i] for i in xrange(0, 3)]
        self.goal_radius = goal_area[3] 
        
        """ Setup operations """
        self.robot_dof = robot.getDOF()        
        return True
            
    def init_serializer(self):
        self.serializer = Serializer()
        self.serializer.create_temp_dir("lqg")        
            
    def clear_stats(self, dir):        
        if os.path.isdir(dir):
            cmd = "rm -rf " + dir + "/*"            
            os.system(cmd)
        else:
            os.makedirs(dir)
            
    def get_average_distance_to_goal_area(self, goal_position, goal_radius, cartesian_coords):        
        avg_dist = 0.0
        goal_pos = np.array(goal_position)        
        for i in xrange(len(cartesian_coords)):            
            cart = np.array(cartesian_coords[i])            
            dist = np.linalg.norm(goal_pos - cart)            
            if dist < goal_radius:
                dist = 0.0
            avg_dist += dist
        if avg_dist == 0.0:
            return avg_dist        
        return np.asscalar(avg_dist) / len(cartesian_coords)          
            
    def problem_setup(self, delta_t, num_links):
        A = np.identity(num_links * 2)
        H = np.identity(num_links * 2)
        W = np.identity(num_links * 2)
        C = self.path_deviation_cost * np.identity(num_links * 2)
        
        '''B = delta_t * np.identity(num_links * 2)                
        V = np.identity(num_links * 2)
        D = self.control_deviation_cost * np.identity(num_links * 2)        
        M_base = np.identity(2 * self.robot_dof)
        N_base = np.identity(2 * self.robot_dof)'''
        
        B = delta_t * np.vstack((np.identity(num_links),
                                 np.zeros((num_links, num_links))))
        V = np.vstack((np.identity(num_links),
                       np.zeros((num_links, num_links))))
        D = self.control_deviation_cost * np.identity(num_links)
        M_base = np.identity(self.robot_dof)
        N_base = np.identity(2 * self.robot_dof)
        
        
        return A, H, B, V, W, C, D, M_base, N_base
            
    def get_avg_path_length(self, paths):
        avg_length = 0.0
        for path in paths:            
            avg_length += len(path[0])
        return avg_length / len(paths)
        
    def set_params(self, config):        
        self.num_paths = config['num_generated_paths']
        self.use_linear_path = config['use_linear_path']        
        self.max_velocity = config['max_velocity']
        self.delta_t = 1.0 / config['control_rate']
        self.start_state = config['start_state']        
        self.num_simulation_runs = config['num_simulation_runs']
        self.num_bins = config['num_bins']
        self.min_process_covariance = config['min_process_covariance']
        self.max_process_covariance = config['max_process_covariance']
        self.covariance_steps = config['covariance_steps']
        self.min_observation_covariance = config['min_observation_covariance']
        self.max_observation_covariance = config['max_observation_covariance']       
        self.discount_factor = config['discount_factor']
        self.illegal_move_penalty = config['illegal_move_penalty']
        self.step_penalty = config['step_penalty']
        self.exit_reward = config['exit_reward']
        self.stop_when_terminal = config['stop_when_terminal']        
        self.enforce_constraints = config['enforce_constraints']
        self.enforce_control_constraints = config['enforce_control_constraints']
        self.sample_size = config['sample_size']
        self.plot_paths = config['plot_paths']
        self.planning_algortihm = config['planning_algorithm']
        self.dynamic_problem = config['dynamic_problem'] 
        self.simulation_step_size = config['simulation_step_size']        
        self.path_timeout = config['path_timeout'] 
        self.continuous_collision = config['continuous_collision_check']
        self.show_viewer = config['show_viewer']   
        self.path_deviation_cost = config['path_deviation_cost'] 
        self.control_deviation_cost = config['control_deviation_cost']
        self.num_control_samples = config['num_control_samples'] 
        self.min_control_duration = config['min_control_duration']
        self.max_control_duration = config['max_control_duration']   
        self.inc_covariance = config['inc_covariance'] 
        self.add_intermediate_states = config['add_intermediate_states']
        self.gravity_constant = config['gravity']

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='LQG-MP.')
    parser.add_argument("-s", "--show", 
                        help="Show the scene without executing LQG-MP", 
                        action="store_true")
    d_help_str = "Get the paths from 'paths.txt'. If no such file exists, generate the paths and serialize them."    
    parser.add_argument("-d", "--deserialize", 
                        help=d_help_str, 
                        action="store_true")
    d_help_str = "Append generated paths to the current paths file."    
    parser.add_argument("-a", "--append_paths", 
                        help=d_help_str, 
                        action="store_true") 
    args = parser.parse_args()  
    LQG(args.show, args.deserialize, args.append_paths)    
    

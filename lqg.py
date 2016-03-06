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
import warnings


class LQG:
    def __init__(self, show_scene, deserialize, append_paths):
        self.abs_path = os.path.dirname(os.path.abspath(__file__))
        """ Reading the config """
        warnings.filterwarnings("ignore")
        self.init_serializer()
        config = self.serializer.read_config("config_lqg.yaml", path=self.abs_path)
        self.set_params(config)
        if self.seed < 0:
            """
            Generate a random seed that will be stored
            """
            self.seed = np.random.randint(0, 42949672)
        np.random.seed(self.seed)  
        
        logging_level = logging.WARN
        if config['verbose']:
            logging_level = logging.DEBUG
        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging_level)        
        np.set_printoptions(precision=16)
        dir = self.abs_path + "/stats/lqg"
        tmp_dir = self.abs_path + "/tmp/lqg"        
        self.utils = Utils()
        if not self.init_robot(self.robot_file):
            logging.error("LQG: Couldn't initialize robot")
            return               
        if not self.setup_scene(self.environment_file, self.robot):
            return
        #self.show_state_distribution(urdf_model_file, environment_file)
        #self.check_continuous_collide(self.robot_file, self.environment_file)
        if show_scene:            
            self.run_viewer(self.robot_file, self.environment_file)
        self.clear_stats(dir)
        logging.info("Start up simulator")
        sim = Simulator()
        path_evaluator = PathEvaluator()
        path_planner = PathPlanningInterface()
        
                
        logging.info("LQG: Generating goal states...")        
        goal_states = get_goal_states("lqg",
                                      self.abs_path,
                                      self.serializer, 
                                      self.obstacles,                                                                           
                                      self.robot,                                    
                                      self.max_velocity,
                                      self.delta_t,
                                      self.start_state,
                                      self.goal_position,
                                      self.goal_radius,
                                      self.planning_algortihm,
                                      self.path_timeout,
                                      self.num_generated_goal_states,
                                      self.continuous_collision,
                                      self.environment_file,
                                      self.num_cores)  
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
                           self.path_timeout,
                           self.continuous_collision,
                           self.num_cores)
        
        if self.dynamic_problem:
            path_planner.setup_dynamic_problem(self.robot_file,
                                               self.environment_file,
                                               self.simulation_step_size,
                                               self.num_control_samples,
                                               self.min_control_duration,
                                               self.max_control_duration,
                                               self.add_intermediate_states,
                                               self.rrt_goal_bias,
                                               self.control_sampler)       
        path_planner.set_start_and_goal(self.start_state, goal_states, self.goal_position, self.goal_radius)         
        A, H, B, V, W, C, D, M_base, N_base = self.problem_setup(self.delta_t, self.robot_dof)
        time_to_generate_paths = 0.0
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
                paths = self.serializer.deserialize_paths(self.abs_path + "/paths.txt", self.robot_dof)
                #paths = [paths[26]]
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
                    self.serializer.serialize_paths(self.abs_path + "/paths.txt", paths, append_paths, self.robot_dof)                    
                    paths = self.serializer.deserialize_paths(self.abs_path + "/paths.txt", self.robot_dof)
            
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
                    M = self.calc_covariance_value(self.robot, m_covs[j], M_base)
                    #M = m_covs[j] * M_base
                    
                    """ The observation error covariance matrix """
                    N = self.calc_covariance_value(self.robot, 
                                                   self.min_observation_covariance, 
                                                   N_base, 
                                                   covariance_type='observation')                    
                elif self.inc_covariance == "observation":
                    M = self.calc_covariance_value(self.robot, 
                                                   self.min_process_covariance,
                                                   M_base)
                    N = self.calc_covariance_value(self.robot, 
                                                   m_covs[j],
                                                   N_base, 
                                                   covariance_type='observation')                    
                P_t = np.array([[0.0 for k in xrange(2 * self.robot_dof)] for l in xrange(2 * self.robot_dof)]) 
                path_evaluator.setup(A, B, C, D, H, M, N, V, W,                                     
                                     self.robot, 
                                     self.sample_size, 
                                     self.obstacles,
                                     self.goal_position,
                                     self.goal_radius,
                                     self.show_viewer_evaluation,
                                     self.robot_file,
                                     self.environment_file,
                                     self.num_cores)
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
                 state_covariances,
                 deviation_covariances,
                 estimated_deviation_covariances) = path_evaluator.evaluate_paths(paths, P_t, 0)
                 
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
                                  self.show_viewer_simulation,
                                  self.robot_file,
                                  self.environment_file)
                if self.dynamic_problem:
                    sim.setup_dynamic_problem(self.simulation_step_size)                
                successes = 0
                num_collisions = 0 
                rewards_cov = []
                final_states= []
                num_steps = 0
                collided_num = 0
                print "LQG: Running " + str(self.num_simulation_runs) + " simulations..."              
                for k in xrange(self.num_simulation_runs):
                    self.serializer.write_line("log.log", tmp_dir, "RUN #" + str(k + 1) + " \n")
                    print "simulation run: " + str(k)
                    (x_true, 
                     x_tilde,
                     x_tilde_linear, 
                     x_estimate,
                     z,
                     P_t, 
                     current_step, 
                     total_reward,
                     terminal,
                     estimated_s,
                     estimated_c,                     
                     history_entries) = sim.simulate_n_steps(xs, us, zs,                                                             
                                                             control_durations,
                                                             xs[0],
                                                             np.array([0.0 for i in xrange(2 * self.robot_dof)]),
                                                             np.array([0.0 for i in xrange(2 * self.robot_dof)]),
                                                             xs[0],
                                                             np.array([[0.0 for k in xrange(2 * self.robot_dof)] for l in xrange(2 * self.robot_dof)]),
                                                             0.0,                                                           
                                                             0,
                                                             len(xs) - 1,
                                                             deviation_covariances,
                                                             estimated_deviation_covariances)
                    if terminal:
                        successes += 1
                    rewards_cov.append(total_reward)
                    #n, min_max, mean, var, skew, kurt = scipy.stats.describe(np.array(rewards_cov))                    
                    collided = False
                    for l in xrange(len(history_entries)):
                        history_entries[l].set_estimated_covariance(state_covariances[l])                        
                        history_entries[l].serialize(tmp_dir, "log.log")
                        if history_entries[l].collided:                            
                            num_collisions += 1
                            collided = True                        
                    if collided:
                        collided_num += 1
                    num_steps += history_entries[-1].t
                    final_states.append(history_entries[-1].x_true)                                         
                    self.serializer.write_line("log.log", tmp_dir, "Reward: " + str(total_reward) + " \n") 
                    self.serializer.write_line("log.log", tmp_dir, "\n")
                    
                """ Calculate the distance to goal area
                """  
                ee_position_distances = [] 
                for state in final_states:
                    joint_angles = v_double()
                    joint_angles[:] = [state[y] for y in xrange(len(state) / 2)]
                    ee_position = v_double()
                    self.robot.getEndEffectorPosition(joint_angles, ee_position)
                    ee_position = np.array([ee_position[y] for y in xrange(len(ee_position))])
                    dist = np.linalg.norm(np.array(self.goal_position - ee_position))
                    if dist < self.goal_radius:
                        dist = 0.0
                    ee_position_distances.append(dist)
                n, min_max, mean_distance_to_goal, var, skew, kurt = scipy.stats.describe(np.array(ee_position_distances))                         
                
                self.serializer.write_line("log.log", tmp_dir, "################################# \n")
                self.serializer.write_line("log.log",
                                           tmp_dir,
                                           "inc_covariance: " + str(self.inc_covariance) + "\n")
                if self.inc_covariance == "process":                  
                    self.serializer.write_line("log.log",
                                               tmp_dir,
                                               "Process covariance: " + str(m_covs[j]) + " \n")
                    self.serializer.write_line("log.log",
                                               tmp_dir,
                                               "Observation covariance: " + str(self.min_observation_covariance) + " \n")
                    
                elif self.inc_covariance == "observation":                    
                    self.serializer.write_line("log.log",
                                               tmp_dir,
                                               "Process covariance: " + str(self.min_process_covariance) + " \n")
                    self.serializer.write_line("log.log",
                                               tmp_dir,
                                               "Observation covariance: " + str(m_covs[j]) + " \n")
                    
                num_steps /= self.num_simulation_runs
                self.serializer.write_line("log.log", tmp_dir, "Mean number of steps: " + str(num_steps) + " \n")
                self.serializer.write_line("log.log", tmp_dir, "Objective value of best path: " + str(objective) + " \n")                
                self.serializer.write_line("log.log", tmp_dir, "Mean num collisions per run: " + str(float(num_collisions) / float(self.num_simulation_runs)) + " \n")
                print "collision_prob " + str(collided_num / float(self.num_simulation_runs))
                print "total num collisions " + str(num_collisions)    
                print "mean num collisions " + str(float(num_collisions) / float(self.num_simulation_runs))
                
                self.serializer.write_line("log.log", tmp_dir, "Length best path: " + str(len(xs)) + " \n")
                self.serializer.write_line("log.log", tmp_dir, "Index of best path: " + str(path_index) + " \n")
                self.serializer.write_line("log.log", 
                                           tmp_dir, 
                                           "Average distance to goal area: " + str(mean_distance_to_goal) + " \n")
                self.serializer.write_line("log.log", tmp_dir, "Num successes: " + str(successes) + " \n")
                print "succ " + str((100.0 / self.num_simulation_runs) * successes)
                self.serializer.write_line("log.log", tmp_dir, "Percentage of successful runs: " + str((100.0 / self.num_simulation_runs) * successes) + " \n")
                self.serializer.write_line("log.log", tmp_dir, "Mean planning time: " + str(mean_planning_time) + " \n")
                
                n, min_max, mean, var, skew, kurt = scipy.stats.describe(np.array(rewards_cov))
                print "mean_rewards " + str(mean)
                #plt.plot_histogram_from_data(rewards_cov)
                #sleep
                self.serializer.write_line("log.log", tmp_dir, "Mean rewards: " + str(mean) + " \n")
                self.serializer.write_line("log.log", tmp_dir, "Reward variance: " + str(var) + " \n")
                self.serializer.write_line("log.log", 
                                      tmp_dir, 
                                      "Reward standard deviation: " + str(np.sqrt(var)) + " \n")
                self.serializer.write_line("log.log", tmp_dir, "Seed: " + str(self.seed) + " \n")
                cmd = "mv " + tmp_dir + "/log.log " + dir + "/log_lqg_" + str(m_covs[j]) + ".log"
                os.system(cmd)
                
            
            if self.plot_paths:
                self.serializer.save_paths(best_paths, 'best_paths.yaml', True, path=dir)                      
            #self.serializer.save_stats(stats, path=dir)
            
            cmd = "cp " + self.abs_path + "/config_lqg.yaml " + dir           
            os.system(cmd)
            
            if not os.path.exists(dir + "/environment"):
                os.makedirs(dir + "/environment") 
            
            cmd = "cp " + self.abs_path + "/" + str(self.environment_file) + " " + str(dir) + "/environment"
            os.system(cmd) 
            
            if not os.path.exists(dir + "/model"):
                os.makedirs(dir + "/model")
                
            cmd = "cp " + self.abs_path + "/" + self.robot_file + " " + dir + "/model"
            os.system(cmd)
        print "LQG: Time to generate paths: " + str(time_to_generate_paths) + " seconds"                   
        print "Done"
        
    def calc_covariance_value(self, robot, error, covariance_matrix, covariance_type='process'):
        active_joints = v_string()
        robot.getActiveJoints(active_joints)        
        if covariance_type == 'process':
            if self.dynamic_problem:            
                torque_limits = v_double()
                robot.getJointTorqueLimits(active_joints, torque_limits)
                torque_limits = [torque_limits[i] for i in xrange(len(torque_limits))]
                for i in xrange(len(torque_limits)):
                    torque_range = 2.0 * torque_limits[i]
                    covariance_matrix[i, i] = np.square((torque_range / 100.0) * error)
            else:
                for i in xrange(self.robot_dof):
                    covariance_matrix[i, i] = np.square((self.max_velocity / 100.0) * error)
        else:
            lower_position_limits = v_double()
            upper_position_limits = v_double()
            velocity_limits = v_double()            
            robot.getJointLowerPositionLimits(active_joints, lower_position_limits)
            robot.getJointUpperPositionLimits(active_joints, upper_position_limits)
            robot.getJointVelocityLimits(active_joints, velocity_limits)            
            for i in xrange(self.robot_dof):
                position_range = upper_position_limits[i] - lower_position_limits[i]
                covariance_matrix[i, i] = np.square((position_range / 100.0) * error)            
                velocity_range = 2.0 * velocity_limits[i]
                covariance_matrix[i + self.robot_dof, i + self.robot_dof] = np.square((velocity_range / 100.0) * error)          
        return covariance_matrix
        
        
    def init_robot(self, urdf_model_file):
        self.robot = Robot(self.abs_path + "/" + urdf_model_file)
        self.robot.enforceConstraints(self.enforce_constraints)
        self.robot.setGravityConstant(self.gravity_constant)
        self.robot.setAccelerationLimit(self.acceleration_limit)
        """ Setup operations """
        self.robot_dof = self.robot.getDOF()        
        if len(self.start_state) != 2 * self.robot_dof:
            logging.error("LQG: Start state dimension doesn't fit to the robot state space dimension")
            return False
        return True
        
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
        
        #x = [0.0, -np.pi / 2.0, 0.0, 0.0, 0.0, 0.0]
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
        
    def check_continuous_collide(self, model_file, env_file):
        self.robot.setViewerBackgroundColor(0.6, 0.8, 0.6)
        self.robot.setViewerSize(1280, 768)
        self.robot.setupViewer(model_file, env_file)        
        x1 = [0.0115186,
              0.1404386,
              -0.17235397,
              -0.22907283,
              0.88922755,
              1.35271925,
              -3.38231963,
              -1.51419397]
        x2 = [0.04028825,
              0.18442375,
              -0.25677913,
              -0.33145854,
              0.81281939,
              1.51040733,
              -2.37524124,
              -3.43838968]
        #x1 = [np.pi - np.pi / 4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #x1 = [np.pi / 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        #x2 = [np.pi / 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        coll_t = 0.0
        k = 0
        while True:
            k += 1            
            cjvals = v_double()
            cjvels = v_double()
            cjvals_arr = [x1[i] for i in xrange(len(x1) / 2)]
            cjvels_arr = [x1[i] for i in xrange(len(x1) / 2, len(x1))]
            cjvals[:] = cjvals_arr
            cjvels[:] = cjvels_arr
            particle_joint_values = v2_double()
            
            ja_start = v_double()
            ja_goal = v_double()
            ja_start[:] = [x1[i] for i in xrange(len(x1) / 2)]
            ja_goal[:] = [x2[i] for i in xrange(len(x2) / 2)]
            collision_objects_start = self.robot.createRobotCollisionObjects(ja_start)
            collision_objects_goal = self.robot.createRobotCollisionObjects(ja_goal)
            in_collision_discrete = False
            in_collision_continuous = False             
            t0 = time.time()
            breaking = False      
            for o in self.obstacles:                
                #in_collision_discrete_start = o.inCollisionDiscrete(collision_objects_start)
                #in_collision_discrete_goal = o.inCollisionDiscrete(collision_objects_goal)
                for i in xrange(len(collision_objects_start)):
                    in_collision_continuous = o.inCollisionContinuous([collision_objects_start[i], collision_objects_goal[i]])
                    if in_collision_continuous:                        
                        break
                if in_collision_continuous:
                    break                              
                '''if in_collision_discrete_start:                    
                    print "start collides discrete"
                    in_collision_discrete = True
                    break
                if in_collision_discrete_goal:
                    print "goal collides discrete"
                    in_collision_discrete = True
                    break
                if in_collision_continuous:
                    print "collides continuous"
                    break                     
            print "in collision discrete " + str(in_collision_discrete)
            print "in collision continuous " + str(in_collision_continuous)'''
            print "in collision continuous " + str(in_collision_continuous)
            coll_t += time.time() - t0
            if k == 10000:
                print coll_t / k
                sleep
            self.robot.updateViewerValues(cjvals, 
                                          cjvels,
                                          particle_joint_values,
                                          particle_joint_values)
            #time.sleep(10)
            time.sleep(0.03) 
            
        
    def run_viewer(self, model_file, env_file):
        show_viewer = True
        rot = v_double()
        trans = v_double()
        rot[:] = [-1.0, 0.0, 0.0, 0.0]
        trans[:] = [0.0, 0.0, 3.0]        
        if show_viewer:
            self.robot.setViewerBackgroundColor(0.6, 0.8, 0.6)
            self.robot.setViewerSize(1280, 768)
            self.robot.setupViewer(model_file, env_file)
        fx = 0.0
        fy = 0.0
        fz = 0.0
        f_roll = 0.0
        f_pitch = 0.0
        f_yaw = 0.0         
        x = [0.0 for i in xrange(2 * self.robot_dof)]
        x = [0.0, 0.0, 0.0, 0.0,
             0.0,
             0.0,
             0.0,
             0.0]
        x_true = [0.0 for i in xrange(2 * self.robot_dof)]        
        integration_times = [] 
        collision_check_times1 = []
        collision_check_times2 = []     
        
        y = 0
        while True:                    
            #u_in = [3.0, 1.5, 0.0, 0.0, 0.0, 0.0]
            u_in = [0.0 for i in xrange(self.robot_dof)]
            #u_in[0] = 150.0
            #u_in[1] = 70.0
            current_state = v_double()            
            current_state[:] = x            
            control = v_double()            
            control[:] = u_in
                       
            control_error = v_double()
            ce = [0.0 for i in xrange(self.robot_dof)]
            control_error[:] = ce
            result = v_double()            
            
            t0 = time.time()                    
            self.robot.propagate(current_state,
                                 control,
                                 control_error,
                                 self.simulation_step_size,
                                 0.03,
                                 result)
            t = time.time() - t0
            integration_times.append(t)
            if y == 10000:
                t_sum = sum(integration_times)
                t_mean = t_sum / len(integration_times)
                print "mean integration times: " + str(t_mean)
                t_sum = sum(collision_check_times1)
                t_mean = t_sum / len(collision_check_times1)
                print "mean collision check times old " + str(t_mean)
                t_mean = sum(collision_check_times2) / len(collision_check_times2)
                print "mean collision check times new " + str(t_mean)
                sleep            
            
            ja_start = v_double()            
            ja_start[:] = [current_state[i] for i in xrange(len(current_state) / 2)]                     
            collision_objects_start = self.robot.createRobotCollisionObjects(ja_start)
            joint_angles = v_double()
            joint_angles[:] = [result[i] for i in xrange(len(result) / 2)]
            collision_objects_goal = self.robot.createRobotCollisionObjects(joint_angles)
            #print "ee_velocity " + str([ee_velocity[i] for i in xrange(len(ee_velocity))])
            t_coll_check1 = time.time()            
            t2 = time.time() - t_coll_check1
            collision_check_times2.append(t2)            
            
            '''
            Get the end effector position
            '''
            #ee_position = v_double()            
            #self.robot.getEndEffectorPosition(joint_angles, ee_position)                      
            #ee_collision_objects = self.robot.createEndEffectorCollisionObject(joint_angles)
            in_collision = False
            t_c = time.time()            
            for o in self.obstacles:                
                if o.inCollisionDiscrete(collision_objects_goal):                                        
                    in_collision = True
                    break                              
                '''for i in xrange(len(collision_objects_start)):                        
                    if o.inCollisionContinuous([collision_objects_start[i], collision_objects_goal[i]]):
                        in_collision = True
                        break'''               
                if in_collision:
                    break
                    
            t = time.time() - t_c            
            collision_check_times1.append(t)          
                                
                                                                          
            x = np.array([result[i] for i in xrange(len(result))])
            cjvals = v_double()
            cjvels = v_double()
            cjvals_arr = [x[i] for i in xrange(len(x) / 2)]
            cjvels_arr = [x[i] for i in xrange(len(x) / 2, len(x))]
            cjvals[:] = cjvals_arr
            cjvels[:] = cjvels_arr
            particle_joint_values = v2_double()
            if show_viewer:
                self.robot.updateViewerValues(cjvals, 
                                              cjvels,
                                              particle_joint_values,
                                              particle_joint_values)
            time.sleep(0.03) 
            
            y += 1
            print y
            
    """
    ================================================================
    """
        
        
    def setup_scene(self,                    
                    environment_file,
                    robot):
        """ Load the obstacles """         
        self.obstacles = self.utils.loadObstaclesXML(self.abs_path + "/" + environment_file)      
        
        """ Load the goal area """
        goal_area = v_double()
        self.utils.loadGoalArea(self.abs_path + "/" + environment_file, goal_area)
        if len(goal_area) == 0:
            print "ERROR: Your environment file doesn't define a goal area"
            return False
        self.goal_position = [goal_area[i] for i in xrange(0, 3)]
        self.goal_radius = goal_area[3]
        return True
            
    def init_serializer(self):
        self.serializer = Serializer()
        self.serializer.create_temp_dir(self.abs_path, "lqg")        
            
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
        self.show_viewer_evaluation = config['show_viewer_evaluation']
        self.show_viewer_simulation = config['show_viewer_simulation']   
        self.path_deviation_cost = config['path_deviation_cost'] 
        self.control_deviation_cost = config['control_deviation_cost']
        self.num_control_samples = config['num_control_samples'] 
        self.min_control_duration = config['min_control_duration']
        self.max_control_duration = config['max_control_duration']   
        self.inc_covariance = config['inc_covariance'] 
        self.add_intermediate_states = config['add_intermediate_states']
        self.gravity_constant = config['gravity']
        self.num_generated_goal_states = config['num_generated_goal_states']
        self.robot_file = config['robot_file']        
        self.environment_file = config['environment_file']
        self.rrt_goal_bias = config['rrt_goal_bias']
        self.control_sampler = config['control_sampler']
        self.seed = config['seed']
        self.num_cores = config['num_cores']
        self.acceleration_limit = config['acceleration_limit']

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
    

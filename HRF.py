import sys
import argparse
import time
import numpy as np
import os
import glob
import scipy
import kalman as kalman
from plan_adjuster import PlanAdjuster
from serializer import Serializer
from libutil import *
import logging
from librobot import v_string, Robot, v_obstacle
from librobot import v_string, Robot
from util_py import check_positive_definite, get_goal_states, copyToTmp
from simulator import Simulator
from path_evaluator import PathEvaluator
from path_planning_interface import PathPlanningInterface
from libobstacle import Obstacle, Terrain
from gen_ik_solution import IKSolutionGenerator
import warnings
import subprocess

class HRF:
    def __init__(self):
        self.abs_path = os.path.dirname(os.path.abspath(__file__))
        cmd = "rm -rf " + self.abs_path + "/tmp"
        popen = subprocess.Popen(cmd, cwd=self.abs_path, shell=True)
        popen.wait()
        """ Reading the config """
        warnings.filterwarnings("ignore")
        self.init_serializer()
        config = self.serializer.read_config("config_hrf.yaml", path=self.abs_path)
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
        dir = self.abs_path + "/stats/hrf"
        tmp_dir = self.abs_path + "/tmp/hrf"
        self.clear_stats(dir)
        
        if self.init_modules() == False:
            return
        A, H, B, V, W, C, D, M_base, N_base = self.problem_setup(self.delta_t, self.robot_dof)
        self.plan_adjuster.set_model_matrices(A, B, V)        
        if check_positive_definite([C, D]):
            m_covs = np.linspace(self.min_process_covariance, 
                                 self.max_process_covariance, 
                                 self.covariance_steps)
            n_covs = np.linspace(self.min_observation_covariance, 
                                 self.max_observation_covariance,
                                 self.covariance_steps)
            
        for j in xrange(len(m_covs)):                       
            M = self.calc_covariance_value(self.robot, m_covs[j], M_base)               
                    
            """ The observation error covariance matrix """
            N = self.calc_covariance_value(self.robot, 
                                           n_covs[j], 
                                           N_base, 
                                           covariance_type='observation')          
            
            mean_number_planning_steps = 0.0
            number_of_steps = 0.0
            mean_planning_time = 0.0
            num_generated_paths_run = 0.0
            successful_runs = 0
            num_collisions = 0.0
            linearization_error = 0.0
            final_states= []
            rewards_cov = []
            for k in xrange(self.num_simulation_runs):
                logging.info("HRF: Generating goal states...")               
                goal_states = self.create_feasible_problem(self.num_obstacles)            
                self.setup()
                self.setup_after_init(A, B, C, D, H, M, N, V, W) 
                print "HRF: Run " + str(k + 1)                                
                self.serializer.write_line("log.log", tmp_dir, "RUN #" + str(k + 1) + " \n")
                current_step = 0
                x_true = np.array([self.start_state[m] for m in xrange(len(self.start_state))])
                x_estimated = np.array([self.start_state[m] for m in xrange(len(self.start_state))])                                              
                P_t = np.array([[0.0 for i in xrange(2 * self.robot_dof)] for i in xrange(2 * self.robot_dof)]) 
                P_ext_t = np.array([[0.0 for i in xrange(2 * self.robot_dof)] for i in xrange(2 * self.robot_dof)]) 
                deviation_covariance = np.array([[0.0 for i in xrange(2 * self.robot_dof)] for i in xrange(2 * self.robot_dof)])
                estimated_deviation_covariance = np.array([[0.0 for i in xrange(2 * self.robot_dof)] for i in xrange(2 * self.robot_dof)])              
                total_reward = 0.0
                terminal = False 
                last_x_true = None               
                
                """
                Obtain a nominal path
                """
                self.path_planner.set_start_and_goal(x_estimated, goal_states, self.goal_position, self.goal_radius)
                t0 = time.time()                
                (xs, 
                 us, 
                 zs,
                 control_durations, 
                 num_generated_paths, 
                 objective,
                 state_covariances,
                 deviation_covariances,
                 estimated_deviation_covariances, 
                 mean_gen_times, 
                 mean_eval_times,
                 total_gen_times,
                 total_eval_times) = self.path_planner.plan_and_evaluate_paths(1, 
                                                                               0, 
                                                                               current_step, 
                                                                               self.evaluation_horizon, 
                                                                               P_t,
                                                                               deviation_covariance,
                                                                               estimated_deviation_covariance, 
                                                                               0.0)
                while True: 
                    print "current step " + str(current_step) 
                    '''if current_step == 7:
                        x_true = np.array([last_x_true[k] for k in xrange(len(last_x_true))])
                        for k in xrange(len(last_x_true) / 2, len(last_x_true)):
                            last_x_true[k] = 0.0'''   
                    """
                    Predict system state at t+1 using nominal path
                    """
                    
                    """ Get state matrices """
                    #As, Bs, Vs, Ms, Hs, Ws, Ns = sim.get_linear_model_matrices(xs, us, control_durations)
                    As, Bs, Vs, Ms, Hs, Ws, Ns = self.sim.get_linear_model_matrices([x_estimated], [us[0]], control_durations)
                    
                    """ Predict using EKF """
                    (x_predicted_temp, P_predicted) = kalman.predict_state(self.robot,
                                                                           x_estimated,                                                                     
                                                                           us[0], 
                                                                           control_durations[0],
                                                                           self.simulation_step_size, 
                                                                           As[0],
                                                                           Bs[0],
                                                                           Vs[0],
                                                                           Ms[0],
                                                                           P_ext_t,
                                                                           self.dynamic_problem)                    
                    
                    """ Make sure x_predicted fulfills the constraints """                 
                    if self.enforce_constraints:     
                        x_predicted_temp = self.sim.check_constraints(x_predicted_temp)
                    predicted_collided = True              
                    if not self.sim.is_in_collision([], x_predicted_temp)[0]:                                                                                                    
                        x_predicted = x_predicted_temp
                        predicted_collided = False
                    else: 
                        print "X_PREDICTED COLLIDES!"
                        x_predicted = x_estimated         
                        for l in xrange(len(x_predicted) / 2, len(x_predicted)):                            
                            x_predicted[l] = 0 
                            
                    last_x_true = np.array([x_true[k] for k in xrange(len(x_true))])  
                    
                    """
                    Execute path for 1 time step
                    """                    
                    (x_true,                     
                     x_tilde,
                     x_tilde_linear, 
                     x_estimated_dash,
                     z, 
                     P_t, 
                     current_step, 
                     total_reward,
                     terminal,
                     estimated_s,
                     estimated_c,
                     history_entries) = self.sim.simulate_n_steps(xs, us, zs,
                                                                  control_durations,
                                                                  x_true,
                                                                  x_estimated,
                                                                  P_t,
                                                                  total_reward,                                                                 
                                                                  current_step,
                                                                  1,
                                                                  0.0,
                                                                  0.0,
                                                                  max_num_steps=self.max_num_steps)
                    print "set best reward " + str(len(history_entries))
                    history_entries[-1].set_best_reward(objective)
                                        
                     
                    """
                    Process history entries
                    """                    
                    try:
                        deviation_covariance = deviation_covariances[len(history_entries) - 1]
                        estimated_deviation_covariance = estimated_deviation_covariances[len(history_entries) - 1]
                    except:
                        print "what: len(deviation_covariances) " + str(len(deviation_covariances))
                        print "len(history_entries) " + str(len(history_entries))
                        print "len(xs) " + str(len(xs))
                    
                    history_entries[0].set_replanning(True)                                           
                    for l in xrange(len(history_entries)):
                        try:
                            history_entries[l].set_estimated_covariance(state_covariances[l])
                        except:
                            print "l " + str(l)
                            print "len(state_covariances) " + str(len(state_covariances))                                                   
                        
                        if history_entries[l].collided:                            
                            num_collisions += 1                            
                        linearization_error += history_entries[l].linearization_error
                    if (current_step == self.max_num_steps) or terminal:
                        history_entries[len(history_entries) - 2].set_best_reward(objective)
                        history_entries[-1].set_best_reward(None)
                        for l in xrange(len(history_entries)):                            
                            history_entries[l].serialize(tmp_dir, "log.log")
                        final_states.append(history_entries[-1].x_true)                        
                        if terminal:
                            print "Terminal state reached"
                            successful_runs += 1
                        break                    
                        
                    
                    
                    """
                    Plan new trajectories from predicted state
                    """
                    self.path_planner.set_start_and_goal(x_predicted, goal_states, self.goal_position, self.goal_radius) 
                    t0 = time.time()                   
                    paths = self.path_planner.plan_paths(self.num_paths, 0, planning_timeout=self.timeout, min_num_paths=0)
                    mean_planning_time += time.time() - t0
                    mean_number_planning_steps += 1.0
                    num_generated_paths_run += len(paths)                    
                    
                    """
                    Filter update
                    """
                    (x_estimated_temp, P_ext_t) = kalman.filter_update(x_predicted, P_predicted, z, Hs[0], Ws[0], Ns[0])
                    
                    
                    """ Make sure x_estimated fulfills the constraints """                 
                    if self.enforce_constraints:     
                        x_estimated_temp = self.sim.check_constraints(x_estimated_temp) 
                    in_collision, colliding_obstacle = self.sim.is_in_collision([], x_estimated_temp)
                    if in_collision:
                        history_entries[-1].set_estimate_collided(True)                        
                        for l in xrange(len(x_estimated) / 2, len(x_estimated)):                            
                            x_estimated[l] = 0
                    elif history_entries[-1].collided and self.knows_collision:
                        for l in xrange(len(x_estimated) / 2, len(x_estimated)):                            
                            x_estimated[l] = 0
                    else:
                        x_estimated = x_estimated_temp                         
                        history_entries[-1].set_estimate_collided(False)
                    
                    
                    """
                    Adjust plan
                    """ 
                    t0 = time.time()                   
                    (xs_adj, us_adj, zs_adj, control_durations_adj) = self.plan_adjuster.adjust_plan(self.robot,
                                                                                                    (xs, us, zs, control_durations),                                                                                                
                                                                                                     x_estimated,
                                                                                                     P_t)
                    if self.show_viewer_simulation:
                        self.sim.update_viewer(x_true, 
                                               x_estimated, 
                                               z, 
                                               control_duration=0.03, 
                                               colliding_obstacle=self.sim.colliding_obstacle)
                    
                    """
                    Evaluate the adjusted plan and the planned paths
                    """
                    #if self.is_terminal(xs_adj[-1]) and not len(xs_adj)==1:
                    if not len(xs_adj)<=1:
                        """
                        Only evaluate the adjusted path if it's > 1
                        """
                        paths.extend([[xs_adj, us_adj, zs_adj, control_durations_adj]])
                    if len(paths) == 0:
                        """
                        Running out of paths
                        """
                        print "Error: Couldn't generate an evaluate any paths"
                        final_states.append(history_entries[-1].x_true)
                        break
                    
                    (path_index,
                     xs, 
                     us, 
                     zs, 
                     control_durations, 
                     objective, 
                     state_covariances,
                     deviation_covariances,
                     estimated_deviation_covariances) = self.path_evaluator.evaluate_paths(paths, 
                                                                                           P_t, 
                                                                                           current_step)
                    
                    history_entries[-1].serialize(tmp_dir, "log.log")
                    mean_planning_time += time.time() - t0
                    if path_index == None:
                        """
                        We couldn't evaluate any paths
                        """
                        final_states.append(history_entries[-1].x_true)
                        break
                    
                    if self.show_viewer_simulation:
                        self.visualize_paths(self.robot, [xs])    
                                                 
                rewards_cov.append(total_reward)                
                self.serializer.write_line("log.log", 
                                           tmp_dir,
                                           "Reward: " + str(total_reward) + " \n") 
                self.serializer.write_line("log.log", 
                                           tmp_dir,
                                           "\n")
                number_of_steps += current_step
            mean_planning_time_per_step = mean_planning_time / mean_number_planning_steps 
            mean_planning_time /= self.num_simulation_runs
            mean_num_generated_paths_step = num_generated_paths_run / mean_number_planning_steps
                                
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
            logging.info("HRF: Done. total_reward is " + str(total_reward))
            try:
                n, min_max, mean_distance_to_goal, var, skew, kurt = scipy.stats.describe(np.array(ee_position_distances))
            except:
                print ee_position_distances                                         
                
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
            mean_linearization_error = linearization_error / number_of_steps
            number_of_steps /= self.num_simulation_runs
            self.serializer.write_line("log.log", tmp_dir, "Mean number of steps: " + str(number_of_steps) + " \n") 
            self.serializer.write_line("log.log", tmp_dir, "Mean number of generated paths per step: " + str(mean_num_generated_paths_step) + " \n")                            
            self.serializer.write_line("log.log", tmp_dir, "Mean num collisions per run: " + str(float(num_collisions) / float(self.num_simulation_runs)) + " \n")
            self.serializer.write_line("log.log", 
                                       tmp_dir, 
                                       "Average distance to goal area: " + str(mean_distance_to_goal) + " \n")
            self.serializer.write_line("log.log", tmp_dir, "Num successes: " + str(successful_runs) + " \n")
            print "succ " + str((100.0 / self.num_simulation_runs) * successful_runs)
            self.serializer.write_line("log.log", tmp_dir, "Percentage of successful runs: " + str((100.0 / self.num_simulation_runs) * successful_runs) + " \n")
            self.serializer.write_line("log.log", tmp_dir, "Mean planning time per run: " + str(mean_planning_time) + " \n")
            self.serializer.write_line("log.log", tmp_dir, "Mean planning time per planning step: " + str(mean_planning_time_per_step) + " \n")
            
            n, min_max, mean, var, skew, kurt = scipy.stats.describe(np.array(rewards_cov))
            print "mean_rewards " + str(mean)
            #plt.plot_histogram_from_data(rewards_cov)
            #sleep
            self.serializer.write_line("log.log", tmp_dir, "Mean rewards: " + str(mean) + " \n")
            self.serializer.write_line("log.log", tmp_dir, "Reward variance: " + str(var) + " \n")
            self.serializer.write_line("log.log", tmp_dir, "Mean linearisation error: " + str(mean_linearization_error) + " \n")
            self.serializer.write_line("log.log", 
                                       tmp_dir, 
                                       "Reward standard deviation: " + str(np.sqrt(var)) + " \n")
            self.serializer.write_line("log.log", tmp_dir, "Seed: " + str(self.seed) + " \n")
            cmd = "mv " + tmp_dir + "/log.log " + dir + "/log_hrf_" + str(m_covs[j]) + ".log"
            os.system(cmd)
        cmd = "cp " + self.abs_path + "/config_hrf.yaml " + dir           
        os.system(cmd)
        
        if not os.path.exists(dir + "/environment"):
            os.makedirs(dir + "/environment") 
            
        cmd = "cp " + self.abs_path + "/" + str(self.environment_file) + " " + str(dir) + "/environment"
        os.system(cmd) 
            
        if not os.path.exists(dir + "/model"):
            os.makedirs(dir + "/model")
                
        cmd = "cp " + self.abs_path + "/" + self.robot_file + " " + dir + "/model"
        os.system(cmd)
        print "Done."
        
    def setup_dynamic_problem(self):
        self.path_planner.setup_dynamic_problem(self.simulation_step_size,
                                                self.num_control_samples,
                                                self.min_control_duration,
                                                self.max_control_duration,
                                                self.add_intermediate_states,
                                                self.rrt_goal_bias,
                                                self.control_sampler)
        self.path_evaluator.setup_dynamic_problem()
        self.sim.setup_dynamic_problem(self.simulation_step_size)        
        
    def init_modules(self):
        self.utils = Utils()
        if not self.init_robot(self.robot_file):
            logging.error("HRF: Couldn't initialize robot")
            return False         
        if not self.setup_scene(self.environment_file, self.robot):
            return False
        
        self.sim = Simulator() 
        self.plan_adjuster = PlanAdjuster()       
        self.path_evaluator = PathEvaluator()
        self.path_planner = PathPlanningInterface()
        if self.show_viewer_simulation:
            self.robot.setupViewer(self.robot_file, self.environment_file)
        return True
    
    def setup_after_init(self,
                         A,
                         B, 
                         C,
                         D,
                         H,
                         M,
                         N,
                         V,
                         W):
        self.path_planner.setup_path_evaluator(A, B, C, D, H, M, N, V, W,                                     
                                               self.robot, 
                                               self.sample_size, 
                                               self.obstacles,
                                               self.goal_position,
                                               self.goal_radius,                                              
                                               self.robot_file,
                                               self.environment_file)
        self.sim.setup_problem(A, B, C, D, H, V, W, M, N,
                               self.robot, 
                               self.enforce_control_constraints,
                               self.obstacles, 
                               self.goal_position, 
                               self.goal_radius,
                               self.max_velocity,                                  
                               self.show_viewer_simulation,
                               self.robot_file,
                               self.environment_file,
                               self.knows_collision)
        self.path_evaluator.setup(A, B, C, D, H, M, N, V, W,                                     
                                  self.robot, 
                                  self.sample_size, 
                                  self.obstacles,
                                  self.goal_position,
                                  self.goal_radius,
                                  self.show_viewer_evaluation,
                                  self.robot_file,
                                  self.environment_file,
                                  self.num_cores)
            
        self.plan_adjuster.setup(self.robot,
                                 M, 
                                 H, 
                                 W, 
                                 N, 
                                 C, 
                                 D,
                                 self.dynamic_problem, 
                                 self.enforce_control_constraints) 
        if self.dynamic_problem:
            self.setup_dynamic_problem()
        
        
    def setup(self):
        obst = v_obstacle()
        obst[:] = self.obstacles
        self.robot.removeObstacles()
        self.robot.addObstacles(obst)
        self.sim.setup_reward_function(self.discount_factor, self.step_penalty, self.illegal_move_penalty, self.exit_reward)
        self.path_planner.setup(self.robot,                         
                                self.obstacles,  
                                self.max_velocity, 
                                self.delta_t, 
                                self.use_linear_path,
                                self.planning_algorithm,
                                self.path_timeout,
                                self.continuous_collision,
                                self.num_cores)
        print "dynamic problem " + str(self.dynamic_problem)        
        self.plan_adjuster.set_max_joint_velocities_linear_problem(np.array([self.max_velocity for i in xrange(self.robot_dof)]))
        self.path_evaluator.setup_reward_function(self.step_penalty, 
                                                  self.illegal_move_penalty, 
                                                  self.exit_reward, 
                                                  self.discount_factor)
        self.plan_adjuster.set_simulation_step_size(self.simulation_step_size)
        
        self.sim.set_stop_when_colliding(self.replan_when_colliding)
        self.path_planner.setup_reward_function(self.step_penalty, 
                                                self.exit_reward, 
                                                self.illegal_move_penalty, 
                                                self.discount_factor)
        
    def create_feasible_problem(self, num_obstacles):
        ik_solution_generator = IKSolutionGenerator()        
        while True:
            print "Creating random scene..." 
            self.create_random_obstacles(num_obstacles)
            ik_solution_generator.setup(self.robot,
                                        self.obstacles,
                                        self.max_velocity,
                                        self.delta_t,
                                        self.planning_algorithm,
                                        self.path_timeout,
                                        self.continuous_collision,
                                        self.num_cores)
            if self.dynamic_problem:
                ik_solution_generator.setup_dynamic_problem(self.simulation_step_size,
                                                            self.num_control_samples,
                                                            self.min_control_duration,
                                                            self.max_control_duration,
                                                            self.add_intermediate_states,
                                                            self.rrt_goal_bias,
                                                            self.control_sampler)
            goal_states = ik_solution_generator.generate(self.start_state, 
                                                         self.goal_position, 
                                                         self.goal_radius,
                                                         self.num_generated_goal_states)        
            if len(goal_states) != 0:
                return goal_states
         
            
                    
    def is_terminal(self, x):
        """
        Check if x is terminal
        """
        state = v_double()
        state[:] = [x[j] for j in xrange(len(x) / 2)]
        ee_position_arr = v_double()
        self.robot.getEndEffectorPosition(state, ee_position_arr)
        ee_position = np.array([ee_position_arr[j] for j in xrange(len(ee_position_arr))])
        norm = np.linalg.norm(ee_position - self.goal_position)               
        if norm - 0.01 <= self.goal_radius:                       
            return True
        return False
      
    def visualize_x(self, robot, x, color=None):
        cjvals = v_double()
        cjvels = v_double()
        cjvals[:] = [x[i] for i in xrange(len(x) / 2)]
        cjvels[:] = [x[i] for i in xrange(len(x) / 2, len(x))]
        particle_joint_values = v2_double()
        robot.updateViewerValues(cjvals, 
                                 cjvels, 
                                 particle_joint_values,
                                 particle_joint_values)              
                    
    def visualize_paths(self, robot, paths, colors=None):
        robot.removePermanentViewerParticles()        
        particle_joint_values = v2_double()
        particle_joint_colors = v2_double()
        pjvs = []
        for k in xrange(len(paths)):
            for p in paths[k]:
                particle = v_double()
                particle[:] = [p[i] for i in xrange(len(p) / 2)]
                pjvs.append(particle)
                if colors == None:
                    color = v_double()
                    color[:] = [0.0, 0.0, 0.0, 0.7]
                    particle_joint_colors.append(color)        
                else:
                    c = v_double()
                    c[:] = color
                    particle_joint_colors.append(c)
                
        particle_joint_values[:] = pjvs
        self.robot.addPermanentViewerParticles(particle_joint_values,
                                               particle_joint_colors)        
        
                    
        
    def init_serializer(self):        
        self.serializer = Serializer()
        self.serializer.create_temp_dir(self.abs_path, "hrf")
        
    def setup_viewer(self, robot):
        robot.setupViewer(model_file, env_file)
        
    def init_robot(self, urdf_model_file):
        self.robot = Robot(self.abs_path + "/" + urdf_model_file)
        self.robot.enforceConstraints(self.enforce_constraints)
        self.robot.setGravityConstant(self.gravity_constant)
        self.robot.setAccelerationLimit(self.acceleration_limit)
        """ Setup operations """
        self.robot_dof = self.robot.getDOF()        
        if len(self.start_state) != 2 * self.robot_dof:
            logging.error("HRF: Start state dimension doesn't fit to the robot state space dimension")
            return False
        return True 
    
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
    
    def create_random_obstacles(self, n):
        self.obstacles = []        
        for i in xrange(n): 
            name = "obst_" + str(i)           
            x_pos = np.random.uniform(-1.0, 4.0)
            y_pos = np.random.uniform(-1.0, 4.0)
            z_pos = np.random.uniform(3.0, 6.0)
        
            x_size = 0.25
            y_size = 0.25
            z_size = 0.25
            
            obst = self.utils.generateObstacle(name,
                                               x_pos,
                                               y_pos,
                                               z_pos,
                                               x_size,
                                               y_size,
                                               z_size)            
            self.obstacles.append(obst[0])
                
    
    def clear_stats(self, dir):
        if os.path.isdir(dir):
            cmd = "rm -rf " + dir + "/*"            
            os.system(cmd)
        else:
            os.makedirs(dir)
            
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
        self.planning_algorithm = config['planning_algorithm']
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
        self.max_num_steps = config['max_num_steps']
        self.evaluation_horizon = config['horizon']
        self.timeout = config['timeout']
        self.seed = config['seed']
        self.num_cores = config['num_cores']
        self.replan_when_colliding = config['replan_when_colliding']
        self.acceleration_limit = config['acceleration_limit']
        self.knows_collision = config['knows_collision']
        self.num_obstacles = config['num_obstacles']
        

if __name__ == "__main__":
    HRF()
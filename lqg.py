import numpy as np
import plot as Plot
import time
import copy
import os
import glob
from scipy.linalg import solve_discrete_are
from scipy.stats import norm, vonmises, multivariate_normal
from scipy import linalg, signal
from path_planner import PathPlanner
from kinematics import Kinematics
from multiprocessing import Process, Queue, cpu_count
from EMD import *
from serializer import Serializer
from obstacle import Obstacle
import collections

class LQG:
    def __init__(self):        
        serializer = Serializer()
        config = serializer.read_config("config.yaml") 
        obstacle_params = serializer.load_obstacles("obstacles.yaml", path="obstacles")
        self.obstacles = self.construct_obstacles(obstacle_params)                      
        self.set_params(config)        
        A = np.identity(self.num_links)
        H = np.identity(self.num_links)
        B = self.delta_t * np.identity(self.num_links)
        V = np.identity(self.num_links)
        W = np.identity(self.num_links)
        C = 2.0 * np.identity(self.num_links)
        D = 2.0 * np.identity(self.num_links)        
        if self.check_positive_definite([C, D]):            
            print "min " + str(self.min_covariance)
            print "max " + str(self.max_covariance)
            print "steps " + str(self.covariance_steps)
            m_covs = np.linspace(self.min_covariance, self.max_covariance, self.covariance_steps)
            emds = []   
                   
            if self.use_paths_from_file and len(glob.glob(os.path.join("stats", "paths.yaml"))) == 1:
                print "Loading paths from file"
                in_paths = serializer.load_paths("paths.yaml", path="stats") 
                paths = []               
                for path in in_paths:
                    xs = []
                    us = []
                    zs = []
                    for j in xrange(len(path)):
                        xs.append([path[j][i] for i in xrange(self.num_links)])
                        us.append([path[j][self.num_links + i] for i in xrange(self.num_links)])
                        zs.append([path[j][2 * self.num_links + i] for i in xrange(self.num_links)])
                    paths.append([xs, us, zs])
            else:
                paths = self.plan_paths(self.num_paths, 0)                   
                serializer.save_paths(paths, "paths.yaml", self.overwrite_paths_file, path="stats")
                #paths = serializer.load_paths("paths.yaml", path="stats")
            print "len paths " + str(len(paths))
            
            """ Determine average path length """
            avg_path_length = self.get_avg_path_length(paths)            
            serializer.save_avg_path_lengths(avg_path_length, path="stats")
                                       
            cart_coords = []  
            best_paths = []
            mean_rewards = []          
            for j in xrange(len(m_covs)):
                print "Evaluating covariance " + str(m_covs[j])
                """
                The process noise covariance matrix
                """
                M = m_covs[j] * np.identity(self.num_links)
                
                """
                The observation noise covariance matrix
                """
                N = self.observation_covariance * np.identity(self.num_links)
                
                '''if use_paths_from_file:
                    xs = paths[j][0]
                    us = paths[j][1]
                    zs = paths[j][2]
                else:'''                
                xs, us, zs = self.evaluate_paths(A, B, C, D, H, M, N, V, W, paths)
                                
                best_paths.append([[xs[i] for i in xrange(len(xs))], 
                                   [us[i] for i in xrange(len(us))],
                                   [zs[i] for i in xrange(len(zs))]])
                
                #self.save_path(xs, us, zs)                
                cartesian_coords, mean_reward = self.simulate(xs, us, zs, A, B, C, D, H, V, W, M, N, self.num_simulation_runs, j)                
                cart_coords.append([cartesian_coords[i] for i in xrange(len(cartesian_coords))])                
                emds.append(calc_EMD(cartesian_coords, self.num_bins))
                mean_rewards.append(mean_reward)            
            stats = dict(m_cov = m_covs.tolist(), emd = emds)
            serializer.save_paths(best_paths, 'best_paths.yaml', True, path="stats")
            serializer.save_cartesian_coords(cart_coords, path="stats")            
            serializer.save_stats(stats, path="stats")
            print mean_rewards
            serializer.save_rewards(mean_rewards, path="stats")
            cmd = "cp config.yaml stats/"            
            os.system(cmd)
            cmd = "cp obstacles/obstacles.yaml stats/"
            os.system(cmd)
            
    def get_avg_path_length(self, paths):
        avg_length = 0.0
        for path in paths:            
            avg_length += len(path[0])
        return avg_length / len(paths)            
            
    def construct_obstacles(self, obstacle_params):
        obstacle_list = []
        if not obstacle_params == None:
            for o in obstacle_params:
                obstacle_list.append(Obstacle(o[0], o[1], o[2], o[3]))
        return obstacle_list        
        
    def check_positive_definite(self, matrices):
        for m in matrices:
            try:
                np.linalg.cholesky(m)
            except:
                print "Matrices are not positive definite. Fix that!"
                return False
        return True
        
    def set_params(self, config):
        self.num_paths = config['num_generated_paths']
        self.use_linear_path = config['use_linear_path']
        self.num_cores = cpu_count()
        #self.num_cores = 2
        self.num_links = config['num_links']
        self.max_velocity = config['max_velocity']
        self.delta_t = 1.0 / config['control_rate']
        self.theta_0 = config['init_joint_angles']
        self.goal_position = config['goal_position']
        self.goal_radius = config['goal_radius']
        self.num_simulation_runs = config['num_simulation_runs']
        self.num_bins = config['num_bins']
        self.min_covariance = config['min_covariance']
        self.max_covariance = config['max_covariance']
        self.covariance_steps = config['covariance_steps']
        self.observation_covariance = config['observation_covariance']        
        self.use_paths_from_file = config['use_paths_from_file']        
        self.overwrite_paths_file = config['overwrite_paths_file']
        self.discount_factor = config['discount_factor']
        self.illegal_move_penalty = config['illegal_move_penalty']
        self.step_penalty = config['step_penalty']
        self.exit_reward = config['exit_reward']
        self.stop_when_terminal = config['stop_when_terminal']
        self.kinematics = Kinematics(self.num_links)    
    
    def get_jacobian(self, links, state):
        s0 = np.sin(state[0])
        c0 = np.cos(state[0])
        s1 = np.sin(state[0] + state[1])
        c1 = np.cos(state[0] + state[1])
        
        if len(state) == 2:
            return np.array([[-links[0] * np.sin(state[0]) - links[1] * np.sin(state[0] + state[1]), -links[1] * np.sin(state[0] + state[1])],
                             [links[0] * np.cos(state[0]) + links[1] * np.cos(state[0] + state[1]), links[1] * np.cos(state[0] + state[1])]])
        elif len(state) == 3:
            s2 = np.sin(state[0] + state[1] + state[2])
            c2 = np.cos(state[0] + state[1] + state[2])
            return np.array([[-links[0] * s0 - links[1] * s1 -links[2] * s2, -links[1] * s1 - links[2] * s2, -links[2] * s2],
                             [links[0] * c0 + links[1] * c1 + links[2] * c2, links[1] * c1 + links[2] * c2, links[2] * c2],
                             [1.0, 1.0, 1.0]])
        return None
        
    def plan_paths(self, num, sim_run):        
        jobs = collections.deque()
        ir = collections.deque()
        path_queue = Queue()
        paths = []        
        for i in xrange(num):
            print "path num " + str(i)
            p = Process(target=self.construct_path, args=(path_queue, sim_run,))
            p.start()
            jobs.append(p)
            ir.append(1)
            if len(ir) >= self.num_cores - 1 or i == num - 1:
                '''for job in jobs:                    
                    if job.is_alive():
                        print "is alive " + str(job.is_alive())
                        print "joining"                                            
                        job.join()
                        print "JOINED"'''
                if i == num - 1:
                    if num == self.num_cores - 1:
                        while not path_queue.qsize() == num:
                            time.sleep(0.0001)
                    else:
                        while not path_queue.qsize() == num % (self.num_cores - 1):
                            time.sleep(0.0001)
                else:
                    while not path_queue.qsize() == self.num_cores - 1:
                        time.sleep(0.0001)                                        
                jobs.clear()
                ir.clear()
                q_size = path_queue.qsize()                
                for j in xrange(q_size):
                    p = path_queue.get()                    
                    paths.append([[p[0][i].tolist() for i in xrange(len(p[0]))], 
                                  [p[1][i].tolist() for i in xrange(len(p[0]))], 
                                  [p[2][i].tolist() for i in xrange(len(p[0]))]])        
        return paths
        
    def construct_path(self, queue, sim_run):
        path_planner = PathPlanner()
        path_planner.set_params(self.num_links, 
                                self.max_velocity, 
                                self.delta_t, 
                                self.use_linear_path,
                                sim_run)
        path_planner.setup_ompl()
        path_planner.set_start_state(self.theta_0)
        path_planner.set_goal_region(self.goal_position, self.goal_radius) 
        path_planner.set_obstacles(self.obstacles)             
        xs, us, zs = path_planner.plan_path()
        print "putting in queue"
        queue.put((xs, us, zs))
        return
    
    def evaluate_paths(self, A, B, C, D, H, M, N, V, W, paths):        
        min_trace = 1000.0
        min_collision_sum = 10000.0
        best_paths = []       
        for l in xrange(len(paths)): 
            xs = paths[l][0]
            us = paths[l][1]
            zs = paths[l][2]
            Ls = self.compute_gain(A, B, C, D, len(xs) - 1)
            P_t = np.array([[0.0 for i in xrange(self.num_links)] for i in xrange(self.num_links)])
            P_0 = np.copy(P_t)
            NU = np.copy(P_t)
            
            Q_t = np.vstack((np.hstack((M, NU)), 
                             np.hstack((NU, N))))
            R_t = np.vstack((np.hstack((P_0, NU)),
                             np.hstack((NU, NU))))
            ee_distributions = []
            ee_approx_distr = []
            collision_probs = []
            for i in xrange(0, len(xs) - 1):
                P_hat_t = self.compute_p_hat_t(A, P_t, V, M)
                
                
                K_t = self.compute_kalman_gain(H, P_hat_t, W, N)            
                
                P_t = self.compute_P_t(K_t, H, P_hat_t) 
                
                F_0 = np.hstack((A, np.dot(B, Ls[i])))
                F_1 = np.hstack((np.dot(np.dot(K_t, H), A), 
                                 np.add(A, np.subtract(np.dot(B, Ls[i]), np.dot(np.dot(K_t, H), A)))))            
                F_t = np.vstack((F_0, F_1))
                
                G_t = np.vstack((np.hstack((V, NU)), 
                                 np.hstack((np.dot(np.dot(K_t, H), V), np.dot(K_t, W)))))
                
                """ Compute R """            
                FRF = np.dot(np.dot(F_t, R_t), np.transpose(F_t))
                GQG = np.dot(np.dot(G_t, Q_t), np.transpose(G_t))
                R_t = np.add(np.dot(np.dot(F_t, R_t), np.transpose(F_t)), np.dot(G_t, np.dot(Q_t, np.transpose(G_t))))
                
                #print "R_t " + str(R_t)
                Gamma_t = np.vstack((np.hstack((np.identity(self.num_links), NU)), 
                                     np.hstack((NU, Ls[i]))))
                #print "Gamma_t " + str(Gamma_t)
                
                Cov = np.dot(np.dot(Gamma_t, R_t), np.transpose(Gamma_t))
                cov_state = np.array([[Cov[j, k] for k in xrange(self.num_links)] for j in xrange(self.num_links)])
                j = self.get_jacobian([1.0 for k in xrange(self.num_links)], xs[i])
                EE_covariance = np.dot(np.dot(j, cov_state), np.transpose(j))
                EE_covariance = np.array([[EE_covariance[j, k] for k in xrange(2)] for j in xrange(2)])
                probs = 0.0 
                if len(self.obstacles) > 0:
                    #print "get collision probs"
                    p = self.kinematics.get_end_effector_position(xs[i])
                    probs = self.get_probability_of_collision(p, EE_covariance)
                    collision_probs.append(probs)                    
                #prob_collision = self.get_probability_of_collision               
            if len(collision_probs) > 0:
                collision_sum = sum(collision_probs) / len(collision_probs)
                print "collision_sum " + str(collision_sum)
                if collision_sum == 0.0:
                    best_paths.append([(paths[l][0], paths[l][1], paths[l][2]), EE_covariance])
                elif collision_sum < min_collision_sum:
                    min_collision_sum = collision_sum
                    best_paths.append([(paths[l][0], paths[l][1], paths[l][2]), EE_covariance])
                    #best_path = (paths[l][0], paths[l][1], paths[l][2])
            else:
                best_paths.append([(paths[l][0], paths[l][1], paths[l][2]), EE_covariance])
        
        for path in best_paths:
            #print "paths[1] " + str(path[1])
            tr = np.trace(path[1])
            #print "trace " + str()
            if tr < min_trace:
                min_trace = tr
                best_path = path[0]                   
        return best_path
        
    def get_probability_of_collision(self, p, cov):        
        samples = multivariate_normal.rvs(p, cov, 10)        
        ps = multivariate_normal.pdf(samples, p, cov, allow_singular=True)
        ps /= sum(ps)
        prob = 0.0
        for i in xrange(len(samples)):
            for o in self.obstacles:
                if o.collides(samples[i]):
                    prob += ps[i]
        return prob    
        
    def simulate(self, 
                 xs, 
                 us, 
                 zs,
                 A,
                 B,
                 C,
                 D,
                 H,
                 V,
                 W,
                 M,
                 N,
                 num_simulation_runs,
                 run):
        Ls = self.compute_gain(A, B, C, D, len(xs) - 1)
        cart_coords = []
        mean_reward = 0.0
        for j in xrange(num_simulation_runs):
            print "simulation run " + str(j) + " for run " + str(run) 
            x_true = xs[0]
            x_tilde = xs[0]        
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
                    x_true = self.apply_control(x_true, np.add(u_dash, us[i]), A, B, V, M)
                        
                    ee_position = self.kinematics.get_end_effector_position(x_true)                
                    discount = np.power(self.discount_factor, i)               
                      
                    if self.is_terminal(ee_position):
                        terminal_state_reached = True                        
                        reward += discount * self.exit_reward                        
                        print "Terminal state reached: reward = " + str(reward)                        
                    else:                        
                        if self.check_collision(ee_position):                            
                            reward += discount * (-1.0 * self.illegal_move_penalty)
                            #reward -= np.array([self.illegal_move_penalty])
                        else:
                            reward += discount * (-1.0 * self.step_penalty)
                            #reward -= np.array([self.step_penalty])                            
                                        
                    """
                    Obtain an observation
                    """
                    z_t = self.get_observation(x_true, H, N, W)
                    z_dash_t = z_t - zs[i]
                            
                    """
                    Kalman prediction and update
                    """
                    x_tilde_dash_t, P_dash = self.kalman_predict(x_tilde, u_dash, A, B, P_t, V, M)
                    x_tilde, P_t = self.kalman_update(x_tilde_dash_t, z_dash_t, H, P_dash, W, N)                   
            ee_position = self.kinematics.get_end_effector_position(x_true)
            cart_coords.append(ee_position.tolist())
            mean_reward += reward            
        mean_reward /= num_simulation_runs                        
        return cart_coords, np.asscalar(mean_reward)
    
    def is_terminal(self, ee_position):        
        if np.linalg.norm(ee_position - self.goal_position) < self.goal_radius:            
            return True
        return False        
    
    def check_collision(self, ee_position):
        """
        Is the given end effector position in collision with an obstacle?
        """              
        for obstacle in self.obstacles:
            if obstacle.collides(ee_position):
                return True
        return False            
    
    def save_path(self, xs, us, zs):
        print "Saving path"
        for file in glob.glob("path.txt"):
            os.remove(file)        
        with open("path.txt", "a+") as f:
            for i in xrange(len(xs)):
                x = ""
                u = ""
                z = ""
                for j in xrange(self.num_links):                    
                    x += str(xs[i][j]) + " "                    
                    u += str(us[i][j]) + " "                    
                    z += str(zs[i][j]) + " "
                string = x + u + z + " \n"
                f.write(string)
        
    def load_path(self, file):
        print "loading path"
        xs = []
        us = []
        zs = [] 
        with open(file, 'r+') as f:
            for line in f.readlines():
                line = line.split(" ")
                x = [float(line[i]) for i in xrange(self.num_links)]
                u = [float(line[self.num_links + i]) for i in xrange(self.num_links)]
                z = [float(line[2 * self.num_links + i]) for i in xrange(self.num_links)]                
                xs.append(x)
                us.append(u)
                zs.append(z)
        return xs, us, zs
        
    def apply_control(self, x_dash, u_dash, A, B, V, M):        
        m = self.get_random_joint_angles([0.0 for i in xrange(self.num_links)], M)        
        x_new = np.add(np.add(np.dot(A, x_dash), np.dot(B, u_dash)), np.dot(V, m))        
        return x_new
    
    def get_observation(self, true_theta, H, N, W):
        return np.add(np.dot(H, true_theta), np.dot(W, self.get_random_joint_angles([0.0 for i in xrange(self.num_links)], N)))
        
        
    def get_random_joint_angles(self, mu, cov):        
        #with self.lock:
        """
        Random numbers need to be generated by using a lock and creating a new random seed in order to avoid
        correlations between different processes
        """
        #np.random.seed()
        return np.random.multivariate_normal(mu, cov)
    
    def kalman_predict(self, x_tilde, u, A, B, P_t, V, M):
        x_tilde_dash_t = np.add(np.dot(A, x_tilde), np.dot(B, u))               
        P_hat_t = self.compute_p_hat_t(A, P_t, V, M)
        return x_tilde_dash_t, P_hat_t
    
    def compute_p_hat_t(self, A, P_t, V, M):
        return np.add(np.dot(np.dot(A, P_t), np.transpose(A)), np.dot(np.dot(V, M), np.transpose(V)))        
    
    def kalman_update(self, x_tilde_dash_t, z_dash_t, H, P_dash_t, W, N):
        K_t = self.compute_kalman_gain(H, P_dash_t, W, N)
        x_tilde_t = self.compute_state_estimate(x_tilde_dash_t, z_dash_t, H, K_t)
        P_t = self.compute_P_t(K_t, H, P_dash_t)
        return x_tilde_t, P_t
    
    def compute_kalman_gain(self, H, P_dash_t, W, N):
        return np.dot(np.dot(P_dash_t, np.transpose(H)), linalg.inv(np.add(np.dot(np.dot(H, P_dash_t), np.transpose(H)), np.dot(np.dot(W, N), np.transpose(W)))))
        
    
    def compute_state_estimate(self, x_tilde_dash_t, z_dash_t, H, K_t):        
        return np.add(x_tilde_dash_t, np.dot(K_t, np.subtract(z_dash_t, np.dot(H, x_tilde_dash_t))))
    
    def compute_P_t(self, K_t, H, P_hat_t):
        return np.dot(np.subtract(np.identity(self.num_links), np.dot(K_t, H)), P_hat_t)        
    
    def compute_gain(self, A, B, C, D, l):
        S = np.copy(C)
        Ls = []        
        for i in xrange(l):            
            L = -np.dot(np.dot(np.dot(linalg.inv(np.add(np.dot(np.dot(np.transpose(B), S), B), D)), np.transpose(B)), S), A)
            Ls.append(L)
            S = np.add(C, np.add(np.dot(np.dot(np.transpose(A), S), A), np.dot(np.dot(np.dot(np.transpose(A), S), B), L)))
        Ls = Ls[::-1]
        return Ls
        
                      
        S = linalg.solve_discrete_are(A, B, C, D)
        L = -np.dot(np.dot(np.dot(linalg.inv(np.add(np.dot(np.dot(np.transpose(B), S), B), D)), np.transpose(B)), S), A)
        #L = -1.0 * np.dot(np.dot(np.dot(np.linalg.inv(np.dot(np.dot(np.transpose(B), S), B) + D), np.transpose(B)), S), A)        
        return L
    
    def _enforce_limits(self, state):
        """
        Not used at the moment
        """
        for i in xrange(len(state)):
            if state[i] < -np.pi:
                state[i] = -np.pi
            elif state[i] > np.pi:
                state[i] = np.pi 
        return state
            
        

if __name__ == "__main__":
    LQG()
    
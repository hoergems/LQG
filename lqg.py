import numpy as np
import plot as Plot
import time
import copy
import os
import glob
from scipy.linalg import solve_discrete_are
from scipy.stats import norm, vonmises
from scipy import linalg, signal
from path_planner import PathPlanner
from kinematics import Kinematics
from multiprocessing import Process, Queue, cpu_count
from EMD import *
from serializer import Serializer
import collections

class LQG:
    def __init__(self):        
        serializer = Serializer()
        config = serializer.read_config("config.yaml")        
        self.set_params(config)        
        A = np.identity(self.num_links)
        H = np.identity(self.num_links)
        B = self.delta_t * np.identity(self.num_links)
        V = np.identity(self.num_links)
        W = np.identity(self.num_links)
        C = 2.0 * np.identity(self.num_links)
        D = 2.0 * np.identity(self.num_links)        
        if self.check_positive_definite([C, D]):
            n_cov = 0.001
            print "min " + str(self.min_covariance)
            print "max " + str(self.max_covariance)
            print "steps " + str(self.covariance_steps)
            m_covs = np.linspace(self.min_covariance, self.max_covariance, self.covariance_steps)
            emds = []
            
            paths = []
            use_paths_from_file = False            
            if len(glob.glob("paths.yaml")) == 1:
                print "Loading paths from file"
                paths = serializer.load_paths('paths.yaml')
                use_paths_from_file = True            
            print "len paths " + str(len(paths))
            cart_coords = []            
            for j in xrange(len(m_covs)):
                print "Evaluating covariance " + str(m_covs[j])
                """
                The process noise covariance matrix
                """
                M = m_covs[j] * np.identity(self.num_links)
                
                """
                The observation noise covariance matrix
                """
                N = n_cov * np.identity(self.num_links)
                
                if use_paths_from_file:
                    xs = paths[j][0]
                    us = paths[j][1]
                    zs = paths[j][2]
                else:
                    xs, us, zs = self.plan_paths(A, B, C, D, H, M, N, V, W, self.goal_position, self.num_paths, j)
                    paths.append([[xs[i].tolist() for i in xrange(len(xs))], 
                                  [us[i].tolist() for i in xrange(len(us))],
                                  [zs[i].tolist() for i in xrange(len(zs))]])
                #self.save_path(xs, us, zs)                
                cartesian_coords = self.simulate(xs, us, zs, A, B, C, D, H, V, W, M, N, self.num_simulation_runs, j)                
                cart_coords.append([cartesian_coords[i] for i in xrange(len(cartesian_coords))])                
                emds.append(calc_EMD(cartesian_coords, self.num_bins))            
            stats = dict(m_cov = m_covs.tolist(), emd = emds)
            serializer.save_paths(paths)
            serializer.save_cartesian_coords(cart_coords)            
            serializer.save_stats(stats)
        
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
        self.num_cores = cpu_count()
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
        
    def plan_paths(self, A, B, C, D, H, M, N, V, W, goal_position, num, sim_run):        
        jobs = collections.deque()
        ir = collections.deque()
        path_queue = Queue()
        paths = []        
        for i in xrange(num):
            print "path num " + str(i)
            p = Process(target=self.evaluate_path, args=(A, B, C, D, H, M, N, V, W, path_queue, sim_run,))
            p.start()
            jobs.append(p)
            ir.append(1)
            if len(ir) >= self.num_cores - 1 or i == num - 1:
                for job in jobs:                         
                    job.join()                                        
                jobs.clear()
                ir.clear()
                q_size = path_queue.qsize()
                for j in xrange(q_size):
                    paths.append(path_queue.get())                    
                
        tr = 1000.0
        best_index = 0
        for i in xrange(len(paths)):            
            if paths[i][0] < tr:
                tr = paths[i][0]
                best_index = copy.copy(i)
        print "best path: " + str(best_index)
        print "best trace: " + str(tr)              
        return paths[best_index][1][0], paths[best_index][1][1], paths[best_index][1][2]
        
    
    def evaluate_path(self, A, B, C, D, H, M, N, V, W, queue, sim_run):
        path_planner = PathPlanner()
        path_planner.set_params(self.num_links, self.max_velocity, self.delta_t, sim_run)
        path_planner.setup_ompl()
        path_planner.set_start_state(self.theta_0)
        path_planner.set_goal_region(self.goal_position, self.goal_radius)              
        xs, us, zs = path_planner.plan_path()
        
        Ls = self.compute_gain(A, B, C, D, len(xs))
        P_t = np.array([[0.0 for i in xrange(self.num_links)] for i in xrange(self.num_links)])
        P_0 = np.copy(P_t)
        NU = np.copy(P_t)
        
        Q_t = np.vstack((np.hstack((M, NU)), 
                         np.hstack((NU, N))))
        R_t = np.vstack((np.hstack((P_0, NU)),
                         np.hstack((NU, NU))))
        ee_distributions = []
        ee_approx_distr = []
        for i in xrange(1, len(xs)):
            P_hat_t = self.compute_p_hat_t(A, P_t, V, M)
            
            
            K_t = self.compute_kalman_gain(H, P_hat_t, W, N)            
            
            P_t = self.compute_P_t(K_t, H, P_hat_t) 
            
            F_0 = np.hstack((A, np.dot(B, Ls[i - 1])))
            F_1 = np.hstack((np.dot(np.dot(K_t, H), A), 
                             np.add(A, np.subtract(np.dot(B, Ls[i-1]), np.dot(np.dot(K_t, H), A)))))            
            F_t = np.vstack((F_0, F_1))
            
            G_t = np.vstack((np.hstack((V, NU)), 
                             np.hstack((np.dot(np.dot(K_t, H), V), np.dot(K_t, W)))))
            
            """ Compute R """            
            FRF = np.dot(np.dot(F_t, R_t), np.transpose(F_t))
            GQG = np.dot(np.dot(G_t, Q_t), np.transpose(G_t))
            R_t = np.add(np.dot(np.dot(F_t, R_t), np.transpose(F_t)), np.dot(G_t, np.dot(Q_t, np.transpose(G_t))))            
            
            
            #print "R_t " + str(R_t)
            Gamma_t = np.vstack((np.hstack((np.identity(self.num_links), NU)), 
                                 np.hstack((NU, Ls[i - 1]))))
            #print "Gamma_t " + str(Gamma_t)
            Cov = np.dot(np.dot(Gamma_t, R_t), np.transpose(Gamma_t))
            cov_state = np.array([[Cov[j, k] for k in xrange(self.num_links)] for j in xrange(self.num_links)])            
            j = self.get_jacobian([1.0 for k in xrange(self.num_links)], xs[i])
            
            EE_covariance = np.dot(np.dot(j, cov_state), np.transpose(j))
        print "cov state " + str(cov_state)
        print "cov end effector " + str(EE_covariance)    
        queue.put([np.trace(EE_covariance), (xs, us, zs)])
        
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
        for j in xrange(num_simulation_runs):
            print "simulation run " + str(j) + " for run " + str(run) 
            x_true = xs[0]
            x_tilde = xs[0]        
            u_dash = np.array([0.0 for j in xrange(self.num_links)])        
            P_t = np.array([[0.0 for k in xrange(self.num_links)] for l in xrange(self.num_links)])          
            for i in xrange(0, len(xs) - 1):                
                """
                Generate u_dash using LQG
                """                
                u_dash = np.dot(Ls[i], x_tilde)
                    
                """
                Generate a true state
                """            
                x_true = self.apply_control(x_true, u_dash + us[i], A, B, V, M)                     
                    
                """
                Obtain an observation
                """
                z_t = self.get_observation(x_true, H, N)
                z_dash_t = z_t - zs[i]
                    
                """
                Kalman prediction and update
                """
                x_tilde_dash_t, P_dash = self.kalman_predict(x_tilde, u_dash, A, B, P_t, V, M)
                x_tilde, P_t = self.kalman_update(x_tilde_dash_t, z_dash_t, H, P_dash, W, N)
                        
            ee_position = self.kinematics.get_end_effector_position(x_true)
            cart_coords.append(ee_position.tolist())            
        return cart_coords
    
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
        return np.add(np.add(np.dot(A, x_dash), np.dot(B, u_dash)), np.dot(V, m))
    
    def get_observation(self, true_theta, H, N):
        return np.dot(H, true_theta) + self.get_random_joint_angles([0.0 for i in xrange(self.num_links)], N)        
        
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
    
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
from EMD import EMD, histEMD
from serializer import Serializer
import collections

class LQG:
    def __init__(self):
        serializer = Serializer("config.yaml")
        config = serializer.read_config()        
        self.set_params(config)        
        self.A = np.identity(self.num_links)
        self.H = np.identity(self.num_links)
        self.B = self.delta_t * np.identity(self.num_links)
        self.V = np.identity(self.num_links)
        self.W = np.identity(self.num_links)
        self.C = 2.0 * np.identity(self.num_links)
        self.D = 2.0 * np.identity(self.num_links)
        if self.check_positive_definite([self.C, self.D]):
            n_cov = 0.001
            m_covs = np.linspace(0.00001, 0.01, 100)
            emds = []
            paths = []
            for j in xrange(len(m_covs)):
                
                """
                The process noise covariance matrix
                """
                self.M = m_covs[j] * np.identity(self.num_links)
                
                """
                The observation noise covariance matrix
                """
                self.N = n_cov * np.identity(self.num_links)
                   
                '''if len(glob.glob("path.txt")) == 1:
                    xs, us, zs = self.load_path("path.txt")
                else:'''
                xs, us, zs = self.plan_paths(self.goal_position, self.num_paths, j)
                paths.append([[xs[i].tolist() for i in xrange(len(xs))], 
                              [us[i].tolist() for i in xrange(len(us))],
                              [zs[i].tolist() for i in xrange(len(zs))]])
                #self.save_path(xs, us, zs)
                
                emds.append(self.simulate(xs, us, zs, self.num_simulation_runs))
            stats = dict(m_cov = m_covs.tolist(), emd = emds)
            serializer.save_paths(paths)            
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
        self.num_cores = cpu_count() - 1
        self.num_links = config['num_links']
        self.max_velocity = config['max_velocity']
        self.delta_t = 1.0 / config['control_rate']
        self.theta_0 = config['init_joint_angles']
        self.goal_position = config['goal_position']
        self.goal_radius = config['goal_radius']
        self.num_simulation_runs = config['num_simulation_runs']
        self.num_bins = config['num_bins']
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
        
    def plan_paths(self, goal_position, num, sim_run):
        jobs = collections.deque()
        ir = collections.deque()
        path_queue = Queue()
        paths = []        
        for i in xrange(num):
            print "path num " + str(i)
            p = Process(target=self.evaluate_path, args=(path_queue, sim_run,))
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
        
    
    def evaluate_path(self, queue, sim_run): 
        path_planner = PathPlanner()
        path_planner.set_params(self.num_links, self.max_velocity, self.delta_t, sim_run)
        path_planner.setup_ompl()
        path_planner.set_start_state(self.theta_0)
        path_planner.set_goal_region(self.goal_position, self.goal_radius)              
        xs, us, zs = path_planner.plan_path()
        
        Ls = self.compute_gain(self.A, self.B, self.C, self.D, len(xs))
        P_t = np.array([[0.0 for i in xrange(self.num_links)] for i in xrange(self.num_links)])
        P_0 = np.copy(P_t)
        NU = np.copy(P_t)
        
        Q_t = np.vstack((np.hstack((self.M, NU)), 
                         np.hstack((NU, self.N))))
        R_t = np.vstack((np.hstack((P_0, NU)),
                         np.hstack((NU, NU))))
        ee_distributions = []
        ee_approx_distr = []
        for i in xrange(1, len(xs)):
            P_hat_t = self.compute_p_hat_t(self.A, P_t, self.V, self.M)
            
            
            K_t = self.compute_kalman_gain(self.H, P_hat_t, self.W, self.N)            
            
            P_t = self.compute_P_t(K_t, self.H, P_hat_t) 
            
            F_0 = np.hstack((self.A, np.dot(self.B, Ls[i - 1])))
            F_1 = np.hstack((np.dot(np.dot(K_t, self.H), self.A), 
                             np.add(self.A, np.subtract(np.dot(self.B, Ls[i-1]), np.dot(np.dot(K_t, self.H), self.A)))))            
            F_t = np.vstack((F_0, F_1))
            
            G_t = np.vstack((np.hstack((self.V, NU)), 
                             np.hstack((np.dot(np.dot(K_t, self.H), self.V), np.dot(K_t, self.W)))))
            
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
        
    def simulate(self, xs, us, zs, num_simulations):
        cartesian_coords = []          
        x_trues = []     
        for k in xrange(num_simulations):
            print "simulation run " + str(k)            
            x_true = xs[0]
            x_tilde = xs[0]        
            u_dash = np.array([0.0 for j in xrange(self.num_links)])        
            P_t = np.array([[0.0 for k in xrange(self.num_links)] for l in xrange(self.num_links)])
            Ls = self.compute_gain(self.A, self.B, self.C, self.D, len(xs) - 1)            
            for i in xrange(0, len(xs) - 1):                
                """
                Generate u_dash using LQG
                """                
                u_dash = np.dot(Ls[i], x_tilde)
                
                """
                Generate a true state
                """
                x_true = self.apply_control(x_true, u_dash + us[i], self.A, self.B, self.V, self.M)                
                
                
                """
                Obtain an observation
                """
                z_t = self.get_observation(x_true, self.H, self.N)
                z_dash_t = z_t - zs[i]
                
                """
                Kalman prediction and update
                """
                x_tilde_dash_t, P_dash = self.kalman_predict(x_tilde, u_dash, self.A, self.B, P_t, self.V, self.M)
                x_tilde, P_t = self.kalman_update(x_tilde_dash_t, z_dash_t, self.H, P_dash, self.W, self.N)                      
            x_trues.append(x_true)            
            ee_position = self.kinematics.get_end_effector_position(x_true)            
            cartesian_coords.append(np.array([ee_position[l] for l in xrange(len(ee_position))]))
        X = np.array([coords[0] for coords in cartesian_coords])
        Y = np.array([coords[1] for coords in cartesian_coords])
        histogram_range = [[-3.1, 3.1], [-3.1, 3.1]]
        
        """
        The historgram from the resulting cartesian coordinates
        """ 
        print "Calculating histograms..."      
        H, xedges, yedges = self.get_2d_histogram(X, 
                                                  Y, 
                                                  histogram_range,
                                                  bins=self.num_bins)
        """
        The histogram from a delta distribution located at the goal position
        """
        H_delta, xedges_delta, yedges_delta = self.get_2d_histogram([0.0], 
                                                                    [-3.0], 
                                                                    histogram_range, 
                                                                    bins=self.num_bins)
        
        print "Calculating EMD..."
        emd = self.compute_earth_mover(H, H_delta)
        print "EMD is " + str(emd)
        #Plot.plot_histogram(H, xedges, yedges)
        return emd    
        
    def compute_earth_mover(self, H1, H2):
        """
        Computes the earth mover's distance between two equally sized histograms
        """        
        feature1 = []
        feature2 = []        
        w1 = []
        w2 = []
        for i in xrange(len(H1)):
            for j in xrange(len(H1[i])):
                if H1[i, j] != 0.0:
                    feature1.append([i, j])
                    w1.append(H1[i, j]) 
        for i in xrange(len(H2)):
            for j in xrange(len(H2[i])):
                if H2[i, j] != 0.0:
                    feature2.append([i, j])
                    w2.append(H2[i, j])
        return histEMD(np.array(feature1),
                       np.array(feature2),
                       np.array([[w1[i]] for i in xrange(len(w1))]),
                       np.array([[w2[i]] for i in xrange(len(w2))]))
    
    def get_2d_histogram(self, X, Y, range, bins=200):
        H, xedges, yedges = np.histogram2d(X, Y, bins=bins, range=range, normed=True)
        H = np.rot90(H)
        H = np.flipud(H)
        sum = 0.0
        for i in xrange(len(H)):
            for j in xrange(len(H[i])):
                sum += H[i, j]        
        for i in xrange(len(H)):
            for j in xrange(len(H[i])):
                H[i, j] /= sum        
        return H, xedges, yedges
    
    def save_path(self, xs, us, zs):
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
        return np.random.multivariate_normal(mu, cov)
        for i in xrange(len(samp)):
            if samp[i] < -np.pi:
                samp[i] = -np.pi
            elif samp[i] > np.pi:
                samp[i] = np.pi 
        return samp
    
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
    
import numpy as np
import plot as Plot
import time
import copy
from scipy.linalg import solve_discrete_are
from scipy.stats import norm, vonmises
from scipy import linalg, signal
from path_planner import PathPlanner
from kinematics import Kinematics
from multiprocessing import Process, Queue
import collections

class LQG:
    def __init__(self):           
        self.set_params()
        '''self.path_planner = PathPlanner()
        self.path_planner.set_params(2, self.max_velocity, self.delta_t)
        self.path_planner.setup_ompl()'''
        
        self.theta_0 = np.array([0.0, 0.0])
        self.goal_position = [-2.0, 0.0]
        self.goal_radius = 0.01
        '''self.path_planner.set_start_state(theta_0)
        self.path_planner.set_goal_region(goal_position, goal_radius)'''
        self.A = np.identity(2)
        self.H = np.identity(2)
        
        
        """
        The process noise covariance matrix
        """
        self.M = np.array([[0.001, 0.0],
                           [0.0, 0.001]])
        
        """
        The observation noise covariance matrix
        """
        self.N = np.array([[0.001, 0.0],
                           [0.0, 0.001]])
        
        self.B = np.array([[self.delta_t, 0.0],
                           [0.0, self.delta_t]])
        
        self.V = np.identity(2)
        self.W = np.identity(2)

        #self.C = np.identity(2)
        self.C = np.array([[2.0, 0.0],
                           [0.0, 2.0]])
        self.D = np.array([[2.0, 0.0],
                           [0.0, 2.0]])
        
        if self.check_positive_definite([self.C, self.D]):
            xs, us, zs = self.plan_paths(self.goal_position, 1)
            #return
            self.simulate(xs, us, zs, 500) 
        
    def check_positive_definite(self, matrices):
        for m in matrices:
            try:
                np.linalg.cholesky(m)
            except:
                print "Matrices are not positive definite. Fix that!"
                return False
        return True
        
    def set_params(self):
        self.max_velocity = 2.0
        self.delta_t = 1.0 / 30.0 
        self.kinematics = Kinematics()    
    
    def get_jacobian(self, links, state):
        return np.array([[-links[0] * np.sin(state[0]) - links[1] * np.sin(state[0] + state[1]), -links[1] * np.sin(state[0] + state[1])],
                         [links[0] * np.cos(state[0]) + links[1] * np.cos(state[0] + state[1]), links[1] * np.cos(state[0] + state[1])]])
        
    def plan_paths(self, goal_position, num):
        jobs = collections.deque()
        ir = collections.deque()
        path_queue = Queue()
        paths = []
        num_cores = 4
        for i in xrange(num):
            print "path num " + str(i)
            p = Process(target=self.do2, args=(path_queue,))
            p.start()
            jobs.append(p)
            ir.append(1)
            if len(ir) >= num_cores - 1 or i == num - 1:
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
            print paths[i][0]
            if paths[i][0] < tr:
                tr = paths[i][0]
                best_index = copy.copy(i)
        print "best path: " + str(best_index)
        print "best trace: " + str(tr)
        sets = []
        for i in xrange(len(paths)):
            s = []
            for j in xrange(len(paths[i][1][0])):
                s.append(np.array(paths[i][1][0][j]))
            sets.append(np.array(s))
        #sets = [np.array([np.array([paths[i][1][0][0], paths[i][1][0][1]])]) for i in xrange(len(paths))]
        
        Plot.plot_2d_n_sets(sets,
                            x_range=[-3.0, 3.0], 
                            y_range=[-3.0, 3.0], 
                            plot_type='lines',
                            idx=best_index,
                            lw=4)
        return paths[best_index][1][0], paths[i][1][1], paths[i][1][2]
        print len(paths)
    
    def do2(self, queue): 
        path_planner = PathPlanner()
        path_planner.set_params(2, self.max_velocity, self.delta_t)
        path_planner.setup_ompl()
        path_planner.set_start_state(self.theta_0)
        path_planner.set_goal_region(self.goal_position, self.goal_radius)              
        xs, us, zs = path_planner.plan_path()        
        set = np.array([np.array([x[0], x[1]]) for x in xs])
        
        Ls = self.compute_gain(self.A, self.B, self.C, self.D, len(xs))        
        
        P_t = np.array([[0.0, 0.0],
                        [0.0, 0.0]])
        P_0 = np.array([[0.0, 0.0],
                        [0.0, 0.0]])
        NU = np.array([[0.0, 0.0],
                       [0.0, 0.0]])
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
            #print "L " + str(Ls[i - 1])
            #print "F " + str(F_t)
            
            G_t = np.vstack((np.hstack((self.V, NU)), 
                             np.hstack((np.dot(np.dot(K_t, self.H), self.V), np.dot(K_t, self.W)))))
            #print "G_t " + str(G_t)           
            
            
            """ Compute R """            
            FRF = np.dot(np.dot(F_t, R_t), np.transpose(F_t))
            #print "FRF " + str(FRF)            
            GQG = np.dot(np.dot(G_t, Q_t), np.transpose(G_t))
            #print "GQG " + str(GQG)
            R_t = np.add(np.dot(np.dot(F_t, R_t), np.transpose(F_t)), np.dot(G_t, np.dot(Q_t, np.transpose(G_t))))            
            
            
            #print "R_t " + str(R_t)
            Gamma_t = np.vstack((np.hstack((np.identity(2), NU)), 
                                 np.hstack((NU, Ls[i - 1]))))
            #print "Gamma_t " + str(Gamma_t)
            Cov = np.dot(np.dot(Gamma_t, R_t), np.transpose(Gamma_t))
            cov_state = np.array([[Cov[0, 0], Cov[0, 1]],
                                  [Cov[1, 0], Cov[1, 1]]])
            j = self.get_jacobian([1.0, 1.0], xs[i])
            cov_state = np.array([[Cov[0, 0], Cov[0, 1]],
                                  [Cov[1, 0], Cov[1, 1]]])
            EE_covariance = np.dot(np.dot(j, cov_state), np.transpose(j))
            print "i " + str(i)
            print "cov " + str(cov_state)
            '''if i == len(xs) - 1:
                joint_samples = np.random.multivariate_normal(xs[i], cov_state, 200)
                ee_positions = np.random.multivariate_normal(self.kinematics.get_end_effector_position(xs[i]),
                                                             EE_covariance,
                                                             200)
                #
                ee_distributions.append(np.array(ee_positions))
                ee_positions = [self.kinematics.get_end_effector_position(state) for state in joint_samples]
                ee_distributions.append(np.array(ee_positions))
                
                ee_distributions.append(np.array([self.kinematics.get_end_effector_position(xs[i])]))'''
            
        queue.put([np.trace(EE_covariance), (xs, us, zs)])
        
        '''Plot.plot_2d_n_sets([distr for distr in ee_distributions], 
                            ['d full approx', 'd full real', 'mean full'], 
                            x_range=[-3.0, 3.0], 
                            y_range=[-3.0, 3.0], 
                            plot_type='points')'''
        #print len(xs)  
        
        
    def simulate(self, xs, us, zs, num_simulations):
        cartesian_coords = []        
        for k in xrange(num_simulations):
            print "simulation run " + str(k)
            x_true = xs[0]
            x_tilde = xs[0]        
            u_dash = np.array([0.0, 0.0])        
            P_t = np.array([[0.0, 0.0],
                            [0.0, 0.0]])
            Ls = self.compute_gain(self.A, self.B, self.C, self.D, len(xs) - 1)
            ee_cov = np.array([[0.0, 0.0],
                               [0.0, 0.0]])
            data = []
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
                data.append([xs[i + 1], x_true, z_t, u_dash + us[i]])
            ee_position = self.kinematics.get_end_effector_position(x_true)
            cartesian_coords.append(np.array([ee_position[l] for l in xrange(len(ee_position))]))            
            '''lin = np.linspace(0.0, len(xs) - 1, len(xs) - 1)        
            set1 = np.array([np.array([lin[i], data[i][0][0]]) for i in xrange(len(lin))])
            set2 = np.array([np.array([lin[i], data[i][1][0]]) for i in xrange(len(lin))])
            set3 = np.array([np.array([lin[i], data[i][2][0]]) for i in xrange(len(lin))])
            set4 = np.array([np.array([lin[i], data[i][3][0]]) for i in xrange(len(lin))])
            Plot.plot_2d_n_sets([set1, set2, set3, set4], ['x_s', 'x_true', 'z', 'u_dash'], x_range=[0.0, len(xs)], y_range=[-np.pi, np.pi])'''
        Plot.plot_2d_n_sets([np.array(cartesian_coords)], 
                            ['c'], 
                            x_range=[-3.0, 3.0],
                            y_range=[-3.0, 3.0],
                            plot_type='points')
            
    def generate_path(self, length, A, B):
        us = [np.array([1.8, 0.9]) for i in xrange(length + 1)]
        xs = [np.array([0.0, 0.0])]
        for i in xrange(length):
            xs.append(self.apply_velocity(xs[-1], A, B, us[i], M=0)) 
        zs = xs[:]
        return xs, us, zs        
        
    def apply_control(self, x_dash, u_dash, A, B, V, M):
        m = self.get_random_joint_angles([0.0, 0.0], M)
        return np.add(np.add(np.dot(A, x_dash), np.dot(B, u_dash)), np.dot(V, m))
        return np.dot(A, x_dash) + np.dot(B, u_dash) + np.dot(V, m)
        
    def apply_velocity(self, theta, A, B, velocity, M=0):
        if M == 0:
            theta_res = np.dot(A, theta) + np.dot(B, velocity)
        else:        
            theta_res = np.dot(A, theta) + np.dot(B, velocity) + self.get_random_joint_angles([0.0, 0.0], M)
        '''for i in xrange(len(theta_res)):
            if theta_res[i] < -np.pi:
                theta_res[i] = -np.pi
            elif theta_res[i] > np.pi:
                theta_res[i] = np.pi'''
        return theta_res
    
    def get_observation(self, true_theta, H, N):
        return np.dot(H, true_theta) + self.get_random_joint_angles([0.0, 0.0], N)        
        
    def get_random_joint_angles(self, mu, cov):
        return np.random.multivariate_normal(mu, cov)
    
    def kalman_estimation(self, x_hat, u, z, A, B, H, P_t, V, W, M, N):
        x_hat_t, P_hat_t = self.kalman_predict(x_hat, u, A, B, P_t, V, M)
        return self.kalman_update(x_hat_t, z, H, P_hat_t, W, N)
    
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
        return np.dot(np.subtract(np.identity(2), np.dot(K_t, H)), P_hat_t)        
    
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
        

if __name__ == "__main__":
    LQG()
    
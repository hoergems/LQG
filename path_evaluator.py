import numpy as np
import kalman as kalman
from scipy.stats import multivariate_normal
from kinematics import Kinematics
from multiprocessing import Process, Queue, cpu_count
import collections
import time

class PathEvaluator:
    def __init__(self):
        pass
    
    def setup(self, A, B, C, D, H, M, N, V, W, num_links, obstacles, verbose):
        self.kinematics = Kinematics(num_links)
        self.A = A
        self.B = B
        self.C = C
        self.D = D
        self.H = H
        self.M = M
        self.N = N
        self.V = V
        self.W = W 
        self.num_links = num_links
        self.obstacles = obstacles
        self.num_cores = cpu_count() - 1
        self.verbose = verbose
        self.w1 = 3.0
        self.w2 = 0.1

    def get_probability_of_collision(self, mean, cov):
            samples = multivariate_normal.rvs(mean, cov, 10)
            pdf = multivariate_normal.pdf(samples, mean, cov, allow_singular=True)
            sum_pdf = sum(pdf)
            #pdf = [s / sum(pdf) for s in pdf]
            pdfs = []
            for i in xrange(len(samples)):
                p1 = self.kinematics.get_link_n_position(samples[i], 1)            
                p2 = self.kinematics.get_link_n_position(samples[i], 2)            
                p3 = self.kinematics.get_link_n_position(samples[i], 3)
                for obstacle in self.obstacles:
                    if obstacle.manipulator_collides([[np.array([0, 0]), p1], [p1, p2], [p2, p3]]):                                    
                        pdfs.append(pdf[i]) 
                        break
            the_sum = 0.0
            if len(pdfs) > 0:
                the_sum = sum(pdfs)                  
            return the_sum
    
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
    
    def evaluate_paths(self, paths, horizon=-1):
        jobs = collections.deque() 
        eval_queue = Queue()
        evaluated_paths = []
        for i in xrange(len(paths)):
            if self.verbose:
                print "Evaluate path " + str(i)
            p = Process(target=self.evaluate, args=(i, eval_queue, paths[i], horizon,))
            p.start()
            jobs.append(p)           
            
            if len(jobs) == self.num_cores - 1 or i == len(paths) - 1:
                if i == len(paths) - 1 and not len(jobs) == self.num_cores - 1:
                    while not eval_queue.qsize() == len(paths) % (self.num_cores - 1):
                        time.sleep(0.00001)
                else:
                    while not eval_queue.qsize() == self.num_cores - 1:
                        time.sleep(0.00001)
                jobs.clear()
                q_size = eval_queue.qsize()
                for j in xrange(q_size):                     
                    evaluated_paths.append(eval_queue.get())        
        min_objective = 1000000.0
        best_path = evaluated_paths[0][1]
        min_objective = evaluated_paths[0][0]        
        for p in evaluated_paths:
            if p[0] < min_objective:
                min_objective = p[0]
                best_path = p[1]                
        return best_path
    
    def evaluate(self, index, eval_queue, path, horizon):
        min_objective = 1000000.0        
        
        xs = path[0]
        us = path[1]
        zs = path[2]
            
        horizon_L = horizon 
        if horizon == -1 or len(xs) - 1 < horizon:            
            horizon_L = len(xs) - 1
        Ls = kalman.compute_gain(self.A, self.B, self.C, self.D, horizon_L)
        P_t = np.array([[0.0 for i in xrange(self.num_links)] for i in xrange(self.num_links)])
        P_0 = np.copy(P_t)
        NU = np.copy(P_t)
                
        Q_t = np.vstack((np.hstack((self.M, NU)), 
                         np.hstack((NU, self.N))))
        R_t = np.vstack((np.hstack((P_0, NU)),
                         np.hstack((NU, NU))))
        ee_distributions = []
        ee_approx_distr = []
        collision_probs = []            
                             
        for i in xrange(0, horizon_L):                
            P_hat_t = kalman.compute_p_hat_t(self.A, P_t, self.V, self.M)
            K_t = kalman.compute_kalman_gain(self.H, P_hat_t, self.W, self.N)
            P_t = kalman.compute_P_t(K_t, self.H, P_hat_t, self.num_links)
            F_0 = np.hstack((self.A, np.dot(self.B, Ls[i])))
            F_1 = np.hstack((np.dot(np.dot(K_t, self.H), self.A), 
                             np.add(self.A, np.subtract(np.dot(self.B, Ls[i]), np.dot(np.dot(K_t, self.H), self.A)))))            
            F_t = np.vstack((F_0, F_1))
            G_t = np.vstack((np.hstack((self.V, NU)), 
                             np.hstack((np.dot(np.dot(K_t, self.H), self.V), np.dot(K_t, self.W)))))
                    
            """ Compute R """            
            FRF = np.dot(np.dot(F_t, R_t), np.transpose(F_t))
            GQG = np.dot(np.dot(G_t, Q_t), np.transpose(G_t))
            R_t = np.add(np.dot(np.dot(F_t, R_t), np.transpose(F_t)), np.dot(G_t, np.dot(Q_t, np.transpose(G_t))))
            Gamma_t = np.vstack((np.hstack((np.identity(self.num_links), NU)), 
                                 np.hstack((NU, Ls[i]))))
                    
                    
            Cov = np.dot(np.dot(Gamma_t, R_t), np.transpose(Gamma_t))
            cov_state = np.array([[Cov[j, k] for k in xrange(self.num_links)] for j in xrange(self.num_links)])
            j = self.get_jacobian([1.0 for k in xrange(self.num_links)], xs[i])
            EE_covariance = np.dot(np.dot(j, cov_state), np.transpose(j))
            EE_covariance = np.array([[EE_covariance[j, k] for k in xrange(2)] for j in xrange(2)])
            probs = 0.0
            if len(self.obstacles) > 0 and np.trace(self.M) != 0.0:
                #print "get collision probs"                    
                probs = self.get_probability_of_collision(xs[i], cov_state)
                collision_probs.append(probs)
            else:
                collision_probs.append(0.0)
        if float(horizon_L) == 0.0:
            collsion_sum = 0.0
        else:
            collision_sum = self.w1 * sum(collision_probs) / float(horizon_L)
        tr = self.w2 * np.trace(EE_covariance)
        if self.verbose:
            print "collision sum: " + str(collision_sum)            
            print "trace: " + str(tr)
        objective_p = collision_sum + tr
        eval_queue.put((objective_p, path))
        '''if objective_p < min_objective:
            min_objective = objective_p
            best_path = (paths[l][0], paths[l][1], paths[l][2])        
        return best_path'''
    
    
        

if __name__ == "__main__":
    PathEvaluator()
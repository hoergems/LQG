import numpy as np
import scipy
import kalman as kalman
from scipy.stats import multivariate_normal
from libkinematics import *
from libutil import *
from libintegrate import *
from multiprocessing import Process, Queue, cpu_count
import collections
import time
import logging
from threading import Lock

class PathEvaluator:
    def __init__(self):
        pass
    
    def setup(self, A, B, C, D, H, M, N, V, W, 
              link_dimensions, 
              workspace_dimension, 
              sample_size, 
              obstacles,
              joint_constraints,
              enforce_constraints,
              goal_position,
              goal_radius,
              w1, 
              w2):       
        self.A = A
        self.B = B
        self.C = C
        self.D = D
        self.H = H
        self.M = M
        self.N = N
        self.V = V
        self.W = W 
        self.link_dimensions = link_dimensions
        self.obstacles = obstacles
        self.joint_constraints = joint_constraints
        self.enforce_constraints = enforce_constraints
        self.sample_size = sample_size
        self.num_cores = cpu_count() - 1
        #self.num_cores = 2 
        self.goal_position = goal_position 
        self.goal_radius = goal_radius   
        self.w1 = w1
        self.w2 = w2
        self.workspace_dimension = workspace_dimension
        self.mutex = Lock()        
        
        axis = v2_int()
        ax1 = v_int()
        ax2 = v_int()
        ax1[:] = [0, 0, 1]
        if workspace_dimension == 2:
            ax2[:] = [0, 0, 1]            
        elif workspace_dimension == 3:
            ax2[:] = [0, 1, 0]
        axis[:] = [ax1, ax2, ax1]
        self.kinematics = Kinematics()
        self.kinematics.setLinksAndAxis(self.link_dimensions, axis)
        self.utils = Utils()
        self.integrate = Integrate()
        self.dynamic_problem = False
        
    def setup_dynamic_problem(self, control_duration):
        self.dynamic_problem = True 
        self.control_duration = control_duration       
        
    def setup_reward_function(self, step_penalty, collision_penalty, exit_reward, discount):
        self.step_penalty = step_penalty
        self.collision_penalty = collision_penalty
        self.exit_reward = exit_reward
        self.discount = discount
        
    def check_constraints(self, sample):
        valid = True
        for i in xrange(len(sample)):
            if ((sample[i] < -self.joint_constraints[i] or 
                 sample[i] > self.joint_constraints[i])):
                valid = False
                break
        return valid
    
    def is_terminal(self, state):                       
        ee_position_arr = self.kinematics.getEndEffectorPosition(state)                
        ee_position = np.array([ee_position_arr[j] for j in xrange(len(ee_position_arr))])        
        if np.linalg.norm(ee_position - self.goal_position) < self.goal_radius:
            print "TERMINAL STATE " + str([state[i] for i in xrange(len(state))])                           
            return True        
        return False        
    
    def get_expected_state_reward(self, mean, cov):
        with self.mutex:
            np.random.seed()              
        samples = multivariate_normal.rvs(mean, cov, self.sample_size)        
        pdf = multivariate_normal.pdf(samples, mean, cov, allow_singular=True) 
        pdf /= sum(pdf)        
        expected_reward = 0.0
        for i in xrange(len(samples)):
            if self.enforce_constraints:
                if not self.check_constraints(samples[i]):
                    continue
            vec = [samples[i][j] for j in xrange(len(self.link_dimensions))]
            joint_angles = v_double()
            joint_angles[:] = vec
            collides = False
            terminal = False
            
            collision_structures = self.utils.createManipulatorCollisionStructures(joint_angles, 
                                                                                   self.link_dimensions,
                                                                                   self.kinematics)
                
            for obstacle in self.obstacles:
                if obstacle.inCollisionDiscrete(collision_structures):                                            
                    expected_reward -= pdf[i] * self.collision_penalty                        
                    collides = True
                    break
            if not collides:
                if self.is_terminal(joint_angles):
                    expected_reward += pdf[i] * self.exit_reward
                    terminal = True
                else:
                    expected_reward -= pdf[i] * self.step_penalty
                       
        return (expected_reward, terminal)    
    
    def get_jacobian(self, links, state):
        s0 = np.sin(state[0])
        c0 = np.cos(state[0])
        s1 = np.sin(state[0] + state[1])
        c1 = np.cos(state[0] + state[1])
            
        if len(state) == 2:
            return np.array([[-links[0] * np.sin(state[0]) - links[1] * np.sin(state[0] + state[1]), -links[1] * np.sin(state[0] + state[1])],
                            [links[0] * np.cos(state[0]) + links[1] * np.cos(state[0] + state[1]), links[1] * np.cos(state[0] + state[1])]])
        elif len(state) == 3:
            l1 = links[0]
            l2 = links[1]
            l3 = links[2]
            c1 = np.cos(state[0])
            c2 = np.cos(state[1])
            c3 = np.cos(state[2])
            s1 = np.sin(state[0])
            s2 = np.sin(state[1])
            s3 = np.sin(state[2])
            if self.workspace_dimension == 2:
                O_0 = np.array([0.0, 
                                0.0, 
                                0.0])
                O_1 = np.array([l1*c1, 
                                l1*s1, 
                                0.0])
                O_2 = np.array([l2*(c1*c2-s1*s2)+l1*c1,
                                l2*(c2*s1+c1*s2)+l1*s1,
                                0.0])
                O_3 = np.array([l3*(c3*(c1*c2-s1*s2)+s3*(-c1*s2-c2*s1))+l2*(c1*c2-s1*s2)+l1*c1,
                                l3*(c3*(c2*s1+c1*s2)+s3*(-s1*s2+c1*c2))+l2*(c2*s1+c1*s2)+l1*s1,
                                0.0])
                z_0 = np.array([0.0, 0.0, 1.0])
                z_1 = np.array([0.0, 0.0, 1.0])
                z_2 = np.array([0.0, 0.0, 1.0])
            else:
                O_0 = np.array([0.0, 
                                0.0, 
                                0.0])
                O_1 = np.array([l1*c1,
                                l1*s1,
                                0.0])
                O_2 = np.array([c1*(l2*c2+l1),
                                s1*(l2*c2+l1),
                                -l2*s2])
                O_3 = np.array([l3*c1*(c2*c3-s2*s3)+c1*(l2*c2+l1),
                                l3*s1*(c2*c3-s2*s3)+s1*(l2*c2+l1),
                                l3*(-c3*s2-c2*s3)-l2*s2])
                z_0 = np.array([0.0, 0.0, 1.0])
                z_1 = np.array([-s1, c1, 0.0])
                z_2 = np.array([-s1, c1, 0.0])
            
            v1 = np.vstack((np.array([np.cross(z_0, np.subtract(O_3, O_0))]).T, np.array([z_0]).T))
            v2 = np.vstack((np.array([np.cross(z_1, np.subtract(O_3, O_1))]).T, np.array([z_1]).T))
            v3 = np.vstack((np.array([np.cross(z_2, np.subtract(O_3, O_2))]).T, np.array([z_2]).T))
            
            j = np.hstack((np.hstack((v1, v2)), v3))            
            return j
        
    def evaluate_path(self, path, P_t, current_step, horizon=-1):       
        objective_p = self.evaluate(0, path, P_t, current_step, horizon)
        return (path, objective_p)
    
    def evaluate_paths(self, paths, P_t, current_step, horizon=-1):
        jobs = collections.deque() 
        eval_queue = Queue()
        evaluated_paths = []
        paths_tmp = []        
        for i in xrange(len(paths)):
            if len(paths[i][0]) != 0:
                paths_tmp.append(paths[i])
        for i in xrange(len(paths_tmp)):            
            logging.info("PathEvaluator: Evaluate path " + str(i))            
            p = Process(target=self.evaluate, args=(i, paths_tmp[i], P_t, current_step, horizon, eval_queue,))
            p.start()
            jobs.append(p)           
                
            if len(jobs) == self.num_cores - 1 or i == len(paths_tmp) - 1:
                if i == len(paths_tmp) - 1 and not len(jobs) == self.num_cores - 1:
                    while not eval_queue.qsize() == len(paths_tmp) % (self.num_cores - 1):
                        time.sleep(0.00001)
                else:
                    while not eval_queue.qsize() == self.num_cores - 1:
                        time.sleep(0.00001)
                jobs.clear()
                q_size = eval_queue.qsize()
                for j in xrange(q_size):                     
                    evaluated_paths.append(eval_queue.get())
        path_rewards = [evaluated_paths[i][0] for i in xrange(len(evaluated_paths))]               
        
        best_path = evaluated_paths[0][1]        
        best_objective = path_rewards[0]        
        for i in xrange(1, len(path_rewards)):                        
            if path_rewards[i] > best_objective:
                best_objective = path_rewards[i]
                best_path = evaluated_paths[i][1]
        logging.info("PathEvaluator: Objective value for the best path is " + str(best_objective))
        return best_path[0], best_path[1], best_path[2], best_objective
    
    def get_linear_model_matrices(self, state_path, control_path):
        As = []
        Bs = []
        Vs = []
        Ms = []
        Hs = []
        Ws = []
        Ns = []
        if self.dynamic_problem:            
            As.append(np.identity((len(state_path[0]))))
            Bs.append(np.identity((len(state_path[0]))))
            Vs.append(np.identity((len(state_path[0]))))
            Ms.append(np.identity((len(state_path[0]))))
            Hs.append(np.identity((len(state_path[0]))))
            Ws.append(np.identity((len(state_path[0]))))
            Ns.append(np.identity((len(state_path[0]))))
            for i in xrange(len(state_path)):
                state = v_double()
                state[:] = state_path[i]
                controls = control_path[i]
                
                t0 = time.time()
                A = self.integrate.getProcessMatrices(state, self.control_duration)                
                Matr_list = [A[i] for i in xrange(len(A))]
                
                A_list = np.array([Matr_list[i] for i in xrange(len(state)**2)])
                           
                B_list = np.array([Matr_list[i] for i in xrange(len(state)**2, 2 * len(state)**2)])
                              
                V_list = np.array([Matr_list[i] for i in xrange(2 * len(state)**2, 
                                                                3 * len(state)**2)])
               
                A_Matr = A_list.reshape(len(state), len(state)).T
                V_Matr = V_list.reshape(len(state), len(state)).T
                B_Matr = B_list.reshape(len(state), len(state)).T  
                
                
                As.append(A_Matr)
                Bs.append(B_Matr)
                Vs.append(V_Matr)
                
                Ms.append(self.M)
                Hs.append(self.H)
                Ws.append(self.W)
                Ns.append(self.N)
        else:
            for i in xrange(len(state_path) + 1):
                As.append(self.A)
                Bs.append(self.B)
                Vs.append(self.V)
                Ms.append(self.M)
                Hs.append(self.H)
                Ws.append(self.W)
                Ns.append(self.N)
            
        return As, Bs, Vs, Ms, Hs, Ws, Ns
    
    def evaluate(self, index, path, P_t, current_step, horizon, eval_queue=None):
        xs = path[0]
        us = path[1]
        zs = path[2]        
        horizon_L = horizon + 1 
        if horizon == -1 or len(xs) < horizon_L:            
            horizon_L = len(xs)
        
        As, Bs, Vs, Ms, Hs, Ws, Ns = self.get_linear_model_matrices(xs, us)        
        
        #Ls = kalman.compute_gain(self.A, self.B, self.C, self.D, horizon_L - 1)
        Ls = kalman.compute_gain(As, Bs, self.C, self.D, horizon_L - 1)
        
        NU = np.array([[0.0 for i in xrange(2 * len(self.link_dimensions))] for i in xrange(2 * len(self.link_dimensions))])
                
        Q_t = np.vstack((np.hstack((self.M, NU)), 
                         np.hstack((NU, self.N))))
        R_t = np.vstack((np.hstack((np.copy(P_t), NU)),
                         np.hstack((NU, NU))))       
        
        ee_distributions = []
        ee_approx_distr = []
        collision_probs = []
        path_rewards = []
        path_rewards.append(np.power(self.discount, current_step) * self.get_expected_state_reward(xs[0], P_t)[0])        
        #sleep
        Cov = 0                
        for i in xrange(1, horizon_L):                              
            P_hat_t = kalman.compute_p_hat_t(As[i], P_t, Vs[i], Ms[i])
            K_t = kalman.compute_kalman_gain(Hs[i], P_hat_t, Ws[i], Ns[i])
            P_t = kalman.compute_P_t(K_t, Hs[i], P_hat_t, 2 * len(self.link_dimensions))
            
            F_0 = np.hstack((As[i], np.dot(Bs[i], Ls[i - 1])))
            F_1 = np.hstack((np.dot(K_t, np.dot(Hs[i], As[i])), 
                             As[i] + np.dot(Bs[i], Ls[i - 1]) - np.dot(K_t, np.dot(Hs[i], As[i]))))            
            F_t = np.vstack((F_0, F_1))                              
            G_t = np.vstack((np.hstack((Vs[i], NU)), 
                             np.hstack((np.dot(np.dot(K_t, Hs[i]), Vs[i]), np.dot(K_t, Ws[i])))))
            G_t_1 = np.vstack((np.hstack((Vs[i], NU)),
                               np.hstack((np.dot(K_t, np.dot(Hs[i], Vs[i])), np.dot(K_t, Ws[i])))))            
            """ Compute R """    
            R_t = np.dot(F_t, np.dot(R_t, F_t.T)) + np.dot(G_t, np.dot(Q_t, G_t.T)) 
            L = np.identity(2 * len(self.link_dimensions))
            if i != horizon_L - 1:
                L = Ls[i]
                
            Gamma_t = np.vstack((np.hstack((np.identity(2 * len(self.link_dimensions)), NU)), 
                                 np.hstack((NU, L))))
                             
            Cov = np.dot(Gamma_t, np.dot(R_t, Gamma_t.T))                       
            cov_state = np.array([[Cov[j, k] for k in xrange(2 * len(self.link_dimensions))] for j in xrange(2 * len(self.link_dimensions))])
            
            
            (state_reward, terminal) = self.get_expected_state_reward(xs[i], cov_state)
            path_rewards.append(np.power(self.discount, current_step + i) * state_reward)
            '''if terminal:
                break'''             
        path_reward = sum(path_rewards)               
        logging.info("========================================")
        logging.info("PathEvaluator: reward for path " + 
                     str(index) + 
                     " is " + 
                     str(path_reward))
        logging.info("========================================")        
        if not eval_queue==None:
            eval_queue.put((path_reward, path, Cov))
        else:
            return path_reward
        

if __name__ == "__main__":
    PathEvaluator()
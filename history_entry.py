import os
import glob
import numpy as np

class HistoryEntry:    
    def __init__(self, 
                 t, 
                 x_true,
                 x_nominal,
                 x_true_linear, 
                 x_estimate,
                 x_dash,
                 x_dash_linear,
                 linearization_error,
                 action,
                 nominal_action,
                 observation,
                 covariance,
                 collided,
                 estimate_collided,
                 terminal,
                 reward,
                 estimated_covariance=None):
        self.t = t        
        self.x_true = np.array([x_true[i] for i in xrange(len(x_true))])
        self.x_nominal = np.array([x_nominal[i] for i in xrange(len(x_nominal))])
        self.x_true_linear = np.array([x_true_linear[i] for i in xrange(len(x_true_linear))])
        self.x_estimate = np.array([x_estimate[i] for i in xrange(len(x_estimate))])
        self.x_dash = np.array([x_dash[i] for i in xrange(len(x_dash))])
        self.x_dash_linear = np.array([x_dash_linear[i] for i in xrange(len(x_dash_linear))])
        if not action == None:
            self.action = np.array([action[i] for i in xrange(len(action))])
        if not nominal_action == None:
            self.nominal_action = np.array([nominal_action[i] for i in xrange(len(nominal_action))])
        if not observation == None:
            self.observation = np.array([observation[i] for i in xrange(len(observation))])
        self.covariance = covariance
        self.collided = collided
        self.estimate_collided = estimate_collided
        self.terminal = terminal
        self.reward = reward
        self.linearization_error = linearization_error
        self.estimated_covariance = estimated_covariance
        self.s_tilde = None
        self.u_dash = None
        self.colliding_obstacle = None
        self.colliding_state = None
        self.replanning = False
        self.estimation_error = None
        self.estimation_error_normalized = None
        self.best_reward = None
        
    def set_best_reward(self, best_reward):
        self.best_reward = best_reward
        
    def set_estimation_error(self, estimation_error):
        self.estimation_error = estimation_error
        
    def set_estimation_error_normalized(self, estimation_error):
        self.estimation_error_normalized = estimation_error
        
    def set_action(self, action):
        self.action = np.array([action[i] for i in xrange(len(action))])
        
    def set_nominal_action(self, nominal_action):
        self.nominal_action = np.array([nominal_action[i] for i in xrange(len(nominal_action))])
        
    def set_observation(self, observation):
        self.observation = np.array([observation[i] for i in xrange(len(observation))])
        
    def set_reward(self, reward):
        self.reward = reward
        
    def set_collided(self, collided):
        self.collided = collided
        
    def set_estimate_collided(self, collided):
        self.estimate_collided = collided
        
    def set_terminal(self, terminal):
        self.terminal = terminal
        
    def set_linearization_error(self, error):
        self.linearization_error = error
        
    def set_estimated_covariance(self, estimated_covariance):        
        self.estimated_covariance = estimated_covariance
        
    def set_s_dash_estimated(self, s_tilde):
        self.s_tilde = np.array([s_tilde[i] for i in xrange(len(s_tilde))])
        
    def set_u_dash(self, u_dash):
        self.u_dash = np.array([u_dash[i] for i in xrange(len(u_dash))])
        
    def set_colliding_obstacle(self, obstacle_name):
        self.colliding_obstacle = obstacle_name
        
    def set_colliding_state(self, colliding_state):
        self.colliding_state = np.array([colliding_state[i] for i in xrange(len(colliding_state))])
        
    def set_replanning(self, replanning):
        self.replanning = replanning
        
    def set_estimated_state(self, estimated_state):
        self.x_estimate = np.array([estimated_state[i] for i in xrange(len(estimated_state))])
        
    def serialize(self, path, file):        
        with open(os.path.join(path, file), 'a') as f:                    
            f.write("t = " + str(self.t) + " \n")
            
            state_str = "S: " 
            for i in xrange(len(self.x_true)):
                state_str += str(self.x_true[i]) + " "
            f.write(state_str + " \n")
            
            state_str = "S_NOMINAL: "
            for i in xrange(len(self.x_nominal)):
                state_str += str(self.x_nominal[i]) + " "
            f.write(state_str + " \n")
            
            state_str = "S_LINEAR: "
            for i in xrange(len(self.x_true_linear)):
                state_str += str(self.x_true_linear[i]) + " "
            f.write(state_str + " \n")
            
            state_str = "S_ESTIMATED: " 
            for i in xrange(len(self.x_estimate)):
                state_str += str(self.x_estimate[i]) + " "
            f.write(state_str + " \n")
            
            state_dash_string = "S_DASH: "
            for i in xrange(len(self.x_dash)):
                state_dash_string += str(self.x_dash[i]) + " "
            f.write(state_dash_string + " \n") 
            
            state_dash_linear_string = "S_DASH_LINEAR: "
            for i in xrange(len(self.x_dash)):
                state_dash_linear_string += str(self.x_dash_linear[i]) + " "
            f.write(state_dash_linear_string + " \n")
            
            if self.s_tilde != None: 
                state_dash_string = "S_DASH_ESTIMATED: "
                for i in xrange(len(self.s_tilde)):
                    state_dash_string += str(self.s_tilde[i]) + " "
                f.write(state_dash_string + " \n")
                
            if self.u_dash != None:
                gain_str = "U_DASH: "
                for i in xrange(len(self.u_dash)):
                    gain_str += str(self.u_dash[i]) + " "
                f.write(gain_str + " \n")
            
            p_str = "COVARIANCE: " 
            for i in xrange(len(self.covariance)):
                for j in xrange(len(self.covariance[i])):
                    p_str += str(self.covariance[i][j]) + " "
            f.write(p_str + " \n")
            
            p_str = "ESTIMATED_COVARIANCE: "
            for i in xrange(len(self.estimated_covariance)):
                for j in xrange(len(self.estimated_covariance[i])):
                    p_str += str(self.estimated_covariance[i][j]) + " "
            f.write(p_str + " \n") 
            
            if not self.terminal:
                try:                
                    action_str = "A: "            
                    for i in xrange(len(self.action)):
                        action_str += str(self.action[i]) + " "
                    f.write(action_str + " \n")
                
                    action_str = "NOMINAL_ACTION: "
                    for i in xrange(len(self.nominal_action)):
                        action_str += str(self.nominal_action[i]) + " "
                    f.write(action_str + " \n")
                
                    obs_str = "O: " 
                    for i in xrange(len(self.observation)):
                        obs_str += str(self.observation[i]) + " "
                    f.write(obs_str + " \n")
                except:
                    pass 
            
            rew_str = "R: " + str(self.reward)
            f.write(rew_str + " \n")
            
            coll_string = "collided: "
            if self.collided:
                coll_string += "true"
            else:
                coll_string += "false"
            f.write(coll_string + " \n")
            
            coll_string = "estimate collided: "
            if self.estimate_collided:
                coll_string += "true"
            else:
                coll_string += "false"
            f.write(coll_string + " \n") 
            
            coll_string = "colliding obstacle: " + str(self.colliding_obstacle)
            f.write(coll_string + " \n")
            
            coll_string = "colliding state: "
            if self.colliding_state == None:
                coll_string += "None"
            else:             
                for i in xrange(len(self.colliding_state)):
                    coll_string += str(self.colliding_state[i]) + " "
            f.write(coll_string + " \n")
            
            coll_str = "Replanning: "
            if self.replanning:
                coll_str += "true \n" 
            else:
                coll_str += "false \n"
            f.write(coll_str)          
            
            term_string = "Terminal: true"
            if not self.terminal:
                term_string = "Terminal: false"
            f.write(term_string + " \n")
            
            err_string = "Estimation error: "
            if not self.estimation_error == None:
                err_string += str(self.estimation_error)
            else:
                err_string += "None"
            f.write(err_string + " \n")
            
            err_string = "Estimation error normalized: "
            if not self.estimation_error == None:
                err_string += str(self.estimation_error_normalized)
            else:
                err_string += "None"
            f.write(err_string + " \n")
            
            err_string = "Linearization error: " + str(self.linearization_error)
            f.write(err_string + " \n")
            
            if self.best_reward != None:
                best_reward_str = "Best reward: " + str(self.best_reward)
                f.write(best_reward_str + " \n") 
            
            f.write(" \n")
               
                       
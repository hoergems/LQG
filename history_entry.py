import os
import glob

class HistoryEntry:    
    def __init__(self, 
                 t, 
                 x_true,
                 x_true_linear, 
                 x_estimate,
                 x_dash,
                 x_dash_linear,
                 linearization_error,
                 action,
                 observation,
                 covariance,
                 collided,
                 estimate_collided,
                 terminal,
                 reward):
        self.t = t        
        self.x_true = x_true
        self.x_true_linear = x_true_linear
        self.x_estimate = x_estimate
        self.x_dash = x_dash
        self.x_dash_linear = x_dash_linear
        self.action = action
        self.observation = observation
        self.covariance = covariance
        self.collided = collided
        self.estimate_collided = estimate_collided
        self.terminal = terminal
        self.reward = reward
        self.linearization_error = linearization_error
        
    def set_action(self, action):
        self.action = action
        
    def set_observation(self, observation):
        self.observation = observation
        
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
        
    def serialize(self, path, file):        
        with open(os.path.join(path, file), 'a') as f:                    
            f.write("t = " + str(self.t) + " \n")
            
            state_str = "S: " 
            for i in xrange(len(self.x_true)):
                state_str += str(self.x_true[i]) + " "
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
            
            p_str = "COVARIANCE: " 
            for i in xrange(len(self.covariance)):
                for j in xrange(len(self.covariance[i])):
                    p_str += str(self.covariance[i][j]) + " "
            f.write(p_str + " \n") 
            
            if not self.terminal:
                action_str = "A: "            
                for i in xrange(len(self.action)):
                    action_str += str(self.action[i]) + " "
                f.write(action_str + " \n")
            
            
                obs_str = "O: " 
                for i in xrange(len(self.observation)):
                    obs_str += str(self.observation[i]) + " "
                f.write(obs_str + " \n") 
            
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
            
            term_string = "Terminal: true"
            if not self.terminal:
                term_string = "Terminal: false"
            f.write(term_string + " \n")
            
            err_string = "Linearization error: " + str(self.linearization_error)
            f.write(err_string + " \n")
            
            f.write(" \n")
               
                       
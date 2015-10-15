import os
import glob

class HistoryEntry:    
    def __init__(self, 
                 t, 
                 x_true, 
                 x_estimate,
                 action,
                 observation,
                 covariance,
                 collided,
                 terminal,
                 reward):
        self.t = t        
        self.x_true = x_true
        self.x_estimate = x_estimate
        self.action = action
        self.observation = observation
        self.covariance = covariance
        self.collided = collided
        self.terminal = terminal
        self.reward = reward
        
    def set_action(self, action):
        self.action = action
        
    def set_observation(self, observation):
        self.observation = observation
        
    def set_reward(self, reward):
        self.reward = reward
        
    def set_collided(self, collided):
        self.collided = collided
        
    def set_terminal(self, terminal):
        self.terminal = terminal
        
    def serialize(self, path, file):        
        with open(os.path.join(path, file), 'a') as f:                    
            f.write("t = " + str(self.t) + " \n")
            
            state_str = "S: " 
            for i in xrange(len(self.x_true)):
                state_str += str(self.x_true[i]) + " "
            f.write(state_str + " \n")
            
            state_str = "S_ESTIMATED: " 
            for i in xrange(len(self.x_estimate)):
                state_str += str(self.x_estimate[i]) + " "
            f.write(state_str + " \n") 
            
            p_str = "COVARIANCE: " 
            for i in xrange(len(self.covariance)):
                for j in xrange(len(self.covariance[i])):
                    p_str += str(self.covariance[i][j]) + " "
            f.write(p_str + " \n") 
            
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
            
            term_string = "Terminal: true"
            if not self.terminal:
                term_string = "Terminal: false"
            f.write(term_string + " \n")
            
            f.write(" \n")
               
                       
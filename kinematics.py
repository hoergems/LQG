import numpy as np

class Kinematics:
    def __init__(self, num_links):
        #theta = [1.5, 2.7]
        self.num_links = num_links
        theta = [0.0, 0.0, np.pi / 2.0]
        print "theta " + str(theta)
        print "cart " + str(self.get_end_effector_position(theta))
        
        
    def get_end_effector_position(self, state):
        """
        Gets the end effector position from a joint state
        """        
        state2 = [state[i] for i in xrange(self.num_links)]        
        SE2s = [self.SE2(1.0, 0.0, state2[i]) for i in xrange(len(state2))]
        SE2 = np.identity(3)
        for i in xrange(len(SE2s)):
            SE2 = np.dot(SE2, SE2s[i])
        return np.array([SE2[i][2] for i in xrange(2)])        
        
    def SE2(self, x, y, theta):
        return np.array([[np.cos(theta), -np.sin(theta), x * np.cos(theta) - y * np.sin(theta)],
                         [np.sin(theta), np.cos(theta), x * np.sin(theta) + y * np.cos(theta)],
                         [0.0, 0.0, 1.0]])
    
    
        
if __name__ == "__main__":
    Kinematics()
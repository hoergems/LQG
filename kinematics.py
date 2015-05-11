import numpy as np

class Kinematics:
    def __init__(self):
        #theta = [1.5, 2.7]
        theta = [-np.pi, -1.0]
        print "theta " + str(theta)
        print "cart " + str(self.get_end_effector_position(theta))
        
        
    def get_end_effector_position(self, state):
        """
        Gets the end effector position from a joint state
        """
        SE2_1 = self.SE2(1.0, 0.0, state[0])
        SE2_2 = self.SE2(1.0, 0.0, state[1])
        SE2 = np.dot(SE2_1, SE2_2)
        return np.array([SE2[0][2], SE2[1][2]])
        
    def SE2(self, x, y, theta):
        return np.array([[np.cos(theta), -np.sin(theta), x * np.cos(theta) - y * np.sin(theta)],
                         [np.sin(theta), np.cos(theta), x * np.sin(theta) + y * np.cos(theta)],
                         [0.0, 0.0, 1.0]])
        
if __name__ == "__main__":
    Kinematics()
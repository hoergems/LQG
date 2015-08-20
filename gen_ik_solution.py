import openravepy
from openravepy import *
from openravepy.misc import InitOpenRAVELogging
from kin import *
from obstacle import *
from serializer import Serializer
from path_planner import *
from gen_ik_solution import *
import json
import time
import numpy as np

class IKSolutionGenerator:
    def __init__(self):
        self.serializer = Serializer()
    
    def setup(self, num_links, workspace_dimension, max_velocity, delta_t, joint_constraints, robot_file, environment_file, verbose_rrt):
        """
        Generate the obstacles
        """
        environment = self.serializer.load_environment(file=environment_file, path="")
        obstacles = []
        terrain = Terrain("default", 0.0, 1.0, True)
        for obstacle in environment:
            print obstacle
            obstacles.append(Obstacle(obstacle[0][0], obstacle[0][1], obstacle[0][2], obstacle[1][0], obstacle[1][1], obstacle[1][2], terrain))
        self.path_planner = PathPlanner()
        self.path_planner.set_params(num_links, workspace_dimension, max_velocity, delta_t, False, 0, joint_constraints, verbose_rrt)
        self.path_planner.setup_ompl()
        self.path_planner.set_obstacles(obstacles)
        
        self.env = openravepy.Environment()
        self.env.StopSimulation()
        self.env.Load(robot_file)
        self.env.Load(environment_file)
        
        self.robot = self.env.GetRobots()[0]
        self.robot.SetActiveManipulator("arm")          

    def generate(self, start_state, goal_position, workspace_dimension):
        self.path_planner.set_start_state(start_state)
        possible_ik_solutions = []        
        if workspace_dimension == 2:
            InitOpenRAVELogging()
            ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(self.robot,
                                                                                    iktype=IkParameterizationType.TranslationXY2D)
            if not ikmodel.load():
                ikmodel.autogenerate()
                
            #robot.SetDOFValues(joint_start, robot.GetManipulator('arm').GetArmIndices())
            target=ikmodel.manip.GetTransform()[0:2,3]
            print "target " + str(target)
            target[0] = goal_position[0]
            target[1] = goal_position[1]
            
            possible_ik_solutions = ikmodel.manip.FindIKSolutions(IkParameterization(target,IkParameterization.Type.TranslationXY2D), False)            
            
        elif workspace_dimension == 3:            
            solution = self.legIK(goal_position[0], goal_position[1], goal_position[2])
            sol1 = solution
            sol2 = [solution[0], -solution[1], -solution[2]]
            possible_ik_solutions = [sol1, sol2]
            
        solutions = []
        n = 0
        for i in xrange(len(possible_ik_solutions)):            
            ik_solution = [possible_ik_solutions[i][k] for k in xrange(len(start_state))]            
            path = self.path_planner.plan_path(ik_solution)
            if len(path[0]) != 0:                
                solutions.append(path[0][-1])                
            n += 1
        if not len(solutions) == 0:
            #print "IK solution is " + str(solution[0])
            return solutions
        else:
            print "Couldn't find a valid IK solution. Defined problem seems to be infeasible."
            return []
        
            
        
    def legIK(self, x, y, z, links=[1.0, 1.0, 1.0]):    
        angles = []
        angles.append(np.arctan2(y, x))
        x_im = np.sqrt(np.square(x) + np.square(y)) - links[0]        
        x_true = np.sqrt(np.square(x_im) + np.square(z))           
        alpha1 = np.arccos(np.abs(z) / x_true)           
        a = np.square(links[2]) - np.square(links[1]) - np.square(x_true)        
        b = -2.0 * links[1] * x_true        
        alpha2 = np.arccos(a / b)
        if np.isnan(alpha2):
            rospy.logerr("a " + str(a))
            rospy.logerr("b " + str(b))
        alpha3 = alpha1 + alpha2 - (np.pi / 2.0)
        beta = np.arccos((np.square(x_true) - np.square(links[2]) - np.square(links[1])) / (-2.0 * links[2] * links[1]))        
        
        angles.append(alpha3)
        angles.append(-1.0 * (np.pi - beta))
        '''else:
            angles.append(-1.0 * alpha3) 
            angles.append(np.pi - beta)'''
        for angle in angles:
            if np.isnan(angle):
                print "ANGLE IS NAN"
                return []                
        return angles
    
if __name__ == "__main__":
    IKSolutionGenerator()
    
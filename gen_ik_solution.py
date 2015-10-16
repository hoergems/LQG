import openravepy
from openravepy import *
from openravepy.misc import InitOpenRAVELogging
from obstacle import *
from serializer import Serializer
from path_planning_interface import *
from gen_ik_solution import *
from kin import *
import json
import time
import numpy as np
import logging

class IKSolutionGenerator:
    def __init__(self):
        self.serializer = Serializer()
    
    def setup(self, 
              link_dimensions, 
              workspace_dimension, 
              obstacles, 
              max_velocity, 
              delta_t, 
              joint_constraints,
              enforce_constraints,
              robot_file, 
              environment_file,
              planning_algorithm):        
        """
        Generate the obstacles
        """
        logging.info("IKSolutionGenerator: Setup")
        self.link_dimensions = link_dimensions
        self.path_planner = PathPlanningInterface()
        self.path_planner.setup(link_dimensions, 
                                workspace_dimension, 
                                obstacles, 
                                max_velocity, 
                                delta_t, 
                                False, 
                                joint_constraints,
                                enforce_constraints,
                                planning_algorithm)
        logging.info("IKSolutionGenerator: Create OpenRAVE environment")
        self.env = openravepy.Environment()
        self.env.StopSimulation()
        logging.info("IKSolutionGenerator: Loading robot file: " + str(robot_file))
        self.env.Load(robot_file)
        logging.info("IKSolutionGenerator: Loading environment file")
        self.env.Load(environment_file)
        logging.info("IKSolutionGenerator: OpenRAVE initialized")
        self.robot = self.env.GetRobots()[0]
        self.robot.SetActiveManipulator("arm")   

    def generate(self, start_state, goal_position, workspace_dimension):        
        possible_ik_solutions = []        
        if workspace_dimension == 2:
            InitOpenRAVELogging()
            ikmodel = openravepy.databases.inversekinematics.InverseKinematicsModel(self.robot,
                                                                                    iktype=IkParameterizationType.TranslationXY2D)
            if not ikmodel.load():
                ikmodel.autogenerate()
                
            #robot.SetDOFValues(joint_start, robot.GetManipulator('arm').GetArmIndices())
            target=ikmodel.manip.GetTransform()[0:2,3]
            logging.info("IKSolutionGenerator: target " + str(target))
            target[0] = goal_position[0]
            target[1] = goal_position[1]
            
            possible_ik_solutions = ikmodel.manip.FindIKSolutions(IkParameterization(target,IkParameterization.Type.TranslationXY2D), False)            
            
        elif workspace_dimension == 3:            
            solution, unique = self.legIK(goal_position[0], 
                                          goal_position[1], 
                                          goal_position[2],
                                          self.link_dimensions)
            sol1 = solution
            #if unique:
            possible_ik_solutions = [sol1]
            #else:
            #    sol2 = [solution[0], -solution[1], -solution[2]]
            #    possible_ik_solutions = [sol1, sol2]
            
        solutions = []
        n = 0
        logging.info("IKSolutionGenerator: " + str(len(possible_ik_solutions)) + " possible ik solutions found")
        for i in xrange(len(possible_ik_solutions)):        
            logging.info("IKSolutionGenerator: Checking ik solution " + str(i) + " for validity")            
            ik_solution = [possible_ik_solutions[i][k] for k in xrange(len(start_state))] 
            self.path_planner.set_start_and_goal(start_state, [ik_solution])           
            path = self.path_planner.plan_paths(1, 0)            
            if len(path) != 0:
                logging.info("IKSolutionGenerator: ik solution " + str(i) + " is a valid ik solution")                
                solutions.append(path[0][0][-1])                
            n += 1
        self.path_planner = None        
        if not len(solutions) == 0: 
            print "IKSolutionGenerator: Found " + str(len(solutions)) + " valid goal states"           
            return solutions        
        else:
            logging.error("IKSoultionGenerator: Couldn't find a valid IK solution. Defined problem seems to be infeasible.")
            return []
        
            
        
    def legIK(self, x, y, z, links):           
        angles = []
        angles.append(np.arctan2(y, x))
        x_im = np.sqrt(np.square(x) + np.square(y)) - links[0][0]        
        x_true = np.sqrt(np.square(x_im) + np.square(z))           
        alpha1 = np.arccos(np.abs(z) / x_true)           
        a = np.square(links[2][0]) - np.square(links[1][0]) - np.square(x_true)        
        b = -2.0 * links[1][0] * x_true        
        alpha2 = np.arccos(a / b)
        if np.isnan(alpha2):
            logging.error("IKSolutionGenerator: Couldn't generate an inverse kinematic solution for goal position " + str((x, y, z)))
            return [], True
        alpha3 = alpha1 + alpha2 - (np.pi / 2.0)
        beta = np.arccos((np.square(x_true) - np.square(links[2][0]) - np.square(links[1][0])) / (-2.0 * links[2][0] * links[1][0]))        
        
        if z < 0:
            angles.append(-alpha3)
            angles.append(np.pi - beta)
        else:
            angles.append(alpha3)
            angles.append(-(np.pi - beta))
        
        for angle in angles:
            if np.isnan(angle):
                logging.error("IKSolutionGenerator: Couldn't generate an inverse kinematic solution for goal position " + str((x, y, z)))
                return [], True
        if ((angles[1] > 0 and angles[2] < 0) or
            (angles[1] < 0 and angles[2] > 0)):
            return angles, False
                        
        return angles, True
    
if __name__ == "__main__":
    IKSolutionGenerator()
    
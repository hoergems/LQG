from libobstacle import *
from serializer import Serializer
from path_planning_interface import *
from gen_ik_solution import *
import ik
import time
import logging

class IKSolutionGenerator:
    def __init__(self):
        self.serializer = Serializer()    
    
    def setup(self,
              robot,
              obstacles, 
              max_velocity, 
              delta_t,
              planning_algorithm,
              path_timeout,
              continuous_collision,
              num_cores):        
        """
        Generate the obstacles
        """                
        self.robot = robot        
        logging.info("IKSolutionGenerator: Setup")
        self.link_dimensions = v2_double()
        self.robot.getActiveLinkDimensions(self.link_dimensions)        
        self.path_planner = PathPlanningInterface()
        self.obstacles = obstacles
        self.path_planner.setup(robot, 
                                obstacles, 
                                max_velocity, 
                                delta_t, 
                                False,
                                planning_algorithm,
                                path_timeout,
                                continuous_collision,
                                num_cores)        
        
    def transform_goal(self, goal_position):
        """
        Transform goal position to first joint frame
        """
        active_joints = v_string()
        self.robot.getActiveJoints(active_joints)
        joint_origins = v2_double()
        self.robot.getJointOrigin(active_joints, joint_origins) 
        joint_origin_first_joint = [joint_origins[0][i] for i in xrange(3)]
        goal_position = [goal_position[i] - joint_origin_first_joint[i] for i in xrange(len(goal_position))]
        return goal_position   

    def generate(self, 
                 start_state, 
                 goal_position, 
                 goal_threshold,
                 num_generated_goal_states): 
        """
        Goal position is w.r.t. base frame
        """       
        goal_position = self.transform_goal(goal_position)         
        possible_ik_solutions = ik.get_goal_states(self.robot, 
                                                   goal_position, 
                                                   self.obstacles, 
                                                   num=num_generated_goal_states)            
        solutions = []
        n = 0
        logging.warn("IKSolutionGenerator: " + str(len(possible_ik_solutions)) + " possible ik solutions found")
        for i in xrange(len(possible_ik_solutions)):                   
            logging.info("IKSolutionGenerator: Checking ik solution " + str(i) + " for validity")            
            ik_solution = [possible_ik_solutions[i][k] for k in xrange(len(start_state) / 2)]                        
            ik_solution.extend([0.0 for j in xrange(len(start_state) / 2)])            
            self.path_planner.set_start_and_goal(start_state, [ik_solution], goal_position, goal_threshold)                   
            path = self.path_planner.plan_paths(1, 0)            
            if len(path) != 0:
                logging.warn("IKSolutionGenerator: ik solution " + str(i) + " is a valid ik solution")
                solutions.append(ik_solution)
            else:
                logging.warn("IKSolutionGenerator: Path has length 0")                
            n += 1
        self.path_planner = None        
        if not len(solutions) == 0: 
            print "IKSolutionGenerator: Found " + str(len(solutions)) + " valid goal states"            
            return solutions                  
        else:
            logging.error("IKSoultionGenerator: Couldn't find a valid IK solution. Defined problem seems to be infeasible.")
            self.path_planner = None            
            return []
    
if __name__ == "__main__":
    IKSolutionGenerator()
    
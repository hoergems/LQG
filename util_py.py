import os
import shutil
import difflib
import logging
import numpy as np
import time
from difflib import Differ
from gen_ik_solution import IKSolutionGenerator

def dist(x1, x2):
    """ Calulate the L2-norm between state x1 and x2 """
    sum = 0.0
    for i in xrange(len(x1)):
        sum += np.power((x1[i] - x2[i]), 2)
    return np.sqrt(sum)

def check_positive_definite(matrices):
    """ Check if the matrices are positive-definite
    """
    for m in matrices:
        try:
            np.linalg.cholesky(m)
        except:
            logging.error("Cost matrices are not positive definite. Fix that!")
            return False
    return True

def compareEnvironmentToTmpFiles(problem, environment_file):
    if not os.path.exists("tmp/" + problem):
        os.makedirs("tmp/" + problem)                   
        return False
        
    if not (os.path.exists('tmp/' + problem + '/' + environment_file) and
            os.path.exists('tmp/' + problem + '/config_' + str(problem) + '.yaml')):                
        return False
        
    with open(environment_file, 'r') as f1, open('tmp/' + problem + '/' + environment_file, 'r') as f2:
        missing_from_b = [
            diff[2:] for diff in Differ().compare(f1.readlines(), f2.readlines())
            if diff.startswith('-')
        ]
        if len(missing_from_b) != 0:                
            return False    
        
    with open('config_' + str(problem) + '.yaml', 'r') as f1, open('tmp/' + problem + '/config_' + str(problem) + '.yaml', 'r') as f2:
        missing_from_b = [
            diff[2:] for diff in Differ().compare(f1.readlines(), f2.readlines())
            if diff.startswith('-')
        ]
            
        for i in xrange(len(missing_from_b)):
            if ("num_links" in missing_from_b[i] or
                "workspace_dimensions" in missing_from_b[i] or
                "goal_position" in missing_from_b[i] or
                "goal_radius" in missing_from_b[i] or
                "joint_constraints" in missing_from_b[i]):
                return False
        
    """ If same, use existing goalstates """
    if os.path.exists("tmp/" + problem + "/goalstates.txt"):
        return True
    return False

def get_goal_states(problem, 
                    serializer, 
                    obstacles,                                         
                    robot,
                    max_velocity,
                    delta_t,
                    start_state,
                    goal_position,
                    goal_threshold,
                    planning_algorithm,
                    path_timeout,
                    num_generated_goal_states,
                    continuous_collision,                    
                    environment_file,
                    num_cores):     
    if not compareEnvironmentToTmpFiles(problem, environment_file):        
        ik_solution_generator = IKSolutionGenerator()          
        ik_solution_generator.setup(robot,
                                    obstacles,
                                    max_velocity,
                                    delta_t,
                                    planning_algorithm,
                                    path_timeout,
                                    continuous_collision,
                                    num_cores)
        ik_solutions = ik_solution_generator.generate(start_state, 
                                                      goal_position, 
                                                      goal_threshold,
                                                      num_generated_goal_states)        
        if len(ik_solutions) == 0:
            return []
        serializer.serialize_ik_solutions([ik_solutions[i] for i in xrange(len(ik_solutions))], path='tmp/' + problem, file='goalstates.txt')
        copyToTmp(problem, environment_file)    
    else:
        ik_solutions = serializer.deserialize_joint_angles(path="tmp/" + problem, file="goalstates.txt")          
    return ik_solutions

def copyToTmp(problem, environment_file):    
    if not os.path.exists("tmp/" + problem + "/environment"):
        os.makedirs("tmp/" + problem + "/environment")
    shutil.copy2(environment_file, 'tmp/' + problem + '/' + environment_file)        
    shutil.copy2('config_' + str(problem) + '.yaml', 'tmp/' + problem + '/config_' + str(problem) + '.yaml')

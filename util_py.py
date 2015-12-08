import os
import shutil
import difflib
import logging
import numpy as np
import time
from difflib import Differ
from gen_ik_solution import IKSolutionGenerator

def check_positive_definite(matrices):
    for m in matrices:
        try:
            np.linalg.cholesky(m)
        except:
            logging.error("MPC: Matrices are not positive definite. Fix that!")
            return False
    return True

def compareEnvironmentToTmpFiles(problem, model_file):
    if not os.path.exists("tmp/" + problem):
        os.makedirs("tmp/" + problem)                   
        return False
        
    if not (os.path.exists('tmp/' + problem + '/env.xml') and
            os.path.exists('tmp/' + problem + '/config_' + str(problem) + '.yaml') and
            os.path.exists(model_file)):                
        return False
        
    with open("environment/env.xml", 'r') as f1, open('tmp/' + problem + '/env.xml', 'r') as f2:
        missing_from_b = [
            diff[2:] for diff in Differ().compare(f1.readlines(), f2.readlines())
            if diff.startswith('-')
        ]
        if len(missing_from_b) != 0:                
            return False
        
    with open(model_file, 'r') as f1, open(model_file, 'r') as f2:               
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
                    model_file,                     
                    robot,
                    max_velocity,
                    delta_t,
                    start_state,
                    goal_position,
                    goal_threshold,
                    planning_algorithm,
                    path_timeout):     
    if not compareEnvironmentToTmpFiles(problem, model_file):
        ik_solution_generator = IKSolutionGenerator()          
        ik_solution_generator.setup(robot,
                                    obstacles,
                                    max_velocity,
                                    delta_t,
                                    model_file,
                                    "environment/env.xml",
                                    planning_algorithm,
                                    path_timeout)
        ik_solutions = ik_solution_generator.generate(start_state, goal_position, goal_threshold)
        ik_solution_generator.destroy()
        if len(ik_solutions) == 0:
            return None
        serializer.serialize_ik_solutions([ik_solutions[i] for i in xrange(len(ik_solutions))], path='tmp/' + problem, file='goalstates.txt')
        copyToTmp(problem)    
    else:
        ik_solutions = serializer.deserialize_joint_angles(path="tmp/" + problem, file="goalstates.txt")          
    return ik_solutions

def copyToTmp(problem):
    shutil.copy2("environment/env.xml", 'tmp/' + problem + '/env.xml')
    shutil.copy2("model/test.xml", 'tmp/' + problem + '/test.xml')    
    shutil.copy2('config_' + str(problem) + '.yaml', 'tmp/' + problem + '/config_' + str(problem) + '.yaml')

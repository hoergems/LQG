import os
import shutil
import difflib
import logging
import numpy as np
from difflib import Differ
from gen_ik_solution import *

def check_positive_definite(matrices):
    for m in matrices:
        try:
            np.linalg.cholesky(m)
        except:
            logging.error("MPC: Matrices are not positive definite. Fix that!")
            return False
    return True

def compareEnvironmentToTmpFiles(problem):
    if not os.path.exists("tmp/" + problem):
        os.makedirs("tmp/" + problem)            
        return False
        
    if not (os.path.exists('tmp/' + problem + '/env.xml') and
            os.path.exists('tmp/' + problem + '/config_' + str(problem) + '.yaml')):            
        return False
        
    with open("environment/env.xml", 'r') as f1, open('tmp/' + problem + '/env.xml', 'r') as f2:
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
                "goal_radius" in missing_from_b[i]):
                return False
        
    """ If same, use existing goalstates """
    if os.path.exists("tmp/" + problem + "/goalstates.txt"):
        return True
    return False
        
    try:        
        shutil.copy2('tmp/' + problem + '/goalstates.txt', "goalstates.txt")
    except:
        return False
    return True

def get_goal_states(problem, 
                    serializer, 
                    obstacles, 
                    num_links, 
                    workspace_dimension,
                    max_velocity,
                    delta_t,
                    joint_constraints,
                    theta_0,
                    goal_position):
    #goal_states = [np.array(gs) for gs in serializer.load_goal_states("goal_states.yaml")]
    #return goal_states
    if not compareEnvironmentToTmpFiles(problem):                     
        ik_solution_generator = IKSolutionGenerator()
        model_file = "model/model.xml"
        if workspace_dimension == 3:
            model_file = "model/model3D.xml"
        ik_solution_generator.setup(num_links,
                                    workspace_dimension,
                                    obstacles,
                                    max_velocity,
                                    delta_t,
                                    joint_constraints,
                                    model_file,
                                    "environment/env.xml")
        ik_solutions = ik_solution_generator.generate(theta_0, goal_position, workspace_dimension)
            
        serializer.serialize_ik_solutions([ik_solutions[i] for i in xrange(len(ik_solutions))], path='tmp/' + problem, file='goalstates.txt')
        copyToTmp(problem)    
    else:
        ik_solutions = serializer.deserialize_joint_angles(path="tmp/" + problem, file="goalstates.txt")          
    return ik_solutions

def copyToTmp(problem):
    shutil.copy2("environment/env.xml", 'tmp/' + problem + '/env.xml')
    #shutil.copy2("goalstates.txt", 'tmp/' + problem + '/goalstates.txt')
    shutil.copy2('config_' + str(problem) + '.yaml', 'tmp/' + problem + '/config_' + str(problem) + '.yaml')
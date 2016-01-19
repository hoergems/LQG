import numpy as np
from librobot import v_double, v2_double, v_string

def get_goal_states(robot, goal_position, obstacles, num=1):
    """ Get 'num' states for which the end-effector is close to the goal position
    """
    solutions = []    
    goal_position = np.array(goal_position)
    dof = robot.getDOF()
    while len(solutions) < num:
        breaking = False   
        state = get_random_state(robot)       
        dist = 10000000.0
        while dist > 0.0003:
            ee_position = transform_ee_position(robot, get_end_effector_position(robot, state))
            delta_p = goal_position - ee_position
            delta_p = [delta_p[i] for i in xrange(len(delta_p))]
            delta_p.extend([0.0 for i in xrange(3)])
            delta_p = np.array(delta_p)
            jacobian_inv = get_jacobian_inverse(robot, state)
            delta_state = np.dot(jacobian_inv, delta_p)        
            state += delta_state        
            old_dist = dist
            dist = np.linalg.norm(delta_p)
            if old_dist - dist < 1e-10:
                breaking = True
                break
        if not breaking:            
            if check_constraints(robot, state) and not in_collision(robot, state, obstacles):
                #print state
                #sleep
                solutions.append([state[k] for k in xrange(len(state))])
    
    return solutions

def in_collision(robot, state, obstacles):
    collidable_obstacles = [o for o in obstacles if not o.isTraversable()]
    joint_angles = v_double()
    joint_angles[:] = [state[i] for i in xrange(len(state))]
    collision_objects = robot.createRobotCollisionObjects(joint_angles)
    for obstacle in collidable_obstacles:
        if obstacle.inCollisionDiscrete(collision_objects):                               
            return True
    return False
    
def check_constraints(robot, state):
    active_joints = v_string()    
    robot.getActiveJoints(active_joints)
    joint_lower_limits = v_double()
    joint_upper_limits = v_double()
    robot.getJointLowerPositionLimits(active_joints, joint_lower_limits)
    robot.getJointUpperPositionLimits(active_joints, joint_upper_limits)
    for i in xrange(len(state)):
        if (state[i] < joint_lower_limits[i] + 0.00000001 or 
            state[i] > joint_upper_limits[i] - 0.00000001):
            return False
    return True
    
def get_random_state(robot):
    active_joints = v_string()    
    robot.getActiveJoints(active_joints)
    joint_lower_limits = v_double()
    joint_upper_limits = v_double()
    robot.getJointLowerPositionLimits(active_joints, joint_lower_limits)
    robot.getJointUpperPositionLimits(active_joints, joint_upper_limits)
    state = [np.random.uniform(joint_lower_limits[i], joint_upper_limits[i]) for i in xrange(len(active_joints))]
    return np.array(state)    
    
def transform_ee_position(robot, position):
    """
    Transform position to first joint frame
    """
    active_joints = v_string()
    robot.getActiveJoints(active_joints)
    joint_origins = v2_double()
    robot.getJointOrigin(active_joints, joint_origins) 
    joint_origin_first_joint = [joint_origins[0][i] for i in xrange(3)]
    position = [position[i] - joint_origin_first_joint[i] for i in xrange(len(position))]
    return np.array(position)  
    
def get_end_effector_position(robot, state):
    s = v_double()
    s[:] = [state[i] for i in xrange(len(state))]
    ee_position = v_double()
    robot.getEndEffectorPosition(s, ee_position)
    return np.array([ee_position[i] for i in xrange(len(ee_position))])
    
def get_jacobian_inverse(robot, state):
    ee_jacobian = get_jacobian(robot, state)
    return np.linalg.pinv(ee_jacobian)
    
def get_jacobian(robot, state):
    s = v_double()
    s[:] = [state[i] for i in xrange(len(state))]
    ee_jacobian = v2_double()
    robot.getEndEffectorJacobian(s, ee_jacobian)
    jac = []
    for i in xrange(len(ee_jacobian)):
        jac_el = [ee_jacobian[i][j] for j in xrange(len(ee_jacobian[i]))]
        jac.append(jac_el)
    return np.array(jac)
    
    
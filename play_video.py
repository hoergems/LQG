import os
import glob
import argparse
import numpy as np
import time
import sys
from librobot import Robot, v_double, v2_double, v_string
from libutil import *
from libobstacle import *

class Play:
    def __init__(self, 
                 algorithm, 
                 dir, 
                 numb,
                 particles,                 
                 play_failed,
                 play_success, 
                 user_input, 
                 nominal_trajectory,
                 colliding_states,
                 covariance,
                 particle_limit,
                 min_max,
                 draw_base_link,
                 robot_file=None,
                 environment_file=None):        
        robot_files = glob.glob(os.path.join(os.path.join(dir + "/model/", "*.urdf")))
        environment_files = glob.glob(os.path.join(os.path.join(dir + "/environment/", "*.xml")))
        
        self.utils = Utils()
        
        if len(robot_files) > 1:
            if robot_file == None:
                print "Error: Multiple robot files found and no robot file provided"
                return
        if len(environment_files) > 1:
            if environment_file == None:
                print "Error: Multiple environment files found and no environment file provided" 
                return
        self.robot_file = None
        if len(robot_files) == 1:
            self.robot_file = robot_files[0]
        else:
            for i in xrange(len(robot_files)):
                if robot_file in robot_files[i]:
                    self.robot_file = robot_files[i]
        if self.robot_file == None:
            print "Error loading robot file"
            return
        
        self.environment_file = None
        if len(environment_files) == 1:
            self.environment_file = environment_files[0]
        else:
            for i in xrange(len(environment_files)):
                if environment_file in environment_files[i]:
                    self.environment_file = environment_files[i]
        if self.environment_file == None:
            print "Error loading environment file"
            return
        
        if play_failed == True and play_success == True:
            print "Error: Can't play only successful and only failed runs at the same time"
            return
        
        self.viewer_initialized = False
        self.draw_base_link = draw_base_link
        if algorithm == None:
            print "No algorithm provided. Use the -a flag"
            return
        if dir == None:
            print "No directory provided. Use the -d flag"
            return
        elif not os.path.isdir(dir):
            print "Provided directory doesn't exist"
            return       
        if numb == None:
            numb = 0
        if not self.init_robot():
            return
        if not self.init_environment():
            return
        print "PLAY"
        
        self.user_input = user_input
        self.play_runs(dir, 
                       algorithm,
                       particles,
                       particle_limit,
                       min_max,                       
                       numb, 
                       play_success,
                       play_failed, 
                       nominal_trajectory,
                       colliding_states,
                       covariance)
        
    def is_terminal(self, state):
        ja = v_double()
        ja[:] = [state[j] for j in xrange(len(state) / 2)]
        ee_position_arr = v_double()
        self.robot.getEndEffectorPosition(ja, ee_position_arr)
        ee_position = np.array([ee_position_arr[j] for j in xrange(len(ee_position_arr))])
        norm = np.linalg.norm(ee_position - self.goal_position)               
        if norm - 0.01 <= self.goal_radius:                       
            return True
        return False        
        
    def play_runs(self, 
                  dir, 
                  algorithm,
                  play_particles,
                  particle_limit, 
                  min_max,                 
                  numb,
                  play_success, 
                  play_failed, 
                  nominal_trajectory,
                  colliding_states,
                  covariance):      
        files = glob.glob(os.path.join(os.path.join(dir, "*.log")))
        first_particle = 1
        log_files = []
        for file in files:
            file_str = file.split("/")[-1]            
            if algorithm in file_str:
                if covariance != None:
                    if str(covariance) in file_str:                        
                        log_files.append(file) 
                else:
                    log_files.append(file)
        with open(sorted(log_files)[numb]) as f:
            states = []
            all_particles = []
            particles = []
            nominal_states = []
            col = []
            col_obstacles = []
            coll_states = []
            terminal = False
            collided = False
            for line in f:                
                if "S: " in line:
                    line_arr = line.rstrip("\n ").split(" ")
                    state = np.array([float(line_arr[i]) for i in xrange(1, len(line_arr))])
                    states.append(state)
                    coll_states.append(None)                    
                elif "S_NOMINAL: " in line and nominal_trajectory:
                    line_arr = line.rstrip("\n ").split(" ")                    
                    state = np.array([float(line_arr[i]) for i in xrange(1, len(line_arr))])
                    nominal_states.append(state)
                elif "Terminal:" in line:
                    term_line = line.rstrip("\n ").split(": ")[1]
                    if term_line == 'true':
                        terminal = True
                elif "collided: " in line or "Collision detected" in line:                    
                    if not "estimate" in line:
                        if "collided: " in line:
                            if line.rstrip("\n ").split(": ")[1] == "true":
                                col.append(True)
                                collided = True
                            else:
                                col.append(False)
                        elif "Collision detected" in line:
                            if line.rstrip("\n").split(": ")[2] == "True":                                
                                col.append(True)
                                collided = True
                            else:
                                col.append(False)
                elif "colliding obstacle:" in line:
                    col_obstacles.append(line.split(":")[1].strip())  
                elif "colliding state:" in line or "Colliding state:" in line:                                        
                    coll_state_str = line.split(":")[1].strip()
                    coll_state = None
                    if not coll_state_str == "None" and colliding_states:
                        coll_state = np.array([float(k) for k in coll_state_str.split(" ")])                    
                    coll_states[-1] = coll_state                                   
                elif "PARTICLES BEGIN" in line:
                    particles = []
                elif "PARTICLES END" in line:
                    all_particles.append(particles)
                elif "p: " in line and not "step" in line:
                    line_arr = line.rstrip("\n").split(": ")[1].split(" ")
                    try:
                        particle = np.array([float(line_arr[i]) for i in xrange(len(line_arr))])
                    except Exception as e:
                        print e
                        print line_arr
                        print line
                        return
                    particles.append(particle)                   
                elif "Final State:" in line:
                    line_arr = line.rstrip("\n ").split(": ")[1].split(" ")
                    state = np.array([float(line_arr[i]) for i in xrange(len(line_arr))])
                    states.append(state)
                    terminal = self.is_terminal(state)
                elif "S_ESTIMATED" in line:
                    first_particle = 0                                      
                    particles = []
                    line_arr = line.rstrip("\n").split(":")[1].strip(" ").split(" ")
                    particle = [float(line_arr[i]) for i in xrange(len(line_arr))]
                    all_particles.append([particle])
                elif ("RUN #" in line or 
                      "Run #" in line or
                      "#####" in line) and len(states) != 0:                    
                    if play_failed:                        
                        if terminal == False:
                            self.show_nominal_path(nominal_states)                            
                            self.play_states(states, 
                                             col,
                                             col_obstacles,
                                             coll_states,
                                             all_particles,
                                             play_particles,
                                             particle_limit,
                                             min_max,                                             
                                             first_particle)
                    elif play_success:
                        if terminal == True and collided == False:
                            self.show_nominal_path(nominal_states)                            
                            self.play_states(states, 
                                             col,
                                             col_obstacles,
                                             coll_states,
                                             all_particles,
                                             play_particles,
                                             particle_limit,
                                             min_max,                                             
                                             first_particle)
                    else:
                        
                        self.show_nominal_path(nominal_states)                        
                        self.play_states(states, 
                                         col, 
                                         col_obstacles,
                                         coll_states, 
                                         all_particles,
                                         play_particles,
                                         particle_limit,
                                         min_max,                                          
                                         first_particle)
                    states = []
                    coll_states = []
                    nominal_states = []
                    col = []
                    all_particles = []
                    collided = False
                    
    def drawBaseLink(self):                       
        joint_names = v_string()
        self.robot.getJointNames(joint_names)                
        first_joint_name = v_string()
        first_joint_name.append(joint_names[0])
        joint_pose = v2_double()
        self.robot.getJointOrigin(first_joint_name, joint_pose)        
        box_dims = v_double()
        box_name = "base_"
        """
        xy-coordinates
        """
        box_dims.extend([joint_pose[0][i] for i in xrange(2)])
        box_dims.append(0.0)        
        box_dims.extend([0.1, 0.1, joint_pose[0][2]])
        self.robot.drawBox(box_name, box_dims)
        
    def init_viewer(self):
        if not self.viewer_initialized:
            print "SET"            
            self.robot.setViewerBackgroundColor(0.6, 0.8, 0.6)
            self.robot.setViewerSize(1280, 768)
            self.robot.setupViewer(self.robot_file, self.environment_file)
            self.viewer_initialized = True
            if self.draw_base_link:                
                self.drawBaseLink()       
                    
    def show_nominal_path(self, path):
        """ Shows the nominal path in the viewer """
        self.init_viewer()
        self.robot.removePermanentViewerParticles()
        particle_joint_values = v2_double()
        particle_joint_colors = v2_double()
        pjvs = []
        for p in path:
            particle = v_double()
            particle[:] = [p[i] for i in xrange(len(p) / 2)]
            pjvs.append(particle)
                
            color = v_double()
            color[:] = [0.0, 0.0, 0.0, 0.7]
            particle_joint_colors.append(color)            
        particle_joint_values[:] = pjvs
        self.robot.addPermanentViewerParticles(particle_joint_values,
                                               particle_joint_colors)
        
    def getMinMaxParticles(self, particles):
        dim = len(particles[0][0]) / 2
        new_particles = []
        for i in xrange(len(particles)):
            mins = [particles[i][0][k] for k in xrange(dim)]
            maxs = [particles[i][0][k] for k in xrange(dim)]
            min_particles = [particles[i][0] for k in xrange(dim)]
            max_particles = [particles[i][0] for k in xrange(dim)]
            for j in xrange(len(particles[i])):
                for k in xrange(dim):
                    if particles[i][j][k] < mins[k]:
                        mins[k] = particles[i][j][k]
                        min_particles[k] = particles[i][j]
                    if particles[i][j][k] > maxs[k]:
                        maxs[k] = particles[i][j][k]
                        max_particles[k] = particles[i][j]
            min_particles.extend(max_particles)
            new_particles.append(min_particles)
        return new_particles
                    
    def play_states(self, 
                    states, 
                    col, 
                    col_obstacles,
                    coll_states, 
                    particles,
                    play_particles,
                    particle_limit,
                    min_max,                    
                    first_particle):
        self.init_viewer()
        if min_max == True:
            particles = self.getMinMaxParticles(particles)
        self.robot.setParticlePlotLimit(particle_limit + 1)        
        for i in xrange(len(states)):
            cjvals = v_double()
            cjvels = v_double()
            cjvals_arr = [states[i][j] for j in xrange(len(states[i]) / 2)]
            cjvels_arr = [states[i][j] for j in xrange(len(states[i]) / 2, len(states[i]))]
            cjvals[:] = cjvals_arr
            cjvels[:] = cjvels_arr            
            particle_joint_values = v2_double()
            particle_joint_colors = v2_double()            
            if i >= 1 and len(particles) > 0 and play_particles == True:
                for k in xrange(len(particles[i - first_particle])):
                    if k < particle_limit:
                        particle = v_double()
                        particle_color = v_double()
                        particle_vec = [particles[i - first_particle][k][t] for t in xrange(len(particles[i - first_particle][k]) / 2)]                        
                        particle[:] = particle_vec
                        particle_color[:] = [0.2, 0.8, 0.5, 0.2]
                        particle_joint_values.append(particle)
                        particle_joint_colors.append(particle_color)
            if not i == len(coll_states) and coll_states[i] != None:                
                part = v_double()
                part[:] = [coll_states[i][k] for k in xrange(len(coll_states[i]))]
                particle_color = v_double()
                particle_color[:] = [0.0, 0.0, 1.0, 0.0]
                particle_joint_values.append(part)
                particle_joint_colors.append(particle_color)
            self.robot.updateViewerValues(cjvals, 
                                          cjvels,
                                          particle_joint_values,
                                          particle_joint_colors)
            for o in self.obstacles:
                self.robot.setObstacleColor(o.getName(), 
                                            o.getStandardDiffuseColor(),
                                            o.getStandardAmbientColor())            
            try:                
                if col_obstacles[i] != None:                    
                    diffuse_col = v_double()
                    ambient_col = v_double()
                    diffuse_col[:] = [0.5, 0.0, 0.0, 0.0]
                    ambient_col[:] = [0.8, 0.0, 0.0, 0.0]
                    for o in self.obstacles:
                        if o.getName() == col_obstacles[i]:
                            self.robot.setObstacleColor(o.getName(), 
                                                    diffuse_col, 
                                                    ambient_col)
                            break
            except:
                pass
                         
            
            if self.user_input:
                raw_input("Press Enter to continue...")
            else:
                time.sleep(0.1)
            
    def is_in_collision(self, previous_state, state):
        """
        Is the given end effector position in collision with an obstacle?
        """
        collidable_obstacles = [o for o in self.obstacles if not o.isTraversable()]
        joint_angles_goal = v_double()
        joint_angles_goal[:] = [state[i] for i in xrange(self.robot_dof)]
        print previous_state
        print state
        collision_objects_goal = self.robot.createRobotCollisionObjects(joint_angles_goal)        
        if len(previous_state) > 0:
            """
            Perform continuous collision checking if previous state is provided
            """            
            joint_angles_start = v_double()        
            joint_angles_start[:] = [previous_state[i] for i in xrange(self.robot_dof)]
            collision_objects_start = self.robot.createRobotCollisionObjects(joint_angles_start)
            
            for obstacle in collidable_obstacles:                
                for i in xrange(len(collision_objects_start)):                    
                    if obstacle.inCollisionContinuous([collision_objects_start[i], collision_objects_goal[i]]):
                        print "return true"
                        return True, obstacle
        
            return False, None
        else:            
            for obstacle in collidable_obstacles:
                if obstacle.inCollisionDiscrete(collision_objects_goal):                               
                    return True, None
            return False, None  
        
    def init_robot(self):
        self.robot = Robot(self.robot_file)
        self.robot_dof = self.robot.getDOF()        
        return True  
    
    def init_environment(self):
        self.obstacles = self.utils.loadObstaclesXML(self.environment_file)
        goal_area = v_double()
        self.utils.loadGoalArea(self.environment_file, goal_area)
        if len(goal_area) == 0:
            print "ERROR: Your environment file doesn't define a goal area"
            return False
        self.goal_position = [goal_area[i] for i in xrange(0, 3)]
        self.goal_radius = goal_area[3]
        return True    
        
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='LQG-MP.')
    parser.add_argument("-a", "--algorithm", help="The algorithm to play")
    parser.add_argument("-d", "--directory", help="The directory of the logfiles")
    parser.add_argument("-n", "--numb", nargs='?', help="The number of covariance value to play", type=int, const=0)
    parser.add_argument("-cov", "--covariance", nargs="?", help="play runs from covariance value", type=float, const=0)
    parser.add_argument("-nt", "--nominal_trajectory", 
                        help="Show the nominal trajectory", 
                        action="store_true")
    
    parser.add_argument("-f", "--play_failed", 
                        help="Play only the failed runs", 
                        action="store_true")
    parser.add_argument("-s", "--play_success",
                        help="Play only the successful runs",
                        action="store_true")
    parser.add_argument("-p", "--particles",
                        help="Show particles",
                        action="store_true")
    parser.add_argument("-u", "--user_input",
                        help="Wait for user input",
                        action="store_true")
    parser.add_argument("-cs", "--colliding_states",
                        help="Show the colliding states",
                        action="store_true") 
    parser.add_argument("-r", "--robot_file",
                        help="The robot file to use")
    parser.add_argument("-e", "--environment_file",
                        help="The environment file to use") 
    parser.add_argument("-pl", "--particle_limit",
                        nargs="?",
                        help="The number of particles to plot",
                        type=int,
                        const=0,
                        default=50)
    parser.add_argument("-b", "--draw_base",
                        help="Draw base link",
                        action="store_true")
    parser.add_argument("-mm", "--min_max",
                        help="Show min/max particles",
                        action="store_true") 
    args = parser.parse_args()
    if args.algorithm == None:
        print "Error: No algorithm provided. Run 'python play_video.py --help' for command line options"
        sys.exit()
    if args.directory == None:
        print "Error: No directory for the logs provided. Run 'python play_video.py --help' for command line options" 
        sys.exit()
    
    Play(args.algorithm, 
         args.directory, 
         args.numb,
         args.particles, 
         args.play_failed,
         args.play_success, 
         args.user_input,  
         args.nominal_trajectory, 
         args.colliding_states,
         args.covariance,
         args.particle_limit,
         args.min_max,
         args.draw_base,
         args.robot_file,
         args.environment_file)
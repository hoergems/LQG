import sys
import numpy as np
import scipy
import plot as Plot
import glob
import os
import logging
from serializer import Serializer
from scipy.stats import multivariate_normal
from libkinematics import *
from libutil import *
from EMD import *
import sets
import random

class PlotStats:
    def __init__(self, save, algorithm):
        dir = "stats/" + str(algorithm)
        if not os.path.isdir(dir):
            os.makedirs(dir)       
        self.save = save        
        serializer = Serializer()
        self.process_covariances = []
        for logfile in glob.glob(dir + "/*.log"):
            self.process_covariances.append(serializer.read_process_covariance(logfile))
        self.setup_kinematics(serializer, dir=dir)        
        self.create_video(serializer, dir)
        print "Setting up Kinematics"
        
        print "Kinematics setup"
        logging.info("PlotStats: plotting paths")    
        self.plot_paths(serializer, dir=dir)
        self.plot_paths(serializer, best_paths=True, dir=dir)
        
        logging.info("PlotStats: plotting average distance to goal")
        self.plot_stat("Average distance to goal area", "avg_distance", dir=dir)
        logging.info("PlotStats: plotting mean rewards")
        self.plot_stat("Mean rewards", "mean_rewards", dir=dir)
        logging.info("PlotStats: plotting % successful runs")
        self.plot_stat("Num success", "succesful_runs", dir=dir)
        self.plot_stat("Reward variance", "sample_variances", dir=dir)
        self.plot_stat("Reward standard deviation", "reward_standand_deviation", dir=dir)        
        logging.info("PlotStats: plotting mean planning times")
        self.plot_stat("Mean planning time", "mean_planning_time", dir=dir) 
        self.plot_stat("Mean num collisions per run", "mean_num_collision_per_run", dir=dir)
        logging.info("PlotStats: plotting mean number of generated paths")
        self.plot_mean_num_generated_paths(serializer,
                                           dir,
                                           filename="mean_num_generated_paths_per_step*.yaml",
                                           output="mean_num_generated_paths_per_step.pdf")
        self.plot_mean_num_generated_paths(serializer,
                                           dir,
                                           filename="mean_num_generated_paths_per_run*.yaml",
                                           output="mean_num_generated_paths_per_run.pdf")
        logging.info("PlotStats: plotting mean number of steps per run")
        self.plot_mean_num_steps(serializer,
                                 dir,
                                 filename="mean_num_planning_steps_per_run*.yaml",
                                 output="mean_num_planning_steps_per_run.pdf")
        self.plot_mean_num_steps(serializer,
                                 dir,
                                 filename="mean_num_steps_per_run*.yaml",
                                 output="mean_num_steps_per_run.pdf")
        cart_coords = serializer.load_cartesian_coords(dir, "cartesian_coords_" + algorithm + ".yaml")        
        logging.info("PlotStats: plotting EMD graph...")
       
        self.plot_emd_graph(serializer, cart_coords, dir=dir)
        logging.info("PlotStats: plotting histograms...")        
        self.save_histogram_plots(serializer, cart_coords, dir=dir)
        
    def setup_kinematics(self, serializer, dir='stats'):
        config = serializer.read_config(path=dir)
        u = Utils()
        model_file = os.getcwd() + "/" + dir + "/model/model.xml"
        if config['workspace_dimension'] == 3:
            model_file = os.getcwd() + "/" + dir + "/model/model3D.xml"
        print "model file: " + model_file 
        link_dimensions = u.getLinkDimensions(model_file)       
        axis = v2_int()
        ax1 = v_int()
        ax2 = v_int()
        ax1[:] = [0, 0, 1]
        if config['workspace_dimension'] == 2:
            ax2[:] = [0, 0, 1]            
        elif config['workspace_dimension'] == 3:
            ax2[:] = [0, 1, 0]
        axis[:] = [ax1, ax2, ax1]
        self.kinematics = Kinematics()
        self.kinematics.setLinksAndAxis(link_dimensions, axis)
               
        
    def clear_stats(self):
        for file in glob.glob("stats/*"):
            os.remove(file)
            
    def plot_paths(self, serializer, best_paths=False, dir="stats"):
        config = serializer.read_config(path=dir)
        utils = Utils()
        model_file = "model.xml" 
        if config['workspace_dimension'] == 3:
            model_file = "model3D.xml"
        link_dimensions = utils.getLinkDimensions(os.getcwd() + "/" + dir + "/model/" + model_file)
        try:
            if config['plot_paths']:
                dim = len(link_dimensions)        
                colors = []
                if best_paths:
                    paths = serializer.load_paths("best_paths.yaml", path=dir)
                    filename = "best_paths.png"
                else:
                    paths = serializer.load_paths("paths.yaml", path=dir)            
                    filename = "paths.png"
                sets = []
                for path in paths:
                                        
                    path_coords = []
                    for elem in path:
                        state = [elem[i] for i in xrange(dim)]
                        state_v = v_double()
                        state_v[:] = state
                        path_coords_v = self.kinematics.getEndEffectorPosition(state_v)
                        path_coords_elem = [path_coords_v[i] for i in xrange(len(path_coords_v))]
                        path_coords.append(path_coords_elem)
                    sets.append(np.array(path_coords))
                    
                    colors.append(None)                
                max_x = -100000.0 
                max_y = -100000.0                
                for s in sets:                    
                    for c in s:                                                
                        if c[0] > max_x:
                            max_x = c[0]                        
                        if c[1] > max_y:
                            max_y = c[1]
                obstacles = serializer.load_environment("env.xml", path=dir + "/environment")        
                if not obstacles == None:
                    for obstacle in obstacles:                
                        point1 = [obstacle[0][0] - obstacle[1][0], obstacle[0][1] - obstacle[1][1]]
                        point2 = [obstacle[0][0] - obstacle[1][0], obstacle[0][1] + obstacle[1][1]]
                        point3 = [obstacle[0][0] + obstacle[1][0], obstacle[0][1] + obstacle[1][1]]
                        point4 = [obstacle[0][0] + obstacle[1][0], obstacle[0][1] - obstacle[1][1]]
                        sets.append(np.array([point1, point2]))                
                        sets.append(np.array([point2, point3]))
                        sets.append(np.array([point3, point4]))
                        sets.append(np.array([point4, point1])) 
                        colors.extend(['k' for j in xrange(4)])                       
                Plot.plot_2d_n_sets(sets,
                                    colors=colors, 
                                    xlabel='x', 
                                    ylabel='y', 
                                    x_range=[-max_x * 1.1, max_x * 1.1], 
                                    y_range=[-max_y * 1.1, max_y * 1.1],
                                    plot_type="lines",
                                    show_legend=False,
                                    save=self.save,
                                    path=dir,
                                    filename=filename)
        except Exception as e:
            print e
            
    def draw_particles(self, mean, cov, size):
        samples = multivariate_normal.rvs(mean, cov, size)
        return samples
            
    def create_video(self, serializer, dir='stats'):
        try:
            os.makedirs(dir + "/mov")
        except Exception as e:
            print e
        config = serializer.read_config(path=dir)
        utils = Utils()
        model_file = "model.xml" 
        if config['workspace_dimension'] == 3:
            model_file = "model3D.xml"
        link_dimensions = utils.getLinkDimensions(os.getcwd() + "/" + dir + "/model/" + model_file)        
        map_size = sum([link_dimension[0] for link_dimension in link_dimensions])
        environment = serializer.load_environment(path=dir + "/environment")        
        sets = []
        color_map = []
        plot_particles = True
        for obstacle in environment:
            point1 = [obstacle[0][0] - obstacle[1][0], obstacle[0][1] - obstacle[1][1], obstacle[0][2] - obstacle[1][2]]
            point2 = [obstacle[0][0] - obstacle[1][0], obstacle[0][1] - obstacle[1][1], obstacle[0][2] + obstacle[1][2]]
            point3 = [obstacle[0][0] - obstacle[1][0], obstacle[0][1] + obstacle[1][1], obstacle[0][2] - obstacle[1][2]]
            point4 = [obstacle[0][0] - obstacle[1][0], obstacle[0][1] + obstacle[1][1], obstacle[0][2] + obstacle[1][2]]
            point5 = [obstacle[0][0] + obstacle[1][0], obstacle[0][1] - obstacle[1][1], obstacle[0][2] - obstacle[1][2]]
            point6 = [obstacle[0][0] + obstacle[1][0], obstacle[0][1] - obstacle[1][1], obstacle[0][2] + obstacle[1][2]]
            point7 = [obstacle[0][0] + obstacle[1][0], obstacle[0][1] + obstacle[1][1], obstacle[0][2] - obstacle[1][2]]
            point8 = [obstacle[0][0] + obstacle[1][0], obstacle[0][1] + obstacle[1][1], obstacle[0][2] + obstacle[1][2]]
            if config['workspace_dimension'] == 2:
                sets.append(np.array([point1, point3]))
                sets.append(np.array([point3, point7]))
                sets.append(np.array([point7, point5]))
                sets.append(np.array([point5, point1]))
                color_map.extend(['#000000' for t in xrange(4)])
            elif config['workspace_dimension'] == 3:
                sets.append(np.array([point1, point3]))
                sets.append(np.array([point3, point7]))
                sets.append(np.array([point7, point5]))
                sets.append(np.array([point5, point1]))
                                
                sets.append(np.array([point2, point4]))
                sets.append(np.array([point4, point8]))
                sets.append(np.array([point8, point6]))
                sets.append(np.array([point6, point2]))
                                
                sets.append(np.array([point1, point2]))
                sets.append(np.array([point3, point4]))
                sets.append(np.array([point7, point8]))
                sets.append(np.array([point5, point6]))
                color_map.extend(['#000000' for t in xrange(12)])
        balls = [[config['goal_position'], config["goal_radius"]]]
        color_map.extend(['green' for i in xrange(len(balls))])
        state = None
        cov_num  = -1
        for process_covariance in self.process_covariances:
            for file in glob.glob(dir + "/*.log"):                
                if str(process_covariance) in file:
                    cov_num += 1
                    run_num = -1
                    step_num = -1
                    with open(file, 'r') as f:                        
                        for line in f:                                                       
                            if "RUN #" in line:                                
                                run_num += 1
                                step_num = -1                 
                            elif "S:" in line:
                                line_arr = line.split(":")[1].strip().split(" ")
                                state = [float(line_arr[j]) for j in xrange(len(link_dimensions))] 
                            elif "S_ESTIMATED:" in line:
                                line_arr = line.split(":")[1].strip().split(" ")
                                estimated_state = [float(line_arr[j]) for j in xrange(len(link_dimensions))]                           
                            elif "COVARIANCE:" in line:
                                temp_sets = []
                                color_map_temp = []
                                step_num += 1  
                                if plot_particles:
                                    covariance = line.split(":")[1].strip().split(" ")
                                    
                                    covariance = [float(cov) for cov in covariance]
                                    covariance = np.array([[covariance[0], covariance[1], covariance[2]],
                                                           [covariance[6], covariance[7], covariance[8]],
                                                           [covariance[12], covariance[13], covariance[14]]])                                    
                                    particles = self.draw_particles(estimated_state, covariance, 50)                                    
                                    for particle in particles:
                                        angles = v_double()
                                        angles[:] = particle
                                        link_1_position = self.kinematics.getPositionOfLinkN(angles, 1)
                                        link_2_position = self.kinematics.getPositionOfLinkN(angles, 2)
                                        link_3_position = self.kinematics.getPositionOfLinkN(angles, 3)
                                        temp_sets.append(np.array([[0.0, 0.0, 0.0], [link_1_position[0], link_1_position[1], link_1_position[2]]]))
                                        temp_sets.append(np.array([[link_1_position[0], link_1_position[1], link_1_position[2]], 
                                                                   [link_2_position[0], link_2_position[1], link_2_position[2]]]))
                                        temp_sets.append(np.array([[link_2_position[0], link_2_position[1], link_2_position[2]], 
                                                                   [link_3_position[0], link_3_position[1], link_3_position[2]]]))
                                        color_map_temp.extend(['#aaabbb' for t in xrange(3)])                                                              
                                angles = v_double()
                                angles[:] = state
                                img_filename = "img_" + str(cov_num) + "_" + str(run_num) + "_" + str(step_num) + ".png"
                                link_1_position = self.kinematics.getPositionOfLinkN(angles, 1)
                                link_2_position = self.kinematics.getPositionOfLinkN(angles, 2)
                                link_3_position = self.kinematics.getPositionOfLinkN(angles, 3)
                                temp_sets.append(np.array([[0.0, 0.0, 0.0], [link_1_position[0], link_1_position[1], link_1_position[2]]]))
                                temp_sets.append(np.array([[link_1_position[0], link_1_position[1], link_1_position[2]], 
                                                          [link_2_position[0], link_2_position[1], link_2_position[2]]]))
                                temp_sets.append(np.array([[link_2_position[0], link_2_position[1], link_2_position[2]], 
                                                          [link_3_position[0], link_3_position[1], link_3_position[2]]]))
                                color_map_temp.extend(['#0000ff' for n in xrange(3)])                                
                                
                                angles = v_double()
                                angles[:] = estimated_state
                                link_1_position = self.kinematics.getPositionOfLinkN(angles, 1)
                                link_2_position = self.kinematics.getPositionOfLinkN(angles, 2)
                                link_3_position = self.kinematics.getPositionOfLinkN(angles, 3)
                                temp_sets.append(np.array([[0.0, 0.0, 0.0], [link_1_position[0], link_1_position[1], link_1_position[2]]]))
                                temp_sets.append(np.array([[link_1_position[0], link_1_position[1], link_1_position[2]], 
                                                          [link_2_position[0], link_2_position[1], link_2_position[2]]]))
                                temp_sets.append(np.array([[link_2_position[0], link_2_position[1], link_2_position[2]], 
                                                          [link_3_position[0], link_3_position[1], link_3_position[2]]]))
                                color_map_temp.extend(['#ff0000' for n in xrange(3)])
                                temp_sets.extend(sets)
                                color_map_temp.extend(color_map)        
                                                          
                                if config['workspace_dimension'] == 2:
                                    circles = []
                                    for ball in balls:
                                        circles.append([ball[0][0], ball[0][1], ball[1]])
                                    Plot.plot_2d_n_sets(temp_sets,
                                                        circles=circles,
                                                        xlabel="x",
                                                        ylabel="y",
                                                        x_range=[-map_size * 1.1, map_size * 1.1], 
                                                        y_range=[-map_size * 1.1, map_size * 1.1],
                                                        plot_type="lines",
                                                        show_legend=False,
                                                        color_map=color_map_temp,
                                                        save=self.save,
                                                        path=dir + "/mov",
                                                        filename=img_filename)
        sleep 
    
    def plot_particles(self, serializer, particle_limit=0):
        config = serializer.read_config('config.yaml', path="stats")
        for file in glob.glob(os.path.join("stats", "particles*.png")):
            os.remove(file)
        if config['plot_particles']:
            particles = serializer.load("particles.yaml", path="stats")
        state_paths = serializer.load("state_paths.yaml", path="stats") 
        
        link_dimensions = self.utils.getLinkDimensions(os.getcwd() + "/stats/model/model.xml")
        map_size = sum([link_dimension[0] for link_dimension in link_dimensions])
        axis = v2_int()
        ax1 = v_int()
        ax2 = v_int()
        ax1[:] = [0, 0, 1]
        if config['workspace_dimensions'] == 2:
            ax2[:] = [0, 0, 1]            
        elif config['workspace_dimensions'] == 3:
            ax2[:] = [0, 1, 0]
            
        axis[:] = [ax1, ax2, ax1] 
        kinematics = Kinematics()
        kinematics.setLinksAndAxis(link_dimensions, axis)        
        environment = serializer.load_environment()        
            
        for file in glob.glob(os.path.join("stats", "particles*.png")):
            os.remove(file)
                
        sets = [] 
        color_map = []
        for obstacle in environment:
            point1 = [obstacle[0][0] - obstacle[1][0], obstacle[0][1] - obstacle[1][1], obstacle[0][2] - obstacle[1][2]]
            point2 = [obstacle[0][0] - obstacle[1][0], obstacle[0][1] - obstacle[1][1], obstacle[0][2] + obstacle[1][2]]
            point3 = [obstacle[0][0] - obstacle[1][0], obstacle[0][1] + obstacle[1][1], obstacle[0][2] - obstacle[1][2]]
            point4 = [obstacle[0][0] - obstacle[1][0], obstacle[0][1] + obstacle[1][1], obstacle[0][2] + obstacle[1][2]]
            point5 = [obstacle[0][0] + obstacle[1][0], obstacle[0][1] - obstacle[1][1], obstacle[0][2] - obstacle[1][2]]
            point6 = [obstacle[0][0] + obstacle[1][0], obstacle[0][1] - obstacle[1][1], obstacle[0][2] + obstacle[1][2]]
            point7 = [obstacle[0][0] + obstacle[1][0], obstacle[0][1] + obstacle[1][1], obstacle[0][2] - obstacle[1][2]]
            point8 = [obstacle[0][0] + obstacle[1][0], obstacle[0][1] + obstacle[1][1], obstacle[0][2] + obstacle[1][2]]
            if config['workspace_dimensions'] == 2:
                sets.append(np.array([point1, point3]))
                sets.append(np.array([point3, point7]))
                sets.append(np.array([point7, point5]))
                sets.append(np.array([point5, point1]))
                color_map.extend(['#000000' for t in xrange(4)])
            elif config['workspace_dimensions'] == 3:
                sets.append(np.array([point1, point3]))
                sets.append(np.array([point3, point7]))
                sets.append(np.array([point7, point5]))
                sets.append(np.array([point5, point1]))
                                
                sets.append(np.array([point2, point4]))
                sets.append(np.array([point4, point8]))
                sets.append(np.array([point8, point6]))
                sets.append(np.array([point6, point2]))
                                
                sets.append(np.array([point1, point2]))
                sets.append(np.array([point3, point4]))
                sets.append(np.array([point7, point8]))
                sets.append(np.array([point5, point6]))
                color_map.extend(['#000000' for t in xrange(12)])
        balls = [[config['goal_position'], config["goal_radius"]]]
        color_map.extend(['green' for i in xrange(len(balls))])       
        for i in xrange(len(state_paths)):
            for j in xrange(len(state_paths[i])):                
                for k in xrange(len(state_paths[i][j])):
                    temp_sets = []
                    color_map_temp = []
                    if config['plot_particles'] and not k == 0:
                        for l in xrange(len(particles[i][j][k - 1])):
                            if not particle_limit == 0:
                                if l > particle_limit + 1:
                                    continue
                            angles = v_double()
                            angles[:] = particles[i][j][k - 1][l]
                            link_1_position = kinematics.getPositionOfLinkN(angles, 1)
                            link_2_position = kinematics.getPositionOfLinkN(angles, 2)
                            link_3_position = kinematics.getPositionOfLinkN(angles, 3)
                            temp_sets.append(np.array([[0.0, 0.0, 0.0], [link_1_position[0], link_1_position[1], link_1_position[2]]]))
                            temp_sets.append(np.array([[link_1_position[0], link_1_position[1], link_1_position[2]], 
                                                      [link_2_position[0], link_2_position[1], link_2_position[2]]]))
                            temp_sets.append(np.array([[link_2_position[0], link_2_position[1], link_2_position[2]], 
                                                      [link_3_position[0], link_3_position[1], link_3_position[2]]]))
                            color_map_temp.extend(['#aaabbb' for t in xrange(3)])                                                        
                        angles = v_double();
                        angles[:] = state_paths[i][j][k]   
                        link_1_position = kinematics.getPositionOfLinkN(angles, 1)
                        link_2_position = kinematics.getPositionOfLinkN(angles, 2)
                        link_3_position = kinematics.getPositionOfLinkN(angles, 3)
                            
                        temp_sets.append(np.array([[0.0, 0.0, 0.0], [link_1_position[0], link_1_position[1], link_1_position[2]]]))
                        temp_sets.append(np.array([[link_1_position[0], link_1_position[1], link_1_position[2]], 
                                                  [link_2_position[0], link_2_position[1], link_2_position[2]]]))
                        temp_sets.append(np.array([[link_2_position[0], link_2_position[1], link_2_position[2]], 
                                                  [link_3_position[0], link_3_position[1], link_3_position[2]]]))
                        color_map_temp.extend(['#0000ff' for t in xrange(3)])
                        
                        """
                        Plot the goal area
                        """                       
                        
                        temp_sets.extend(sets)
                        color_map_temp.extend(color_map)                       
                        if config['workspace_dimensions'] == 2:
                            circles = []
                            for ball in balls:
                                circles.append([ball[0][0], ball[0][1], ball[1]])
                            Plot.plot_2d_n_sets(temp_sets,
                                                circles=circles,
                                                xlabel="x",
                                                ylabel="y",
                                                x_range=[-map_size * 1.1, map_size * 1.1], 
                                                y_range=[-map_size * 1.1, map_size * 1.1],
                                                plot_type="lines",
                                                show_legend=False,
                                                color_map=color_map_temp,
                                                save=self.save,
                                                path="stats",
                                                filename="particles" + str(i) + "_" + str(j) + "_" + str(k) + ".png")
                        elif config['workspace_dimensions'] == 3:
                             Plot.plot_3d_n_sets(sets=temp_sets,
                                                balls=balls, 
                                                colormap=color_map_temp, 
                                                show_legend=False, 
                                                save=self.save, 
                                                path="stats", 
                                                filename="particles" + str(i) + "_" + str(j) + "_" + str(k) + ".png")
    
    def plot_stat(self, stat_str, output_file_str, dir="stats"):
        files = glob.glob(os.path.join(os.path.join(dir, "*.log")))     
        num_succ_runs = [] 
        num_succ_runs_sets = [] 
        labels = []
        m_covs = []
        data = []
        color_map = []
        d = dict()
        for file in sorted(files):
            file_str = file.split("/")[2].split("_")[1]
            if not file_str in d:
                d[file_str] = []                 
            m_cov = 0.0
            succ = 0
            with open(file, "r") as f:                
                for line in f:
                    if "Process covariance:" in line:
                        m_cov = float(line.split(" ")[2])
                    elif stat_str in line:
                        float_found = False                        
                        succ_l = line.split(" ")
                        for s in succ_l:
                            if not float_found:
                                try:
                                    succ = float(s)
                                    float_found = True
                                except:
                                    pass
            m_covs.append(m_cov)
            num_succ_runs.append(succ)
            d[file_str].append(np.array([m_cov, succ]))        
        for key in d:
            color_map.append(self.gen_random_color())
            num_succ_runs_sets.append(np.array(d[key]))
            labels.append(key)        
        
        min_m = min(num_succ_runs) 
        max_m = max(num_succ_runs) 
        min_cov = min(m_covs)
        max_cov = max(m_covs)
        Plot.plot_2d_n_sets(num_succ_runs_sets,
                            labels=labels,
                            xlabel="joint covariance",
                            ylabel=stat_str,
                            x_range=[min(m_covs), max(m_covs)],
                            y_range=[min_m, max_m * 1.05],
                            show_legend=True,
                            lw=2,
                            color_map=color_map,
                            save=self.save,
                            filename=dir + "/" + output_file_str + ".pdf")
        
    def gen_random_color(self):
        return "#%06x" % random.randint(0, 0xFFFFFF)
        
    def plot_mean_num_steps(self, serializer, dir="stats", filename="", output=""):
        if filename == "":
            filename = "mean_num_steps_per_run*.yaml"
        if output == "":
            output = "mean_num_steps_per_run.pdf" 
        print filename      
        stats = serializer.load_stats('stats.yaml', path=dir)
        m_cov = stats['m_cov']
        sets = []
        labels = []
        mean_num_steps = []
        for file in glob.glob(os.path.join(os.path.join(dir, filename))):
            file_str = file            
            try:
                file_str = file.split("/")[2].split(".")[0].split("_")[-1]
            except:
                pass
                   
            #mean_rewards = serializer.load_stats('rewards.yaml', path="stats")
            mean_num_steps.append(serializer.load_stats(file))            
            data = []
            for k in xrange(len(m_cov)):
                data.append(np.array([m_cov[k], mean_num_steps[-1][k]]))
            sets.append(np.array(data))
            labels.append(file_str)        
        if not len(mean_num_steps) == 0:
            min_m = [min(m) for m in mean_num_steps]
            max_m = [max(m) for m in mean_num_steps]
            Plot.plot_2d_n_sets(sets,
                                labels=labels,
                                xlabel="joint covariance",
                                ylabel="mean number of steps per run",
                                x_range=[m_cov[0], m_cov[-1]],
                                y_range=[min(min_m)*0.95, max(max_m) * 1.05],
                                show_legend=True,
                                save=self.save,
                                filename=dir + "/" + output)
        
    def plot_mean_num_generated_paths(self, serializer, dir="stats", filename="", output=""):
        if filename == "":
            filename = "mean_num_generated_paths_per_step*.yaml"
        if output == "":
            output = "mean_num_generated_paths_per_step.pdf" 
        print filename      
        stats = serializer.load_stats('stats.yaml', path=dir)
        m_cov = stats['m_cov']
        sets = []
        labels = []
        mean_num_generated_paths = []
        for file in glob.glob(os.path.join(os.path.join(dir, filename))):
            file_str = file            
            try:
                file_str = file.split("/")[2].split(".")[0].split("_")[-1]
            except:
                pass
                   
            #mean_rewards = serializer.load_stats('rewards.yaml', path="stats")
            mean_num_generated_paths.append(serializer.load_stats(file))            
            data = []
            for k in xrange(len(m_cov)):
                data.append(np.array([m_cov[k], mean_num_generated_paths[-1][k]]))
            sets.append(np.array(data))
            labels.append(file_str)        
        if not len(mean_num_generated_paths) == 0:
            min_m = [min(m) for m in mean_num_generated_paths]
            max_m = [max(m) for m in mean_num_generated_paths]
            Plot.plot_2d_n_sets(sets,
                                labels=labels,
                                xlabel="joint covariance",
                                ylabel="mean num generated paths",
                                x_range=[m_cov[0], m_cov[-1]],
                                y_range=[min(min_m)*0.95, max(max_m) * 1.05],
                                show_legend=True,
                                save=self.save,
                                filename=dir + "/" + output)
        
    def plot_emd_graph(self, serializer, cartesian_coords, dir="stats"):
        stats = serializer.load_stats('stats.yaml', path=dir) 
        config = serializer.read_config(path=dir)
        goal_position = config['goal_position']
        utils = Utils()
        model_file = "model.xml" 
        if config['workspace_dimension'] == 3:
            model_file = "model3D.xml"
        link_dimensions = utils.getLinkDimensions(os.getcwd() + "/" + dir + "/model/" + model_file)     
        #emd = stats['emd']
        m_cov = stats['m_cov']
        
        emds = []
        for k in xrange(len(cartesian_coords)):
            #cart_coords.append([cartesian_coords[i] for i in xrange(len(cartesian_coords))])                
            emds.append(calc_EMD(cartesian_coords[k], config['num_bins'], goal_position, link_dimensions))
        
        arr = np.array([np.array([m_cov[i], emds[i]]) for i in xrange(len(emds))])
        Plot.plot_2d_n_sets([arr], 
                            xlabel='joint covariance', 
                            ylabel='Wasserstein distance', 
                            x_range=[m_cov[0], m_cov[-1]], 
                            y_range=[0, max(emds)],
                            show_legend=False,
                            save=self.save,
                            path=dir,
                            filename="emd.png")        
        
    def save_histogram_plots(self, serializer, cart_coords, dir="stats"):
        config = serializer.read_config(path=dir)
        utils = Utils()
        model_file = "model.xml" 
        if config['workspace_dimension'] == 3:
            model_file = "model3D.xml"
        link_dimensions = utils.getLinkDimensions(os.getcwd() + "/" + dir + "/model/" + model_file)
        dim = sum([l[0] for l in link_dimensions])
        for k in xrange(len(cart_coords)):                    
            X = np.array([cart_coords[k][i][0] for i in xrange(len(cart_coords[0]))])
            Y = np.array([cart_coords[k][i][1] for i in xrange(len(cart_coords[0]))])
            histogram_range = [[-dim * 1.1, dim * 1.1], [-dim * 1.1, dim * 1.1]]
            H, xedges, yedges = get_2d_histogram(X, Y, histogram_range, bins=config['num_bins'])        
            Plot.plot_histogram(H, xedges, yedges, save=self.save, path=dir, filename="hist"+ str(k) + ".png")
            
    
    
        
if __name__ == "__main__":
    logging_level = logging.DEBUG
    logging.basicConfig(format='%(levelname)s: %(message)s', level=logging_level)
    if len(sys.argv) > 2:
        algorithm = sys.argv[1]
        if "save" in sys.argv[2]:
            PlotStats(True, algorithm)
            sys.exit()       
        PlotStats(False)
        sys.exit() 
    else:
        logging.error("Wrong number of arguments. Should be 'python plot_stats.py ALGORITHM SAVE'")
        sys.exit()   
    PlotStats(False)
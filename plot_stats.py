import sys
import numpy as np
import scipy
import plot as Plot
import glob
import os
import logging
from serializer import Serializer
from scipy.stats import multivariate_normal
#from librobot import *
#from libutil import *
from EMD import EMD
import sets
import random
import argparse
from emd import emd
from tabulate import tabulate

class PlotStats:
    def __init__(self, dir, save_plots, show_particles, plot_emds, collision_is_failed):        
        if not os.path.isdir(dir):
            print "Error: Directory doesn't exist"
            return
        self.cleanup(dir=dir)
        #self.config_file = glob.glob(os.path.join(dir + "/config_*"))[0]
        self.color_dict = dict()
        self.color_dict["abt"] = "#ff0000"
        self.color_dict["lqg"] = "#00ff00"
        self.color_dict["hfr"] = "#0000ff"  
        self.linestyles = dict()
        self.linestyles['abt'] = "solid"
        self.linestyles['lqg'] = "dashed"
        self.linestyles['hfr'] = 'dashdot'      
              
        self.save = save_plots        
        serializer = Serializer()
        
        '''if self.setup_robot(dir) and plot_emds:            
            self.plot_emds(show_particles, dir=dir)'''
        '''self.plot_estimate_error(dir=dir)
        self.plot_linearisation_error(dir=dir)
        return'''
               
        
        self.plot_num_succesful_runs("succ_stats", 
                                     dir=dir, 
                                     finish_when_collided=collision_is_failed)
        self.plot_stat_from_txt_file("succ_stats", 
                                     "num_succ", 
                                     dir=dir, 
                                     y_label="Succesful runs in %")
        reward_model = {}
        reward_model["step_penalty"] = -1.0
        reward_model["collision_penalty"] = -500.0
        reward_model["exit_reward"] = 1000.0
        self.plot_reward(reward_model, "reward_stats", dir=dir, finish_when_collided=collision_is_failed)        
        self.plot_stat_from_txt_file("reward_stats", 
                                     "mean_rewards", 
                                     dir=dir, 
                                     y_label="mean reward")
        #sleep    
        try:
            self.save_estimation_error("estimation_error_stats", dir=dir)
            self.plot_number_of_steps_stats("num_step_stats", dir=dir)
            self.plot_stat_from_txt_file("estimation_error_stats", 
                                         "mean_estimation_error", 
                                         dir=dir, 
                                         y_label="mean estimation error")
        except:
            print "WARNING: Estimation error stats could not be plotted"
        
        print "saving collision information"
        self.plot_bla(["collided", "Trans: Collision detected", "Collision"], "collision_stats", dir=dir)
        print "saving reward information"
        #self.plot_bla(["Reward:"], "reward_stats", dir=dir)
        
        
        self.to_latex_table(dir=dir) 
        
        
        logging.info("Plotting average distance to goal")
        try:        
            self.plot_stat("Average distance to goal area", "avg_distance", dir=dir)
        except:
            pass
        
        
        
        logging.info("Plotting mean number of histories per step")
        self.plot_stat("Mean number of histories per step", "mean_num_histories", dir=dir)
                 
        logging.info("Plotting mean rewards")
        #self.plot_stat("Mean rewards", "mean_rewards", dir=dir)        
        
        logging.info("Plotting number of succesful runs")
        self.plot_stat("Num success", "succesful_runs", dir=dir)
        
        logging.info("Plotting % successful runs")
        self.plot_stat("Percentage of successful runs", "percentage_succesful_runs", dir=dir, y_label="Succesful runs in %")
        
        logging.info("Plotting reward variance")
        #self.plot_stat("Reward variance", "sample_variances", dir=dir)
        
        logging.info("Plotting reward standard deviation")
        #self.plot_stat("Reward standard deviation", "reward_standand_deviation", dir=dir)
        
        logging.info("Plotting mean number of steps")
        self.plot_stat("Mean number of steps", "mean_num_steps", dir=dir)
              
        logging.info("Plotting mean planning times")
        self.plot_stat("Mean planning time", "mean_planning_time", dir=dir)
        
        logging.info("Plotting mean num generated paths per step")
        self.plot_stat("Mean number of generated paths per step", "mean_num_generated_paths_per_step", dir=dir)
        
        logging.info("Plotting mean number of collisions per run") 
        self.plot_stat("Mean num collisions per run", "mean_num_collision_per_run", dir=dir)
        logging.info("PlotStats: plotting mean number of generated paths")
        '''self.plot_mean_num_generated_paths(serializer,
                                           dir,
                                           filename="mean_num_generated_paths_per_step*.yaml",
                                           output="mean_num_generated_paths_per_step.pdf")'''
        '''self.plot_mean_num_generated_paths(serializer,
                                           dir,
                                           filename="mean_num_generated_paths_per_run*.yaml",
                                           output="mean_num_generated_paths_per_run.pdf")'''
        logging.info("PlotStats: plotting mean number of steps per run")
        
        
    def cleanup(self, dir="stats"):
        txtfiles = glob.glob(os.path.join(dir, "*.txt"))
        for file in txtfiles:
            print "removing " + file
            try:
                os.remove(file)
            except:
                pass
        texfiles = glob.glob(os.path.join(dir, "*.tex"))        
        for file in txtfiles:
            try:
                os.remove(file)
            except Exception as e:
                print e
                pass
            
    ##################################################################################
    def plot_linearisation_error(self, dir="stats"):
        files = glob.glob(os.path.join(dir, "log*.log"))        
        stats_sets = []        
        labels = []
        min_e = 1000000000000
        max_e = -1000000000000        
        color_map = [self.gen_random_color() for i in xrange(len(files))]        
        for file in files:
            set_1 = []
            n = 0
            labels.append(file.split("_")[-1].split(".log")[0])            
            with open(file, "r") as f:
                lines = f.readlines()                
                for i in xrange(len(lines)):
                    if "Linearization error:" in lines[i]: 
                        try:       
                            err = float(lines[i].split(":")[1].strip())
                            if err < min_e:
                                min_e = err
                            if err > max_e:
                                max_e = err       
                            elem = [n, err]
                            n+=1
                            set_1.append(elem)
                        except:
                            pass                        
            stats_sets.append(np.array(set_1))
        Plot.plot_2d_n_sets(stats_sets,
                            labels=labels,
                            xlabel="step",
                            ylabel="linearisation error",
                            x_range=[0, n],
                            y_range=[min_e, max_e * 1.05],
                            show_legend=True,
                            lw=3,
                            color_map=color_map,
                            save=self.save,
                            filename=dir + "/linearisation_error.png")
    
    def plot_estimate_error(self, dir="stats", normalized=False):
        files = glob.glob(os.path.join(dir, "log*.log"))        
        stats_sets = []        
        labels = []
        min_e = 1000000000000
        max_e = -1000000000000        
        color_map = [self.gen_random_color() for i in xrange(len(files))]        
        for file in files:
            set_1 = []
            n = 0
            labels.append(file.split("_")[-1].split(".log")[0])            
            with open(file, "r") as f:
                lines = f.readlines()                
                for i in xrange(len(lines)): 
                    if normalized:               
                        if "Estimation error normalized:" in lines[i]: 
                            try:       
                                err = float(lines[i].split(":")[1].strip())
                                if err < min_e:
                                    min_e = err
                                if err > max_e:
                                    max_e = err       
                                elem = [n, err]
                                n+=1
                                set_1.append(elem)
                            except:
                                pass
                    else:
                        if "Estimation error:" in lines[i]: 
                            try:       
                                err = float(lines[i].split(":")[1].strip())
                                if err < min_e:
                                    min_e = err
                                if err > max_e:
                                    max_e = err       
                                elem = [n, err]
                                n+=1
                                set_1.append(elem)
                            except:
                                pass                        
            stats_sets.append(np.array(set_1))
        Plot.plot_2d_n_sets(stats_sets,
                            labels=labels,
                            xlabel="step",
                            ylabel="estimation error joint angles",
                            x_range=[0, n],
                            y_range=[min_e, max_e * 1.05],
                            show_legend=True,
                            lw=3,
                            color_map=color_map,
                            save=self.save,
                            filename=dir + "/estimation_error.png")
        
    ##################################################################################
    
    def to_latex_table_reward_stats(self, dir="stats"):
        files_temp = glob.glob(os.path.join(dir, "out_*.txt")) 
        files = []
        for file in files_temp:
            if "reward" in file:
                files.append(file)
        print files
        
    def to_latex_table(self, dir="stats"):
        files = glob.glob(os.path.join(dir, "out_*.txt"))       
        print "files " + str(files)
        for file in files:            
            headers = [0.1]
            table_entries = []
            latex_filename = file.split("/")[-1].split(".txt")[0] + ".tex" 
            cov = 0.0           
            with open(file, "r") as f: 
                lines = f.readlines()               
                for i in xrange(len(lines)):
                    if "mean" in lines[i]:
                        mean = float(lines[i].strip().split(": ")[1])
                        print lines[i]
                        print mean
                    if "alg" in lines[i]:
                        headers.append(lines[i].strip().split(" ")[1])                        
                        cov = float(lines[i].strip().split(" ")[2].split(":")[0])
                        print headers
                    else:
                        value_found = False
                        for k in xrange(len(table_entries)):                           
                            if table_entries[k][0] == lines[i].strip().split(":")[0]:
                                value_found = True                                
                                table_entries[k].append(float(lines[i].strip().split(":")[1].strip()))
                                break
                            if "conf" in lines[i].strip().split(":")[0]:
                                if "confidence" in table_entries[k][0]:
                                    value_found = True
                                    conv = float(lines[i].strip().split(": ")[1])
                                    val1 = mean - conf
                                    val2 = mean + conf
                                    table_entries[k].append(str(val1) + ", " + str(val2))           
                        if not len(lines[i].split()) == 0:
                            if not value_found:
                                print lines[i].strip().split(":")[0]
                                if "conf" in lines[i].strip().split(":")[0]: 
                                    conf = float(lines[i].strip().split(":")[1].strip())
                                    val1 = mean - conf
                                    val2 = mean + conf
                                    e = str(val1) + ", " + str(val2)                                                                    
                                    table_entries.append(["confidence", e])                                    
                                else:
                                    table_entries.append([lines[i].strip().split(":")[0], float(lines[i].strip().split(":")[1].strip())])
                    
                table = tabulate(table_entries, headers=headers, tablefmt="latex")
                with open(os.path.join(dir, latex_filename), "a+") as f:
                    f.write(table)    
          
        
    def plot_emds(self, show_particles, dir="stats"):        
        files = glob.glob(os.path.join(os.path.join(dir, "*.log")))
        files = [files[i] for i in xrange(len(files)) if not "abt" in files[i] and not "hrf" in files[i]]
        
        d = dict()
        d["lqg"] = [] 
        cov_string = "" 
        cov_value = 0.0
        stats_sets = []
        color_map = []
        labels = []
        m_covs = [] 
        emd_vals = []             
        for file in files:
            print file
            num_steps = 0
            with open(file, 'r') as f:
                for line in f:
                    if "inc_covariance" in line:
                        inc_cov = line.rstrip("\n").split(": ")[1]
                        if inc_cov == "process":
                            cov_string = "Process covariance"
                        elif inc_cov == "observation":
                            cov_string = "Observation covariance"            
            with open(file, 'r') as f:
                for line in f:
                    if cov_string in line:
                        cov_value = float(line.rstrip("\n ").split(": ")[1])
                        m_covs.append(cov_value)                                                                 
            with open(file, 'r') as f:
                for line in f:
                    if "Length best path: " in line:
                        num_steps = int(line.split(" ")[3]) 
                      
            #print file.split("/")         
            #file_str = file.split("/")[2].split("_")[1]
            emds = []
            for n in xrange(num_steps):                
                print "step " + str(n)                
                particles = []
                state_nominal = None
                covariance = None
                with open(file, 'r') as f:
                    t_found = False                
                    for line in f:
                        if t_found:                        
                            if "S: " in line:                            
                                line_str = line.split(" ")
                                #line_arr = line_str[1:len(line_str) - 2]
                                line_arr = line_str[1:len(line_str) - 2]
                                particles.append(np.array([float(l) for l in line_arr][0:2 * self.robot.getDOF()])) 
                            elif "S_NOMINAL: " in line:                            
                                line_str = line.split(" ")
                                line_arr = line_str[1:len(line_str) - 2]
                                state_nominal = np.array([float(l) for l in line_arr][0:2 * self.robot.getDOF()]) 
                            elif "ESTIMATED_COVARIANCE: " in line:
                                line_str = line.split(" ")
                                line_arr = line_str[1:len(line_str) - 2]                             
                                line_arr = np.array([float(el) for el in line_arr])
                                covariance = line_arr.reshape((2 * self.robot.getDOF(), 2 * self.robot.getDOF()))
                                                    
                        if "t = " in line:
                            if "t = " + str(n) + " " in line:                            
                                t_found = True
                            else:
                                t_found = False
                #mult_normal = multivariate_normal.pdf(particles, state_nominal, covariance)                
                """
                Calculate 2 * N - dimensional histogram
                """
                
                num_bins = 5
                X = np.array([[particles[i][j] for j in xrange(len(particles[i]))] for i in xrange(len(particles))])
                                
                h_x = np.histogramdd(X, bins=num_bins, normed=False)
                coords = []
                weights = []
                num_dimensions = len(h_x[1])
                hist_coord_arr = [0 for i in xrange(num_dimensions)]                
                m = 0
                for i in xrange(0, (num_bins**num_dimensions)):
                    """
                    Get the histogram coordinates (in state space coordinates)
                    """           
                    sample_arr = []                    
                    for k in xrange(num_dimensions):
                        lower_edge = h_x[1][k][hist_coord_arr[k]]
                        upper_edge = h_x[1][k][hist_coord_arr[k] + 1]
                        c = lower_edge + (upper_edge - lower_edge) / 2.0                                              
                        sample_arr.append(c)
                    coords.append(sample_arr)
                    
                    """
                    Get the histogram weights for the histogram coordinates
                    """
                    histogram_elem = h_x[0]
                    for k in hist_coord_arr:
                        histogram_elem = histogram_elem[k]
                    weights.append(histogram_elem)
                    
                    if i < (num_bins**num_dimensions) - 1:          
                        hist_coord_arr[m] += 1                    
                        if hist_coord_arr[m] >= num_bins:
                            while hist_coord_arr[m] >= num_bins - 1:
                                m += 1
                            hist_coord_arr[m] += 1
                            for k in xrange(0, m):
                                hist_coord_arr[k] = 0
                            m = 0
                sum_weights = sum(weights)
                weights /= sum(weights)
                samples = multivariate_normal.rvs(state_nominal, covariance, 1000)
                #samples = [sample[dim[0]:dim[1]] for sample in samples]
                
                sample_weights = [multivariate_normal.pdf(samples[i], state_nominal, covariance, allow_singular=True) for i in xrange(len(samples))]
                sample_weights /= sum(sample_weights)                
                
                sample_coords = np.array([[samples[i][j] for j in xrange(len(samples[i]))] for i in xrange(len(samples))])
                particle_weights = []
                particle_coords = []
                for i in xrange(len(coords)):
                    if weights[i] != 0:
                        particle_coords.append([coords[i][j] for j in xrange(len(coords[i]))])
                        particle_weights.append(weights[i])
                particle_coords = np.array([[particle_coords[i][j] for j in xrange(len(particle_coords[i]))] for i in xrange(len(particle_coords))])
                particle_weights = np.array(particle_weights)                
                emd_v = emd(particle_coords, sample_coords, X_weights=particle_weights, Y_weights=sample_weights)
                print emd_v
                emds.append(emd_v)
                if show_particles:
                    min_x = -np.pi
                    min_y = np.pi
                    min_z = -np.pi
                    max_x = np.pi
                    max_y = -np.pi
                    max_z = np.pi                            
                    sets = [np.array(particles), np.array(samples)]                
                    Plot.plot_3d_sets(sets,
                                      x_scale=[min_x, max_x],
                                      y_scale=[min_y, max_y],
                                      z_scale=[min_z, max_z], 
                                      colormap=['r', 'g'])
            emd_mean = sum(emds) / len(emds)            
            print "emd_mean " + str(emd_mean)
            emd_vals.append(float(emd_mean))
            d["lqg"].append(np.array([float(cov_value), float(emd_mean)]))
        print d
                   
        for k in d:
            color_map.append(self.gen_random_color())           
            from operator import itemgetter            
            d[k] = sorted(d[k], key=itemgetter(0))
            d[k] = [np.array(d[k][i]) for i in xrange(len(d[k]))]
            stats_sets.append(np.array(d[k]))
            labels.append(k)
        min_m = min(emd_vals)
        if min_m > 0:
            min_m = 0.0
        else:
            min_m -= -0.1 * min_m 
        max_m = max(emd_vals)
        Plot.plot_2d_n_sets(stats_sets,
                            labels=labels,
                            xlabel="joint covariance",
                            ylabel="EMD",
                            x_range=[min(m_covs), max(m_covs)],
                            y_range=[min_m, max_m * 1.05],
                            show_legend=True,
                            lw=3,
                            color_map=color_map,
                            save=self.save,
                            filename=dir + "/EMD.png")
        
    def save_estimation_error(self, output_file_str, dir="stats", y_label=""):
        all_log_files = glob.glob(os.path.join(os.path.join(dir, "*.log")))
        log_files = []
        d = dict()
        m_covs = []
        for file in all_log_files:
            if "hrf" in file or "lqg" in file:
                log_files.append(file)
        for file in sorted(log_files):
            m_cov = float(str(file.split("/")[-1].split("_")[-1].split(".")[0] + "." + str(file.split("/")[-1].split("_")[-1].split(".")[1])))
            m_covs.append(m_cov)
        for i in xrange(len(m_covs)):
            out_files = glob.glob(os.path.join(os.path.join(dir, "out_" + str(output_file_str) + "*")))
            for out_file in out_files:
                os.remove(out_file)
        for file in sorted(log_files):
            num_runs = 0.0
            all_vals = []
            vals = []
            vals_per_run = []
            file_str = "alg: " + file.split("/")[-1].split("_")[1]
            with open(file, "r") as f:
                for line in f:                    
                    if "inc_covariance: " in line:                        
                        inc_covariance = line.rstrip("\n").split(": ")[1]
                        if inc_covariance == 'process':
                            cov_str = "Process covariance:"
                        elif inc_covariance == 'observation':
                            cov_str = "Observation covariance:" 
            with open(file, "r") as f:
                for line in f:
                    if cov_str in line:                                             
                        m_cov = float(line.split(" ")[2])
                    elif ("RUN #" in line or 
                          "Run #" in line):
                        num_runs += 1
                        if len(vals) != 0:
                            vals_per_run.append(vals)
                            all_vals.extend(vals)
                            vals = []
                    elif "############" in line:
                        vals_per_run.append(vals)
                        all_vals.extend(vals)
                        vals = []
                    else:
                        if "S:" in line:                            
                            state = np.array([float(el) for el in line.split("S: ")[1].strip().split(" ")])                            
                        elif "S_ESTIMATED:" in line:
                            estimated_state = np.array([float(el) for el in line.split("S_ESTIMATED: ")[1].strip().split(" ")])
                            vals.append(np.linalg.norm(state - estimated_state))
            n, min_max, mean, var, skew, kurt = scipy.stats.describe(np.array(all_vals))
            arr = []            
            for i in xrange(int(num_runs)):                            
                arr.append(sum(vals_per_run[i]))            
            n, min_max, mean2, var2, skew, kurt = scipy.stats.describe(np.array(arr))
            with open(os.path.join(dir, "out_" + str(output_file_str) + "_" + str(m_cov) + ".txt"), "a+") as f:
                f.write(file_str + " " + str(m_cov) + ": \n")
                f.write("mean per run: " + str(mean2) + " \n")
                f.write("variance: " + str(var2) + " \n")
                f.write("min: " + str(min_max[0]) + " \n")
                f.write("max: " + str(min_max[1]) + " \n")
                f.write("skewness: " + str(skew) + " \n")
                f.write("kurtosis: " + str(kurt) + " \n") 
                f.write("conf: 0.0 \n")       
        
    def plot_number_of_steps_stats(self, output_file_str, dir="stats"):
        files = glob.glob(os.path.join(os.path.join(dir, "*.log")))        
        d = dict()
        m_covs = []
        for file in sorted(files):
            m_cov = float(str(file.split("/")[-1].split("_")[-1].split(".")[0] + "." + str(file.split("/")[-1].split("_")[-1].split(".")[1])))
            m_covs.append(m_cov)
        for i in xrange(len(m_covs)):
            out_files = glob.glob(os.path.join(os.path.join(dir, "out_" + str(output_file_str) + "*")))
            for out_file in out_files:
                os.remove(out_file) 
        for file in sorted(files):
            num_runs = 0
            ts = []
            file_str = "alg: " + file.split("/")[-1].split("_")[1]
            with open(file, "r") as f:
                for line in f:
                    if "inc_covariance: " in line:                        
                        inc_covariance = line.rstrip("\n").split(": ")[1]
                        if inc_covariance == 'process':
                            cov_str = "Process covariance:"
                        elif inc_covariance == 'observation':
                            cov_str = "Observation covariance:"
            with open(file, "r") as f:
                t = 0
                for line in f:
                    if cov_str in line:                                             
                        m_cov = float(line.split(" ")[2])
                    if "t =" in line:
                        t = float(line.split("=")[1].strip())
                    elif ("RUN #" in line or 
                          "Run #" in line):
                        num_runs += 1
                        if t != 0:
                            ts.append(t)
                    elif "############" in line:
                        ts.append(t)
            n, min_max, mean, var, skew, kurt = scipy.stats.describe(np.array(ts))
            with open(os.path.join(dir, "out_" + str(output_file_str) + "_" + str(m_cov) + ".txt"), "a+") as f:
                f.write(file_str + " " + str(m_cov) + ": \n")
                f.write("mean per run: " + str(mean) + " \n")
                f.write("variance: " + str(var) + " \n")
                f.write("min: " + str(min_max[0]) + " \n")
                f.write("max: " + str(min_max[1]) + " \n")
                f.write("skewness: " + str(skew) + " \n")
                f.write("kurtosis: " + str(kurt) + " \n \n")
                
    def get_files(self, output_file_str, dir):
        files = glob.glob(os.path.join(os.path.join(dir, "*.log")))
        m_covs = []
        algorithm = ""
        for file in sorted(files):
            m_cov = float(str(file.split("/")[-1].split("_")[-1].split(".")[0] + "." + str(file.split("/")[-1].split("_")[-1].split(".")[1])))
            m_covs.append(m_cov)
        for i in xrange(len(m_covs)):
            out_files = glob.glob(os.path.join(os.path.join(dir, "out_" + str(output_file_str) + "*")))
            for out_file in out_files:
                os.remove(out_file)
        return m_covs, sorted(files)
                
    def plot_num_succesful_runs(self, output_file_str, dir="stats", finish_when_collided=False):
        m_covs, files = self.get_files(output_file_str, dir)
        d = dict()
        algorithm = ""
        for file in files: 
            num_succ = 0.0           
            num_not_succ = 0.0
            num_runs = 0.0
            file_str = "alg: " + file.split("/")[-1].split("_")[1]
            if not file_str in d:
                d[file_str] = []
            with open(file, "r") as f:
                for line in f:
                    if "inc_covariance: " in line:                        
                        inc_covariance = line.rstrip("\n").split(": ")[1]
                        if inc_covariance == 'process':
                            cov_str = "Process covariance:"
                        elif inc_covariance == 'observation':
                            cov_str = "Observation covariance:"            
            with open(file, "r") as f:
                algorithm = file.split("/")[-1].split("_")[1]
                block = False
                for line in f:                                      
                    if cov_str in line:                                             
                        m_cov = float(line.split(" ")[2])
                    if "collided: true" in line or "Collision detected: True" in line:
                        if not block and finish_when_collided:                      
                            num_not_succ += 1                        
                            block = True
                    elif ("RUN #" in line or 
                          "Run #" in line): 
                        num_runs += 1                       
                        block = False
                    elif "Num successes:" in line:
                        num_succ = float(line.strip().split(": ")[1])
            actual_successful_runs = num_runs - num_not_succ
            print "file " + str(file)
            print "num_runs " + str(num_runs)
            print "num success " + str(num_succ)
            print "actual_successful_runs " + str(actual_successful_runs)
            print "num not success " + str(num_not_succ)
            print "diff " + str(num_succ - num_not_succ)
            print " "
            percentage_succ_runs = (100.0 / num_runs) * actual_successful_runs
            with open(os.path.join(dir, "out_" + str(output_file_str) + "_" + str(m_cov) + ".txt"), "a+") as f:
                f.write(file_str + " " + str(m_cov) + ": \n")
                f.write("mean per run: " + str(percentage_succ_runs) + " \n")
                f.write("variance: " + str(0.0) + " \n")
                f.write("min: " + str(0) + " \n")
                f.write("max: " + str(0) + " \n")
                f.write("conf: 0.0 \n")
        
                
    def mean_confidence_interval(self, data, confidence=0.95):
        print data
        a = 1.0 * np.array(data)
        n = len(a)
        m, se = np.mean(a), scipy.stats.sem(a)
        h = se * scipy.stats.t._ppf((1+confidence)/2., n-1)
        return m, h
                        
    def plot_reward(self, reward_model, output_file_str, dir="stats", finish_when_collided=False):
        m_covs, files = self.get_files(output_file_str, dir)
        d = dict()
        algorithm = ""
        arrs = []
        for file in files:
            vals = []
            vals_per_run = []
            all_vals = []
            num_runs = 0.0
            algorithm = ""
            file_str = "alg: " + file.split("/")[-1].split("_")[1]
            if not file_str in d:
                d[file_str] = []
            with open(file, "r") as f:
                for line in f:
                    if "inc_covariance: " in line:                        
                        inc_covariance = line.rstrip("\n").split(": ")[1]
                        if inc_covariance == 'process':
                            cov_str = "Process covariance:"
                        elif inc_covariance == 'observation':
                            cov_str = "Observation covariance:"            
            with open(file, "r") as f:
                algorithm = file.split("/")[-1].split("_")[1]
                reward_set = False
                block = False
                for line in f:                                      
                    if cov_str in line:                                             
                        m_cov = float(line.split(" ")[2])
                    elif ("RUN #" in line or 
                          "Run #" in line):
                        num_runs += 1
                        block = False
                        if len(vals) != 0:
                            vals_per_run.append(vals)
                            all_vals.extend(vals)
                            vals = []
                    elif "############" in line:
                        vals_per_run.append(vals)
                        all_vals.extend(vals)
                        vals = [] 
                        block  = False            
                    elif "t = " in line:
                        if not block:                                               
                            vals.append(reward_model['step_penalty'])                                                
                    elif "collided: true" in line or "Collision detected: True" in line:
                        if not block:                      
                            vals[-1] = reward_model['collision_penalty']
                        if finish_when_collided:
                            block = True
                    elif "Terminal: true" in line or "Terminal: True" in line:
                        if not block:
                            vals[-1] = reward_model['exit_reward']
                    elif "R: " in line and "abt" in algorithm:
                        if not block:
                            vals[-1] = float(line.strip().split(": ")[1])                        
            arr = []            
            for i in xrange(int(num_runs)): 
                if algorithm == 'hrf':
                    print sum(vals_per_run[i])                        
                arr.append(sum(vals_per_run[i]))
            arrs.append(arr)
            mean, conf = self.mean_confidence_interval(arr)            
            
            #Plot.plot_histogram_from_data(arr, bin_width=1.0, title=algorithm)        
            n, min_max, mean2, var2, skew, kurt = scipy.stats.describe(np.array(arr))
            with open(os.path.join(dir, "out_" + str(output_file_str) + "_" + str(m_cov) + ".txt"), "a+") as f:
                f.write(file_str + " " + str(m_cov) + ": \n")
                f.write("mean per run: " + str(mean2) + " \n")
                f.write("variance: " + str(var2) + " \n")
                f.write("min: " + str(min_max[0]) + " \n")
                f.write("max: " + str(min_max[1]) + " \n")
                f.write("conf: " + str(conf) + " \n")
        absmin = 1000000000
        absmax = -1000000000
        for i in xrange(len(arrs)):
            for j in xrange(len(arrs[i])):
                if arrs[i][j] < absmin:
                    absmin = arrs[i][j]
                if arrs[i][j] > absmax:
                    absmax = arrs[i][j]
        print "absmin " + str(absmin)
        print "absmax " + str(absmax)
        for i in xrange(len(arrs)):
            '''Plot.plot_histogram_from_data(arrs[i], 
                                          bin_width=30.0, 
                                          absmin=absmin,
                                          absmax=absmax)'''
            pass             
                                           
            
        
    def plot_bla(self, stat_strings, output_file_str, dir="stats", y_label=""):        
        files = glob.glob(os.path.join(os.path.join(dir, "*.log")))        
        d = dict()
        m_covs = []
        for file in sorted(files):
            m_cov = float(str(file.split("/")[-1].split("_")[-1].split(".")[0] + "." + str(file.split("/")[-1].split("_")[-1].split(".")[1])))
            m_covs.append(m_cov)
        for i in xrange(len(m_covs)):
            out_files = glob.glob(os.path.join(os.path.join(dir, "out_" + str(output_file_str) + "*")))
            for out_file in out_files:
                os.remove(out_file)               
        for file in sorted(files):
            num_runs = 0.0
            all_vals = []
            vals = []
            vals_per_run = []
            file_str = "alg: " + file.split("/")[-1].split("_")[1]
            if not file_str in d:
                d[file_str] = []           
            with open(file, "r") as f:
                for line in f:
                    if "inc_covariance: " in line:                        
                        inc_covariance = line.rstrip("\n").split(": ")[1]
                        if inc_covariance == 'process':
                            cov_str = "Process covariance:"
                        elif inc_covariance == 'observation':
                            cov_str = "Observation covariance:"            
            with open(file, "r") as f:
                for line in f:                    
                    if cov_str in line:                                             
                        m_cov = float(line.split(" ")[2])
                    elif ("RUN #" in line or 
                          "Run #" in line):                        
                        num_runs += 1                        
                        if len(vals) != 0:
                            vals_per_run.append(vals)
                            all_vals.extend(vals)
                            vals = []                            
                    elif "############" in line:
                        vals_per_run.append(vals)
                        all_vals.extend(vals)
                        vals = []
                    else:
                        for i in xrange(len(stat_strings)):                            
                            if stat_strings[i] in line and stat_strings[i][0] == line[0]:
                                if not "estimate" in line:                                             
                                    if ("true" in line or 
                                        "false" in line or
                                        "True" in line or
                                        "False" in line):
                                        if "true" in line or "True" in line:
                                            vals.append(1)
                                        else:
                                            print "APPEND!!!!"
                                            vals.append(0)
                                    else: 
                                        try:                                       
                                            vals.append(float(line.rstrip("\n").split(":")[-1].rstrip(" ")))
                                        except:
                                            print line.rstrip("\n")
                                                                
            print all_vals
            n, min_max, mean, var, skew, kurt = scipy.stats.describe(np.array(all_vals))
            arr = []            
            for i in xrange(int(num_runs)):                            
                arr.append(sum(vals_per_run[i]))            
            n, min_max, mean2, var2, skew, kurt = scipy.stats.describe(np.array(arr))
            
            
            with open(os.path.join(dir, "out_" + str(output_file_str) + "_" + str(m_cov) + ".txt"), "a+") as f:
                f.write(file_str + " " + str(m_cov) + ": \n")
                f.write("mean per run: " + str(mean2) + " \n")
                f.write("variance: " + str(var2) + " \n")
                f.write("min: " + str(min_max[0]) + " \n")
                f.write("max: " + str(min_max[1]) + " \n")                
                #f.write(file_str + " " + str(m_cov) + " " + str(mean) + ", " + str(var) + " \n")
             
    def plot_stat_from_txt_file(self, stat_str, output_file_str, dir="stats", y_label=""):
        if y_label == "":
            y_label = stat_str        
        possible_files = glob.glob(os.path.join(os.path.join(dir, "*.txt")))
        files = []
        stats_sets = []
        labels = []
        color_map = []
        linestyles = []
        d = dict()        
        for file in sorted(possible_files):            
            if stat_str in file:
                files.append(file)              
        for file in files:
            with open(file, "r") as f:
                cov_val = 0.0 
                mean = 0.0
                conf_low = 0.0
                conf_high = 0.0              
                for line in f:                    
                    if "alg" in line:
                        alg = line.split(": ")[1].split(" ")[0]
                        cov_val = float(line.split(" ")[2].split(":")[0])                                              
                        if not alg in d:
                            d[alg] = []                        
                    if "mean per run:" in line:
                        mean = float(line.split(": ")[1].strip())                        
                    elif "variance" in line:
                        standard_deviation = np.sqrt(float(line.strip().split(": ")[1]))
                        #d[alg].append([cov_val, mean, standard_deviation])
                    elif "conf:" in line:                        
                        conf = float(line.strip().split(": ")[1])                  
                        d[alg].append([cov_val, mean, conf])
                        
                        
        m_covs = []
        min_m = 100000.0
        max_m = -100000.0 
        print d       
        for key in d.keys():
            if key in self.color_dict:
                color_map.append(self.color_dict[key])
            else:
                color_map.append(self.gen_random_color())
            linestyles.append(self.linestyles[key])
            labels.append(key)
            stats_sets.append(np.array(d[key]))            
            for i in xrange(len(d[key])):
                print "d[key] " + str(d[key])
                m_covs.append(d[key][i][0])
                if d[key][i][1] > max_m:
                    max_m = d[key][i][1]
                if d[key][i][1] + d[key][i][2] > max_m:
                    max_m = d[key][i][1] + d[key][i][2]
                if d[key][i][1] < min_m:
                    min_m = d[key][i][1]
                if d[key][i][1] - d[key][i][2] < min_m:
                    min_m = d[key][i][1] - d[key][i][2]
        
        x_range=[min(m_covs), max(m_covs) + 0.5]
        y_range=[min_m, max_m + max_m * 0.1] 
        if stat_str == "succ_stats":
            y_range = [0, max_m + max_m * 0.1]                  
        
        Plot.plot_2d_n_sets(stats_sets,
                            labels=labels,
                            xlabel="joint covariance",
                            ylabel=y_label,
                            x_range=x_range,
                            y_range=y_range,
                            show_legend=True,
                            lw=3,
                            linestyles=linestyles,
                            color_map=color_map,
                            save=self.save,
                            filename=dir + "/" + output_file_str + ".png")
        
    def plot_stat(self, stat_str, output_file_str, dir="stats", y_label=""):
        if y_label == "":
            y_label = stat_str        
        files = glob.glob(os.path.join(os.path.join(dir, "*.log")))                 
        num_succ_runs = [] 
        stats_sets = [] 
        labels = []
        m_covs = []
        data = []
        color_map = []
        linestyles = []
        d = dict()               
        for file in sorted(files):
            file_str = file.split("/")[-1].split("_")[1]            
            if not file_str in d:
                d[file_str] = []                 
            m_cov = -1
            succ = -1
            cov_str = ""
            
            with open(file, "r") as f:
                logging.info("Processing " + str(file))
                for line in f:
                    if "inc_covariance: " in line:                        
                        inc_covariance = line.rstrip("\n").split(": ")[1]
                        if inc_covariance == 'process':
                            cov_str = "Process covariance:"
                        elif inc_covariance == 'observation':
                            cov_str = "Observation covariance:"
            with open(file, "r") as f:                                         
                for line in f:                                                                        
                    if cov_str in line:                                             
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
            if m_cov != -1:                        
                m_covs.append(m_cov)
                num_succ_runs.append(succ)
                d[file_str].append([m_cov, succ])             
        for k in d:
            if k in self.color_dict:
                color_map.append(self.color_dict[k])
            else:
                color_map.append(self.gen_random_color())
            linestyles.append(self.linestyles[k])
            
            from operator import itemgetter            
            d[k] = sorted(d[k], key=itemgetter(0))
            d[k] = [np.array(d[k][i]) for i in xrange(len(d[k]))]
            stats_sets.append(np.array(d[k]))
            labels.append(k)
        min_m = min(num_succ_runs)
        if min_m > 0:
            min_m = 0.0
        else:
            min_m -= -0.1 * min_m 
        max_m = max(num_succ_runs) 
        min_cov = min(m_covs)
        max_cov = max(m_covs)
        with open(dir + "/" + output_file_str + ".txt", 'a+') as f:            
            for i in xrange(len(stats_sets)):
                string = labels[i]
                f.write(string + " \n") 
                string = ""
                for j in xrange(len(stats_sets[i])):
                    string = str(stats_sets[i][j][0]) + ", "  + str(stats_sets[i][j][1])
                    f.write(string + " \n")
                f.write("\n")
        Plot.plot_2d_n_sets(stats_sets,
                            labels=labels,
                            xlabel="joint covariance",
                            ylabel=y_label,
                            x_range=[min(m_covs), max(m_covs)],
                            y_range=[min_m, max_m * 1.05],
                            show_legend=True,
                            lw=3,
                            linestyles=linestyles,
                            color_map=color_map,
                            save=self.save,
                            filename=dir + "/" + output_file_str + ".png")
        
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
    parser = argparse.ArgumentParser(description='LQG-MP plotting')    
    parser.add_argument("-d", "--directory", help="Path to the robot model file")
    parser.add_argument("-s", "--save", 
                        help="Save the plots", 
                        action="store_true")
    parser.add_argument("-p", "--particles", 
                        help="Show particles", 
                        action="store_true")
    parser.add_argument("-e", "--emd", 
                        help="Plot emds", 
                        action="store_true")
    parser.add_argument("-cf", "--collision_is_failed",
                        help="A run in which the robot collides, counts as an unsuccessful run",
                        action="store_true")
    args = parser.parse_args() 
    PlotStats(args.directory, args.save, args.particles, args.emd, args.collision_is_failed)
    '''return
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
    PlotStats(False)'''
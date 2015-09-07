import sys
import numpy as np
import plot as Plot
import glob
import os
import logging
from serializer import Serializer
from kin import *
from util import *
from EMD import *

class PlotStats:
    def __init__(self, save, algorithm):
        dir = "stats/" + str(algorithm)
        if not os.path.isdir(dir):
            os.makedirs(dir)      
        self.save = save
        logging.info("PlotStats: Loading cartesian coordinates...") 
        serializer = Serializer()
        self.setup_kinematics(serializer, dir=dir)
        logging.info("PlotStats: plotting paths")    
        self.plot_paths(serializer, dir=dir)
        self.plot_paths(serializer, best_paths=True, dir=dir)
        
        logging.info("PlotStats: plotting average distance to goal")
        self.plot_average_dist_to_goal(serializer, dir=dir)
        logging.info("PlotStats: plotting mean rewards")
        self.plot_rewards(serializer, dir=dir, show_legend=True)
        logging.info("PlotStats: plotting % successful runs")
        self.plot_num_successes(serializer, dir=dir)
        self.plot_sample_variances(serializer, dir=dir, show_legend=True)
        self.plot_sample_standard_deviations(serializer, dir=dir, show_legend=True)        
        logging.info("PlotStats: plotting mean planning times")
        self.plot_mean_planning_times(serializer, dir=dir) 
        self.plot_mean_planning_times(serializer, 
                                      dir, 
                                      filename="mean_planning_times_per_run*.yaml",
                                      output="mean_planning_times_per_run.pdf")
        cart_coords = serializer.load_cartesian_coords(dir, "cartesian_coords_" + algorithm + ".yaml")
        logging.info("PlotStats: plotting EMD graph...")
        self.plot_emd_graph(serializer, cart_coords, dir=dir) 
        logging.info("PlotStats: plotting histograms...")        
        self.save_histogram_plots(serializer, cart_coords, dir=dir)
        
    def setup_kinematics(self, serializer, dir='stats'):
        config = serializer.read_config(path=dir)
        links = v2_double()
        axis = v2_int()
        
        link = v_double()
        ax1 = v_int()
        ax2 = v_int()
        link[:] = [1.0, 0.0, 0.0]
        links[:] = [link for i in xrange(config['num_links'])]
        
        ax1[:] = [0, 0, 1]
        if config['workspace_dimension'] == 2:
            ax2[:] = [0, 0, 1]            
        elif config['workspace_dimension'] == 3:
            ax2[:] = [0, 1, 0]
            
        axis[:] = [ax1, ax2, ax1]
        
        self.kinematics = Kinematics()
        self.kinematics.setLinksAndAxis(links, axis)
               
        
    def clear_stats(self):
        for file in glob.glob("stats/*"):
            os.remove(file)
            
    def plot_average_dist_to_goal(self, serializer, dir="stats"):
        config = serializer.read_config(path=dir)
        stats = serializer.load_stats('stats.yaml', path=dir)
        m_cov = stats['m_cov'] 
        labels = []       
        max_avg_distance = 0.0
        sets = []
        cart_coords = []
        files = glob.glob(os.path.join(os.path.join(dir, "cartesian_coords*")))
        
        for file in sorted(files):
            file_str = file
            try:                
                file_str = file.split("/")[-1].split("_")[-1].split(".")[0]
                file_str = "avg_distance_" + file_str
                #file_str = file.split("/")[1].split(".")[0].split("_")[1] 
                label_string = ""
                for i in xrange(len(file_str.split("_"))):
                    if i != 0:
                        label_string += " " + file_str.split("_")[i]
                    else:
                        label_string += file_str.split("_")[i]
                labels.append(label_string)               
            except Exception as e:
                logging.error("PlotStats: " + str(e))
            data = []
            cart_coords = serializer.load_cartesian_coords(dir, file.split("/")[-1])
            dists = []
            if len(cart_coords) > 0:            
                for k in xrange(len(m_cov)):
                    for coords in cart_coords[k]:                
                        dists.append(np.linalg.norm(np.array(coords) - np.array(config['goal_position'])))            
                    avg_distance = 0.0
                    for d in dists:
                        avg_distance += d
                    avg_distance /= len(dists)
                    if avg_distance > max_avg_distance:
                        max_avg_distance = avg_distance                        
                    data.append(np.array([m_cov[k], avg_distance]))
                sets.append(np.array(data))
        if len(sets) > 0:    
            Plot.plot_2d_n_sets(sets,
                                labels=labels,
                                xlabel="joint covariance",
                                ylabel="average distance to goal",
                                x_range=[m_cov[0], m_cov[-1]],
                                y_range=[0, max_avg_distance],
                                show_legend=True,
                                save=self.save,
                                filename=dir + "/avg_distance.pdf")
        return
        
        Plot.plot_2d_n_sets([np.array(data)],
                            xlabel="joint covariance",
                            ylabel="average distance to goal",
                            x_range=[m_cov[0], m_cov[-1]],
                            y_range=[0, max_avg_distance],
                            show_legend=False,
                            save=self.save,
                            filename=dir + "/avg_distance.png")
        
    def plot_num_successes(self, serializer, dir="stats"):
        config = serializer.read_config(path=dir)    
        stats = serializer.load_stats('stats.yaml', path=dir)
        m_cov = stats['m_cov']    
        filename = glob.glob(os.path.join(dir, "num_successes_*.yaml"))[0]
        
        files = glob.glob(os.path.join(os.path.join(dir, "num_successes*.yaml")))      
        num_succ_runs = [] 
        num_succ_runs_sets = [] 
        labels = []
        for file in sorted(files):
            file_str = file
            try:
                file_str = file.split("/")[-1].split(".")[0]
                #file_str = file.split("/")[1].split(".")[0].split("_")[1]                
            except Exception as e:
                logging.error("PlotStats: " + str(e))   
            
            num_succ_runs.append(serializer.load_stats(file))            
            data = []
            for k in xrange(len(m_cov)):
                data.append(np.array([m_cov[k], num_succ_runs[-1][k]]))
            num_succ_runs_sets.append(np.array(data))
            label_string = ""
            for i in xrange(len(file_str.split("_"))):
                if i != 0:
                    label_string += " " + file_str.split("_")[i]
                else:
                    label_string += file_str.split("_")[i]
            labels.append(label_string)
        min_m = [min(m) for m in num_succ_runs]
        max_m = [max(m) for m in num_succ_runs]
               
        Plot.plot_2d_n_sets(num_succ_runs_sets,
                            labels=labels,
                            xlabel="joint covariance",
                            ylabel="% successful runs",
                            x_range=[m_cov[0], m_cov[-1]],
                            y_range=[min(min_m), max(max_m) * 1.05],
                            show_legend=True,
                            save=self.save,
                            filename=dir + "/successful_runs.pdf")
           
        
        
    def plot_sample_variances(self, serializer, dir="stats", show_legend=False):
        stats = serializer.load_stats('stats.yaml', path=dir)
        m_cov = stats['m_cov']
        sample_variances_sets = []
        labels = []
        sample_variances = []
        files = glob.glob(os.path.join(os.path.join(dir, "sample_variances*.yaml")))
        for file in sorted(files):
            file_str = file
            try:
                file_str = file.split("/")[-1].split(".")[0]
                #file_str = file.split("/")[1].split(".")[0].split("_")[1]                
            except Exception as e:
                logging.error("PlotStats: " + str(e))   
            
            sample_variances.append(serializer.load_stats(file))            
            data = []
            for k in xrange(len(m_cov)):
                data.append(np.array([m_cov[k], sample_variances[-1][k]]))
            sample_variances_sets.append(np.array(data))
            label_string = ""
            for i in xrange(len(file_str.split("_"))):
                if i != 0:
                    label_string += " " + file_str.split("_")[i]
                else:
                    label_string += file_str.split("_")[i]
            labels.append(label_string)        
        min_m = [min(m) for m in sample_variances]
        max_m = [max(m) for m in sample_variances]        
        Plot.plot_2d_n_sets(sample_variances_sets,
                            labels=labels,
                            xlabel="joint covariance",
                            ylabel="sample variance",
                            x_range=[m_cov[0], m_cov[-1]],
                            y_range=[min(min_m), max(max_m) * 1.05],
                            show_legend=show_legend,
                            save=self.save,
                            filename=dir + "/sample_variance.pdf")
        
    def plot_sample_standard_deviations(self, serializer, dir="stats", show_legend=False):
        stats = serializer.load_stats('stats.yaml', path=dir)
        m_cov = stats['m_cov']
        sample_deviations_sets = []
        labels = []
        sample_deviations = []
        files = glob.glob(os.path.join(os.path.join(dir, "sample_standard_deviations*.yaml")))
        for file in sorted(files):
            file_str = file
            try:
                file_str = file.split("/")[-1].split(".")[0]
                #file_str = file.split("/")[1].split(".")[0].split("_")[1]                
            except Exception as e:
                logging.error("PlotStats: " + str(e))   
            
            sample_deviations.append(serializer.load_stats(file))            
            data = []
            for k in xrange(len(m_cov)):
                data.append(np.array([m_cov[k], sample_deviations[-1][k]]))
            sample_deviations_sets.append(np.array(data))
            label_string = ""
            for i in xrange(len(file_str.split("_"))):
                if i != 0:
                    label_string += " " + file_str.split("_")[i]
                else:
                    label_string += file_str.split("_")[i]
            labels.append(label_string)        
        min_m = [min(m) for m in sample_deviations]
        max_m = [max(m) for m in sample_deviations]        
        Plot.plot_2d_n_sets(sample_deviations_sets,
                            labels=labels,
                            xlabel="joint covariance",
                            ylabel="sample standard deviation",
                            x_range=[m_cov[0], m_cov[-1]],
                            y_range=[min(min_m), max(max_m) * 1.05],
                            show_legend=show_legend,
                            save=self.save,
                            filename=dir + "/sample_standard_deviations.pdf")
        
            
    def plot_rewards(self, serializer, dir="stats", show_legend=False):        
        stats = serializer.load_stats('stats.yaml', path=dir)
        m_cov = stats['m_cov']
        mean_rewards_sets = []
        labels = []
        mean_rewards = []
        
        files = glob.glob(os.path.join(os.path.join(dir, "mean_rewards*.yaml")))
        for file in sorted(files):
            file_str = file
            try:
                file_str = file.split("/")[-1].split(".")[0]
                #file_str = file.split("/")[1].split(".")[0].split("_")[1]                
            except Exception as e:
                logging.error("PlotStats: " + str(e))   
            
            mean_rewards.append(serializer.load_stats(file))            
            data = []
            for k in xrange(len(m_cov)):
                data.append(np.array([m_cov[k], mean_rewards[-1][k]]))
            mean_rewards_sets.append(np.array(data))
            label_string = ""
            for i in xrange(len(file_str.split("_"))):
                if i != 0:
                    label_string += " " + file_str.split("_")[i]
                else:
                    label_string += file_str.split("_")[i]
            labels.append(label_string)        
        min_m = [min(m) for m in mean_rewards]
        max_m = [max(m) for m in mean_rewards]               
        Plot.plot_2d_n_sets(mean_rewards_sets,
                            labels=labels,
                            xlabel="joint covariance",
                            ylabel="mean reward",
                            x_range=[m_cov[0], m_cov[-1]],
                            y_range=[min(min_m), max(max_m) * 1.05],
                            show_legend=show_legend,
                            save=self.save,
                            filename=dir + "/mean_rewards.pdf")
        
    def plot_mean_planning_times(self, serializer, dir="stats", filename="", output=""): 
        if filename == "":
            filename = "mean_planning_times_per_step*.yaml"
        if output == "":
            output = "mean_planning_times_per_step.pdf"       
        stats = serializer.load_stats('stats.yaml', path=dir)
        m_cov = stats['m_cov']
        sets = []
        labels = []
        mean_planning_times = []
        for file in glob.glob(os.path.join(os.path.join(dir, filename))):
            file_str = file
            try:
                file_str = file.split("/")[1].split(".")[0].split("_")[1]
            except:
                pass
                   
            #mean_rewards = serializer.load_stats('rewards.yaml', path="stats")
            mean_planning_times.append(serializer.load_stats(file))            
            data = []
            for k in xrange(len(m_cov)):
                data.append(np.array([m_cov[k], mean_planning_times[-1][k]]))
            sets.append(np.array(data))
            labels.append(file_str)        
        if not len(mean_planning_times) == 0:
            min_m = [min(m) for m in mean_planning_times]
            max_m = [max(m) for m in mean_planning_times]
            Plot.plot_2d_n_sets(sets,
                                labels=labels,
                                xlabel="joint covariance",
                                ylabel="mean planning times (seconds)",
                                x_range=[m_cov[0], m_cov[-1]],
                                y_range=[min(min_m), max(max_m) * 1.05],
                                show_legend=True,
                                save=self.save,
                                filename=dir + "/" + output)
        
    def plot_emd_graph(self, serializer, cartesian_coords, dir="stats"):
        stats = serializer.load_stats('stats.yaml', path=dir) 
        config = serializer.read_config(path=dir)       
        #emd = stats['emd']
        m_cov = stats['m_cov']
        
        emds = []
        for k in xrange(len(cartesian_coords)):
            #cart_coords.append([cartesian_coords[i] for i in xrange(len(cartesian_coords))])                
            emds.append(calc_EMD(cartesian_coords[k], config['num_bins']))
        
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
        
        for k in xrange(len(cart_coords)):                    
            X = np.array([cart_coords[k][i][0] for i in xrange(len(cart_coords[0]))])
            Y = np.array([cart_coords[k][i][1] for i in xrange(len(cart_coords[0]))])
            histogram_range = [[-3.1, 3.1], [-3.1, 3.1]]
            H, xedges, yedges = get_2d_histogram(X, Y, histogram_range, bins=config['num_bins'])        
            Plot.plot_histogram(H, xedges, yedges, save=self.save, path=dir, filename="hist"+ str(k) + ".png")
            
    def plot_paths(self, serializer, best_paths=False, dir="stats"):
        config = serializer.read_config(path=dir)
        dim = config['num_links']        
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
        obstacles = serializer.load_environment("env.xml", path=dir + "/environment")        
        if not obstacles == None:
            for obstacle in obstacles:                
                point1 = [obstacle[0][0] - obstacle[1][0] / 2.0, obstacle[0][1] - obstacle[1][1] / 2.0]
                point2 = [obstacle[0][0] - obstacle[1][0] / 2.0, obstacle[0][1] + obstacle[1][1] / 2.0]
                point3 = [obstacle[0][0] + obstacle[1][0] / 2.0, obstacle[0][1] + obstacle[1][1] / 2.0]
                point4 = [obstacle[0][0] + obstacle[1][0] / 2.0, obstacle[0][1] - obstacle[1][1] / 2.0]
                sets.append(np.array([point1, point2]))                
                sets.append(np.array([point2, point3]))
                sets.append(np.array([point3, point4]))
                sets.append(np.array([point4, point1])) 
                colors.extend(['k' for j in xrange(4)])                       
        Plot.plot_2d_n_sets(sets,
                            colors=colors, 
                            xlabel='x', 
                            ylabel='y', 
                            x_range=[-3.5, 3.5], 
                            y_range=[-3.5, 3.5],
                            plot_type="lines",
                            show_legend=False,
                            save=self.save,
                            path=dir,
                            filename=filename)    
    
        
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
        logging.error("Wrong number of arguments. Should be 'python plot_stats ALGORITHM SAVE'")
        sys.exit()   
    PlotStats(False)
import sys
import numpy as np
import plot as Plot
import glob
import os
from serializer import Serializer
from kinematics import Kinematics
from EMD import *

class PlotStats:
    def __init__(self, save, algorithm):
        dir = "stats/" + str(algorithm)
        if not os.path.isdir(dir):
            os.makedirs(dir)      
        self.save = save
        print "Loading cartesian coordinates..."  
        serializer = Serializer()
        print "plotting paths"    
        self.plot_paths(serializer, dir=dir)
        self.plot_paths(serializer, best_paths=True, dir=dir)
        
        print "plotting average distance to goal"
        self.plot_average_dist_to_goal(serializer, dir=dir)
        print "plotting mean rewards"
        self.plot_mean_rewards(serializer, dir=dir, show_legend=True)        
        print "plotting mean planning times"
        self.plot_mean_planning_times(serializer, dir=dir) 
        cart_coords = serializer.load_cartesian_coords(dir, "cartesian_coords_" + algorithm + ".yaml")
        print "plotting EMD graph..."
        self.plot_emd_graph(serializer, cart_coords, dir=dir) 
        print "plotting histograms..."        
        self.save_histogram_plots(serializer, cart_coords, dir=dir)       
        
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
        for file in glob.glob(os.path.join(os.path.join(dir, "cartesian_coords*"))):
            file_str = file
            try:                
                file_str = file.split("/")[-1].split("_")[-1].split(".")[0]
                file_str = "avg_distance_" + file_str
                #file_str = file.split("/")[1].split(".")[0].split("_")[1] 
                labels.append(file_str)               
            except Exception as e:
                print e
            data = []
            cart_coords = serializer.load_cartesian_coords(dir, file.split("/")[-1])
            dists = []            
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
            
    def plot_mean_rewards(self, serializer, dir="stats", show_legend=False):        
        stats = serializer.load_stats('stats.yaml', path=dir)
        m_cov = stats['m_cov']
        sets = []
        labels = []
        mean_rewards = []
        for file in glob.glob(os.path.join(os.path.join(dir, "rewards*"))):
            file_str = file
            try:
                file_str = file.split("/")[-1].split(".")[0]
                #file_str = file.split("/")[1].split(".")[0].split("_")[1]                
            except Exception as e:
                print e
                
                   
            #mean_rewards = serializer.load_stats('rewards.yaml', path="stats")
            mean_rewards.append(serializer.load_stats(file))            
            data = []
            for k in xrange(len(m_cov)):
                data.append(np.array([m_cov[k], mean_rewards[-1][k]]))
            sets.append(np.array(data))
            labels.append(file_str)
        print mean_rewards
        min_m = [min(m) for m in mean_rewards]
        max_m = [max(m) for m in mean_rewards]
        print "min_m " + str(min_m)
        print "max_m " + str(max_m)
        
        Plot.plot_2d_n_sets(sets,
                            labels=labels,
                            xlabel="joint covariance",
                            ylabel="mean reward",
                            x_range=[m_cov[0], m_cov[-1]],
                            y_range=[min(min_m), max(max_m)],
                            show_legend=show_legend,
                            save=self.save,
                            filename=dir + "/mean_rewards.pdf")
        
    def plot_mean_planning_times(self, serializer, dir="stats"):        
        stats = serializer.load_stats('stats.yaml', path=dir)
        m_cov = stats['m_cov']
        sets = []
        labels = []
        mean_planning_times = []
        for file in glob.glob(os.path.join(os.path.join(dir, "mean_planning_times.yaml"))):
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
        print mean_planning_times
        if not len(mean_planning_times) == 0:
            min_m = [min(m) for m in mean_planning_times]
            max_m = [max(m) for m in mean_planning_times]
            print "min_m " + str(min_m)
            print "max_m " + str(max_m)
            
            Plot.plot_2d_n_sets(sets,
                                labels=labels,
                                xlabel="joint covariance",
                                ylabel="mean planning times (seconds)",
                                x_range=[m_cov[0], m_cov[-1]],
                                y_range=[min(min_m), max(max_m)],
                                show_legend=True,
                                save=self.save,
                                filename=dir + "/mean_planning_times.png")
        
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
        kinematics = Kinematics(dim)
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
                path_coords.append(kinematics.get_end_effector_position(state))
            sets.append(np.array(path_coords)) 
        obstacles = serializer.load_obstacles("obstacles.yaml", path=dir)
        print "obstacles " + str(obstacles)
        if not obstacles == None:
            for obstacle in obstacles:
                point1 = [obstacle[0] - obstacle[2] / 2.0, obstacle[1] - obstacle[3] / 2.0]
                point2 = [obstacle[0] - obstacle[2] / 2.0, obstacle[1] + obstacle[3] / 2.0]
                point3 = [obstacle[0] + obstacle[2] / 2.0, obstacle[1] + obstacle[3] / 2.0]
                point4 = [obstacle[0] + obstacle[2] / 2.0, obstacle[1] - obstacle[3] / 2.0]
                sets.append(np.array([point1, point2]))
                sets.append(np.array([point2, point3]))
                sets.append(np.array([point3, point4]))
                sets.append(np.array([point4, point1]))
                print "WOW"              
        Plot.plot_2d_n_sets(sets, 
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
    if len(sys.argv) > 1:
        algorithm = sys.argv[1]
        if "save" in sys.argv[2]:
            PlotStats(True, algorithm)
            sys.exit()       
        PlotStats(False)
        sys.exit()   
    PlotStats(False)
import sys
import numpy as np
import plot as Plot
import glob
import os
from serializer import Serializer
from kinematics import Kinematics
from EMD import *

class PlotStats:
    def __init__(self, save):
        if not os.path.isdir("stats"):
            os.makedirs("stats")      
        self.save = save
        print "Loading cartesian coordinates..."  
        serializer = Serializer()
        print "plotting paths"    
        self.plot_paths(serializer)
        self.plot_paths(serializer, best_paths=True)
        cart_coords = serializer.load_cartesian_coords(path="stats")
        print "plotting average distance to goal"
        self.plot_average_dist_to_goal(serializer, cart_coords)
        print "plotting mean rewards"
        self.plot_mean_rewards(serializer)
        print "plotting EMD graph..."
        self.plot_emd_graph(serializer, cart_coords) 
        print "plotting histograms..." 
        self.save_histogram_plots(serializer, cart_coords)        
        
    def clear_stats(self):
        for file in glob.glob("stats/*"):
            os.remove(file)
            
    def plot_mean_rewards(self, serializer):        
        stats = serializer.load_stats('stats.yaml', path="stats")
        m_cov = stats['m_cov']
        sets = []
        for file in glob.glob(os.path.join(os.path.join("stats", "rewards*"))):
            print file        
            #mean_rewards = serializer.load_stats('rewards.yaml', path="stats")
            mean_rewards = serializer.load_stats(file)
            print "mean_rewards " + str(mean_rewards)
            data = []
            for k in xrange(len(m_cov)):
                data.append(np.array([m_cov[k], mean_rewards[k]]))
            sets.append(np.array(data))
            print "APPENDED"
        Plot.plot_2d_n_sets(sets,
                            xlabel="joint covariance",
                            ylabel="mean reward",
                            x_range=[m_cov[0], m_cov[-1]],
                            y_range=[min(mean_rewards), max(mean_rewards)],
                            show_legend=False,
                            save=self.save,
                            filename="stats/mean_rewards.png")       
        
    def plot_average_dist_to_goal(self, serializer, cart_coords):
        config = serializer.read_config("config.yaml", path="stats")
        stats = serializer.load_stats('stats.yaml', path="stats")
        m_cov = stats['m_cov']
        data = []
        max_avg_distance = 0.0
        for k in xrange(len(m_cov)):
            print "cov " + str(m_cov[k])
            dists = []            
            for coords in cart_coords[k]:
                dists.append(np.linalg.norm(np.array(coords) - np.array(config['goal_position'])))            
            avg_distance = 0.0
            for d in dists:
                avg_distance += d
            avg_distance /= len(dists)
            if avg_distance > max_avg_distance:
                max_avg_distance = avg_distance
            print "Average distance for covariacne " + str(m_cov[k]) + " calculated"            
            data.append(np.array([m_cov[k], avg_distance]))
        
        Plot.plot_2d_n_sets([np.array(data)],
                            xlabel="joint covariance",
                            ylabel="average distance to goal",
                            x_range=[m_cov[0], m_cov[-1]],
                            y_range=[0, max_avg_distance],
                            show_legend=False,
                            save=self.save,
                            filename="stats/avg_distance.png")
        
    def plot_emd_graph(self, serializer, cartesian_coords):
        stats = serializer.load_stats('stats.yaml', path="stats") 
        config = serializer.read_config('config.yaml', path="stats")       
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
                            path="stats",
                            filename="emd.png")        
        
    def save_histogram_plots(self, serializer, cart_coords):
        config = serializer.read_config("config.yaml", path="stats")
        
        for k in xrange(len(cart_coords)):                    
            X = np.array([cart_coords[k][i][0] for i in xrange(len(cart_coords[0]))])
            Y = np.array([cart_coords[k][i][1] for i in xrange(len(cart_coords[0]))])
            histogram_range = [[-3.1, 3.1], [-3.1, 3.1]]
            H, xedges, yedges = get_2d_histogram(X, Y, histogram_range, bins=config['num_bins'])        
            Plot.plot_histogram(H, xedges, yedges, save=self.save, path="stats", filename="hist"+ str(k) + ".png")
            
    def plot_paths(self, serializer, best_paths=False):
        config = serializer.read_config('config.yaml', path="stats")
        dim = config['num_links']
        kinematics = Kinematics(dim)
        if best_paths:
            paths = serializer.load_paths("best_paths.yaml", path="stats")
            filename = "best_paths.png"
        else:
            paths = serializer.load_paths("paths.yaml", path="stats")            
            filename = "paths.png"
        sets = []
        for path in paths:
            path_coords = []
            for elem in path:
                state = [elem[i] for i in xrange(dim)]
                path_coords.append(kinematics.get_end_effector_position(state))
            sets.append(np.array(path_coords)) 
        obstacles = serializer.load_obstacles("obstacles.yaml", path="stats")
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
                            path="stats",
                            filename=filename)    
    
        
if __name__ == "__main__":
    if len(sys.argv) > 1:
        if "save" in sys.argv[1]:
            PlotStats(True)
            sys.exit()       
        PlotStats(False)
        sys.exit()   
    PlotStats(False)
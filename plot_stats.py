import sys
import numpy as np
import plot as Plot
import glob
import os
from serializer import Serializer
from EMD import *

class PlotStats:
    def __init__(self, save):        
        self.save = save
        print "Loading cartesian coordinates..."  
        serializer = Serializer()
        cart_coords = serializer.load_cartesian_coords()
        print "plotting average distance to goal"
        self.plot_average_dist_to_goal(serializer, cart_coords)
        print "plotting EMD graph..."
        self.plot_emd_graph(serializer, cart_coords) 
        print "plotting histograms..." 
        self.save_histogram_plots(serializer, cart_coords)        
        
    def plot_average_dist_to_goal(self, serializer, cart_coords):
        config = serializer.read_config("config.yaml")
        stats = serializer.load_stats('stats.yaml')
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
                            filename="histograms/avg_distance.png")
        
    def plot_emd_graph(self, serializer, cartesian_coords):
        stats = serializer.load_stats('stats.yaml') 
        config = serializer.read_config('config.yaml')       
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
                            filename="histograms/emd.png")        
        
    def save_histogram_plots(self, serializer, cart_coords):
        config = serializer.read_config("config.yaml")
        
        for k in xrange(len(cart_coords)):                    
            X = np.array([cart_coords[k][i][0] for i in xrange(len(cart_coords[0]))])
            Y = np.array([cart_coords[k][i][1] for i in xrange(len(cart_coords[0]))])
            histogram_range = [[-3.1, 3.1], [-3.1, 3.1]]
            H, xedges, yedges = get_2d_histogram(X, Y, histogram_range, bins=config['num_bins'])        
            Plot.plot_histogram(H, xedges, yedges, save=self.save, filename="histograms/hist"+ str(k) + ".png")
    
        
if __name__ == "__main__":
    if len(sys.argv) > 1:
        if "save" in sys.argv[1]:
            PlotStats(True)
            sys.exit()       
        PlotStats(False)
        sys.exit()   
    PlotStats(False)
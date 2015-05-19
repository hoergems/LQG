import sys
import numpy as np
import plot as Plot
from serializer import Serializer
from EMD import *

class PlotStats:
    def __init__(self, save):
        self.save = save
        serializer = Serializer()
        self.plot_emd_graph(serializer)  
        self.save_histogram_plots(serializer)      
        
    def plot_emd_graph(self, serializer):
        stats = serializer.load_stats('stats.yaml')        
        emd = stats['emd']
        m_cov = stats['m_cov']
        arr = np.array([np.array([m_cov[i], emd[i]]) for i in xrange(len(emd))])
        Plot.plot_2d_n_sets([arr], 
                            xlabel='joint covariance', 
                            ylabel='Wasserstein distance', 
                            x_range=[0.0, np.pi], 
                            y_range=[0, 700],
                            save=True,
                            filename="histograms/emd.png")        
        
    def save_histogram_plots(self, serializer):
        config = serializer.read_config("config.yaml")
        cart_coords = serializer.load_cartesian_coords()
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
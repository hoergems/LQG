import numpy as np
import plot as Plot
from serializer import Serializer

class PlotStats:
    def __init__(self):
        serializer = Serializer("")
        stats = serializer.load_stats('stats.yaml')
        emd = stats['emd']
        m_cov = stats['m_cov']
        arr = np.array([np.array([m_cov[i], emd[i]]) for i in xrange(len(emd))])
        Plot.plot_2d_n_sets([arr], xlabel='joint covariance', ylabel='Wasserstein distance', x_range=[0.0, 0.01], y_range=[0, 250])
        print emd
        print m_cov
        
        
    
        
if __name__ == "__main__":
    PlotStats()
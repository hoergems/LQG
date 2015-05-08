import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import pylab as p
import numpy as np
from scipy import stats

def plot_2d_three_sets(sets, axis='xy', xrange=[0.0, 1.0], yrange=[0.0, 1.0]):
    if axis == 'xy':
        plt.plot(sets[0][:,0], sets[0][:,1], 'ro', 
                 sets[1][:,0], sets[1][:,1], 'go', 
                 sets[2][:,0], sets[2][:,1], 'bo')
        plt.xlabel('x')
        plt.ylabel('y')
    plt.xlim([xrange[0], xrange[1]])
    plt.ylim([yrange[0], yrange[1]])
    plt.show()
    
def plot_2d_n_sets(sets, labels=[], axis='xy', x_range=[0.0, 1.0], y_range=[0.0, 1.0]):
    ps = []
    if len(labels) != len(sets):
        labels=['default' for i in xrange(len(sets))]
    print len(sets)
    for i in xrange(len(sets)):
        p, = plt.plot(sets[i][:,0], sets[i][:,1], label=labels[i])
        ps.append(p)
    plt.legend(ps)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.xlim([x_range[0], x_range[1]])
    plt.ylim([y_range[0], y_range[1]])
    plt.show()

def plot_2d_two_sets(sets, labels, axis='xy', xrange=[0.0, 1.0], yrange=[0.0, 1.0]):
    if axis == 'xy':
        p1, = plt.plot(sets[0][:,0], sets[0][:,1], label=labels[0])
        p2, = plt.plot(sets[1][:,0], sets[1][:,1], label=labels[1])
        #plt.plot(sets[0][:,0], sets[0][:,1], 'ro', sets[1][:,0], sets[1][:,1], 'bo')
        plt.legend([p1, p2])
        plt.xlabel('x')
        plt.ylabel('y')
    plt.xlim([xrange[0], xrange[1]])
    plt.ylim([yrange[0], yrange[1]])
    plt.show()

def plot_2d_points(points, axis='xy', xrange=[0.0, 1.0], yrange=[0.0, 1.0]):
    if axis == 'xy':
        plt.plot(points[:,0], points[:,1], 'ro')
        plt.xlabel('x')
        plt.ylabel('y')
    elif axis == 'xz':
        plt.plot(points[:,0], points[:,2], 'ro')
        plt.xlabel('x')
        plt.ylabel('z')            
    elif axis == 'yz':
        plt.plot(points[:,1], points[:,2], 'ro')
        plt.xlabel('y')
        plt.ylabel('z')        
        
    plt.xlim([xrange[0], xrange[1]])
    plt.ylim([yrange[0], yrange[1]])
    plt.show()
    
    
def plot_heat_2d(m1, m2, xmin, xmax, ymin, ymax):
    X, Y = np.mgrid[xmin:xmax:1000j, ymin:ymax:1000j]    
    positions = np.vstack([X.ravel(), Y.ravel()])
    values = np.vstack([m1, m2])
    kernel = stats.gaussian_kde(values)    
    Z = np.reshape(kernel(positions).T, X.shape)
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.imshow(np.rot90(Z), cmap=plt.cm.gist_earth_r,
              extent=[xmin, xmax, ymin, ymax])
    ax.plot(m1, m2, 'k.', markersize=2)
    ax.set_xlim([xmin, xmax])
    ax.set_ylim([ymin, ymax])
    plt.show()
        
def plot_3d_points(points, x_scale=[-4.0, 4.0], y_scale=[-4.0, 4.0], z_scale=[-4.0, 4.0]):
    fig = p.figure()
    ax = Axes3D(fig)
    #ax.set_xticks([-1.0, 1.0])
    x = points[:,0]
    y = points[:,1]
    z = points[:,2]
    ax.set_xlabel("v1")
    ax.set_ylabel("v2")
    ax.set_zlabel("alpha")
    ax.view_init(elev=17.0, azim=-58.0)
    #ax.auto_scale_xyz([-1.0, 1.0], [-1.0, 1.0], [-1.0, 1.0])
    ax.scatter(x, y, z, c='r')
    ax.auto_scale_xyz(x_scale, y_scale, z_scale)
    plt.show()
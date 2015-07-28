#!/usr/bin/env python
import yaml
import glob
import os
import numpy as np

class Serializer:
    def __init__(self):
        pass
        
    def read_config(self, filename=None, path=""):
        if filename == None:
            filename = glob.glob(os.path.join(path, "config*"))[0]
            if "mpc" in filename:
                filename = "config_mpc.yaml"
            else:
                filename = "config.yaml"
        try:
            print "filename " + str(filename) 
            return yaml.load(open(os.path.join(path, filename), 'r'), yaml.CLoader)
        except:
            print "Can't read " + path + "/" + filename + ". No such file" 
            return None
        
    def save_stats(self, stats, path=""):
        if not os.path.exists(path):
            os.makedirs(path)
        for file in glob.glob(os.path.join(path, "stats.yaml")):
            os.remove(file)
        with open(os.path.join(path, "stats.yaml"), 'w') as f:
            f.write(yaml.dump(stats, default_flow_style=False))
            
    def save_rewards(self, rewards, path=""):
        if not os.path.exists(path):
            os.makedirs(path)
        for file in glob.glob(os.path.join(path, "rewards.yaml")):
            os.remove(file)
        with open(os.path.join(path, "rewards.yaml"), 'w') as f:
            f.write(yaml.dump(rewards, default_flow_style=False))
            
    def save_avg_path_lengths(self, lengths, path=""):
        if not os.path.exists(path):
            os.makedirs(path)
        for file in glob.glob(os.path.join(path, "avg_path_lengths.yaml")):
            os.remove(file)
        with open(os.path.join(path, "avg_path_lengths.yaml"), 'w') as f:
            f.write(yaml.dump(lengths, default_flow_style=False))
            
    def save_mean_planning_times(self, planning_times, path=""):
        if not os.path.exists(path):
            os.makedirs(path)
        for file in glob.glob(os.path.join(path, "mean_planning_times.yaml")):
            os.remove(file)
        with open(os.path.join(path, "mean_planning_times.yaml"), 'w') as f:
            f.write(yaml.dump(planning_times, default_flow_style=False))
            
    def load_stats(self, filename, path=""):
        with open(os.path.join(path, filename), 'r') as f:
            return yaml.load(f, yaml.CLoader)
            
    def save_paths(self, paths, filename, overwrite, path=""):
        if not os.path.exists(path):
            os.makedirs(path)
        path_arrays = []
        if overwrite:
            for file in glob.glob(os.path.join(path, filename)):
                os.remove(file)            
        else:
            try:
                path_arrays = self.load_paths(filename, path)
                for file in glob.glob(os.path.join(path, filename)):
                    os.remove(file)
            except:
                print "Couldn't load paths.yaml"        
        for i in xrange(len(paths)):                       
            path_arr = []           
            for j in xrange(len(paths[i][0])):
                el = []
                el.extend(paths[i][0][j])
                el.extend(paths[i][1][j])
                el.extend(paths[i][2][j])
                path_arr.append(el)
            path_arrays.append(path_arr)  
        d = dict(paths = path_arrays)        
        with open(os.path.join(path, filename), 'a+') as f:
            f.write(yaml.dump(d, default_flow_style=False))
            
    def save_cartesian_coords(self, cartesian_coords, path=""):
        if not os.path.exists(path):
            os.makedirs(path)
        for file in glob.glob(os.path.join(path, "cartesian_coords.yaml")):
            os.remove(file)
        with open(os.path.join(path, "cartesian_coords.yaml"), 'w') as f:                
            f.write(yaml.dump(cartesian_coords, default_flow_style=False))
            
            
    def load_cartesian_coords(self, path=""):        
        with open(os.path.join(path, "cartesian_coords.yaml"), 'r') as f:
            return yaml.load(f, yaml.CLoader)
    
    def load_paths(self, file, path=""):
        try:        
            paths = yaml.load(open(os.path.join(path, file), 'r'), yaml.CLoader) 
        except IOError:
            print "No such file or directory: " + str(os.path.join(path, file))
            return []       
        return paths['paths']
    
    def load_obstacles(self, file, path=""):
        return yaml.load(open(os.path.join(path, file), 'r'), yaml.CLoader)
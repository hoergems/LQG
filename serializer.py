#!/usr/bin/env python
import yaml
import glob
import os

class Serializer:
    def __init__(self):
        pass
        
    def read_config(self, filename):
        try:            
            return yaml.load(open(filename, 'r'), yaml.CLoader)
        except:
            print "Can't read " + filename + ". No such file" 
            return None
        
    def save_stats(self, stats):
        for file in glob.glob("stats.yaml"):
            os.remove(file)
        with open('stats.yaml', 'w') as f:
            f.write(yaml.dump(stats, default_flow_style=False))
            
    def load_stats(self, filename):
        with open(filename, 'r') as f:
            return yaml.load(f, yaml.CLoader)
            
    def save_paths(self, paths, filename, overwrite):
        path_arrays = []
        if overwrite:
            for file in glob.glob(filename):
                os.remove(file)            
        else:
            try:
                path_arrays = self.load_paths(filename)
                for file in glob.glob(filename):
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
        with open(filename, 'a+') as f:
            f.write(yaml.dump(d, default_flow_style=False))
            
    def save_cartesian_coords(self, cartesian_coords):
        for file in glob.glob('cartesian_coords.yaml'):
            os.remove(file)
        with open('cartesian_coords.yaml', 'w') as f:
            f.write(yaml.dump(cartesian_coords, default_flow_style=False))
            
    def load_cartesian_coords(self):
        with open("cartesian_coords.yaml", 'r') as f:
            return yaml.load(f, yaml.CLoader)
    
    def load_paths(self, file):        
        paths = yaml.load(open(file, 'r'), yaml.CLoader)        
        return paths['paths']
                
        
        

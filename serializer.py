#!/usr/bin/env python
import yaml
import glob
import os

class Serializer:
    def __init__(self, filename):
        self.filename = filename
        
    def read_config(self):
        try:            
            return yaml.load(open(self.filename, 'r'))
        except:
            print "Can't read " + self.filename + ". No such file" 
            return None
        
    def save_stats(self, stats):
        for file in glob.glob("stats.yaml"):
            os.remove(file)
        with open('stats.yaml', 'w') as f:
            f.write(yaml.dump(stats, default_flow_style=False))
            
    def load_stats(self, filename):
        with open(filename, 'r') as f:
            return yaml.load(f)
            
    def save_paths(self, paths):
        for file in glob.glob('paths.yaml'):
            os.remove(file)
        d = dict(paths = paths)        
        with open('paths.yaml', 'w') as f:
            f.write(yaml.dump(d, default_flow_style=False))
            
    def save_cartesian_coords(self, cartesian_coords):
        for file in glob.glob('cartesian_coords.yaml'):
            os.remove(file)
        with open('cartesian_coords.yaml', 'w') as f:
            f.write(yaml.dump(cartesian_coords, default_flow_style=False))
    
    def load_paths(self, file):        
        paths = yaml.load(open(file, 'r'))
        return paths['paths']
                
        
        

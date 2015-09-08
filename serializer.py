#!/usr/bin/env python
import yaml
import glob
import os
import numpy as np
import logging
from obstacle import Obstacle
from xml.dom import minidom

class Serializer:
    def __init__(self):
        pass
        
    def read_config(self, filename=None, path=""):        
        if filename == None:
            filename = glob.glob(os.path.join(path, "config*"))[0]
            if "mpc" in filename:
                filename = "config_mpc.yaml"
            else:
                filename = "config_lqg.yaml"
        try:            
            return yaml.load(open(os.path.join(path, filename), 'r'), yaml.CLoader)
        except:
            logging.error("Serializer: Can't read " + path + "/" + filename + ". No such file") 
            return None
        
    def save_stats(self, stats, path=""):
        if not os.path.exists(path):
            os.makedirs(path)
        for file in glob.glob(os.path.join(path, "stats.yaml")):
            os.remove(file)
        with open(os.path.join(path, "stats.yaml"), 'w') as f:
            f.write(yaml.dump(stats, default_flow_style=False))
            
    def save_rewards(self, rewards, path="", filename=""):
        if not os.path.exists(path):
            os.makedirs(path)
        if filename == "":
            filename = "rewards.yaml"
        for file in glob.glob(os.path.join(path, filename)):
            os.remove(file)
        with open(os.path.join(path, filename), 'w') as f:
            f.write(yaml.dump(rewards, default_flow_style=False))
            
    def save_mean_num_generated_paths(self, mean_num_generated_paths, path="", filename=""):
        if not os.path.exists(path):
            os.makedirs(path)
        if filename == "":
            filename = "mean_num_generated_paths.yaml"
        for file in glob.glob(os.path.join(path, filename)):
            os.remove(file)
        with open(os.path.join(path, filename), 'w') as f:
            f.write(yaml.dump(mean_num_generated_paths, default_flow_style=False))
            
    def save_mean_num_planning_steps(self, mean_num_planning_steps, path="", filename=""):
        if not os.path.exists(path):
            os.makedirs(path)
        if filename == "":
            filename = "mean_num_planning_steps_per_run.yaml"
        for file in glob.glob(os.path.join(path, filename)):
            os.remove(file)
        with open(os.path.join(path, filename), 'w') as f:
            f.write(yaml.dump(mean_num_planning_steps, default_flow_style=False))
            
    def save_avg_path_lengths(self, lengths, path=""):
        if not os.path.exists(path):
            os.makedirs(path)
        for file in glob.glob(os.path.join(path, "avg_path_lengths.yaml")):
            os.remove(file)
        with open(os.path.join(path, "avg_path_lengths.yaml"), 'w') as f:
            f.write(yaml.dump(lengths, default_flow_style=False))
            
    def save_num_successes(self, successes, path="", filename=""):
        if not os.path.exists(path):
            os.makedirs(path)
        if filename == "":
            filename = "num_successes.yaml"
        for file in glob.glob(os.path.join(path, filename)):
            os.remove(file)
        with open(os.path.join(path, filename), 'w') as f:
            f.write(yaml.dump(successes, default_flow_style=False))
            
    def save_mean_planning_times(self, planning_times, path="", filename=""):
        if not os.path.exists(path):
            os.makedirs(path)
        if filename == "":
            filename = "mean_planning_times.yaml"
        for file in glob.glob(os.path.join(path, filename)):
            os.remove(file)
        with open(os.path.join(path, filename), 'w') as f:
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
                logging.error("Serializer: Couldn't load paths.yaml")        
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
            
    def save_cartesian_coords(self, cartesian_coords, path="", filename=None):
        if not os.path.exists(path):
            os.makedirs(path)
        if filename == None:
            filename = "cartesian_coords.yaml"
        for file in glob.glob(os.path.join(path, filename)):
            os.remove(file)
        with open(os.path.join(path, filename), 'w') as f:                
            f.write(yaml.dump(cartesian_coords, default_flow_style=False))           
                 
            
    def load_cartesian_coords(self, path="", file=None): 
        if file == None:
            file = "cartesian_coords.yaml"       
        with open(os.path.join(path, file), 'r') as f:
            return yaml.load(f, yaml.CLoader)
        
    def save_total_rewards(self, total_rewards, path="", filename=None):
        if not os.path.exists(path):
            os.makedirs(path)
        if filename == None:
            filename = "total_rewards.yaml"
        for file in glob.glob(os.path.join(path, filename)):
            os.remove(file)
        with open(os.path.join(path, filename), 'w') as f:                
            f.write(yaml.dump(total_rewards, default_flow_style=False))
            
            
    def load_num_successes(self, path="", filename=None):
        if filename == None:
            filename = "num_successes.yaml"       
        with open(os.path.join(path, filename), 'r') as f:
            return yaml.load(f, yaml.CLoader)
            
    def load_total_rewards(self, path="", file=None): 
        if file == None:
            file = "total_rewards.yaml"       
        with open(os.path.join(path, file), 'r') as f:
            return yaml.load(f, yaml.CLoader)
        
    def save_sample_variances(self, sample_variances, path="", filename=None):
        if not os.path.exists(path):
            os.makedirs(path)
        if filename == None:
            filename = "sample_variances.yaml"
        for file in glob.glob(os.path.join(path, filename)):
            os.remove(file)
        with open(os.path.join(path, filename), 'w') as f:                
            f.write(yaml.dump(sample_variances, default_flow_style=False))
            
    def load_sample_variances(self, path="", file=None): 
        if file == None:
            file = "sample_variances.yaml"       
        with open(os.path.join(path, file), 'r') as f:
            return yaml.load(f, yaml.CLoader)
    
    def load_paths(self, file, path=""):
        try:        
            paths = yaml.load(open(os.path.join(path, file), 'r'), yaml.CLoader) 
        except IOError:
            logging.error("Serializer: No such file or directory: " + str(os.path.join(path, file)))
            return []       
        return paths['paths']
        
    def serialize_ik_solutions(self, ik_solutions, path="", file=""):        
        for f in glob.glob(os.path.join(path, file)):
            os.remove(f)
        with open(os.path.join(path, file), 'w') as f:
            for i in xrange(len(ik_solutions)):
                for j in xrange(len(ik_solutions[i])):
                    f.write(str(ik_solutions[i][j]) + " ")
                if not i == len(ik_solutions) - 1:
                    f.write("\n")
                    
    def deserialize_joint_angles(self, path="", file=""):
        float_arrs = []
        if not file == "":
            with open(os.path.join(path, file), 'r') as f:
                for line in f.readlines():                    
                    arr = line.split(" ")                                       
                    float_arr = [float(arr[k]) for k in xrange(0, len(arr) - 1)]
                    float_arrs.append(float_arr)
            return float_arrs
        return []      
        
    
    def load_environment(self, file="env.xml", path="stats/environment"):
        try:
            xmldoc = minidom.parse(os.path.join(path, file)) 
        except Exception as e:
            logging.error("Serializer: " + str(e))
            return None            
        obstacle_translations = xmldoc.getElementsByTagName('Translation')
        obstacle_dimensions = xmldoc.getElementsByTagName('extents')
        obstacles = []
        for i in xrange(len(obstacle_translations)):
            trans = [float(k) for k in obstacle_translations[i].childNodes[0].nodeValue.split(" ")]
            dim =  [float(k) for k in obstacle_dimensions[i].childNodes[0].nodeValue.split(" ")] 
            obstacles.append([trans, dim])        
        return obstacles 
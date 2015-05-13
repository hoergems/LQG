#!/usr/bin/env python
import yaml

class ConfigReader:
    def __init__(self, filename):
        self.filename = filename
        
    def read_config(self):
        try:            
            return yaml.load(open(self.filename, 'r'))
        except:
            print "Can't read " + self.filename + ". No such file" 
            return None
        

from __future__ import division
from numpy import where, dstack, diff, meshgrid
import numpy as np
from shapely.geometry import Point, LineString
 

class Obstacle:
    def __init__(self, x, y, x_size, y_size):
        self.x = x
        self.y = y
        self.x_size = x_size
        self.y_size = y_size
        self.line1 = LineString([Point(self.x - self.x_size / 2.0, self.y - self.y_size / 2.0), 
                                 Point(self.x - self.x_size / 2.0, self.y + self.y_size / 2.0)])
        self.line2 = LineString([Point(self.x - self.x_size / 2.0, self.y + self.y_size / 2.0), 
                                 Point(self.x + self.x_size / 2.0, self.y + self.y_size / 2.0)])
        self.line3 = LineString([Point(self.x + self.x_size / 2.0, self.y + self.y_size / 2.0), 
                                 Point(self.x + self.x_size / 2.0, self.y - self.y_size / 2.0)])
        self.line4 = LineString([Point(self.x + self.x_size / 2.0, self.y - self.y_size / 2.0), 
                                 Point(self.x - self.x_size / 2.0, self.y - self.y_size / 2.0)])
        
    def collides(self, point):
        """
        Checks if a point is within the obstacle.
        Point is a list type
        """        
        if (point[0] >= self.x - (self.x_size / 2.0) and 
            point[0] <= self.x + (self.x_size / 2.0)):
            
            if (point[1] >= self.y - (self.y_size / 2.0) and
                point[1] <= self.y + (self.y_size / 2.0)):
                """
                Collistion
                """                
                return True
        return False   
    
    def manipulator_collides(self, links):
        for link in links:
            line_m = LineString([Point(link[0].tolist()[0], link[0].tolist()[1]), 
                                 Point(link[1].tolist()[0], link[1].tolist()[1])])
            #print "line_m " + str(line_m)
            inter = line_m.intersection(self.line1)            
            if not inter.is_empty:
                #print "inter " + str(inter)                
                return True
            inter = line_m.intersection(self.line2)
            if not inter.is_empty:                
                return True
            inter = line_m.intersection(self.line3)
            if not inter.is_empty:                
                return True
            inter = line_m.intersection(self.line4)
            if not inter.is_empty:                
                return True 
        return False
    

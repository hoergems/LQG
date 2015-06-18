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
                    
                       
            ''''i = self.seg_intersect(link[0], link[1], p1, p2)            
            if len(i) > 0:
                if (i[1] <= p2[1] and i[1] >= p1[1]):
                    print "i " + str(i)
                    return True
            i = self.seg_intersect(link[0], link[1], p2, p3)            
            if len(i) > 0:
                if (i[0] <= p3[0] and i[0] >= p2[0]):
                    print "i " + str(i)
                    return True
            i = self.seg_intersect(link[0], link[1], p3, p4)            
            if len(i) > 0:
                if (i[1] >= p4[1] and i[1] <= p3[1]):
                    print "i " + str(i)
                    return True
            i = self.seg_intersect(link[0], link[1], p4, p1)            
            if len(i) > 0:
                if (i[0] >= p1[0] and i[1] <= p4[1]):
                    print "i " + str(i)
                    return True'''
            
        return False
            
            
    
    def in_collision(self, point1, point2): 
        if self.collides(point1) or self.collides(point2):        
            return True
            
        p1 = [self.x - self.x_size / 2.0, self.y - self.y_size / 2.0] 
        p2 = [self.x - self.x_size / 2.0, self.y + self.y_size / 2.0]   
        p3 = [self.x + self.x_size / 2.0, self.y + self.y_size / 2.0]
        p4 = [self.x + self.x_size / 2.0, self.y - self.y_size / 2.0]
        
        coll = self.intersection_(point1, point2, p1, p2)
        if len(coll) > 0:            
            return True
            
        coll = self.intersection_(point1, point2, p2, p3)
        if len(coll) > 0:            
            return True
        
        coll = self.intersection_(point1, point2, p3, p4);
        if len(coll) > 0:            
            return True
        
        coll = self.intersection_(point1, point2, p1, p4);
        if len(coll) > 0:            
            return True
        
        return False

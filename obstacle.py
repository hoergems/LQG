

class Obstacle:
    def __init__(self, x, y, x_size, y_size):
        self.x = x
        self.y = y
        self.x_size = x_size
        self.y_size = y_size
        
    def intersection_(self, p1, p2, p3, p4):
        x1 = p1[0] 
        x2 = p2[0]
        x3 = p3[0]
        x4 = p4[0]
        y1 = p1[1] 
        y2 = p2[1]
        y3 = p3[1] 
        y4 = p4[1]
 
        d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
        
        # If d is zero, there is no intersection
        if d == 0:
             return [] 
    
        pre = (x1*y2 - y1*x2)
        post = (x3*y4 - y3*x4)
        x = ( pre * (x3 - x4) - (x1 - x2) * post ) / d
        y = ( pre * (y3 - y4) - (y1 - y2) * post ) / d
 
    
        if (x < min(x1, x2) or 
            x > max(x1, x2) or 
            x < min(x3, x4) or 
            x > max(x3, x4)):
            return [];
        if ( y < min(y1, y2) or
             y > max(y1, y2) or 
             y < min(y3, y4) or 
             y > max(y3, y4) ): 
            return []
        
        return [x, y];
        

        
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
    
    def in_collision(self, point1, point2): 
        if self.collides(point1) or self.collides(point2):        
            return True
            
        p1 = [self.x - self.x_size / 2.0, self.y - self.y_size / 2.0] 
        p2 = [self.x - self.x_size / 2.0, self.y + self.y_size / 2.0]   
        p3 = [self.x + self.x_size / 2.0, self.y + self.y_size / 2.0]
        p4 = [self.x + self.x_size / 2.0, self.y - self.y_size / 2.0]
        
        coll = self.intersection_(point1, point2, p3, p4)
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

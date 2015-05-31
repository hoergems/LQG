

class Obstacle:
    def __init__(self, x, y, x_size, y_size):
        self.x = x
        self.y = y
        self.x_size = x_size
        self.y_size = y_size
        
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
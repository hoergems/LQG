#ifndef AREA_HPP_
#define AREA_HPP_
#include <iostream>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

namespace shared { 
    
class Area {
    public:	
	    enum AreaShape {
	    	SPHERE,
			BOX
	    };
    	
    	Area(double pos_x,
    		 double pos_y,
			 double pos_z,
			 double size_x,
			 double size_y,
			 double size_z);
    	
    	Area(double pos_x,
       		 double pos_y,
   			 double pos_z,
    		 double radius);
    	
        AreaShape getAreaShape() const; 
        
        std::vector<double> getLocation() const;
        
        std::vector<double> getDimension() const;
        
        double getRadius() const;
        
    private:
        std::vector<double> location_;
        std::vector<double> dimension_;
        
        double radius_;
        
        AreaShape shape_;
    	
    };

}

#endif
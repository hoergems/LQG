#ifndef ROBOT_HPP_
#define ROBOR_HPP_
#include <string>
#include <iostream>
#include <urdf/model.h>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

namespace shared {
    class Robot {
        public:
    	    Robot(std::string robot_file);
    	    
    	    void getLinkNames(std::vector<std::string> &link_names);
    	    
    	    void getLinkMasses(std::vector<std::string> &link, std::vector<double> &link_masses);
    	    
    	    void getLinkDimension(std::vector<std::string> &link, std::vector<std::vector<double>> &dimension);
    	    
    	    void getLinkPose(std::vector<std::string> &link, std::vector<std::vector<double>> &pose);
    	    
    	    void getLinkInertialPose(std::vector<std::string> &link, std::vector<std::vector<double>> &pose);
    	    
    	    void getLinkInertias(std::vector<std::string> &link, std::vector<std::vector<double>> &inertias);
    	    
    	    void getJointNames(std::vector<std::string> &joint_names);
    	    
    	    void getJointType(std::vector<std::string> &joint, std::vector<std::string> &type);
    	    
    	    void getJointOrigin(std::vector<std::string> &joints, std::vector<std::vector<double>> &origins);
    	    
    	    void getJointAxis(std::vector<std::string> &joints, std::vector<std::vector<double>> &axis);
    	    
    	    
    
        private:
            std::string robot_file_;
            
            std::shared_ptr<urdf::Model> model_;
            
            std::vector<boost::shared_ptr<urdf::Joint>> joints_;
    
    	
    };
    		
}

#endif
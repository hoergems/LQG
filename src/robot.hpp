#ifndef ROBOT_HPP_
#define ROBOR_HPP_
#include <string>
#include <iostream>
#include <urdf/model.h>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <tinyxml.h>

namespace shared {
    class Robot {
        public:
    	    Robot(std::string robot_file);
    	    
    	    void getLinkNames(std::vector<std::string> &link_names);
    	    
    	    void getLinkMasses(std::vector<std::string> &link, std::vector<double> &link_masses);
    	    
    	    void getLinkDimension(std::vector<std::string> &link, std::vector<std::vector<double>> &dimension);
    	    
    	    void getActiveLinkDimensions(std::vector<std::vector<double>> &dimensions);
    	    
    	    void getLinkPose(std::vector<std::string> &link, std::vector<std::vector<double>> &pose);
    	    
    	    void getLinkInertialPose(std::vector<std::string> &link, std::vector<std::vector<double>> &pose);
    	    
    	    void getLinkInertias(std::vector<std::string> &link, std::vector<std::vector<double>> &inertias);
    	    
    	    void getJointNames(std::vector<std::string> &joint_names);
    	    
    	    void getActiveJoints(std::vector<std::string> &joints);
    	    
    	    void getJointType(std::vector<std::string> &joint, std::vector<std::string> &type);
    	    
    	    void getJointOrigin(std::vector<std::string> &joints, std::vector<std::vector<double>> &origins);
    	    
    	    void getJointAxis(std::vector<std::string> &joints, std::vector<std::vector<double>> &axis);
    	    
    	    
    
        private:
            std::string robot_file_;
            
            std::shared_ptr<urdf::Model> model_;
            
            std::vector<boost::shared_ptr<urdf::Joint>> joints_;
            
            std::vector<std::string> link_names_;
            
            std::vector<std::string> active_link_names_;
            
            std::vector<std::string> joint_names_;
            
            std::vector<std::string> joint_types_;
            
            std::vector<std::string> active_joints_;
            
            std::vector<std::vector<double>> joint_origins_;
            
            std::vector<std::vector<double>> link_origins_;
            
            std::vector<std::vector<double>> active_joint_origins_;
            
            std::vector<std::vector<double>> joint_axes_;
            
            std::vector<double> joint_torque_limits_;
            
            std::vector<double> active_joint_torque_limits_;
            
            std::vector<double> lower_joint_limits_;
            
            std::vector<double> upper_joint_limits_;
            
            std::vector<double> active_lower_joint_limits_;
			
			std::vector<double> active_upper_joint_limits_;
			
			std::vector<double> joint_velocity_limits_;
			
			std::vector<double> active_joint_velocity_limits_;
            
            std::vector<std::vector<double>> active_joint_axes_;
            
            std::vector<std::string> active_links_;
            
            std::vector<double> link_masses_;
            
            std::vector<std::vector<double>> link_inertia_origins_;
            
            std::vector<std::vector<double>> link_inertia_matrices_;
            
            std::vector<std::vector<double>> link_dimensions_;
            
            bool initLinks(TiXmlElement *robot_xml);
            
            bool initJoints(TiXmlElement *robot_xml);
            
            unsigned int get_link_index(std::string &link_name);
            
            std::vector<double> process_origin_(TiXmlElement *xml);
    
    	
    };
    		
}

#endif
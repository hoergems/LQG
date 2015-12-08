#ifndef ROBOT_HPP_
#define ROBOT_HPP_
#include <string>
#include <iostream>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/algorithm/string.hpp>
#include "fcl/BV/BV.h" 
#include "fcl/collision_object.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/shape/geometric_shapes_utility.h"
#include <tinyxml.h>
#include "propagator.hpp"
#include "Kinematics.hpp"

namespace shared {

struct RobotState {
	std::vector<double> joint_values;
	std::vector<double> joint_velocities;
};

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
    	    
    	    void getJointAxis(std::vector<std::string> &joints, std::vector<std::vector<int>> &axis);
    	    
    	    void setState(std::vector<double> &joint_values, std::vector<double> &joint_velocities);
    	    
    	    void getState(std::vector<double> &state);
    	    
    	    void getJointLowerPositionLimits(std::vector<std::string> &joints, std::vector<double> &joint_limits);
    	    
    	    void getJointUpperPositionLimits(std::vector<std::string> &joints, std::vector<double> &joint_limits);
    	    
    	    void getJointVelocityLimits(std::vector<std::string> &joints, std::vector<double> &joint_limits);
    	    
    	    void getJointTorqueLimits(std::vector<std::string> &joints, std::vector<double> &joint_limits);
    	    
    	    void getJointDamping(std::vector<std::string> &joints, std::vector<double> &damping);
    	    
    	    void getPositionOfLinkN(const std::vector<double> &joint_angles, const int &n, std::vector<double> &position);
    	    
    	    void getEndEffectorPosition(const std::vector<double> &joint_angles, std::vector<double> &end_effector_position);
    	    
    	    
    	    void updateViewerValues(const std::vector<double> &current_joint_values,
                                    const std::vector<double> &current_joint_velocities);
    	    
    	    void setupViewer(std::string model_file, std::string environment_file);
    	    
    	    int getDOF();
    	    
    	    void test();
    	    
    	    bool propagate(std::vector<double> &current_state,
		                   std::vector<double> &control_input,
		                   std::vector<double> &control_error,
		                   double simulation_step_size,
		                   double duration,
		                   std::vector<double> &result);
    	    
    	    bool propagate_linear(std::vector<double> &current_state,
    	    		              std::vector<double> &control_input,
    	    		              std::vector<double> &control_error,
    	    		              double duration,
    	    		              std::vector<double> &result);
    	    
    	    void createRobotCollisionStructures(const std::vector<double> &joint_angles, std::vector<fcl::OBB> &collision_structures);
    	    
    	    void createRobotCollisionObjects(const std::vector<double> &joint_angles, std::vector<fcl::CollisionObject> &collision_objects) ;
    	    
    	    std::vector<fcl::CollisionObject> createRobotCollisionObjectsPy(const std::vector<double> &joint_angles);
    	    
    	    std::vector<fcl::OBB> createRobotCollisionStructuresPy(const std::vector<double> &joint_angles);
    	    
    	    void enforceConstraints(bool enforce);
    	    
    
        private:
            std::string robot_file_;
            
            std::vector<std::string> link_names_;
            
            std::vector<std::string> active_link_names_;
            
            std::vector<std::string> joint_names_;
            
            std::vector<std::string> joint_types_;
            
            std::vector<std::string> active_joints_;
            
            std::vector<double> joint_dampings_;
            
            std::vector<std::vector<double>> joint_origins_;
            
            std::vector<std::vector<double>> link_origins_;
            
            std::vector<std::vector<double>> active_joint_origins_;
            
            std::vector<std::vector<int>> joint_axes_;
            
            std::vector<double> joint_torque_limits_;
            
            std::vector<double> active_joint_torque_limits_;
            
            std::vector<double> lower_joint_limits_;
            
            std::vector<double> upper_joint_limits_;
            
            std::vector<double> active_lower_joint_limits_;
			
			std::vector<double> active_upper_joint_limits_;
			
			std::vector<double> joint_velocity_limits_;
			
			std::vector<double> active_joint_velocity_limits_;
            
            std::vector<std::vector<int>> active_joint_axes_;
            
            std::vector<std::string> active_links_;
            
            std::vector<double> link_masses_;
            
            std::vector<std::vector<double>> link_inertia_origins_;
            
            std::vector<std::vector<double>> link_inertia_matrices_;
            
            std::vector<std::vector<double>> link_dimensions_;
            
            std::vector<std::vector<double>> active_link_dimensions_;
            
            bool initLinks(TiXmlElement *robot_xml);
            
            bool initJoints(TiXmlElement *robot_xml);
            
            bool enforce_constraints_;
            
            unsigned int get_link_index(std::string &link_name);
            
            std::vector<double> process_origin_(TiXmlElement *xml);
            
            shared::RobotState robot_state_;
            
            std::shared_ptr<shared::Propagator> propagator_;
            
            std::shared_ptr<shared::Kinematics> kinematics_;
    
    	
    };
    		
}

#endif
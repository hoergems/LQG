#ifndef ROBOT_HPP_
#define ROBOT_HPP_
#include <string>
#include <iostream>
#include <assert.h>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/make_shared.hpp>
#include "fcl/BV/BV.h" 
#include "fcl/collision_object.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/shape/geometric_shapes_utility.h"
#include <tinyxml.h>
#include <rbdl_interface/rbdl_interface.hpp>
#include "propagator.hpp"
#include "Kinematics.hpp"

#ifdef USE_URDF
   #include <viewer_interface/viewer_interface.hpp>
#endif

using std::cout;
using std::endl;

namespace shared {

struct RobotState {
	std::vector<double> joint_values;
	std::vector<double> joint_velocities;
};

struct Link {
	std::string name;
	
	bool active;
	
	std::vector<double> link_dimensions;
	
	std::vector<double> origin;
	
	double mass;
	
	std::vector<double> inertia_origin;
	
	std::vector<double> inertials;
	
	std::vector<double> com_origin;
};

struct Joint {
	std::string name;
	
	std::shared_ptr<shared::Link> parent_link;
	
	std::shared_ptr<shared::Link> child_link;
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
    	    
    	    void getEndEffectorJacobian(const std::vector<double> &joint_angles, 
    	    		                    std::vector<std::vector<double>> &ee_jacobian);
    	    
    	    
    	    
    	    
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
    	    
    	    //void createRobotCollisionStructures(const std::vector<double> &joint_angles, std::vector<fcl::OBB> &collision_structures);
    	    /**
    	     * Creates the collision object for the end effector
    	     */
    	    void createEndEffectorCollisionObject(const std::vector<double> &joint_angles,
    	    		std::vector<std::shared_ptr<fcl::CollisionObject>> &collision_objects); 
    	    
    	    std::vector<std::shared_ptr<fcl::CollisionObject>> 
			        createEndEffectorCollisionObjectPy(const std::vector<double> &joint_angles);
    	    
    	    /**
    	     * Create the robot collision objects
    	     */
    	    void createRobotCollisionObjects(const std::vector<double> &joint_angles, 
    	    		std::vector<std::shared_ptr<fcl::CollisionObject>> &collision_objects);
    	    
    	    std::vector<std::shared_ptr<fcl::CollisionObject>> 
			        createRobotCollisionObjectsPy(const std::vector<double> &joint_angles);
    	    
    	    //std::vector<fcl::AABB> createRobotCollisionStructuresPy(const std::vector<double> &joint_angles);
    	    
    	    void enforceConstraints(bool enforce);
    	    
    	    bool constraintsEnforced();
    	    
    	    /**
    	     * Set the gravity constant
    	     */
    	    void setGravityConstant(double gravity_constant);
    	    
    	    /**
    	     * Set the external force acting on the end-effector
    	     */
    	    void setExternalForce(double f_x, 
    	    		              double f_y, 
								  double f_z,
								  double f_roll,
								  double f_pitch,
								  double f_yaw);
    	    
    	    /**
    	     * Set a joint acceleration limit
    	     */
    	    void setAccelerationLimit(double accelerationLimit);
    	    
    	    /**
    	     * Gets the end-effector velocity for a given state
    	     */
    	    void getEndEffectorVelocity(std::vector<double> &state,
    	    		                    std::vector<double> &ee_velocity);
    	    
    	    /**
    	     * Gets the process matrices in vector form for state x,
    	     * control rho and control duration t_e
    	     */
    	    std::vector<double> getProcessMatrices(std::vector<double> &x, 
                                                   std::vector<double> &rho, 
				                                   double t_e);
#ifdef USE_URDF	    
    	    /**
    	     * Set the size of the attached viewer
    	     */
    	    void setViewerSize(int x, int y);
    	    
    	    /**
    	     * Set the background color of the viewer
    	     */
    	    void setViewerBackgroundColor(double r, double g, double b);
    	    
    	    /**
    	     * Set the viewer camera transformation
    	     */
    	    void setViewerCameraTransform(std::vector<double> &rot, std::vector<double> &trans);
    	    
    	    /**
    	     * Set the maximum number of particles to plot
    	     */
    	    void setParticlePlotLimit(unsigned int particle_plot_limit);
    	    
    	    void updateViewerValues(const std::vector<double> &current_joint_values,
    	                            const std::vector<double> &current_joint_velocities,
    	    					    const std::vector<std::vector<double>> &particle_joint_values,
    	    					    const std::vector<std::vector<double>> &particle_colors);
    	        	    
    	        	    
    	    /**
    	      * Add particles to the viewer which remain when the viewer is updated
    	    */
    	    void addPermanentViewerParticles(const std::vector<std::vector<double>> &particle_joint_values,
    	    							     const std::vector<std::vector<double>> &particle_colors);
    	        	    
    	    /**
    	      * Removes any permanent particles
    	      */
    	    void removePermanentViewerParticles();
    	        	    
    	    void setupViewer(std::string model_file, std::string environment_file);
#endif
        private:
    	    std::vector<shared::Link> links_;
    	    
    	    std::vector<shared::Joint> joints_;    	    
    	    
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
            
#ifdef USE_URDF
            std::shared_ptr<shared::ViewerInterface> viewer_;
#else
            std::shared_ptr<double> viewer_;
#endif
            
            void quatFromRPY(double &roll, double &pitch, double &y, std::vector<double> &quat);
    
            /**
             * Initialize the collision objects for the active links
             */
    	    void initCollisionObjects();
    	    
    	    /**
    	     * A vector holding the collision objects of the active links
    	     */
    	    std::vector<std::shared_ptr<fcl::CollisionObject>> collision_objects_;
    	    
    	    std::shared_ptr<shared::RBDLInterface> rbdl_interface_;
    };
    		
}

#endif
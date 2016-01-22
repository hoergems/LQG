#ifndef VIEWER_INTERFACE_HPP_
#define VIEWER_INTERFACE_HPP_
#include <openrave-core.h>
#include <openrave/environment.h>
#include "viewer.hpp"
#include "urdf_loader.hpp"

namespace shared {

class ViewerInterface {
public:
	ViewerInterface();
	
	~ViewerInterface() {
		if (viewer_setup_) { 
			OpenRAVE::RaveDestroy(); 
		}
	}
	
	bool setupViewer(std::string model_file,
                     std::string environment_file);
	
	void updateRobotValues(const std::vector<double> &current_joint_values,
		   	   		       const std::vector<double> &current_joint_velocities,	
						   const std::vector<std::vector<double>> &particle_joint_values,
						   const std::vector<std::vector<double>> &particle_colors,
		   	   			   OpenRAVE::RobotBasePtr robot);
	
	/**
	 * Add particles that are independent of viewer updates
	 */
	void addPermanentParticles(const std::vector<std::vector<double>> &particle_joint_values,
			                   const std::vector<std::vector<double>> &particle_colors);
	
	/**
	 * Remove the permanently added particles
	 */
	void removePermanentParticles();
	
    /**
     * Set the size of the viewer attached to the environment
     */
    void setViewerSize(int x, int y);

    /**
     * Set the background color of the viewer
     */
    void setBackgroundColor(double &r, double &g, double &b);
    
    /**
     * Set the camera transform
     */
    void setCameraTransform(std::vector<double> &rot, std::vector<double> &trans);
    
    /**
     * Maximum number of particles to plot
     */
    void setParticlePlotLimit(unsigned int particle_plot_limit);

private:
    bool viewer_setup_;
    
    OpenRAVE::EnvironmentBasePtr env_;
    OpenRAVE::RobotBasePtr robot_;
    
    std::string model_file_;
    
    OpenRAVE::RobotBasePtr getRobot();
    
    std::shared_ptr<shared::URDFLoader> urdf_loader_;
    
    std::shared_ptr<shared::RaveViewer> viewer_;
    
    unsigned int particle_plot_limit_;
    
};


}

#endif
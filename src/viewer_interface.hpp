#ifndef VIEWER_INTERFACE_HPP_
#define VIEWER_INTERFACE_HPP_
#include <openrave-core.h>
#include <openrave/environment.h>
#include "viewer.hpp"

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
		   	   			     OpenRAVE::RobotBasePtr robot);


private:
    bool viewer_setup_;
    
    OpenRAVE::EnvironmentBasePtr env_;
    OpenRAVE::RobotBasePtr robot_;
    
    OpenRAVE::RobotBasePtr getRobot();
    
};


}

#endif
#include "viewer_interface.hpp"

using std::cout;
using std::endl;

namespace shared {

ViewerInterface::ViewerInterface ():
	viewer_setup_(false),
	env_(nullptr),
	robot_(nullptr){
	
}

bool ViewerInterface::setupViewer(std::string model_file,
                                  std::string environment_file) {
	if (viewer_setup_) {
		return false;
    }
	
	OpenRAVE::RaveInitialize(true);    
	env_ = OpenRAVE::RaveCreateEnvironment();
	env_->Load(environment_file);
	
	const std::string module_str("or_urdf_plugin");
	if(!OpenRAVE::RaveLoadPlugin(module_str)) {
		cout << "Failed to load the or_urdf_plugin." << endl;
		return false;
	}
	
	OpenRAVE::ModuleBasePtr urdf_module = OpenRAVE::RaveCreateModule(env_, "URDF");
	const std::string cmdargs("");
	env_->AddModule(urdf_module, cmdargs);
	std::stringstream sinput, sout;
	sinput << "load " << model_file;
	
	if (!urdf_module->SendCommand(sout,sinput)) {
		cout << "Failed to load URDF model" << endl;
		return false;
	}
			
				
	std::vector<OpenRAVE::KinBodyPtr> bodies;
	env_->GetBodies(bodies);
	env_->StopSimulation();
	for (auto &k: bodies) {
		cout << k->GetName() << endl;
	}
	
	OpenRAVE::RobotBasePtr robot = getRobot();			
	const std::vector<OpenRAVE::KinBody::LinkPtr> links(robot->GetLinks()); 
	for (size_t i = 0; i < links.size(); i++) {			
		if (links[i]->GetName() == "world") {
			links[i]->SetStatic(true);
		}
		else if (links[i]->GetName() == "end_effector") {
			links[i]->Enable(false);
		}
	}
	
	shared::RaveViewer viewer;
	viewer.testView(env_);
	
	viewer_setup_ = true;
	return true;
	
}

void ViewerInterface::updateRobotValues(const std::vector<double> &current_joint_values,
		                                const std::vector<double> &current_joint_velocities,									 
								        OpenRAVE::RobotBasePtr robot=nullptr) {	
	OpenRAVE::RobotBasePtr robot_to_use(nullptr);
			
	if (robot == nullptr) {
		robot_to_use = getRobot();
	}
	else {
		robot_to_use = robot;
	}
	if (robot_to_use == nullptr) {
		cout << "Propagator: Error: Environment or robot has not been initialized or passed as argument. Can't propagate the state" << endl;
		return;	
	}
	
	std::vector<OpenRAVE::dReal> newJointValues;
		
	std::vector<OpenRAVE::dReal> newJointVelocities;
	for (size_t i = 0; i < current_joint_values.size(); i++) {
		if (current_joint_values[i] < -M_PI) {
			newJointValues.push_back(2.0 * M_PI + current_joint_values[i]);
		}
		else if (current_joint_values[i] > M_PI) {
			newJointValues.push_back(-2.0 * M_PI + current_joint_values[i]);
		}
		else {
		    newJointValues.push_back(current_joint_values[i]);
		}
	}
		
	for (size_t i = 0; i < current_joint_velocities.size(); i++) {
		newJointVelocities.push_back(current_joint_velocities[i]);		
	}
	
	newJointValues.push_back(0);
	newJointVelocities.push_back(0);
	
	robot_to_use->SetDOFValues(newJointValues);
    robot_to_use->SetDOFVelocities(newJointVelocities);	
}

OpenRAVE::RobotBasePtr ViewerInterface::getRobot() {
    std::vector<OpenRAVE::KinBodyPtr> bodies;
    env_->GetBodies(bodies);
    for (auto &body: bodies) {
    	if (body->GetDOF() > 0) {
    		OpenRAVE::RobotBasePtr robot = boost::static_pointer_cast<OpenRAVE::RobotBase>(body);
    		return robot;
    	}    	
    }   
}

}
#include "propagator.hpp"

using std::cout;
using std::endl;

namespace shared {

Propagator::Propagator():	
	env_(nullptr),
	robot_(nullptr),
	enforce_constraints_(false),
	integrator_(new Integrate()),
	viewer_setup_(false){	
}

void Propagator::setup(std::vector<double> &jointsLowerPositionLimit,
		               std::vector<double> &jointsUpperPositionLimit,
		               std::vector<double> &jointsVelocityLimit,
					   bool enforce_constraints) {	
	for (size_t i = 0; i < jointsLowerPositionLimit.size(); i++) {
		jointsLowerPositionLimit_.push_back(jointsLowerPositionLimit[i]);
		jointsUpperPositionLimit_.push_back(jointsUpperPositionLimit[i]);
		jointsVelocityLimit_.push_back(jointsVelocityLimit[i]);
	}
	
	enforce_constraints_ = enforce_constraints;
}

void Propagator::enforce_constraints(bool enforce) {
	enforce_constraints_ = enforce;
}

bool Propagator::setup_viewer(std::string model_file,
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
		cout << "VIEWER SETUP!!!!!!!!!!" << endl;
	viewer_setup_ = true;
    return true;
}

OpenRAVE::RobotBasePtr Propagator::getRobot() {
    std::vector<OpenRAVE::KinBodyPtr> bodies;
    env_->GetBodies(bodies);
    for (auto &body: bodies) {
    	if (body->GetDOF() > 0) {
    		OpenRAVE::RobotBasePtr robot = boost::static_pointer_cast<OpenRAVE::RobotBase>(body);
    		return robot;
    	}    	
    }   
}


void Propagator::update_robot_values(const std::vector<double> &current_joint_values,
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

bool Propagator::propagate_linear(const std::vector<double> &current_joint_values,
                                  const std::vector<double> &control,
                                  const std::vector<double> &control_error,				             		             
                                  const double duration,
                                  std::vector<double> &result) {
	
	std::vector<double> c;
    bool allZeros = true;

	for (size_t i=0; i < control.size(); i++) {
		if (control[i] != 0) {
		    allZeros = false;
		    c.push_back(1.0);            
		}
		else {
		    // Add no uncertainty if joint input is 0
		    c.push_back(0.0);
		}
    }
		
    if (allZeros) {
	    return false;
	}
		
	for (size_t i = 0; i < control.size(); i++) {
	    result.push_back(current_joint_values[i] + 
					     duration * control[i] + 
					     c[i] * control_error[i]);
	}
	for (size_t i = 0; i < control.size(); i++) {
		result.push_back(0.0);
	}
	return true;
	
}
	
bool Propagator::propagate_nonlinear(const std::vector<double> &current_joint_values,
				                     const std::vector<double> &current_joint_velocities,
				                     std::vector<double> &control,
				                     std::vector<double> &control_error_vec,
				                     const double simulation_step_size,
				                     const double duration,
				                     std::vector<double> &result) {
	std::vector<double> state;
	
	for (size_t i = 0; i < current_joint_values.size(); i++) {
		state.push_back(current_joint_values[i]);
	}
	for (size_t i = 0; i < current_joint_values.size(); i++) {
		state.push_back(current_joint_velocities[i]);
	}
	
	std::vector<double> integration_result;
	std::vector<double> inte_times({0.0, duration, simulation_step_size});	
	integrator_->do_integration(state, control, control_error_vec, inte_times, integration_result);
	
	std::vector<double> newJointValues;
	std::vector<double> newJointVelocities;
	
	for (size_t i = 0; i < integration_result.size() / 2; i++) {
		newJointValues.push_back(integration_result[i]);		
	}
	
	for (size_t i = integration_result.size() / 2; i < integration_result.size(); i++) {
		newJointVelocities.push_back(integration_result[i]);		
	}
	
	//Enforce position and velocity limits
	bool legal = true;
	if (enforce_constraints_) {
		for (unsigned int i = 0; i < newJointValues.size(); i++) {
			if (newJointValues[i] < jointsLowerPositionLimit_[i]) {
				legal = false;	    	
				newJointValues[i] = jointsLowerPositionLimit_[i];
				newJointVelocities[i] = 0;				
			}
			else if (newJointValues[i] > jointsUpperPositionLimit_[i]) {
				legal = false;			
				newJointValues[i] = jointsUpperPositionLimit_[i];
				newJointVelocities[i] = 0;
			}
	
			if (newJointVelocities[i] < -jointsVelocityLimit_[i]) {
				newJointVelocities[i] = -jointsVelocityLimit_[i];
				legal = false;
			}
			else if (newJointVelocities[i] > jointsVelocityLimit_[i]) {
				newJointVelocities[i] = jointsVelocityLimit_[i];
				legal = false;
			}		        
		}
	}
	
	for (size_t i = 0; i < newJointValues.size(); i++) {
		result.push_back(newJointValues[i]);
	}
	
	for (size_t i = 0; i < newJointVelocities.size(); i++) {
		result.push_back(newJointVelocities[i]);
	}
	
	return legal;
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(propagate_nonlinear_overload, propagate_nonlinear, 7, 7);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(propagate_linear_overload, propagate_linear, 5, 5);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(update_robot_values_overload, update_robot_values, 2, 3);

BOOST_PYTHON_MODULE(libpropagator) {
    using namespace boost::python;
   
    class_<Propagator>("Propagator", init<>())
							   .def("setup", &Propagator::setup)
							   .def("setup", &Propagator::setup_viewer)
							   .def("propagate", &Propagator::propagate_nonlinear, propagate_nonlinear_overload())
							   .def("propagateLinear", &Propagator::propagate_linear, propagate_linear_overload())
							   .def("updateRobotValues", &Propagator::update_robot_values, update_robot_values_overload())
		                       
										            		 
                        //.def("doIntegration", &Integrate::do_integration)                        
                        //.def("getResult", &Integrate::getResult)
    ;
}


}
#include "propagator.hpp"

using std::cout;
using std::endl;

namespace shared {

Propagator::Propagator():
	damper_(nullptr),
	env_(nullptr),
	robot_(nullptr){	
}

void Propagator::setup(double coulomb, 
		               double viscous,
		               std::vector<double> &jointsLowerPositionLimit,
		               std::vector<double> &jointsUpperPositionLimit,
		               std::vector<double> &jointsLowerVelLimit,
		               std::vector<double> &jointsUpperVelLimit) {
	damper_ = std::make_shared<TorqueDamper>(coulomb, viscous);
	for (size_t i = 0; i < jointsLowerPositionLimit.size(); i++) {
		jointsLowerPositionLimit_.push_back(jointsLowerPositionLimit[i]);
		jointsUpperPositionLimit_.push_back(jointsUpperPositionLimit[i]);
		jointsLowerVelocityLimit_.push_back(jointsLowerVelLimit[i]);
		jointsUpperVelocityLimit_.push_back(jointsUpperVelLimit[i]);
	}
}

bool Propagator::setup_py(std::string model_file,
		                  double coulomb, 
                          double viscous) {
	damper_ = std::make_shared<TorqueDamper>(coulomb, viscous);
	
    OpenRAVE::RaveInitialize(true);    
    env_ = OpenRAVE::RaveCreateEnvironment();    

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
    
    OpenRAVE::RobotBasePtr robot = getRobot();
    
    const std::vector<OpenRAVE::KinBody::LinkPtr> links(robot->GetLinks());    
    links[0]->SetStatic(true); 
    
    /***** Create the physics engine *****/
    const std::string engine = "ode";
    OpenRAVE::PhysicsEngineBasePtr physics_engine_ = OpenRAVE::RaveCreatePhysicsEngine(env_, engine);
    	    
    const OpenRAVE::Vector gravity({0.0, 0.0, -9.81});    
    physics_engine_->SetGravity(gravity);
    env_->SetPhysicsEngine(physics_engine_);
    
    /**
     * Set the joint limits
     */
    const std::vector<OpenRAVE::KinBody::JointPtr> joints(robot->GetJoints());
    for (size_t i = 0; i < joints.size(); i++) {
    	std::vector<OpenRAVE::dReal> lower_limit;
    	std::vector<OpenRAVE::dReal> upper_limit;        
    	joints[i]->GetLimits(lower_limit, upper_limit);
    	
    	jointsLowerPositionLimit_.push_back(lower_limit[0]);
    	jointsUpperPositionLimit_.push_back(upper_limit[0]);
    	
    	jointsLowerVelocityLimit_.push_back(-joints[i]->GetMaxTorque());
    	jointsUpperVelocityLimit_.push_back(joints[i]->GetMaxTorque());
    }
    return true;
}

OpenRAVE::RobotBasePtr Propagator::getRobot() {
    std::vector<OpenRAVE::KinBodyPtr> bodies;
    env_->GetBodies(bodies);
    OpenRAVE::RobotBasePtr robot = boost::static_pointer_cast<OpenRAVE::RobotBase>(bodies[0]);
    return robot;
}

void Propagator::propagate_nonlinear_py(const std::vector<double> &current_joint_values,
                                        const std::vector<double> &current_joint_velocities,
                                        std::vector<double> &control,
                                        std::vector<double> &control_error_vec,
                                        const double simulation_step_size,
                                        const double duration,
                                        std::vector<double> &result) {
	std::vector<double> current_vel;
	OpenRAVE::RobotBasePtr robot = getRobot();
	const std::vector<OpenRAVE::KinBody::JointPtr> joints(robot->GetJoints());
	std::vector<OpenRAVE::dReal> input_torques(control);
	std::vector<OpenRAVE::dReal> damped_torques(control);
	std::vector<OpenRAVE::dReal> torque_error(control_error_vec);
	
	const std::vector<OpenRAVE::dReal> currentJointValues(current_joint_values);
	const std::vector<OpenRAVE::dReal> currentJointVelocities(current_joint_velocities);
	
	robot->SetDOFValues(current_joint_values);
	robot->SetDOFVelocities(current_joint_velocities);
	
	int num_steps = duration / simulation_step_size;
	for (unsigned int i = 0; i < num_steps; i++) {
		robot->GetDOFVelocities(current_vel);
		damper_->damp_torques(current_vel,
		                      damped_torques);
		for (size_t k = 0; k < joints.size(); k++) {
		    input_torques[k] = input_torques[k] + damped_torques[k] + torque_error[k];
		    const std::vector<OpenRAVE::dReal> torques({input_torques[k]});
		    joints[k]->AddTorque(torques);
		}        
		env_->StepSimulation(simulation_step_size);
		env_->StopSimulation();
     }
	
	 std::vector<OpenRAVE::dReal> newJointValues;
	 std::vector<OpenRAVE::dReal> newJointVelocities;	    
	 robot->GetDOFValues(newJointValues);
	 robot->GetDOFVelocities(newJointVelocities);
		
	 //Enforce position and velocity limits
	 for (unsigned int i = 0; i < joints.size(); i++) {
         if (newJointValues[i] < jointsLowerPositionLimit_[i]) {
			newJointValues[i] = jointsLowerPositionLimit_[i];
		 }
		 else if (newJointValues[i] > jointsUpperPositionLimit_[i]) {
			newJointValues[i] = jointsUpperPositionLimit_[i];
		 }

		 if (newJointVelocities[i] < jointsLowerVelocityLimit_[i]) {
			newJointVelocities[i] = jointsLowerVelocityLimit_[i];
		 }
		 else if (newJointVelocities[i] > jointsUpperVelocityLimit_[i]) {
		    newJointVelocities[i] = jointsUpperVelocityLimit_[i];
		 }
			        
	 }
		
	 for (size_t i = 0; i < joints.size(); i++) {
	     result.push_back(newJointValues[i]);
	 }
		
	 for (size_t i = 0; i < joints.size(); i++) {
		 result.push_back(newJointVelocities[i]);
	 }
}
	
void Propagator::propagate_nonlinear(OpenRAVE::EnvironmentBasePtr environment,
				                     OpenRAVE::RobotBasePtr robot,
				                     const std::vector<double> &current_joint_values,
				                     const std::vector<double> &current_joint_velocities,
				                     std::vector<double> &control,
				                     const double simulation_step_size,
				                     const double duration,
				                     std::vector<double> &result) const {	
	std::vector<double> current_vel;
	const std::vector<OpenRAVE::KinBody::JointPtr> joints(robot->GetJoints());
	std::vector<OpenRAVE::dReal> input_torques(control);
	std::vector<OpenRAVE::dReal> damped_torques(control);
	
	const std::vector<OpenRAVE::dReal> currentJointValues(current_joint_values);
	const std::vector<OpenRAVE::dReal> currentJointVelocities(current_joint_velocities);	    
	    
	robot->SetDOFValues(current_joint_values);
	robot->SetDOFVelocities(current_joint_velocities);
	
	int num_steps = duration / simulation_step_size;
	for (unsigned int i = 0; i < num_steps; i++) {
	    robot->GetDOFVelocities(current_vel);
	    damper_->damp_torques(current_vel,
	                          damped_torques);
	    for (size_t k = 0; k < joints.size(); k++) {
	        input_torques[k] = input_torques[k] + damped_torques[k];
	        const std::vector<OpenRAVE::dReal> torques({input_torques[k]});
	        joints[k]->AddTorque(torques);
	    }        
	    environment->StepSimulation(simulation_step_size);
	    environment->StopSimulation();
	}
	
	std::vector<OpenRAVE::dReal> newJointValues;
	std::vector<OpenRAVE::dReal> newJointVelocities;	    
	robot->GetDOFValues(newJointValues);
	robot->GetDOFVelocities(newJointVelocities);
	
	//Enforce position and velocity limits
	for (unsigned int i = 0; i < joints.size(); i++) {
	    if (newJointValues[i] < jointsLowerPositionLimit_[i]) {
		    newJointValues[i] = jointsLowerPositionLimit_[i];
		}
		else if (newJointValues[i] > jointsUpperPositionLimit_[i]) {
		    newJointValues[i] = jointsUpperPositionLimit_[i];
		}

		if (newJointVelocities[i] < jointsLowerVelocityLimit_[i]) {
		    newJointVelocities[i] = jointsLowerVelocityLimit_[i];
		}
		else if (newJointVelocities[i] > jointsUpperVelocityLimit_[i]) {
	        newJointVelocities[i] = jointsUpperVelocityLimit_[i];
		}
		        
	}
	
	for (size_t i = 0; i < joints.size(); i++) {
		result.push_back(newJointValues[i]);
	}
	
	for (size_t i = 0; i < joints.size(); i++) {
		result.push_back(newJointVelocities[i]);
	}
	
	
	
}

BOOST_PYTHON_MODULE(libpropagator) {
    using namespace boost::python;    
   
    class_<Propagator>("Propagator", init<>())
							   .def("setup", &Propagator::setup_py)
							   .def("propagate", &Propagator::propagate_nonlinear_py)
		                       
										            		 
                        //.def("doIntegration", &Integrate::do_integration)                        
                        //.def("getResult", &Integrate::getResult)
    ;
}


}
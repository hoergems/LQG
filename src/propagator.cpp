#include "propagator.hpp"

using std::cout;
using std::endl;

namespace shared {

Propagator::Propagator():
	damper_(nullptr){	
}

void Propagator::setup(double &coulomb, 
		               double &viscous,
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
	
	for (size_t i = 0; i < newJointVelocities.size(); i++) {
		cout << newJointVelocities[i] << ", ";
	}
	cout << endl;
	
	for (size_t i = 0; i < joints.size(); i++) {
		result.push_back(newJointValues[i]);
	}
	
	for (size_t i = 0; i < joints.size(); i++) {
		result.push_back(newJointVelocities[i]);
	}
	
	//Enforce position and velocity limits
	/**for (unsigned int i = 0; i < joints.size(); i++) {
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
	        
	}*/
	
}


}
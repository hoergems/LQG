#include "state_propagator.hpp"

using std::cout;
using std::endl;

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

namespace shared {

StatePropagator::StatePropagator(const ompl::control::SpaceInformationPtr &si, 
                                 double &simulation_step_size,                                 
                                 bool &linear_propagation,
                                 bool &verbose):
    ompl::control::StatePropagator(si),
    space_information_(si),    
    model_setup_(false),
    environment_(nullptr),
    robot_(nullptr),
    propagator_(new Propagator()),
    simulation_step_size_(simulation_step_size),    
    linear_propagation_(linear_propagation),
    linear_integrator_(),
    verbose_(verbose)
{
    
}

void StatePropagator::propagate(const ompl::base::State *state, 
                                const ompl::control::Control *control, 
                                const double duration, 
                                ompl::base::State *result) const {	
    unsigned int dim = space_information_->getStateSpace()->getDimension() / 2;    
    std::vector<double> current_vel;
    if (linear_propagation_) {
    	std::vector<double> state_vec;
    	std::vector<double> control_vec;    	
    	std::vector<double> integration_times({0.0, duration, duration});
    	for (unsigned int i = 0; i < 2 * dim; i++) {
    		state_vec.push_back(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);    		
    		if (i < dim) {
    			control_vec.push_back(control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[i]);
    		}    		
    	}  	
    	
    	//boost::timer t;
    	linear_integrator_.do_integration(state_vec, control_vec, integration_times);
    	//cout << "Integrated in " << t.elapsed() << "seconds" << endl;
    	if (verbose_) {
			cout << "start state: ";
			for (unsigned int i = 0; i < 2 * dim; i++) { 
				cout << state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] << ", ";
			}
			cout << endl;  	
			
			
			cout << "control: ";
			for (unsigned int i = 0; i < dim; i++) { 
				cout << control_vec[i] << ", ";
			}    	
			cout << endl;
			
			cout << "duration: " << duration << endl; 
			
			cout << "result: "; 
			for (unsigned int i = 0; i < 2 * dim; i++) { 
				cout << state_vec[i] << ", ";
			} 
			cout << endl;
			cout << "=================================" << endl;
			//sleep(1);
    	}
    	
    	for (unsigned int i = 0; i < dim; i++) {
    		if (state_vec[i] > M_PI) {
    			state_vec[i] = -2.0 * M_PI + state_vec[i]; 
    			//integration_result[i] = integration_result[i] - 2.0 * M_PI;
    		}
    		else if (state_vec[i] < -M_PI) {
    			state_vec[i] = 2.0 * M_PI + state_vec[i];
    			//integration_result[i] = integration_result[i] + 2.0 * M_PI;
    		}
    	}
    	
    	for (unsigned int i = 0; i < 2 * dim; i++) {
    	    result->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = state_vec[i];
    	}
    	return;
    }
    
    if (verbose_) {
		cout << "State: ";
		for (unsigned int i = 0; i < dim * 2.0; i++) {
			cout << " " << state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
		}
		cout << endl;
	
		cout << "Torques: ";
		for (unsigned int i = 0; i < dim; i++) {
			//cout << " " << control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[i];
			cout << " " << 1.0;
		}
		cout << endl;
    }    
                                
    std::vector<OpenRAVE::dReal> current_joint_values;
    std::vector<OpenRAVE::dReal> current_joint_velocities;
    std::vector<double> input_torque;
    
    for (unsigned int i = 0; i < dim; i++) {
    	current_joint_values.push_back(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);
    	current_joint_velocities.push_back(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i + dim]);
    	input_torque.push_back(control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[i]);
    	//input_torque.push_back(1.0);
    }
    
    current_joint_values.push_back(0.0);
    current_joint_velocities.push_back(0.0);
    
    std::vector<double> propagation_result;
    propagator_->propagate_nonlinear(environment_,
    		                         robot_,
    		                         current_joint_values,
    		                         current_joint_velocities,
    		                         input_torque,
    		                         simulation_step_size_,
    		                         duration,
    		                         propagation_result);
    if (verbose_) {
		cout << "Propagation result: ";
		for (size_t i = 0; i < propagation_result.size(); i++) {
			cout << propagation_result[i] << ", ";
		}
		cout << endl;
		sleep(1);
    }
    for (unsigned int i = 0; i < dim; i++) {
        result->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = propagation_result[i];
        result->as<ompl::base::RealVectorStateSpace::StateType>()->values[i + dim] = propagation_result[i + dim];
    }       
}

bool StatePropagator::canPropagateBackward() const{
    return false;
}

bool StatePropagator::steer(const ompl::base::State* /*from*/, 
                            const ompl::base::State* /*to*/, 
                            ompl::control::Control* /*result*/, 
                            double& /*duration*/) const {
    return false;                            
} 

bool StatePropagator::canSteer() const {
    return false;
}

bool StatePropagator::setupOpenRAVEEnvironment(OpenRAVE::EnvironmentBasePtr environment,
                                               OpenRAVE::RobotBasePtr robot,
                                               double &coulomb,
                                               double &viscous) {
	cout << "setup openrave" << endl;
    environment_ = environment;
    robot_ = robot;
    const std::vector<OpenRAVE::KinBody::JointPtr> joints(robot_->GetJoints());
    std::vector<double> jointsLowerPositionLimit; 
    std::vector<double> jointsUpperPositionLimit;
    std::vector<double> jointsLowerVelocityLimit; 
    std::vector<double> jointsUpperVelocityLimit;
    for (size_t i = 0; i < joints.size(); i++) {
        std::vector<OpenRAVE::dReal> jointLowerLimit;
        std::vector<OpenRAVE::dReal> jointUpperLimit;
        std::vector<OpenRAVE::dReal> jointLowerVelLimit;
        std::vector<OpenRAVE::dReal> jointUpperVelLimit;
        joints[i]->GetLimits(jointLowerLimit, jointUpperLimit);
        joints[i]->GetVelocityLimits(jointLowerVelLimit, jointUpperVelLimit);
        jointsLowerPositionLimit.push_back(jointLowerLimit[0]);
        jointsUpperPositionLimit.push_back(jointUpperLimit[0]);
        jointsLowerVelocityLimit.push_back(jointLowerVelLimit[0]);
        jointsUpperVelocityLimit.push_back(jointUpperVelLimit[0]);        
    }
    propagator_->setup(coulomb, 
    		           viscous,
    		           jointsLowerPositionLimit,
    		           jointsUpperPositionLimit,
    		           jointsLowerVelocityLimit,
    		           jointsUpperVelocityLimit);
    model_setup_ = true;
    return model_setup_;
}

}

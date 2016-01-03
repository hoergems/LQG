#include "propagator.hpp"

using std::cout;
using std::endl;

namespace shared {

Propagator::Propagator():
	enforce_constraints_(false),
	integrator_(new Integrate()){	
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

MatrixXd Propagator::get_ee_jacobian(std::vector<double> &state) {
	const std::vector<double> rho;
	const std::vector<double> zeta;
	return integrator_->get_end_effector_jacobian(state, rho, zeta);
}

void Propagator::enforce_constraints(bool enforce) {
	enforce_constraints_ = enforce;
}

std::shared_ptr<Integrate> Propagator::getIntegrator() {
	return integrator_;
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
	
	// Normalize joint angles to be within [-pi, pi]
	/**for (size_t i = 0; i < newJointValues.size(); i++) {
		if (newJointValues[i] > M_PI) {
			newJointValues[i] = newJointValues[i] - 2.0 * M_PI;
		}
		else if (newJointValues[i] < -M_PI) {
			newJointValues[i] = newJointValues[i] + 2.0 * M_PI;
		}
	}*/
	
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

BOOST_PYTHON_MODULE(libpropagator) {
    using namespace boost::python;
   
    class_<Propagator>("Propagator", init<>())
							   .def("setup", &Propagator::setup)							   
							   .def("propagate", &Propagator::propagate_nonlinear, propagate_nonlinear_overload())
							   .def("propagateLinear", &Propagator::propagate_linear, propagate_linear_overload())		            		 
                        //.def("doIntegration", &Integrate::do_integration)                        
                        //.def("getResult", &Integrate::getResult)
    ;
}


}
#include "state_propagator.hpp"

using std::cout;
using std::endl;

namespace shared {

StatePropagator::StatePropagator(const ompl::control::SpaceInformationPtr &si,
		                         boost::shared_ptr<shared::Robot> &robot,
                                 double &simulation_step_size,
                                 bool &verbose):
    ompl::control::StatePropagator(si),
    space_information_(si),    
    model_setup_(false),    
    robot_(robot),
    simulation_step_size_(simulation_step_size),
    verbose_(verbose)
{
    
}

void StatePropagator::propagate(const ompl::base::State *state, 
                                const ompl::control::Control *control, 
                                const double duration, 
                                ompl::base::State *result) const {	
    unsigned int dim = space_information_->getStateSpace()->getDimension();    
    std::vector<double> current_vel;
    
    if (verbose_) {
		cout << "State: ";
		for (unsigned int i = 0; i < dim; i++) {
			cout << " " << state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i];
		}
		cout << endl;
	
		cout << "Torques: ";
		for (unsigned int i = 0; i < dim / 2; i++) {
			cout << " " << control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[i];			
		}
		cout << endl;
    }    
                                
    std::vector<double> current_state;    
    std::vector<double> control_error_vec;
    std::vector<double> input_torque;    
    for (unsigned int i = 0; i < dim; i++) {
    	current_state.push_back(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);
    }
    
    for (unsigned int i = 0; i < dim / 2; i++) {
    	input_torque.push_back(control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[i]);
    	control_error_vec.push_back(0.0);
    }
    
    double dur = duration;
    double sss = simulation_step_size_;
    
    std::vector<double> propagation_result;
    robot_->propagate(current_state,
    		          input_torque,
    		          control_error_vec,
    		          sss,
    		          dur,
    		          propagation_result);
    if (verbose_) {
		cout << "Propagation result: ";
		for (size_t i = 0; i < propagation_result.size(); i++) {
			cout << propagation_result[i] << ", ";
		}
		cout << endl << endl;
		//sleep(1);
    }
    
    for (unsigned int i = 0; i < dim; i++) {
        result->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] = propagation_result[i];        
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

}

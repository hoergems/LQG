#include "control_sampler.hpp"

using std::cout;
using std::endl;

namespace shared {

UniformControlSampler::UniformControlSampler(const ompl::control::ControlSpace *space):
    ompl::control::ControlSampler(space),
    space_(space),
	low_(space_->as<ompl::control::RealVectorControlSpace>()->getBounds().low),
	high_(space_->as<ompl::control::RealVectorControlSpace>()->getBounds().high)
{
    
}

void UniformControlSampler::sample(ompl::control::Control *control) {    
    for (size_t i = 0; i < space_->getDimension(); i++) {
        control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[i] = rng_.uniformReal(low_[i], high_[i]); 
    }
}

void UniformControlSampler::sample(ompl::control::Control *control, 
		                           const ompl::base::State *state) {	
	sample(control);
}

void UniformControlSampler::sampleNext(ompl::control::Control *control, 
		                               const ompl::control::Control *previous, 
		                               const ompl::base::State *state) {	
	sample(control);
}

/**
 * Discrete control sampler
 */

DiscreteControlSampler::DiscreteControlSampler(const ompl::control::ControlSpace *space):
	ompl::control::ControlSampler(space),
	space_(space),
	low_(space_->as<ompl::control::RealVectorControlSpace>()->getBounds().low),
	high_(space_->as<ompl::control::RealVectorControlSpace>()->getBounds().high)
{
	
}

void DiscreteControlSampler::sample(ompl::control::Control *control) {	
	for (size_t i = 0; i < space_->getDimension(); i++) { 
		int rand_int = rng_.uniformInt(0, 2);
		if (rand_int == 0) {
			control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[i] = low_[i];
		}
		else if (rand_int == 1) {
			control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[i] = 0.0;
		}
		
		else {
			control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[i] = high_[i];
		}
		/**int rand_int = rng_.uniformInt(0, 1);
		if (rand_int == 0) {
			control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[i] = low_[i];
		}
		else{
			control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[i] = high_[i];
		}*/
	}
}

void DiscreteControlSampler::sample(ompl::control::Control *control, 
		                            const ompl::base::State *state) {
	sample(control);
}

void DiscreteControlSampler::sampleNext(ompl::control::Control *control, 
		                                const ompl::control::Control *previous, 
		                                const ompl::base::State *state) {
	sample(control);
}

}

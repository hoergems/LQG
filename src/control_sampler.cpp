#include "control_sampler.hpp"

using std::cout;
using std::endl;

namespace shared {

UniformControlSampler::UniformControlSampler(const ompl::control::ControlSpace *space):
    ompl::control::ControlSampler(space),
    space_(space)
{
    
}

void UniformControlSampler::sample(ompl::control::Control *control) {	
    std::vector<double> low = space_->as<ompl::control::RealVectorControlSpace>()->getBounds().low;
    std::vector<double> high = space_->as<ompl::control::RealVectorControlSpace>()->getBounds().high;
    for (size_t i = 0; i < space_->getDimension(); i++) {
        control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[i] = rng_.uniformReal(low[i], high[i]); 
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
	space_(space){
	
}

void DiscreteControlSampler::sample(ompl::control::Control *control) {	
	std::vector<double> low = space_->as<ompl::control::RealVectorControlSpace>()->getBounds().low;
	std::vector<double> high = space_->as<ompl::control::RealVectorControlSpace>()->getBounds().high;	
	for (size_t i = 0; i < space_->getDimension(); i++) { 
		int rand_int = rng_.uniformInt(0, 2);
		if (rand_int == 0) {
			control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[i] = low[i];
		}
		else if (rand_int == 1) {
			control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[i] = 0.0;
		}
		
		else {
			control->as<ompl::control::RealVectorControlSpace::ControlType>()->values[i] = high[i];
		}
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

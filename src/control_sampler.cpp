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

void UniformControlSampler::sampleNext(ompl::control::Control *control, const ompl::control::Control *previous, const ompl::base::State *state) {	
	sample(control);
}

}

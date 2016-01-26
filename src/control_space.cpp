#include "control_space.hpp"

using std::cout;
using std::endl;

namespace shared {

ControlSpace::ControlSpace(const ompl::base::StateSpacePtr &stateSpace, unsigned int dim):
    ompl::control::RealVectorControlSpace(stateSpace, dim),
    control_sampler_("continuous")
{

}

void ControlSpace::setControlSampler(std::string control_sampler) {
	control_sampler_ = control_sampler;
}

ompl::control::ControlSamplerPtr ControlSpace::allocDefaultControlSampler() const {	
	if (control_sampler_ == "discrete") {		
	    return ompl::control::ControlSamplerPtr(new DiscreteControlSampler(this));
	}
	
	return ompl::control::ControlSamplerPtr(new UniformControlSampler(this));
}

}

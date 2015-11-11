#include "control_space.hpp"

namespace shared {

ControlSpace::ControlSpace(const ompl::base::StateSpacePtr &stateSpace, unsigned int dim):
    ompl::control::RealVectorControlSpace(stateSpace, dim)
{

}

ompl::control::ControlSamplerPtr ControlSpace::allocDefaultControlSampler() const {    
    return ompl::control::ControlSamplerPtr(new UniformControlSampler(this));
}
}

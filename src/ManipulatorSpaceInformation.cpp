#include "ManipulatorSpaceInformation.hpp"

using std::cout;
using std::endl;

namespace shared {

ManipulatorSpaceInformation::ManipulatorSpaceInformation(const ompl::base::StateSpacePtr &stateSpace, 
                                                         const ompl::control::ControlSpacePtr &controlSpace):
		ompl::control::SpaceInformation(stateSpace, controlSpace),
		num_control_samples_(1)
{
    
}

ompl::control::DirectedControlSamplerPtr ManipulatorSpaceInformation::allocDirectedControlSampler() const {	
	ompl::control::DirectedControlSamplerPtr ptr(new ompl::control::SimpleDirectedControlSampler(this));
	boost::dynamic_pointer_cast<ompl::control::SimpleDirectedControlSampler>(ptr)->setNumControlSamples(num_control_samples_);	
	return ptr;
}

void ManipulatorSpaceInformation::setNumControlSamples(unsigned int &num_control_samples) {
	num_control_samples_ = num_control_samples;
}
    
}
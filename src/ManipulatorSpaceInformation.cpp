#include "ManipulatorSpaceInformation.hpp"

using std::cout;
using std::endl;

namespace shared {

ManipulatorSpaceInformation::ManipulatorSpaceInformation(const ompl::base::StateSpacePtr &stateSpace, 
                                                         const ompl::control::ControlSpacePtr &controlSpace):
		ompl::control::SpaceInformation(stateSpace, controlSpace)
{
    
}

ompl::control::DirectedControlSamplerPtr ManipulatorSpaceInformation::allocDirectedControlSampler() const {	
	ompl::control::DirectedControlSamplerPtr ptr(new ompl::control::SimpleDirectedControlSampler(this));
	boost::dynamic_pointer_cast<ompl::control::SimpleDirectedControlSampler>(ptr)->setNumControlSamples(3);
	return ptr;
}
    
}
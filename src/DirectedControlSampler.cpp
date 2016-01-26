#include "DirectedControlSampler.hpp"

using std::cout;
using std::endl;

namespace shared {

ManipulatorDirectedControlSampler::ManipulatorDirectedControlSampler(const ompl::control::SpaceInformation *si, unsigned int k):
		SimpleDirectedControlSampler(si, k), 
        cs_(si->allocControlSampler()), 
    	numControlSamples_(k) {	
}

unsigned int ManipulatorDirectedControlSampler::sampleTo(ompl::control::Control *control, 
		                                                 const ompl::base::State *source, 
		                                                 ompl::base::State *dest)
{    
	return sampleTo(control, NULL, source, dest);	
}

unsigned int ManipulatorDirectedControlSampler::sampleTo(ompl::control::Control *control, 
		                                                 const ompl::control::Control *previous, 
		                                                 const ompl::base::State *source, 
		                                                 ompl::base::State *dest){
	if (numControlSamples_ > 1) {		
		return getBestControl(control, source, dest, previous);
	}
    
	cs_->sample(control);
	const unsigned int minDuration = si_->getMinControlDuration();
	const unsigned int maxDuration = si_->getMaxControlDuration();
	return cs_->sampleStepCount(minDuration, maxDuration);
}

}
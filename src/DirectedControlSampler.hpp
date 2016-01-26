#ifndef _DIRECTED_CONTROL_SAMPLER_MANIPULATOR_HPP_
#define _DIRECTED_CONTROL_SAMPLER_MANIPULATOR_HPP_
#include <ompl/control/SimpleDirectedControlSampler.h>
#include <ompl/control/ControlSampler.h>
#include "ManipulatorSpaceInformation.hpp"

namespace shared {
    class ManipulatorDirectedControlSampler: public ompl::control::SimpleDirectedControlSampler {
    	public:
    	    ManipulatorDirectedControlSampler(const ompl::control::SpaceInformation *si, unsigned int k);
    	    
    	    unsigned int sampleTo(ompl::control::Control *control,
    	    		              const ompl::base::State *source,
    	    		              ompl::base::State *dest);
    	    
    	    unsigned int sampleTo(ompl::control::Control *control,
    	    		              const ompl::control::Control *previous,
    	    		              const ompl::base::State *source,
    	    		              ompl::base::State *dest);
    	    
    	private:
    	    ompl::control::ControlSamplerPtr cs_;
    	    
    	    unsigned int numControlSamples_;
    
    };
}

#endif
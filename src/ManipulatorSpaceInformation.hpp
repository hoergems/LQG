#ifndef M_SI_HPP_
#define M_SI_HPP_
#include <iostream>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/Control.h>
#include <ompl/control/ControlSpace.h>
#include <ompl/control/DirectedControlSampler.h>
#include <ompl/control/SimpleDirectedControlSampler.h>
#include "DirectedControlSampler.hpp"


namespace shared {

    class ManipulatorSpaceInformation : public ompl::control::SpaceInformation
    {
        public:    	
            ManipulatorSpaceInformation(const ompl::base::StateSpacePtr &stateSpace, 
            		                    const ompl::control::ControlSpacePtr &controlSpace);
            
            ompl::control::DirectedControlSamplerPtr allocDirectedControlSampler() const;
            
            double getPropagationStepSize() const;
            
            void setNumControlSamples(unsigned int &num_control_samples);
            
            unsigned int propagateWhileValid(const ompl::base::State *state, 
            		                         const ompl::control::Control *control, 
											 int steps, 
											 std::vector<ompl::base::State*> &result, 
											 bool alloc) const;
            
            //unsigned int getMinControlDuration();
            
        private:
            unsigned int num_control_samples_;
                        
    };

}

#endif
#ifndef M_SI_HPP_
#define M_SI_HPP_
#include <iostream>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/ControlSpace.h>
#include <ompl/control/DirectedControlSampler.h>
#include <ompl/control/SimpleDirectedControlSampler.h>


namespace shared {

    class ManipulatorSpaceInformation : public ompl::control::SpaceInformation
    {
        public:    	
            ManipulatorSpaceInformation(const ompl::base::StateSpacePtr &stateSpace, 
            		                    const ompl::control::ControlSpacePtr &controlSpace);
            
            ompl::control::DirectedControlSamplerPtr allocDirectedControlSampler() const;
            
            double getPropagationStepSize() const;
            
            //unsigned int getMinControlDuration();
                        
    };

}

#endif
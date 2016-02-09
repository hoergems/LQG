#ifndef STATE_PROPAGATOR_TEST_HPP_
#define STATE_PROPAGATOR_TEST_HPP_
#include <iostream>
#include <boost/timer.hpp>
#include <ompl/control/ControlSpace.h>
#include <ompl/control/Control.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "propagator.hpp"
#include "robot.hpp"

namespace shared {
    class StatePropagator: public ompl::control::StatePropagator {
        public:
            StatePropagator(const ompl::control::SpaceInformationPtr &si, 
            		        boost::shared_ptr<shared::Robot> &robot,
                            double &simulation_step_size,
                            bool &verbose);
            
            void propagate(const ompl::base::State *state, 
                           const ompl::control::Control *control, 
                           const double duration, 
                           ompl::base::State *result) const;
                           
            bool canPropagateBackward() const;
            
            bool steer(const ompl::base::State* /*from*/, 
                       const ompl::base::State* /*to*/, 
                       ompl::control::Control* /*result*/, 
                       double& /*duration*/) const;
            
            bool canSteer() const;
            
        private:
            // The OMPL spacei information associated with this state propagator
            const ompl::control::SpaceInformationPtr space_information_;
            
            // Determines if the robo model has been set up
            bool model_setup_;
            
            // The robot model
            boost::shared_ptr<shared::Robot> robot_;
            
            // The simulation step size
            const double simulation_step_size_;
            
            bool verbose_;              
    };

}

#endif

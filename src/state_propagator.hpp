#ifndef STATE_PROPAGATOR_TEST_HPP_
#define STATE_PROPAGATOR_TEST_HPP_
#include <iostream>
#include <boost/timer.hpp>
#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include <ompl/control/ControlSpace.h>
#include <ompl/control/Control.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <openrave-core.h>
#include <openrave/environment.h>

#include "propagator.hpp"
#include "integrate.hpp"

namespace RBD = RigidBodyDynamics;

namespace shared {
    class StatePropagator: public ompl::control::StatePropagator {
        public:
            StatePropagator(const ompl::control::SpaceInformationPtr &si, 
                            double &simulation_step_size,                            
                            bool &linear_propagation,
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
            
            bool setupOpenRAVEEnvironment(OpenRAVE::EnvironmentBasePtr environment,
                                          OpenRAVE::RobotBasePtr robot,                                          
                                          double &coulomb,
                                          double &viscous);
            
        private:
            // The OMPL spacei information associated with this state propagator
            const ompl::control::SpaceInformationPtr space_information_;
            
            // Determines if the robo model has been set up
            bool model_setup_; 
            
            // The OpenRAVE environment
            OpenRAVE::EnvironmentBasePtr environment_;
            
            // The robot model
            OpenRAVE::RobotBasePtr robot_;
            
            std::shared_ptr<Propagator> propagator_;
            
            // The simulation step size
            double simulation_step_size_;
            
            // Determines is a linear model has to be used for state propagation
            bool linear_propagation_;
            
            Integrate linear_integrator_;
            
            bool verbose_;              
    };

}

#endif

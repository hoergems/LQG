#ifndef TORQUE_DAMPER_HPP_
#define TORQUE_DAMPER_HPP_
#include <iostream>
#include <openrave-core.h>

namespace shared {
    class TorqueDamper {
        public:
            TorqueDamper(double coulomb, double viscous);
            
            void damp_torques(std::vector<OpenRAVE::dReal> &current_velocities, 
                              std::vector<OpenRAVE::dReal> &torques);
        private:
            double coulomb_;
            double viscous_;
        
    };
}

#endif

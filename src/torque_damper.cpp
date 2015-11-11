#include "torque_damper.hpp"

using std::cout;
using std::endl;

namespace shared {

TorqueDamper::TorqueDamper(double coulomb, double viscous):
    coulomb_(coulomb),
    viscous_(viscous)
{

}

void TorqueDamper::damp_torques(std::vector<OpenRAVE::dReal> &current_velocities, 
                                std::vector<OpenRAVE::dReal> &torques) {
    
    for (size_t i = 0; i < current_velocities.size(); i++) {    
        if (current_velocities[i] != 0.0) {
            double damped_torque = -(coulomb_ * (current_velocities[i] / fabs(current_velocities[i])) + 
                                     viscous_ * current_velocities[i]);            
            torques[i] = damped_torque;
        }
        else {
        	torques[i] = 0.0;
        }
    }                                 
}

}

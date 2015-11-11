#ifndef PROPAGATOR_HPP_
#define PROPAGATOR_HPP_

#include <openrave-core.h>
#include <openrave/environment.h>
#include <iostream>

#include "torque_damper.hpp"

namespace shared {

   class Propagator {
   public:
	   Propagator();
	   
	   void setup(double &coulomb, 
			      double &viscous,
			      std::vector<double> &jointsLowerPositionLimit,
			      std::vector<double> &jointsUpperPositionLimit,
			      std::vector<double> &jointsLowerVelLimit,
			      std::vector<double> &jointsUpperVelLimit);
	   //void propagate_linear() const;
	   
	   void propagate_nonlinear(OpenRAVE::EnvironmentBasePtr environment,
		         OpenRAVE::RobotBasePtr robot,
		         const std::vector<double> &current_joint_values,
		         const std::vector<double> &current_joint_velocities,
		         std::vector<double> &control,
		         const double simulation_step_size,
		         const double duration,
		         std::vector<double> &result) const;
	   
   private:
	   std::shared_ptr<TorqueDamper> damper_;
	   
	   std::vector<double> jointsLowerPositionLimit_; 
	   std::vector<double> jointsUpperPositionLimit_;
	   std::vector<double> jointsLowerVelocityLimit_; 
	   std::vector<double> jointsUpperVelocityLimit_;
	   
	   
   };

}

#endif
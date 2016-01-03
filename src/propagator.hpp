#ifndef PROPAGATOR_HPP_
#define PROPAGATOR_HPP_
#include <Eigen/Dense>
#include <boost/timer.hpp>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <iostream>
#include "integrate.hpp"

namespace shared {

   class Propagator {
   public:
	   Propagator();
	   
	   void setup(std::vector<double> &jointsLowerPositionLimit,
			      std::vector<double> &jointsUpperPositionLimit,
			      std::vector<double> &jointsVelocityLimit,
				  bool enforce_constraints);
	   
	   bool propagate_linear(const std::vector<double> &current_joint_values,
               const std::vector<double> &control,
               const std::vector<double> &control_error,				             		             
               const double duration,
               std::vector<double> &result);
	   
	   bool propagate_nonlinear(const std::vector<double> &current_joint_values,
		                        const std::vector<double> &current_joint_velocities,
		                        std::vector<double> &control,
		                        std::vector<double> &control_error_vec,
		                        const double simulation_step_size,
		                        const double duration,
		                        std::vector<double> &result);
	   
	   void enforce_constraints(bool enforce);
	   
	   /**
	    * Gets the state dependent end-effector jacobian
	    */
	   MatrixXd get_ee_jacobian(std::vector<double> &state);
	   
	   /**
	    * Gets the underlying integrator
	    */
	   std::shared_ptr<Integrate> getIntegrator();
	   
   private:
	   std::shared_ptr<Integrate> integrator_;
	   
	   std::vector<double> jointsLowerPositionLimit_; 
	   std::vector<double> jointsUpperPositionLimit_;
	   std::vector<double> jointsVelocityLimit_;
	   
	   bool enforce_constraints_;
   };

}

#endif
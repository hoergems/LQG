#ifndef PROPAGATOR_HPP_
#define PROPAGATOR_HPP_
#include <Eigen/Dense>
#include <boost/timer.hpp>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <openrave-core.h>
#include <openrave/environment.h>
#include <iostream>

#include "torque_damper.hpp"
#include "viewer.hpp"
#include "integrate.hpp"

namespace shared {

   class Propagator {
   public:
	   Propagator();
	   
	   void setup(double coulomb, 
			      double viscous,
			      std::vector<double> &jointsLowerPositionLimit,
			      std::vector<double> &jointsUpperPositionLimit,
			      std::vector<double> &jointsLowerVelLimit,
			      std::vector<double> &jointsUpperVelLimit);
	   //void propagate_linear() const;
	   
	   bool setup_py(std::string model_file,
			         std::string environment_file,
			         double coulomb, 
	                 double viscous,
	                 bool show_viewer);
	   
	   void propagate_linear(const std::vector<double> &x_dash,
                             const std::vector<double> &u_dash,
				             const std::vector<double> &x_star_current,
				             const std::vector<double> &u_star_current,
				             const std::vector<double> &x_star_next,								  
                             const std::vector<double> &control_error_vec,	   		                 
                             const double duration,
                             std::vector<double> &result,
                             OpenRAVE::EnvironmentBasePtr environment,
                             OpenRAVE::RobotBasePtr robot);
	   
	   void propagate_nonlinear(const std::vector<double> &current_joint_values,
		         const std::vector<double> &current_joint_velocities,
		         std::vector<double> &control,
		         std::vector<double> &control_error_vec,
		         const double simulation_step_size,
		         const double duration,
		         std::vector<double> &result,		         
		         OpenRAVE::EnvironmentBasePtr environment,
		         OpenRAVE::RobotBasePtr robot);
	   
   private:
	   OpenRAVE::RobotBasePtr getRobot();
	   
	   void createPhysicsEngine(OpenRAVE::EnvironmentBasePtr env);
	   
	   std::shared_ptr<TorqueDamper> damper_;
	   
	   std::shared_ptr<Integrate> integrator_;
	   
	   std::vector<double> jointsLowerPositionLimit_; 
	   std::vector<double> jointsUpperPositionLimit_;
	   std::vector<double> jointsLowerVelocityLimit_; 
	   std::vector<double> jointsUpperVelocityLimit_;
	   
	   OpenRAVE::EnvironmentBasePtr env_;
	   OpenRAVE::RobotBasePtr robot_;
	   
	   bool show_viewer_;
   };

}

#endif
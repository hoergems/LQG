#ifndef PROPAGATOR_HPP_
#define PROPAGATOR_HPP_
#include <Eigen/Dense>
#include <boost/timer.hpp>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <openrave-core.h>
#include <openrave/environment.h>
#include <iostream>
#include "viewer.hpp"
#include "integrate.hpp"

namespace shared {

   class Propagator {
   public:
	   Propagator();
	   
	   void setup(std::vector<double> &jointsLowerPositionLimit,
			      std::vector<double> &jointsUpperPositionLimit,
			      std::vector<double> &jointsVelocityLimit);
	   //void propagate_linear() const;
	   
	   bool setup_viewer(std::string model_file,
			             std::string environment_file);
	   
	   void propagate_linear(const std::vector<double> &x_dash,
                             const std::vector<double> &u_dash,
				             const std::vector<double> &x_star_current,
				             const std::vector<double> &u_star_current,
				             const std::vector<double> &x_star_next,								  
                             const std::vector<double> &control_error_vec,	   		                 
                             const double duration,
                             std::vector<double> &result);
	   
	   void propagate_nonlinear(const std::vector<double> &current_joint_values,
		         const std::vector<double> &current_joint_velocities,
		         std::vector<double> &control,
		         std::vector<double> &control_error_vec,
		         const double simulation_step_size,
		         const double duration,
		         std::vector<double> &result);
	   
	   void update_robot_values(const std::vector<double> &current_joint_values,
	   	   		                const std::vector<double> &current_joint_velocities,									 
	   	   					    OpenRAVE::RobotBasePtr robot);
	   
   private:
	   OpenRAVE::RobotBasePtr getRobot();
	   
	   std::shared_ptr<Integrate> integrator_;
	   
	   std::vector<double> jointsLowerPositionLimit_; 
	   std::vector<double> jointsUpperPositionLimit_;
	   std::vector<double> jointsVelocityLimit_;
	   
	   OpenRAVE::EnvironmentBasePtr env_;
	   OpenRAVE::RobotBasePtr robot_;
	   
	   bool viewer_setup_;
	   
   };

}

#endif
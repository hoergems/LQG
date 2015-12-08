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
	   
	   ~Propagator() {if (viewer_setup_) { OpenRAVE::RaveDestroy(); }}
	   
	   void setup(std::vector<double> &jointsLowerPositionLimit,
			      std::vector<double> &jointsUpperPositionLimit,
			      std::vector<double> &jointsVelocityLimit,
				  bool enforce_constraints);
	   //void propagate_linear() const;
	   
	   bool setup_viewer(std::string model_file,
			             std::string environment_file);
	   
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
	   
	   void update_robot_values(const std::vector<double> &current_joint_values,
	   	   		                const std::vector<double> &current_joint_velocities,									 
	   	   					    OpenRAVE::RobotBasePtr robot);
	   
	   void enforce_constraints(bool enforce);
	   
   private:
	   OpenRAVE::RobotBasePtr getRobot();
	   
	   std::shared_ptr<Integrate> integrator_;
	   
	   std::vector<double> jointsLowerPositionLimit_; 
	   std::vector<double> jointsUpperPositionLimit_;
	   std::vector<double> jointsVelocityLimit_;
	   
	   bool enforce_constraints_;
	   
	   OpenRAVE::EnvironmentBasePtr env_;
	   OpenRAVE::RobotBasePtr robot_;
	   
	   bool viewer_setup_;
	   
   };

}

#endif
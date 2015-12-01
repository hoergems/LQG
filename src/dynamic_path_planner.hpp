#ifndef OMPL_CONTROL_TEST_HPP_
#define OMPL_CONTROL_TEST_HPP_
#include <iostream>
#include <boost/timer.hpp>
#include <boost/thread.hpp>
#include <boost/make_shared.hpp>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include "state_propagator.hpp"
#include "ManipulatorGoalRegion.hpp"
#include "torque_damper.hpp"
#include "control_space.hpp"
#include "Obstacle.hpp"
#include <openrave-core.h>
#include <openrave/environment.h>

#include <ompl/control/ControlSpace.h>
#include <ompl/control/Control.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/est/EST.h>

#include <ompl/base/State.h>
#include <ompl/base/Goal.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/base/MotionValidator.h>
#include "MotionValidator.hpp"

using std::cout;
using std::endl;
namespace shared {

    typedef boost::shared_ptr<ompl::control::PathControl> PathControlPtr;

    class DynamicPathPlanner {
        public:
        		DynamicPathPlanner(int dim, bool verbose);
                            
            	~DynamicPathPlanner() { OpenRAVE::RaveDestroy(); }
        	
            	bool isValid(const ompl::base::State *state);
            	
            	bool isValidPy(std::vector<double> &state);
        	
            	std::vector<std::vector<double>> solve(const std::vector<double> &start_state_vec, double timeout);

                OpenRAVE::EnvironmentBasePtr getEnvironment();

                OpenRAVE::RobotBasePtr getRobot();
                
                void setGoalStates(std::vector<std::vector<double>> &goal_states,
                           		           std::vector<double> &ee_goal_position,
                           		           double ee_goal_threshold);
                
                void setObstacles(const std::vector<std::shared_ptr<Obstacle> > obstacles);
                
                void setKinematics(std::shared_ptr<Kinematics> kinematics);
                
                void setupMotionValidator(bool continuous_collision);

                void setObstaclesPy(boost::python::list &ns);
                
                void setLinkDimensions(std::vector<std::vector<double>> &link_dimensions);
                
                bool setup(std::string model_file,
                		   std::string environment_file,
                		   double simulation_step_size,
						   bool linear_propagation,
						   double coulomb,
						   double viscous,
						   double control_duration); 

        private:
                std::shared_ptr<Kinematics> kinematics_;
                
                ompl::base::MotionValidatorPtr motionValidator_;
                
                boost::shared_ptr<TorqueDamper> damper_;
                
                double accepted_ = 0.0;
                
                double rejected_ = 0.0;

                double control_duration_;

                // Dimension of the state space
                unsigned int state_space_dimension_;

                // Dimension of the control space
                unsigned int control_space_dimension_;

                // The state space
                ompl::base::StateSpacePtr state_space_;

                // The bounds of the state space
                ompl::base::RealVectorBounds state_space_bounds_;

                // The control space
                ompl::control::ControlSpacePtr control_space_;

                // The space information
                ompl::control::SpaceInformationPtr space_information_;

                // The problem definition
                ompl::base::ProblemDefinitionPtr problem_definition_;

                // The planner
                ompl::base::PlannerPtr planner_;
                
                ompl::control::StatePropagatorPtr state_propagator_;

                OpenRAVE::EnvironmentBasePtr env_;
                
                std::vector<std::shared_ptr<Obstacle> > obstacles_;
                
                std::vector<std::vector<double>> goal_states_;
                
                std::vector<double> ee_goal_position_;
                            
                double ee_goal_threshold_;

                // Solve the motion planning problem
                bool solve_(double time_limit);
                
                bool setup_ompl_(OpenRAVE::RobotBasePtr &robot, 
                		         double &simulation_step_size,
                		         bool &linear_propagation,
                		         bool &verbose);
                                   
                ompl::control::ControlSamplerPtr allocUniformControlSampler_(const ompl::control::ControlSpace *control_space);
                
                bool verbose_;
                
                void log_(std::string msg, bool warn);
    };
}

#endif

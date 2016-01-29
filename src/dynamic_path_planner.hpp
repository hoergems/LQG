#ifndef OMPL_CONTROL_TEST_HPP_
#define OMPL_CONTROL_TEST_HPP_
#include <iostream>
#include <boost/timer.hpp>
#include <boost/thread.hpp>
#include <boost/make_shared.hpp>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include "ManipulatorSpaceInformation.hpp"
#include "state_propagator.hpp"
#include "ManipulatorGoalRegion.hpp"
#include "control_space.hpp"
#include "Obstacle.hpp"

#include <ompl/control/ControlSpace.h>
#include <ompl/control/Control.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SpaceInformation.h>
//#include <ompl/control/planners/rrt/RRT.h>
#include "rrt_control.hpp"
#include "EST_control.hpp"
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
#include "robot.hpp"

using std::cout;
using std::endl;
namespace shared {

    typedef boost::shared_ptr<ompl::control::PathControl> PathControlPtr;

    class DynamicPathPlanner {
        public:
        		DynamicPathPlanner(boost::shared_ptr<shared::Robot> &robot, bool verbose);
                            
            	~DynamicPathPlanner() {  }
        	
            	bool isValid(const ompl::base::State *state);
            	
            	bool isValidPy(std::vector<double> &state);
        	
            	std::vector<std::vector<double>> solve(const std::vector<double> &start_state_vec, double timeout);
                
                void setGoalStates(std::vector<std::vector<double>> &goal_states,
                           		           std::vector<double> &ee_goal_position,
                           		           double ee_goal_threshold);
                
                void setObstacles(const std::vector<std::shared_ptr<Obstacle> > obstacles);
                
                void setupMotionValidator(bool continuous_collision);

                void setObstaclesPy(boost::python::list &ns); 
                
                bool setup(double simulation_step_size,
						   double control_duration);
                
                void getAllStates(std::vector<std::vector<double>> &all_states);
                
                void setNumControlSamples(std::vector<int> &num_control_samples);
                
                void setMinMaxControlDuration(std::vector<int> &min_max_control_duration);
                
                void addIntermediateStates(bool add_intermediate_states);
                
                void setRRTGoalBias(double goal_bias);
                
                void setControlSampler(std::string control_sampler);

        private:
                std::shared_ptr<Kinematics> kinematics_;
                
                ompl::base::MotionValidatorPtr motionValidator_;
                
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
                
                std::vector<std::shared_ptr<Obstacle> > obstacles_;
                
                std::vector<std::vector<double>> goal_states_;
                
                std::vector<double> ee_goal_position_;
                            
                double ee_goal_threshold_;
                
                boost::shared_ptr<shared::Robot> robot_;

                // Solve the motion planning problem
                bool solve_(double time_limit);
                
                bool setup_ompl_(double &simulation_step_size,
                		         bool &verbose);
                                   
                ompl::control::ControlSamplerPtr allocUniformControlSampler_(const ompl::control::ControlSpace *control_space);
                
                bool verbose_;
                
                void log_(std::string msg, bool warn);
                
                std::vector<std::vector<double>> all_states_;
    };
}

#endif

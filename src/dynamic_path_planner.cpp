#include "dynamic_path_planner.hpp"
#include <random>
#include <fstream>
#include <iterator>
#include <stdio.h>

using std::cout;
using std::endl;

namespace shared {

DynamicPathPlanner::DynamicPathPlanner(boost::shared_ptr<shared::Robot> &robot,
		                               bool verbose):
	state_space_dimension_(2 * robot->getDOF()),
	control_space_dimension_(robot->getDOF()),
    state_space_(new ompl::base::RealVectorStateSpace(state_space_dimension_)),
    state_space_bounds_(1),
	kinematics_(nullptr),
    control_space_(new ControlSpace(state_space_, control_space_dimension_)),
    space_information_(new ManipulatorSpaceInformation(state_space_, control_space_)),
    problem_definition_(nullptr),
    planner_(nullptr),
    state_propagator_(nullptr),    
    motionValidator_(nullptr),
    verbose_(verbose),
    robot_(robot),
	all_states_()	
{
    
}

void DynamicPathPlanner::setupMotionValidator(bool continuous_collision) {	
	motionValidator_ = boost::make_shared<MotionValidator>(space_information_,
			                                               robot_,
														   continuous_collision,
														   true);	
}

void DynamicPathPlanner::setRRTGoalBias(double goal_bias) {
	boost::static_pointer_cast<RRTControl>(planner_)->setGoalBias(goal_bias);
}

void DynamicPathPlanner::log_(std::string msg, bool warn=false) {
	if (warn) {
		cout << "DynamicPathPlanner: " << msg << endl;
	}
	else if (verbose_) {
		cout << "DynamicPathPlanner: " << msg << endl;
	}
}

bool DynamicPathPlanner::setup(double simulation_step_size,
							   double control_duration) {	
	control_duration_ = control_duration;
	    
	/***** Setup OMPL *****/
	log_("Setting up OMPL");
	setup_ompl_(simulation_step_size, verbose_);
	log_("OMPL setup");
	log_("Setup complete");
	return true;
}

ompl::control::ControlSamplerPtr DynamicPathPlanner::allocUniformControlSampler_(const ompl::control::ControlSpace *control_space) {	
	return nullptr;
    return ompl::control::ControlSamplerPtr(new UniformControlSampler(control_space));
}

void DynamicPathPlanner::setNumControlSamples(std::vector<int> &num_control_samples) {
	unsigned int ncs = (unsigned int)num_control_samples[0];
	boost::static_pointer_cast<ManipulatorSpaceInformation>(space_information_)->setNumControlSamples(ncs);	
}

void DynamicPathPlanner::setControlSampler(std::string control_sampler) {
	boost::static_pointer_cast<shared::ControlSpace>(control_space_)->setControlSampler(control_sampler);
	//control_space_->setControlSampler(control_sampler);
}

void DynamicPathPlanner::setMinMaxControlDuration(std::vector<int> &min_max_control_duration) {
	unsigned int min = (unsigned int)min_max_control_duration[0];
	unsigned int max = (unsigned int)min_max_control_duration[1];
	space_information_->setMinMaxControlDuration(min, max);	
}

void DynamicPathPlanner::addIntermediateStates(bool add_intermediate_states) {
	boost::static_pointer_cast<RRTControl>(planner_)->setIntermediateStates(add_intermediate_states);
}

bool DynamicPathPlanner::setup_ompl_(double &simulation_step_size,
		                             bool &verbose) {
	if (!verbose_) {        
	    ompl::msg::noOutputHandler();
	}
    state_space_bounds_ = ompl::base::RealVectorBounds(state_space_dimension_);    
    //space_information_->setStateValidityChecker(boost::bind(&DynamicPathPlanner::isValid, this, _1));
    space_information_->setMotionValidator(motionValidator_);
    space_information_->setMinMaxControlDuration(1, 1);
    space_information_->setPropagationStepSize(control_duration_);
     
    problem_definition_ = boost::make_shared<ompl::base::ProblemDefinition>(space_information_);
    //planner_ = boost::make_shared<ompl::control::RRT>(space_information_);
    planner_ = boost::make_shared<RRTControl>(space_information_);
    //planner_ = boost::make_shared<ESTControl>(space_information_);
    planner_->setProblemDefinition(problem_definition_);
    boost::static_pointer_cast<RRTControl>(planner_)->setIntermediateStates(true);
    state_propagator_ = boost::make_shared<StatePropagator>(space_information_,
    		                                                robot_,
                                                            simulation_step_size,
                                                            verbose);    
    space_information_->setStatePropagator(state_propagator_);
    
    // Set the bounds    
    ompl::base::RealVectorBounds control_bounds(control_space_dimension_);
    std::vector<std::string> active_joints;
    robot_->getActiveJoints(active_joints);
    
    std::vector<double> lower_position_limits;
    std::vector<double> upper_position_limits;
    std::vector<double> velocity_limits;
    std::vector<double> torque_limits;
    
    robot_->getJointLowerPositionLimits(active_joints, lower_position_limits);
    robot_->getJointUpperPositionLimits(active_joints, upper_position_limits);
    robot_->getJointVelocityLimits(active_joints, velocity_limits);
    robot_->getJointTorqueLimits(active_joints, torque_limits);
    
        
    for (size_t i = 0; i < active_joints.size(); i++) {
        // Set the joints position bounds        
        state_space_bounds_.setLow(i, lower_position_limits[i]);
        state_space_bounds_.setHigh(i, upper_position_limits[i]);

        // Set the joints velocity bounds              
        state_space_bounds_.setLow(i + state_space_dimension_ / 2, -velocity_limits[i]);
        state_space_bounds_.setHigh(i + state_space_dimension_ / 2, velocity_limits[i]);
        
        control_bounds.setLow(i, -torque_limits[i]);
        control_bounds.setHigh(i, torque_limits[i]);
        //torque_bounds = torque_bounds - 0.1;
    }    
    
    state_space_->as<ompl::base::RealVectorStateSpace>()->setBounds(state_space_bounds_);
    control_space_->as<ompl::control::RealVectorControlSpace>()->setBounds(control_bounds);    
    return true;
}

bool DynamicPathPlanner::isValid(const ompl::base::State *state) {	
	std::vector<double> state_vec;
	for (unsigned int i = 0; i < space_information_->getStateSpace()->getDimension(); i++) {
	    state_vec.push_back(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);        
	} 
	all_states_.push_back(state_vec);
    if (static_cast<MotionValidator &>(*motionValidator_).isValid(state_vec)) {
    	accepted_ = accepted_ + 1.0;
    	return true;
    }
    else {
    	rejected_ = rejected_ + 1.0;
    	return false;
    }
    /**if (valid) {
    	accepted_ = accepted_ + 1.0;
    }
    else {
    	cout << "not valid: ";
    	for (unsigned int i = 0; i < space_information_->getStateSpace()->getDimension(); i++) {
    		cout << state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i] << ", ";
    	}
    	cout << endl;
    	rejected_ = rejected_ + 1.0;
    	return false;
    }
    
    all_states_.push_back(state_vec);
    return static_cast<MotionValidator &>(*motionValidator_).isValid(state_vec); */   
}

void DynamicPathPlanner::getAllStates(std::vector<std::vector<double>> &all_states) {	
	for (auto &k: all_states_) {
		all_states.push_back(k);		
	}
}

bool DynamicPathPlanner::isValidPy(std::vector<double> &state) {
    bool valid = static_cast<MotionValidator &>(*motionValidator_).isValid(state);
    return valid;    
}

bool DynamicPathPlanner::solve_(double time_limit) {
    bool solved = false;
    bool hasExactSolution = false;    
    while (!solved && !hasExactSolution) {
        solved = planner_->solve(time_limit);
        
        // Get all the solutions
        std::vector<ompl::base::PlannerSolution> solutions = problem_definition_->getSolutions();
        for (size_t i = 0; i < solutions.size(); i++) {
            if (!solutions[i].approximate_) {
                hasExactSolution = true;                
                break;
            }
        }
        // Check if there's an exact solution
    }    
    return hasExactSolution;
}

void DynamicPathPlanner::setGoalStates(std::vector<std::vector<double>> &goal_states,
                                       std::vector<double> &ee_goal_position,
                                       double ee_goal_threshold) {
    for (size_t i = 0; i < goal_states.size(); i++) {
        goal_states_.push_back(goal_states[i]);
    }
    
    ee_goal_position_.clear();
    for (size_t i = 0; i < ee_goal_position.size(); i++) {
    	ee_goal_position_.push_back(ee_goal_position[i]);
    }
    
    ee_goal_threshold_ = ee_goal_threshold;
}

void DynamicPathPlanner::setObstacles(std::vector<std::shared_ptr<Obstacle> > obstacles) {
    for (size_t i = 0; i < obstacles.size(); i++) {        
        obstacles_.push_back(obstacles[i]);
    }
    static_cast<MotionValidator &>(*motionValidator_).setObstacles(obstacles_);
}

void DynamicPathPlanner::setObstaclesPy(boost::python::list &ns) {
    for (size_t i = 0; i < len(ns); i++) {
    	std::shared_ptr<ObstacleWrapper> obst_wrapper = boost::python::extract<std::shared_ptr<ObstacleWrapper>>(ns[i]);
    	obstacles_.push_back(std::static_pointer_cast<shared::Obstacle>(obst_wrapper));
        //obstacles_.push_back(std::make_shared<Obstacle>(boost::python::extract<Obstacle>(ns[i])));
    }

    static_cast<MotionValidator &>(*motionValidator_).setObstacles(obstacles_);
}

std::vector<std::vector<double>> DynamicPathPlanner::solve(const std::vector<double> &start_state_vec,
		                                                   double timeout) {
    // Set the start and goal state	
    ompl::base::ScopedState<> start_state(state_space_);
    std::vector<std::vector<double>> solution_vector; 
    for (unsigned int i = 0; i < state_space_dimension_; i++) {
        start_state[i] = start_state_vec[i];        
    }
    
    if (!static_cast<MotionValidator &>(*motionValidator_).isValid(start_state_vec)) {
    	cout << "DynamicPathPlanner: ERROR: Start state is not valid!" << endl;
    	return solution_vector;
    }

    ompl::base::GoalPtr gp(new ManipulatorGoalRegion(space_information_,
    		                                         robot_,
    		                                         goal_states_, 
    		                                         ee_goal_position_, 
    		                                         ee_goal_threshold_,
    		                                         true));
    boost::static_pointer_cast<ManipulatorGoalRegion>(gp)->setThreshold(ee_goal_threshold_);
    
    problem_definition_->addStartState(start_state);    
    problem_definition_->setGoal(gp);
    
    //planner_->setGoalBias(0.1);
    planner_->setup();
    bool solved = false;
    
    boost::timer t;
    solved = solve_(timeout);    
    
    if (solved) {
        ompl::base::PlannerSolution planner_solution(problem_definition_->getSolutionPath());                
        PathControlPtr solution_path_ = 
            boost::static_pointer_cast<ompl::control::PathControl>(planner_solution.path_);               
        //cout << "Length of solution path " << solution_path_->length() << endl << endl;
        std::vector<ompl::base::State*> solution_states_(solution_path_->getStates());
        std::vector<ompl::control::Control*> solution_controls_(solution_path_->getControls());
        std::vector<double> control_durations(solution_path_->getControlDurations());
        /**cout << "durations: ";
        for (auto &k: control_durations) {
        	cout << k << ", ";
        }
        cout << endl;*/
        for (size_t i = 0; i < solution_states_.size(); i++) {
            //cout << "State: ";
            std::vector<double> solution_state;
            for (size_t j = 0; j < state_space_dimension_; j++) {
            	solution_state.push_back(solution_states_[i]->as<ompl::base::RealVectorStateSpace::StateType>()->values[j]);
                //cout << solution_states_[i]->as<ompl::base::RealVectorStateSpace::StateType>()->values[j] << ", ";
            }
            //cout << endl;
            
            
            //cout << "Control: ";
            for (size_t j = 0; j < state_space_dimension_ / 2; j++) {
            	if (i < solution_states_.size() - 1) {
                    solution_state.push_back(solution_controls_[i]->as<ompl::control::RealVectorControlSpace::ControlType>()->values[j]);
                    //cout << solution_controls_[i]->as<ompl::control::RealVectorControlSpace::ControlType>()->values[j] << ", ";
            	}
            	else {            		
            		solution_state.push_back(0.0);            		
            	}
            }
            
            //cout << "Duration: " << control_durations[i] << endl;
            /**for (size_t j = 0; j < state_space_dimension_ / 2; j++) { 
            	solution_state.push_back(0.0);
            }*/
            
            //cout << endl;
            
            for (size_t j = 0; j < state_space_dimension_; j++) {
                solution_state.push_back(solution_states_[i]->as<ompl::base::RealVectorStateSpace::StateType>()->values[j]);                
            }
            
            if (i < solution_states_.size() - 1) { 
            	solution_state.push_back(control_durations[i]);
            }
            else {
            	solution_state.push_back(0.0);
            }
            
            solution_vector.push_back(solution_state);          
        }
        //cout << "Solution found in " << t.elapsed() << "seconds" << endl;
        //cout << "accepted " << accepted_ << endl;
        //cout << "rejected " << rejected_ << endl;        
        //return solution_path_; 
    }
    
    return solution_vector;
}

BOOST_PYTHON_MODULE(libdynamic_path_planner) {
    using namespace boost::python;
    
    class_<std::vector<int> > ("v_int")
             .def(vector_indexing_suite<std::vector<int> >());
   
    class_<DynamicPathPlanner>("DynamicPathPlanner", init<boost::shared_ptr<shared::Robot> &, bool>())
							   .def("solve", &DynamicPathPlanner::solve)
		                       .def("setObstacles", &DynamicPathPlanner::setObstaclesPy)
							   .def("setGoalStates", &DynamicPathPlanner::setGoalStates)
							   .def("isValid", &DynamicPathPlanner::isValidPy)
							   .def("setup", &DynamicPathPlanner::setup)
							   .def("setupMotionValidator", &DynamicPathPlanner::setupMotionValidator)
							   .def("getAllStates", &DynamicPathPlanner::getAllStates)
							   .def("setNumControlSamples", &DynamicPathPlanner::setNumControlSamples)
							   .def("setMinMaxControlDuration", &DynamicPathPlanner::setMinMaxControlDuration)
							   .def("addIntermediateStates", &DynamicPathPlanner::addIntermediateStates)
							   .def("setRRTGoalBias", &DynamicPathPlanner::setRRTGoalBias)
							   .def("setControlSampler", &DynamicPathPlanner::setControlSampler)
										            		 
                        //.def("doIntegration", &Integrate::do_integration)                        
                        //.def("getResult", &Integrate::getResult)
    ;
}
 
}
#include "dynamic_path_planner.hpp"
#include <random>
#include <fstream>
#include <iterator>
#include <stdio.h>

using std::cout;
using std::endl;

namespace shared {

DynamicPathPlanner::DynamicPathPlanner(int dim, bool verbose):
	state_space_dimension_(dim),
	control_space_dimension_(dim / 2),
    state_space_(new ompl::base::RealVectorStateSpace(state_space_dimension_)),
    state_space_bounds_(1),
	kinematics_(nullptr),
    control_space_(new ControlSpace(state_space_, control_space_dimension_)),
    space_information_(new ompl::control::SpaceInformation(state_space_, control_space_)),
    problem_definition_(nullptr),
    planner_(nullptr),
    state_propagator_(nullptr),
    env_(nullptr),
    motionValidator_(nullptr),
    verbose_(verbose)
{
    
}


void DynamicPathPlanner::setKinematics(std::shared_ptr<Kinematics> kinematics) {
	kinematics_ = kinematics;
	static_cast<MotionValidator &>(*motionValidator_).setKinematics(kinematics_);
	cout << "Kinematics in motion validator set" << endl;
}

void DynamicPathPlanner::setupMotionValidator(bool continuous_collision) {
	cout << "setup motion validator" << endl;
	motionValidator_ = boost::make_shared<MotionValidator>(space_information_,				                                           
														   continuous_collision,
														   true);
	cout << "Motion validator setup" << endl;
}

bool DynamicPathPlanner::setup(std::string model_file,
		                       std::string environment_file,
		                       double simulation_step_size,
							   bool linear_propagation,
							   double coulomb,
							   double viscous,
							   double control_duration) {	
	control_duration_ = control_duration;	
	
	/***** Initialize OpenRAVE *****/
	OpenRAVE::RaveInitialize(true);    
	env_ = OpenRAVE::RaveCreateEnvironment();    
	env_->Load(environment_file);
	const std::string module_str("or_urdf_plugin");
	if(!OpenRAVE::RaveLoadPlugin(module_str)) {
	    cout << "Failed to load the or_urdf_plugin." << endl;
	    return false;
	}
	    
	OpenRAVE::ModuleBasePtr urdf_module = OpenRAVE::RaveCreateModule(env_, "URDF");
	const std::string cmdargs("");
	env_->AddModule(urdf_module, cmdargs);
	std::stringstream sinput, sout;
	sinput << "load " << model_file;
	if (!urdf_module->SendCommand(sout,sinput)) {
	    cout << "Failed to load URDF model" << endl;
	    return false;
	}
	
	std::vector<OpenRAVE::KinBodyPtr> bodies;
	env_->GetBodies(bodies);
	env_->StopSimulation();
	    
	OpenRAVE::RobotBasePtr robot = getRobot();	

	const std::vector<OpenRAVE::KinBody::LinkPtr> links(robot->GetLinks());    
	links[0]->SetStatic(true);    
	    
	/***** Setup OMPL *****/
	setup_ompl_(robot, simulation_step_size, linear_propagation, verbose_);
	    
	/***** Create the physics engine *****/
	const std::string engine = "ode";
	OpenRAVE::PhysicsEngineBasePtr physics_engine_ = OpenRAVE::RaveCreatePhysicsEngine(env_, engine);
	    
	const OpenRAVE::Vector gravity({0.0, 0.0, -9.81});    
	physics_engine_->SetGravity(gravity);
	env_->SetPhysicsEngine(physics_engine_);
	boost::static_pointer_cast<StatePropagator>(state_propagator_)->setupOpenRAVEEnvironment(env_, 
	    		                                                                             robot,
	    		                                                                             coulomb,
	    		                                                                             viscous);
	return true;
}

OpenRAVE::EnvironmentBasePtr DynamicPathPlanner::getEnvironment() {
    return env_;
}

OpenRAVE::RobotBasePtr DynamicPathPlanner::getRobot() {
	std::vector<OpenRAVE::KinBodyPtr> bodies;
	env_->GetBodies(bodies);
	for (auto &body: bodies) {
	    if (body->GetDOF() > 0) {
	        OpenRAVE::RobotBasePtr robot = boost::static_pointer_cast<OpenRAVE::RobotBase>(body);
	    	return robot;
	    }    	
	}  
}

ompl::control::ControlSamplerPtr DynamicPathPlanner::allocUniformControlSampler_(const ompl::control::ControlSpace *control_space) {	
	return nullptr;
    return ompl::control::ControlSamplerPtr(new UniformControlSampler(control_space));
}

bool DynamicPathPlanner::setup_ompl_(OpenRAVE::RobotBasePtr &robot, 
		                             double &simulation_step_size,
		                             bool &linear_propagation,
		                             bool &verbose) {
    // The state space consists of joint angles + velocity    
    //state_space_dimension_ = robot->GetDOF() * 2;
    //control_space_dimension_ = state_space_dimension_ / 2;    
    //state_space_ = boost::make_shared<ompl::base::RealVectorStateSpace>(state_space_dimension_);    
    state_space_bounds_ = ompl::base::RealVectorBounds(state_space_dimension_);
    //control_space_ = boost::make_shared<ControlSpace>(state_space_, control_space_dimension_);
    
    //space_information_ = boost::make_shared<ompl::control::SpaceInformation>(state_space_, control_space_);    
    space_information_->setStateValidityChecker(boost::bind(&DynamicPathPlanner::isValid, this, _1));
    space_information_->setMotionValidator(motionValidator_);
    space_information_->setMinMaxControlDuration(1, 1);
    space_information_->setPropagationStepSize(control_duration_);
     
    problem_definition_ = boost::make_shared<ompl::base::ProblemDefinition>(space_information_);
    planner_ = boost::make_shared<ompl::control::RRT>(space_information_);
    planner_->as<ompl::control::RRT>()->setIntermediateStates(false);
    //planner_->as<ompl::control::RRT>()->setGoalBias(0.1);
    planner_->setProblemDefinition(problem_definition_);   
    
    state_propagator_ = boost::make_shared<StatePropagator>(space_information_, 
                                                            simulation_step_size,                                                            
                                                            linear_propagation,
                                                            verbose);    
    space_information_->setStatePropagator(state_propagator_);
    
    // Set the bounds
    const std::vector<OpenRAVE::KinBody::JointPtr> joints(robot->GetJoints());
    ompl::base::RealVectorBounds control_bounds(control_space_dimension_);    
        
    for (size_t i = 0; i < joints.size(); i++) {
        std::vector<OpenRAVE::dReal> lower_limit;
        std::vector<OpenRAVE::dReal> upper_limit;        
        joints[i]->GetLimits(lower_limit, upper_limit);        
        
        // Set the joints position bounds        
        state_space_bounds_.setLow(i, lower_limit[0]);
        state_space_bounds_.setHigh(i, upper_limit[0]);

        // Set the joints velocity bounds              
        state_space_bounds_.setLow(i + state_space_dimension_ / 2, -joints[i]->GetMaxVel());
        state_space_bounds_.setHigh(i + state_space_dimension_ / 2, joints[i]->GetMaxVel());
        
        control_bounds.setLow(i, -joints[i]->GetMaxTorque());
        control_bounds.setHigh(i, joints[i]->GetMaxTorque());
        //torque_bounds = torque_bounds - 0.1;
    }    
    
    state_space_->as<ompl::base::RealVectorStateSpace>()->setBounds(state_space_bounds_);
    control_space_->as<ompl::control::RealVectorControlSpace>()->setBounds(control_bounds);    
    return true;
}

bool DynamicPathPlanner::isValid(const ompl::base::State *state) {
    bool valid = state_space_->as<ompl::base::RealVectorStateSpace>()->satisfiesBounds(state);
    if (valid) {
    	accepted_ = accepted_ + 1.0;
    }
    else {
    	rejected_ = rejected_ + 1.0;
    	return false;
    }
    std::vector<double> state_vec;
    for (unsigned int i = 0; i < space_information_->getStateSpace()->getDimension(); i++) {
        state_vec.push_back(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);        
    }
    return static_cast<MotionValidator &>(*motionValidator_).isValid(state_vec);
    
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
        obstacles_.push_back(std::make_shared<Obstacle>(boost::python::extract<Obstacle>(ns[i])));
    }

    static_cast<MotionValidator &>(*motionValidator_).setObstacles(obstacles_);
}

void DynamicPathPlanner::setLinkDimensions(std::vector<std::vector<double>> &link_dimensions) {	
    boost::shared_ptr<MotionValidator> mv = boost::static_pointer_cast<MotionValidator>(space_information_->getMotionValidator());    
    mv->setLinkDimensions(link_dimensions);    
}

std::vector<std::vector<double>> DynamicPathPlanner::solve(const std::vector<double> &start_state_vec,
		                                                   double timeout) {
    // Set the start and goal state
	cout << "solve" << endl;
    ompl::base::ScopedState<> start_state(state_space_);
    std::vector<std::vector<double>> solution_vector; 
    for (unsigned int i = 0; i < state_space_dimension_; i++) {
        start_state[i] = start_state_vec[i];        
    }

    ompl::base::GoalPtr gp(new ManipulatorGoalRegion(space_information_, 
    		                                         goal_states_, 
    		                                         ee_goal_position_, 
    		                                         ee_goal_threshold_, 
    		                                         kinematics_,
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
        cout << "Solution is approximate: ";
        if (planner_solution.approximate_) {
            cout << " True";
        }
        else {
            cout << " False";
        }
        cout << endl;        
        PathControlPtr solution_path_ = 
            boost::static_pointer_cast<ompl::control::PathControl>(planner_solution.path_);               
        cout << "Length of solution path " << solution_path_->length() << endl << endl;
        std::vector<ompl::base::State*> solution_states_(solution_path_->getStates());
        std::vector<ompl::control::Control*> solution_controls_(solution_path_->getControls());
        std::vector<double> control_durations(solution_path_->getControlDurations());
        cout << "durations: ";
        for (auto &k: control_durations) {
        	cout << k << ", ";
        }
        cout << endl;
        for (size_t i = 0; i < solution_states_.size(); i++) {
            cout << "State: ";
            std::vector<double> solution_state;
            for (size_t j = 0; j < state_space_dimension_; j++) {
            	solution_state.push_back(solution_states_[i]->as<ompl::base::RealVectorStateSpace::StateType>()->values[j]);
                cout << solution_states_[i]->as<ompl::base::RealVectorStateSpace::StateType>()->values[j] << ", ";
            }
            cout << endl;
            
            
            cout << "Control: ";
            for (size_t j = 0; j < state_space_dimension_ / 2; j++) {
            	if (i < solution_states_.size() - 1) {
                    solution_state.push_back(solution_controls_[i]->as<ompl::control::RealVectorControlSpace::ControlType>()->values[j]);
                    cout << solution_controls_[i]->as<ompl::control::RealVectorControlSpace::ControlType>()->values[j] << ", ";
            	}
            	else {            		
            		solution_state.push_back(0.0);            		
            	}
            }
            for (size_t j = 0; j < state_space_dimension_ / 2; j++) { 
            	solution_state.push_back(0.0);
            }
            
            cout << endl;
            
            for (size_t j = 0; j < state_space_dimension_; j++) {
                solution_state.push_back(solution_states_[i]->as<ompl::base::RealVectorStateSpace::StateType>()->values[j]);                
            }
            solution_vector.push_back(solution_state);          
        }
        cout << "Solution found in " << t.elapsed() << "seconds" << endl;
        cout << "accepted " << accepted_ << endl;
        cout << "rejected " << rejected_ << endl;        
        //return solution_path_; 
    }
    
    return solution_vector;
}

BOOST_PYTHON_MODULE(libdynamic_path_planner) {
    using namespace boost::python;    
   
    class_<DynamicPathPlanner>("DynamicPathPlanner", init<int, bool>())
							   .def("solve", &DynamicPathPlanner::solve)
		                       .def("setObstacles", &DynamicPathPlanner::setObstaclesPy)
							   .def("setGoalStates", &DynamicPathPlanner::setGoalStates)
							   .def("isValid", &DynamicPathPlanner::isValidPy)
							   .def("setup", &DynamicPathPlanner::setup)
							   .def("setLinkDimensions", &DynamicPathPlanner::setLinkDimensions)
							   .def("setKinematics", &DynamicPathPlanner::setKinematics)
							   .def("setupMotionValidator", &DynamicPathPlanner::setupMotionValidator)
										            		 
                        //.def("doIntegration", &Integrate::do_integration)                        
                        //.def("getResult", &Integrate::getResult)
    ;
}
 
}



/**int main(int argc, char** argv) {
    double coulomb = 0.0;
    double viscous = 1.0;
    double control_duration = 0.005;
    double simulation_step_size = 0.001;    
    double time_limit = 10.0;
    bool linear_propagation = false;
    bool verbose = true;
    const std::string model_file("test.urdf");
    shared::OMPLControl ompl_test(model_file,
                                      control_duration,
                                      simulation_step_size,
                                      coulomb,
                                      viscous,
                                      linear_propagation,
                                      verbose);
    //ompl_test.testNormalDist(control_duration, simulation_step_size, coulomb, viscous);    
    //OpenRAVE::EnvironmentBasePtr env = ompl_test.getEnvironment(); 
    
    //ompl_test.testPhysics(simulation_step_size, coulomb, viscous);    
    shared::PathControlPtr controls = ompl_test.test(time_limit);
    //ompl_test.viewControls(controls,
                           //simulation_step_size);
    //OpenRAVE::RaveDestroy();
    return 0;
}*/



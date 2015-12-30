#include "path_planner.hpp"

using std::cout;
using std::endl;

namespace shared {

PathPlanner::PathPlanner(boost::shared_ptr<shared::Robot> &robot,		                 
                         double delta_t,
                         bool continuous_collision,                         
                         double max_joint_velocity,
                         double stretching_factor,
                         bool check_linear_path,                         
                         bool verbose,						 
                         std::string planner):
    robot_(robot),
    dim_(robot->getDOF()),
    delta_t_(delta_t),
    continuous_collision_(continuous_collision),
    max_joint_velocity_(max_joint_velocity),
    stretching_factor_(stretching_factor),
    planning_range_(delta_t_ * max_joint_velocity_),     
    check_linear_path_(check_linear_path),  
    space_(new ompl::base::RealVectorStateSpace(dim_)),    
    si_(new ompl::base::SpaceInformation(space_)),    
    problem_definition_(new ompl::base::ProblemDefinition(si_)),
    planner_str_(planner),    
    planner_(nullptr),
    motionValidator_(new MotionValidator(si_,
    		                             robot_,
                                         continuous_collision,										 
                                         false)),
    verbose_(verbose)	
{    
	if (!verbose_) {        
        ompl::msg::noOutputHandler();
    }
	
	/**std::vector<std::string> active_joints;
	std::vector<double> joint_velocity_limits;
	robot_->getActiveJoints(active_joints);
	robot_->getJointVelocityLimits(active_joints, joint_velocity_limits);
	double max_velocity = 0.0;
	for (auto &k: joint_velocity_limits) {
		if (k > max_velocity) {
			max_velocity = k;
		}
	}
	
	planning_range_ = delta_t_ * max_velocity;*/
    
    if (planner_str_ == "RRTConnect") {
    	planner_ = boost::shared_ptr<ompl::geometric::RRTConnect>(new ompl::geometric::RRTConnect(si_));
    }
    else if (planner_str_ == "RRT") {
        planner_ = boost::shared_ptr<ompl::geometric::RRT>(new ompl::geometric::RRT(si_));
    }
    else if (planner_str_ == "SBL"){
    	planner_ = boost::shared_ptr<ompl::geometric::SBL>(new ompl::geometric::SBL(si_));
    }
    else if (planner_str_ == "BKPIECE1"){
        planner_ = boost::shared_ptr<ompl::geometric::BKPIECE1>(new ompl::geometric::BKPIECE1(si_));
    }
    else if (planner_str_ == "PDST"){
        planner_ = boost::shared_ptr<ompl::geometric::PDST>(new ompl::geometric::PDST(si_));
    }
    else if (planner_str_ == "STRIDE"){
        planner_ = boost::shared_ptr<ompl::geometric::STRIDE>(new ompl::geometric::STRIDE(si_));
    }
}

void PathPlanner::setup() {	
	std::vector<std::string> activeJoints;
    robot_->getActiveJoints(activeJoints);
    
    std::vector<double> lowerJointPositionPonstraints;
    std::vector<double> upperJointPositionConstraints;
    
    robot_->getJointLowerPositionLimits(activeJoints, lowerJointPositionPonstraints);
    robot_->getJointUpperPositionLimits(activeJoints, upperJointPositionConstraints);
	
	ompl::base::RealVectorBounds bounds(dim_);
	if (robot_->constraintsEnforced()) {
		/** We assume that the there are no joint limits. So the range of each joint
		*** is between -2*Pi and 2*Pi
		*/   
		for (size_t i = 0; i < dim_; i++) {				
			bounds.setLow(i, lowerJointPositionPonstraints[i]);
			bounds.setHigh(i, upperJointPositionConstraints[i]);
		}
	}
	else {
		for (size_t i = 0; i < dim_; i++) {
			bounds.setLow(i, -100000);
			bounds.setHigh(i, 100000);			
		}
	}
	
	/** Apply the bounds to the space */    
   space_->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
    
    /** Set the StateValidityChecker */    
    si_->setStateValidityChecker(boost::bind(&PathPlanner::isValid, this, _1)); 
    
    /** Set the MotionValidation */
    si_->setMotionValidator(motionValidator_);
    
    /** Let the planner know about our problem */    
    planner_->setProblemDefinition(problem_definition_);
    
    /** Use RRTConnect as the planning algorithm */
    if (planner_str_ == "RRT") {    	
    	boost::shared_ptr<ompl::geometric::RRT> planner_ptr = boost::static_pointer_cast<ompl::geometric::RRT>(planner_);
    	planner_ptr->setRange(planning_range_);
    	planner_ptr->setGoalBias(0.1);
    	
    }
    else if (planner_str_ == "RRTConnect") {
    	boost::shared_ptr<ompl::geometric::RRTConnect> planner_ptr = boost::static_pointer_cast<ompl::geometric::RRTConnect>(planner_);
    	planner_ptr->setRange(planning_range_);
    } 
    
    else if (planner_str_ == "SBL") {
        boost::shared_ptr<ompl::geometric::SBL> planner_ptr = boost::static_pointer_cast<ompl::geometric::SBL>(planner_);
        planner_ptr->setRange(planning_range_);
    } 
    else if (planner_str_ == "BKPIECE1") {
        boost::shared_ptr<ompl::geometric::BKPIECE1> planner_ptr = boost::static_pointer_cast<ompl::geometric::BKPIECE1>(planner_);
        planner_ptr->setRange(planning_range_);
    }
    else if (planner_str_ == "PDST") {
        boost::shared_ptr<ompl::geometric::PDST> planner_ptr = boost::static_pointer_cast<ompl::geometric::PDST>(planner_);
        //planner_ptr->setRange(planning_range_);
        planner_ptr->setGoalBias(0.1);
    }
    
    else if (planner_str_ == "STRIDE") {
        boost::shared_ptr<ompl::geometric::STRIDE> planner_ptr = boost::static_pointer_cast<ompl::geometric::STRIDE>(planner_);
        planner_ptr->setRange(planning_range_);
    }
}

void PathPlanner::setObstacles(std::vector<std::shared_ptr<Obstacle> > obstacles) {
    for (size_t i = 0; i < obstacles.size(); i++) {        
        obstacles_.push_back(obstacles[i]);
    }
    static_cast<MotionValidator &>(*motionValidator_).setObstacles(obstacles_);
}

void PathPlanner::setObstaclesPy(boost::python::list &ns) {
    for (size_t i = 0; i < len(ns); i++) {
    	std::shared_ptr<ObstacleWrapper> obst_wrapper = boost::python::extract<std::shared_ptr<ObstacleWrapper>>(ns[i]);
    	obstacles_.push_back(std::static_pointer_cast<shared::Obstacle>(obst_wrapper));
        //obstacles_.push_back(std::make_shared<Obstacle>(boost::python::extract<Obstacle>(ns[i])));
    }

    static_cast<MotionValidator &>(*motionValidator_).setObstacles(obstacles_);
}

ompl::base::MotionValidatorPtr PathPlanner::getMotionValidator() {
    return motionValidator_;
}

bool PathPlanner::isValidPy(std::vector<double> &state) {
    bool valid = static_cast<MotionValidator &>(*motionValidator_).isValid(state);
    return valid;    
}

bool PathPlanner::isValid(const ompl::base::State *state) {    
    std::vector<double> state_vec;
    for (unsigned int i = 0; i < si_->getStateSpace()->getDimension(); i++) {
        state_vec.push_back(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);        
    }
    return static_cast<MotionValidator &>(*motionValidator_).isValid(state_vec);
}

void PathPlanner::clear() {
    planner_->clear();
    if (problem_definition_->hasSolution()) {
        problem_definition_->clearSolutionPaths();
    }    
    
    if (problem_definition_->getStartStateCount() > 0) {        
        problem_definition_->clearStartStates();
    }
}

void PathPlanner::clearAll() {
    clear();    
    if (problem_definition_->getGoal()) {
        problem_definition_->clearGoal();
    }
    
}

/** Generates a linear path from point p1 to point p2 */
std::vector<std::vector<double> > PathPlanner::genLinearPath(std::vector<double> &p1, std::vector<double> &p2) {	
    std::vector<double> vec;
    std::vector<double> vec_res;
    std::vector<std::vector<double> > solution_vector;
    for (size_t i = 0; i < p1.size(); i++) {
        vec.push_back(p2[i] - p1[i]);
        vec_res.push_back(p1[i]);
    }
    
    double length(0.0);
    for (size_t j = 0; j < vec.size(); j++) {
        length += std::pow(vec[j], 2);
    }  
    length = std::sqrt(length);
    int num_points = (int) (length / planning_range_);
    std::vector<double> vec_norm;        
    for (size_t j = 0; j < vec.size(); j++) {
        vec_norm.push_back(vec[j] / length);
    }

    solution_vector.push_back(p1);
    for (int j = 0; j < num_points; j++) {            
        for (size_t k = 0; k < vec_res.size(); k++) {
             vec_res[k] += planning_range_ * vec_norm[k];
        } 
        std::vector<double> vec_append(vec_res);
        solution_vector.push_back(vec_append);
    }

    solution_vector.push_back(p2);    
    return solution_vector;
}

std::vector<double> PathPlanner::sampleGoalVec() {    
    ManipulatorGoalRegion gr(si_, robot_, goal_states_, ee_goal_position_, ee_goal_threshold_, false);
    return gr.sampleGoalVec();
}

void PathPlanner::setGoalStates(std::vector<std::vector<double>> &goal_states,
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

bool PathPlanner::solve_(double time_limit) {
    bool solved = false;
    bool hasExactSolution = false;    
    
    solved = planner_->solve(time_limit);
        
    // Get all the solutions
    std::vector<ompl::base::PlannerSolution> solutions = problem_definition_->getSolutions();
    for (size_t i = 0; i < solutions.size(); i++) {
        if (!solutions[i].approximate_) {
            return true;
            
        }
    }
        // Check if there's an exact solution
       
    return false;
}

/** Solves the motion planning problem */
std::vector<std::vector<double> > PathPlanner::solve(const std::vector<double> &start_state_vec, double timeout) {
	
    std::vector<double> ss_vec;
    ompl::base::ScopedState<> start_state(space_); 
    for (size_t i =0; i < dim_; i++) {
        ss_vec.push_back(start_state_vec[i]);
        start_state[i] = ss_vec[i]; 
    }
    
    std::vector<std::vector<double> > solution_vector;
    boost::shared_ptr<MotionValidator> mv = boost::static_pointer_cast<MotionValidator>(si_->getMotionValidator());    
       
    /** Add the start state to the problem definition */
    if (verbose_) {
        cout << "Adding start state: ";
        for (size_t i = 0; i < dim_; i++) {
            cout << start_state[i] << ", ";
        }
        cout << endl;        
    }
    
    /**if (!isValid(start_state.get())) {        
        cout << "Path planner: ERROR: Start state not valid!" << endl;        
        return solution_vector;    
    }*/
    
    problem_definition_->addStartState(start_state);    
    ompl::base::GoalPtr gp(new ManipulatorGoalRegion(si_, robot_, goal_states_, ee_goal_position_, ee_goal_threshold_, false));
    boost::dynamic_pointer_cast<ompl::base::GoalRegion>(gp)->setThreshold(ee_goal_threshold_); 
    problem_definition_->setGoal(gp);
   
    if (check_linear_path_) {    	
        bool collides = false;       
        std::vector<double> goal_state_vec = boost::dynamic_pointer_cast<ManipulatorGoalRegion>(gp)->sampleGoalVec();    
        std::vector<std::vector<double> > linear_path(genLinearPath(ss_vec, goal_state_vec));
    
        for (size_t i = 1; i < linear_path.size(); i++) {       
           if (!(mv->checkMotion(linear_path[i - 1], linear_path[i], continuous_collision_))) {            
                collides = true;
                break;
            }            
        }
    
        if (!collides) {            
            clear();        
            if (verbose_) {
                cout << "Linear path is a valid solution. Returning linear path of length " << linear_path.size() << endl;
            } 
            
            return augmentPath_(linear_path);
        } 
    
    }
    
    /** Solve the planning problem with a maximum of *timeout* seconds per attempt */    
    bool solved = false;
    boost::timer t;    
    bool approximate_solution = true;
    
    /**while (!solved || approximate_solution) {
        solved = planner_->solve(4.0);        
        //solved = planner_->solve(10.0);
        if (t.elapsed() > timeout) {
        	cout << "Timeout reached." << endl;
        	return solution_vector;
        }
        for (auto &solution : problem_definition_->getSolutions()) {
        	
        	cout << "diff " << solution.difference_ << endl;
        	approximate_solution = false;
        	
        }
    }**/
    solved = solve_(timeout);
    if (!solved) {
    	return solution_vector;
    }
    
    boost::shared_ptr<ompl::geometric::PathGeometric> solution_path = boost::static_pointer_cast<ompl::geometric::PathGeometric>(problem_definition_->getSolutionPath());
    
    solution_vector.push_back(ss_vec);    
    /** We found a solution, so get the solution path */
    
    if (verbose_) {
        cout << "Solution found in " << t.elapsed() << " seconds." << endl;
        cout << "Solution path has length " << solution_path->getStates().size() << endl;
    } 
    
    std::vector<double> vals;
    const bool cont_check = true;
    for (size_t i=1; i<solution_path->getStates().size(); i++) {
       vals.clear();       
       for (unsigned int j = 0; j < dim_; j++) {          
          vals.push_back(solution_path->getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values[j]);
       }
       
       solution_vector.push_back(vals);
    }   
    clear();       
    return augmentPath_(solution_vector);
} 

std::vector<std::vector<double>> PathPlanner::augmentPath_(std::vector<std::vector<double>> &solution_path) {	
	std::vector<std::vector<double>> augmented_path;
	for (size_t i = 0; i < solution_path.size(); i++) {
		std::vector<double> path_element;
		std::vector<double> solution_element;
		std::vector<double> next_solution_element;
		std::vector<double> control;
		std::vector<double> observation;
		for (size_t j = 0; j < dim_; j++) {			
			solution_element.push_back(solution_path[i][j]);			
			if (i != solution_path.size() - 1) {
				next_solution_element.push_back(solution_path[i + 1][j]);
				control.push_back((next_solution_element[j] - solution_element[j]) / delta_t_);
			}
			else {
				control.push_back(0.0);
			}
			observation.push_back(solution_element[j]);
			
		}
		/**for (size_t j = 0; j < dim_; j++) {
			control.push_back(0.0);
		}*/
		
		for (size_t j = 0; j < dim_; j++) {
			solution_element.push_back(0.0);
			observation.push_back(0.0);
		}		
		
		for (size_t j = 0; j < solution_element.size(); j++) {
			path_element.push_back(solution_element[j]);			
		}
		
		for (size_t j = 0; j < control.size(); j++) {
			path_element.push_back(control[j]);
		}
		
		for (size_t j = 0; j < observation.size(); j++) {
			path_element.push_back(observation[j]);
		}
		
		if (i != solution_path.size() - 1) {
			path_element.push_back(delta_t_);
		}
		else {
			path_element.push_back(0.0);
		}
		
		augmented_path.push_back(path_element);
	}	
	return augmented_path;
}

BOOST_PYTHON_MODULE(libpath_planner) {
    using namespace boost::python;  
    
    
    
    class_<PathPlanner>("PathPlanner", init<boost::shared_ptr<shared::Robot>&,    		                                                                     
                                            double,
                                            bool,                                            
                                            double,
                                            double,
                                            bool,                                            
                                            bool,											
                                            std::string>())
                        .def("solve", &PathPlanner::solve)
                        .def("setObstacles", &PathPlanner::setObstaclesPy) 
                        .def("setGoalStates", &PathPlanner::setGoalStates)
                        .def("isValid", &PathPlanner::isValidPy) 
                        .def("setup", &PathPlanner::setup)						
    ;
}

}

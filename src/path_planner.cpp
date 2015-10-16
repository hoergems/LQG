#include "path_planner.hpp"

using std::cout;
using std::endl;

namespace shared {

PathPlanner::PathPlanner(std::shared_ptr<Kinematics> kinematics,
                         int dim,
                         double delta_t,
                         bool continuous_collision,
                         double max_joint_velocity,
                         std::vector<double> joint_constraints,
                         bool enforce_constraints,
                         double stretching_factor, 
                         bool use_rrt_heuristic,
                         bool check_linear_path,                         
                         bool verbose,
                         std::string planner):
    dim_(dim),
    delta_t_(delta_t),
    continuous_collision_(continuous_collision),
    max_joint_velocity_(max_joint_velocity),
    joint_constraints_(joint_constraints),
    enforce_constraints_(enforce_constraints),
    stretching_factor_(stretching_factor),
    planning_range_((1.0 / stretching_factor_) * std::sqrt(dim * std::pow(delta_t_ * max_joint_velocity_, 2))),  
    use_rrt_heuristic_(use_rrt_heuristic),
    check_linear_path_(check_linear_path),  
    space_(new ompl::base::RealVectorStateSpace(dim_)),    
    si_(new ompl::base::SpaceInformation(space_)),    
    problem_definition_(new ompl::base::ProblemDefinition(si_)),
    planner_str_(planner),    
    planner_(nullptr),
    obstacles_(),    
    kinematics_(kinematics), 
    motionValidator_(new MotionValidator(si_, 
                                         kinematics_, 
                                         obstacles_,                                         
                                         max_joint_velocity_, 
                                         delta_t_,
                                         continuous_collision)),
    verbose_(verbose)
{   
    if (!verbose_) {        
        ompl::msg::noOutputHandler();
    }
    
    if (planner_str_ == "RRTConnect") {
    	planner_ = boost::shared_ptr<ompl::geometric::RRTConnect>(new ompl::geometric::RRTConnect(si_));
    }
    else {
        planner_ = boost::shared_ptr<ompl::geometric::RRT>(new ompl::geometric::RRT(si_));
    }
}

void PathPlanner::setup() {
	ompl::base::RealVectorBounds bounds(dim_);
	if (enforce_constraints_) {
		/** We assume that the there are no joint limits. So the range of each joint
		*** is between -2*Pi and 2*Pi
		*/   
		for (size_t i = 0; i < dim_; i++) {			
			bounds.setLow(i, -joint_constraints_[i]);
			bounds.setHigh(i, joint_constraints_[i]);
		}
		
		/** Apply the bounds to the space */    
		space_->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
	}
	else {
		for (size_t i = 0; i < dim_; i++) {			
			bounds.setLow(i, -M_PI + 0.0000001);
			bounds.setHigh(i, M_PI + 0.0000001);
		}
				
		/** Apply the bounds to the space */    
		space_->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
	}
    
    /** Set the StateValidityChecker */    
    si_->setStateValidityChecker(boost::bind(&PathPlanner::isValid, this, _1)); 
    
    /** Set the MotionValidation */
    si_->setMotionValidator(motionValidator_);
    
    /** Let the planner know about our problem */    
    planner_->setProblemDefinition(problem_definition_);
    
    /** Use RRTConnect as the planning algorithm */
    if (planner_str_ == "RRTConnect") {   
    	boost::shared_ptr<ompl::geometric::RRTConnect> planner_ptr = boost::static_pointer_cast<ompl::geometric::RRTConnect>(planner_);
    	planner_ptr->setRange(planning_range_);
    }
    else {
        boost::shared_ptr<ompl::geometric::RRT> planner_ptr = boost::static_pointer_cast<ompl::geometric::RRT>(planner_);
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
        obstacles_.push_back(std::make_shared<Obstacle>(boost::python::extract<Obstacle>(ns[i])));
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
    std::vector<double> angles1;
    for (unsigned int i = 0; i < si_->getStateSpace()->getDimension(); i++) {
        angles1.push_back(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);        
    }
    return static_cast<MotionValidator &>(*motionValidator_).isValid(angles1);
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
    ManipulatorGoalRegion gr(si_, goal_states_);
    return gr.sampleGoalVec();
}

void PathPlanner::setGoalStates(std::vector<std::vector<double>> &goal_states) {
    for (size_t i = 0; i < goal_states.size(); i++) {
        goal_states_.push_back(goal_states[i]);
    }
}

void PathPlanner::setLinkDimensions(std::vector<std::vector<double>> &link_dimensions) {
    boost::shared_ptr<MotionValidator> mv = boost::static_pointer_cast<MotionValidator>(si_->getMotionValidator());
    mv->setLinkDimensions(link_dimensions);
}

/** Solves the motion planning problem */
std::vector<std::vector<double> > PathPlanner::solve(const std::vector<double> &start_state_vec) {
    std::vector<double> ss_vec;
    for (size_t i =0; i < start_state_vec.size(); i++) {
        ss_vec.push_back(start_state_vec[i]);
    }
    
    ompl::base::ScopedState<> start_state(space_);    
    std::vector<std::vector<double> > solution_vector;
    solution_vector.push_back(ss_vec);
    
    boost::shared_ptr<MotionValidator> mv = boost::static_pointer_cast<MotionValidator>(si_->getMotionValidator());
    for (int i=0; i<dim_; i++) {
        start_state[i] = ss_vec[i];       
    }
    
    /** Add the start state to the problem definition */
    if (verbose_) {
        cout << "Adding start state: ";
        for (size_t i = 0; i < start_state_vec.size(); i++) {
            cout << start_state[i] << ", ";
        }
        cout << endl;        
    }
    
    if (!isValid(start_state.get())) {
        if (verbose_) {
            cout << "Path planner: Start state not valid" << endl;
        }
        return solution_vector;    
    }
    
    problem_definition_->addStartState(start_state);    
    ompl::base::GoalPtr gp(new ManipulatorGoalRegion(si_, goal_states_));
    
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
    
        if (!collides && !use_rrt_heuristic_) {            
            clear();        
            if (verbose_) {
                cout << "Linear path is a valid solution. Returning linear path of length " << linear_path.size() << endl;
            }        
            return linear_path;
        } 
    
    }
    
    /** Solve the planning problem with a maximum of 10 seconds per attempt */    
    bool solved = false;
    boost::timer t;    
    
    while (!solved) {
        solved = planner_->solve(10);        
        //solved = planner_->solve(10.0);  
    }
    
    /** We found a solution, so get the solution path */
    boost::shared_ptr<ompl::geometric::PathGeometric> solution_path = boost::static_pointer_cast<ompl::geometric::PathGeometric>(problem_definition_->getSolutionPath());    
    if (verbose_) {
        cout << "Solution found in " << t.elapsed() << " seconds." << endl;
        cout << "Solution path has length " << solution_path->getStates().size() << endl;
    } 
    
    std::vector<double> vals;
    std::vector<double> old_vals;
    std::vector<std::vector<double> > temp_vals;    
    const bool cont_check = true;
    for (size_t i=1; i<solution_path->getStates().size(); i++) {
       vals.clear();
       old_vals.clear();
       for (int j=0; j<dim_; j++) {
          old_vals.push_back(solution_path->getState(i-1)->as<ompl::base::RealVectorStateSpace::StateType>()->values[j]);
          vals.push_back(solution_path->getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values[j]); 
       }  
       if (!mv->checkMotion(old_vals, vals, cont_check)) {
    	   cout << "whaaaaaaaaaaaaaaat" << endl;
    	   sleep(20);
       }       
       solution_vector.push_back(vals);  
       
    }   
    clear();       
    return solution_vector;
} 

BOOST_PYTHON_MODULE(libpath_planner) {
    using namespace boost::python;   
    
    class_<PathPlanner>("PathPlanner", init<std::shared_ptr<Kinematics>, 
                                            int,                                             
                                            double,
                                            bool, 
                                            double,
                                            std::vector<double>,
                                            bool,
                                            double,
                                            bool,
                                            bool,
                                            bool,
                                            std::string>())
                        .def("solve", &PathPlanner::solve)
                        .def("setObstacles", &PathPlanner::setObstaclesPy) 
                        .def("setGoalStates", &PathPlanner::setGoalStates)
                        .def("isValid", &PathPlanner::isValidPy) 
                        .def("setup", &PathPlanner::setup) 
                        .def("setLinkDimensions", &PathPlanner::setLinkDimensions)          
    ;
}

}

#ifndef PATH_PLANNER_HPP_
#define PATH_PLANNER_HPP_
#include <boost/python.hpp>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/Goal.h>
#include "ManipulatorGoalRegion.hpp"
#include <boost/make_shared.hpp>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/base/Path.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/Planner.h>
#include <ompl/util/Console.h>
#include <ompl/base/MotionValidator.h>
#include "MotionValidator.hpp"
#include "Obstacle.hpp"

#include <boost/timer.hpp>

#include <iostream>


namespace shared {
    
    class PathPlanner {
        public:
            
            
            PathPlanner(std::shared_ptr<Kinematics> kinematics,
                        int dim,             
                        double delta_t,
                        bool continuous_collision,
                        double max_joint_velocity,
                        double stretching_factor,                                                
                        bool use_rrt_heuristic,
                        bool check_linear_path,                        
                        bool verbose,
                        std::string planner);
                        
            ~PathPlanner() {clearAll();}
            
            /** Checks of a state is valid */
            bool isValid(const ompl::base::State *state);
            
            bool isValidPy(std::vector<double> &state);
            
            /** Solve the deterministic motion planning problem */
            std::vector<std::vector<double> > solve(const std::vector<double> &start_state_vec);

            /** Setup OMPL. This need to be called before solving the motion planning problem*/
            void setup();

            /** Place the obstacles in the environment */
            void setObstacles(const std::vector<std::shared_ptr<Obstacle> > obstacles);

            void setObstaclesPy(boost::python::list &ns);
            
            std::vector<double> sampleGoalVec();
            
            ompl::base::MotionValidatorPtr getMotionValidator();

            boost::shared_ptr<ManipulatorGoalRegion> getOrCreateGoalRegion();
 
            void setGoalStates(std::vector<std::vector<double>> &goal_states);

            void setLinkDimensions(std::vector<std::vector<double>> &link_dimensions);
            
        private:        
            /** The dimension of the space we're planning in */
            int dim_;

            /** The maximum allowed euclidean distance between two connected nodes */
            double delta_t_;
            
            bool continuous_collision_;
            
            double max_joint_velocity_;
            
            double stretching_factor_;
            
            double planning_range_;
            
            bool use_rrt_heuristic_;
            
            bool check_linear_path_;

            /** The space we're planning in */
            ompl::base::StateSpacePtr space_;

            /** A SpaceInformation pointer */
            ompl::base::SpaceInformationPtr si_;

            /** The definition of the path planning problem */
            ompl::base::ProblemDefinitionPtr problem_definition_;
            
            std::string planner_str_;

            /** A pointer to the path planner */
            ompl::base::PlannerPtr planner_;

            /** A vector of obstacles in the environment */
            std::vector<std::shared_ptr<Obstacle> > obstacles_;
            
            /** A simplifier which can be used to simplify (shorten an smoothen) paths */
            //ompl::geometric::PathSimplifier simplifier_;

            std::shared_ptr<Kinematics> kinematics_;
            
            ompl::base::MotionValidatorPtr motionValidator_;

            bool verbose_;            

            std::vector<std::vector<double>> goal_states_;

            /** Clean memory from unused states */
            void clear();
            
            void clearAll();
            
            /** Generates a linear path from point p1 to point p2 */
            std::vector<std::vector<double> > genLinearPath(std::vector<double> &p1, std::vector<double> &p2);
    };
}

#endif




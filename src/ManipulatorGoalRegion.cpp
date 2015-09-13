#include "ManipulatorGoalRegion.hpp"

#include <fstream>
#include <string>

using std::cout;
using std::endl;

namespace shared {

    ManipulatorGoalRegion::ManipulatorGoalRegion(const ompl::base::SpaceInformationPtr &si,                                  
                                                 std::vector<std::vector<double>> &goal_states):
        ompl::base::GoalSampleableRegion(si),
        state_dimension_(si->getStateDimension()),        
        goal_states_(goal_states)          
    {
        
    }

    double ManipulatorGoalRegion::distanceGoal(const ompl::base::State *st) const
    {        
        double min_dist = 10000.0;
        std::vector<double> v1;
        double* v = st->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        for (unsigned int i = 0; i < state_dimension_; i++) {
           v1.push_back(v[i]);
        }
        for (size_t i = 0; i < goal_states_.size(); i++) {
           std::vector<double> goalState = goal_states_[i];
           double d = utils::euclideanDistance(v1, goalState);
           if (d < min_dist) {
               min_dist = d;
           }
        }
        
        return min_dist;        
    } 

    void ManipulatorGoalRegion::sampleGoal(ompl::base::State *st) const 
    {    
        ompl::RNG rng; 
       
        int rd = rng.uniformInt(0, goal_states_.size() - 1);   
        double* v = st->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        for (unsigned int i = 0; i < state_dimension_; i++) {
           v[i] = goal_states_[rd][i];
        }           
    }
    
    std::vector<double> ManipulatorGoalRegion::sampleGoalVec() const {               
        ompl::RNG rng;        
        std::vector<double> v;         
        int rd = rng.uniformInt(0, goal_states_.size() - 1);
        //int rd = 0;
        if (goal_states_.size() == 0) {cout << "wtf"<<endl;}
        for (unsigned int i = 0; i < state_dimension_; i++) {
           v.push_back(goal_states_[rd][i]);
        }
        
        return v;
    }

    unsigned int ManipulatorGoalRegion::maxSampleCount() const
    {
        return goal_states_.size();
    }
}


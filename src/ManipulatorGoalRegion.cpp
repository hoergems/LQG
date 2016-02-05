#include "ManipulatorGoalRegion.hpp"

#include <fstream>
#include <string>

using std::cout;
using std::endl;

namespace shared {

    ManipulatorGoalRegion::ManipulatorGoalRegion(const ompl::base::SpaceInformationPtr &si,
    		                                     boost::shared_ptr<shared::Robot> robot,
                                                 std::vector<std::vector<double>> &goal_states,
                                                 std::vector<double> &ee_goal_position,
                                                 double &ee_goal_threshold,                                                 
                                                 bool dynamics):
        ompl::base::GoalSampleableRegion(si),
        robot_(robot),        
        state_space_information_(si),
        state_dimension_(si->getStateDimension()),        
        goal_states_(goal_states),
        ee_goal_position_(ee_goal_position),
        ee_goal_threshold_(ee_goal_threshold)        
    {
        if (dynamics) {
        	state_dimension_ = si->getStateDimension() / 2;
        }
    }

    double ManipulatorGoalRegion::distanceGoal(const ompl::base::State *st) const
    {	
        std::vector<double> v1;
        double* v = st->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        for (unsigned int i = 0; i < state_dimension_; i++) {
           v1.push_back(v[i]);          
        }
        
        std::vector<double> ee_position;        
        robot_->getEndEffectorPosition(v1, ee_position);
        
        std::vector<double> ee_g;
        for (size_t i = 0; i < ee_goal_position_.size(); i++) {
        	ee_g.push_back(ee_goal_position_[i]);
        }
        
        double distance = utils::euclideanDistance(ee_position, ee_g);        
        return distance;             
    } 
    
    double ManipulatorGoalRegion::getThreshold() const {
    	cout << "GETTING THRESHOLD" << endl;
    	sleep(10);
    	return ee_goal_threshold_;
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
    
    bool ManipulatorGoalRegion::isSatisfied(const ompl::base::State *st) const {
    	cout << "IS SATISFIED" << endl;
    	std::vector<double> joint_angles;    	
    	for (unsigned int i = 0; i < state_dimension_; i++) {
    		joint_angles.push_back(st->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);
    	}
    	std::vector<double> ee_position;
    	robot_->getEndEffectorPosition(joint_angles, ee_position);
    	double sum(0.0);
    	for (unsigned int i = 0; i < joint_angles.size(); i++) {
    		sum += pow(ee_position[i] - ee_goal_position_[i], 2);
    	}
    	
    	sum = sqrt(sum);    	
    	if (sum < ee_goal_threshold_ - 0.01) {
    		return true;
    	}
    	
    	return false;
    	
    }
}


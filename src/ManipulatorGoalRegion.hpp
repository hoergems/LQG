#ifndef MANIPULATOR_GOAL_REGION_HPP_
#define MANIPULATOR_GOAL_REGION_HPP_

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <ompl/util/RandomNumbers.h>
#include "utils.hpp"
#include <iostream>
#include <random>

namespace shared {    

    class ManipulatorGoalRegion: public ompl::base::GoalSampleableRegion {
        public:
            ManipulatorGoalRegion(const ompl::base::SpaceInformationPtr &si,                                  
                                  std::vector<std::vector<double>> &goal_states,
                                  std::vector<double> &ee_goal_position,
                                  double &ee_goal_threshold,
                                  std::shared_ptr<Kinematics> kinematics);
                                  
            //~ManipulatorGoalRegion() = default;

            double distanceGoal(const ompl::base::State *st) const;

            void sampleGoal(ompl::base::State *st) const;
            
            std::vector<double> sampleGoalVec() const;

            unsigned int maxSampleCount() const;
            
            virtual bool isSatisfied(const ompl::base::State *st) const;

        private:
            /** Forward kinematics */
            std::shared_ptr<Kinematics> kinematics_;
            
            ompl::base::SpaceInformationPtr state_space_information_;
            
            unsigned int state_dimension_;
            
            std::vector<std::vector<double>> goal_states_;
            
            std::vector<double> ee_goal_position_;
            
            double ee_goal_threshold_;

    };

}

#endif

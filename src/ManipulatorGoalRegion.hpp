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
                                  std::vector<std::vector<double>> &goal_states);
                                  
            //~ManipulatorGoalRegion() = default;

            double distanceGoal(const ompl::base::State *st) const;

            void sampleGoal(ompl::base::State *st) const;
            
            std::vector<double> sampleGoalVec() const;

            unsigned int maxSampleCount() const;

        private:
            unsigned int state_dimension_;
            
            std::vector<std::vector<double>> goal_states_;

    };

}

#endif

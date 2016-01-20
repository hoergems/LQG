#ifndef RRT_CONTROL_HPP_
#define RRT_CONTROL_HPP_
#include "ompl/control/planners/PlannerIncludes.h"
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/control/Control.h>
#include <ompl/control/PathControl.h>
#include "ManipulatorSpaceInformation.hpp"
#include "MotionValidator.hpp"
#include <iostream>

namespace shared {
    class RRTControl : public ompl::control::RRT {
        public:
		    RRTControl(const ompl::control::SpaceInformationPtr &si);
	    
	        ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc);
	
    };
}

#endif
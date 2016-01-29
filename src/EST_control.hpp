#ifndef _EST_CONTROL_HPP_
#define _EST_CONTROL_HPP_
#include "ompl/control/planners/est/EST.h"
#include "ompl/control/planners/PlannerIncludes.h"
#include <ompl/base/goals/GoalSampleableRegion.h>
#include "ManipulatorSpaceInformation.hpp"
#include "MotionValidator.hpp"

namespace shared {
	class ESTControl : public ompl::control::EST {
	    public:
		    ESTControl(const ompl::control::SpaceInformationPtr &si);
		    
		    ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc);
		    
	    private:
		    bool addIntermediateStates_;
	};
}

#endif
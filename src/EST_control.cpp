#include "EST_control.hpp"

using std::cout;
using std::endl;

namespace shared {

ESTControl::ESTControl(const ompl::control::SpaceInformationPtr &si):
	ompl::control::EST(si),
	addIntermediateStates_(true)
{
	
}

ompl::base::PlannerStatus ESTControl::solve(const ompl::base::PlannerTerminationCondition &ptc)
{
	ManipulatorSpaceInformation const * msiC_ = static_cast<ManipulatorSpaceInformation const *>(siC_);
    MotionValidator const * motion_validator_ = static_cast<MotionValidator const *>(siC_->getMotionValidator().get());
	checkValidity();
    ompl::base::Goal                   *goal = pdef_->getGoal().get();
    ompl::base::GoalSampleableRegion *goal_s = dynamic_cast<ompl::base::GoalSampleableRegion*>(goal);

    // Initializing tree with start state(s)
    while (const ompl::base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        addMotion(motion);
    }

    if (tree_.grid.size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return ompl::base::PlannerStatus::INVALID_START;
    }

    // Ensure that we have a state sampler AND a control sampler
    if (!sampler_)
        sampler_ = si_->allocValidStateSampler();
    if (!controlSampler_)
        //controlSampler_ = siC_->allocDirectedControlSampler();
    	controlSampler_ = msiC_->allocDirectedControlSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), tree_.size);

    Motion  *solution = NULL;
    Motion *approxsol = NULL;
    double  approxdif = std::numeric_limits<double>::infinity();
    Motion   *rmotion = new Motion(siC_);
    bool       solved = false;

    while (!ptc)
    {
        // Select a state to expand the tree from
        Motion *existing = selectMotion();
        assert (existing);

        // sample a random state (with goal biasing) near the state selected for expansion
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rmotion->state);
        else
        {
            if (!sampler_->sampleNear(rmotion->state, existing->state, maxDistance_))
                continue;
        }
        
        if (addIntermediateStates_) {

			// Extend a motion toward the state we just sampled
			unsigned int duration = controlSampler_->sampleTo(rmotion->control, existing->control,
															  existing->state, rmotion->state);
			
			std::vector<ompl::base::State *> pstates;            
			duration = msiC_->propagateWhileValid(existing->state, rmotion->control, duration, pstates, true); 
			
			// If the system was propagated for a meaningful amount of time, save into the tree
			if (duration >= siC_->getMinControlDuration())
			{
				Motion *lastmotion = existing;
				bool solved = false;
				size_t p = 0;
				for ( ; p < pstates.size(); ++p) {
					/* create a motion */                	
					Motion *motion = new Motion();
				    motion->state = pstates[p];
				    
				    motion->control = msiC_->allocControl();
				    msiC_->copyControl(motion->control, rmotion->control);
				    motion->steps = 1;
				    motion->parent = lastmotion;
				    lastmotion = motion;                    
				    addMotion(motion);
				    
				    double dist = 0.0;
				    solved = goal->isSatisfied(motion->state, &dist);
				    if (solved)
				    {
				        approxdif = dist;
				        solution = motion;
				        break;
				    }
				    
				    if (dist < approxdif)
				    {
				        approxdif = dist;
				        approxsol = motion;
				    }
				}
				
				 while (++p < pstates.size()) {
				     si_->freeState(pstates[p]);
				 }
				 if (solved)
				     break;
				/**Motion *motion = new Motion(siC_);
				//si_->copyState(motion->state, rmotion->state);
				si_->copyState(motion->state, result_states[0]);
				siC_->copyControl(motion->control, rmotion->control);
				motion->steps = duration;
				motion->parent = existing;			   
				
				// save the state
				addMotion(motion);
		
				// Check if this state is the goal state, or improves the best solution so far
				double dist = 0.0;
				solved = goal->isSatisfied(motion->state, &dist);
				if (solved)
				{
					approxdif = dist;
					solution = motion;
					break;
				}
				if (dist < approxdif)
				{
					approxdif = dist;
					approxsol = motion;
				}*/
				
			}
        }
    }

    bool approximate = false;
    if (solution == NULL)
    {
        solution = approxsol;
        approximate = true;
    }

    // Constructing the solution path
    if (solution != NULL)
    {
        lastGoalMotion_ = solution;

        std::vector<Motion*> mpath;
        while (solution != NULL)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        ompl::control::PathControl *path = new ompl::control::PathControl(si_);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i)
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
        solved = true;
        pdef_->addSolutionPath(ompl::base::PathPtr(path), approximate, approxdif, getName());
    }

    // Cleaning up memory
    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states in %u cells", getName().c_str(), tree_.size, tree_.grid.size());

    return ompl::base::PlannerStatus(solved, approximate);
}

}
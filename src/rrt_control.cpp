#include "rrt_control.hpp"

using std::cout;
using std::endl;

namespace shared {

RRTControl::RRTControl(const ompl::control::SpaceInformationPtr &si):
	ompl::control::RRT(si)
{
	
}

ompl::base::PlannerStatus RRTControl::solve(const ompl::base::PlannerTerminationCondition &ptc)
{    
	ManipulatorSpaceInformation const * msiC_ = static_cast<ManipulatorSpaceInformation const *>(siC_);
	MotionValidator const * motion_validator_ = static_cast<MotionValidator const *>(siC_->getMotionValidator().get());
	checkValidity();
    ompl::base::Goal                   *goal = pdef_->getGoal().get();
    ompl::base::GoalSampleableRegion *goal_s = dynamic_cast<ompl::base::GoalSampleableRegion*>(goal);

    while (const ompl::base::State *st = pis_.nextStart())
    {
        Motion *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        msiC_->nullControl(motion->control);
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return ompl::base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_) {
    	controlSampler_ = msiC_->allocDirectedControlSampler();
        //controlSampler_ = siC_->allocDirectedControlSampler();
    }

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution  = NULL;
    Motion *approxsol = NULL;
    double  approxdif = std::numeric_limits<double>::infinity();

    Motion      *rmotion = new Motion(msiC_);
    ompl::base::State  *rstate = rmotion->state;
    ompl::control::Control       *rctrl = rmotion->control;
    ompl::base::State  *xstate = si_->allocState();
    bool valid = false;

    while (ptc == false)
    {
        /* sample random state (with goal biasing) */    	
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
        	valid = false;
        	while (!valid) {
                sampler_->sampleUniform(rstate);
                valid = motion_validator_->isValid(rstate);
        	}

        /* find closest state in the tree */        
        Motion *nmotion = nn_->nearest(rmotion);

        /* sample a random control that attempts to go towards the random state, and also sample a control duration */        
        unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);        
        if (addIntermediateStates_)
        {            
        	// this code is contributed by Jennifer Barry
            std::vector<ompl::base::State *> pstates;            
            cd = msiC_->propagateWhileValid(nmotion->state, rctrl, cd, pstates, true);            
            
            if (cd >= msiC_->getMinControlDuration())
            {
                Motion *lastmotion = nmotion;
                bool solved = false;
                size_t p = 0;
                for ( ; p < pstates.size(); ++p)
                {
                    /* create a motion */                	
                    Motion *motion = new Motion();
                    motion->state = pstates[p];
                    //we need multiple copies of rctrl
                    motion->control = msiC_->allocControl();
                    msiC_->copyControl(motion->control, rctrl);
                    motion->steps = 1;
                    motion->parent = lastmotion;
                    lastmotion = motion;                    
                    nn_->add(motion);
                    
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

                //free any states after we hit the goal
                
                while (++p < pstates.size())
                    si_->freeState(pstates[p]);
                if (solved)
                    break;
            }
            else {            	
                for (size_t p = 0 ; p < pstates.size(); ++p) {
                    si_->freeState(pstates[p]);
                }
            }            
        }
        else
        {
            //cout << "CHECK MOTION" << endl;
        	if (msiC_->checkMotion(nmotion->state, rmotion->state)) {        	
				if (cd >= msiC_->getMinControlDuration())
				{
					/* create a motion */
					Motion *motion = new Motion(msiC_);
					si_->copyState(motion->state, rmotion->state);
					msiC_->copyControl(motion->control, rctrl);
					motion->steps = cd;
					motion->parent = nmotion;
	
					nn_->add(motion);
					double dist = 0.0;
					bool solv = goal->isSatisfied(motion->state, &dist);
					if (solv)
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
        	}
        }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == NULL)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != NULL)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion*> mpath;
        while (solution != NULL)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        ompl::control::PathControl *path = new ompl::control::PathControl(si_);
        for (int i = mpath.size() - 1 ; i >= 0 ; --i)
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
        solved = true;
        pdef_->addSolutionPath(ompl::base::PathPtr(path), approximate, approxdif, getName());
    }

    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        msiC_->freeControl(rmotion->control);
    delete rmotion;
    si_->freeState(xstate);

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return ompl::base::PlannerStatus(solved, approximate);
}

}
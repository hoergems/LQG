#include "MotionValidator.hpp"

using std::cout;
using std::endl;

using namespace fcl;

namespace shared {

MotionValidator::MotionValidator(const ompl::base::SpaceInformationPtr &si,
		                         boost::shared_ptr<shared::Robot> &robot,
                                 bool continuous_collision,
                                 bool dynamics):    
    ompl::base::MotionValidator(si),
    si_(si), 
    robot_(robot),
    continuous_collision_(continuous_collision),
    obstacles_(),
    dim_(si_->getStateSpace()->getDimension())
{	
    if (dynamics) {
    	dim_ = si_->getStateSpace()->getDimension() / 2;
    }
    
}

bool MotionValidator::checkMotion(const std::vector<double> &s1, 
                                  const std::vector<double> &s2, 
                                  const bool &continuous_collision) const {		
	std::vector<std::shared_ptr<fcl::CollisionObject>> collision_objects_goal;
	robot_->createRobotCollisionObjects(s2, collision_objects_goal);
    if (continuous_collision) {    	
    	return !collidesContinuous(s1, s2); 
    } 
    else {    	
        return !collidesDiscrete(s2);
    }
}

/** Check if a motion between two states is valid. This assumes that state s1 is valid */
bool MotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {	
    std::vector<double> angles1;
    std::vector<double> angles2; 
    
    for (unsigned int i = 0; i < dim_; i++) {
        angles1.push_back(s1->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);        
        angles2.push_back(s2->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);        
    }
    if (!satisfiesConstraints(angles2)) {
    	return false;
    }
    
    return checkMotion(angles1, angles2, continuous_collision_);
}

/** Check if a motion between two states is valid. This assumes that state s1 is valid */
bool MotionValidator::checkMotion(const ompl::base::State *s1, 
                                  const ompl::base::State *s2, 
                                  std::pair< ompl::base::State *, double > &/*lastValid*/) const {	
    return checkMotion(s1, s2);
}

bool MotionValidator::satisfiesConstraints(const std::vector<double> &s1) const {
	std::vector<double> joint_angles;
	for (size_t i = 0; i < dim_; i++) {
		joint_angles.push_back(s1[i]);
	}
	std::vector<double> lower_bounds = si_->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->getBounds().low;
	std::vector<double> upper_bounds = si_->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->getBounds().high;
	for (size_t i = 0; i < dim_; i++) {
		if (s1[i] < lower_bounds[i]) {			
			return false;			
		}
		else if (s1[i] > upper_bounds[i]) {			
			return false;			
		}
	}
	
	return true;
}

bool MotionValidator::isValid(const ompl::base::State *state) const{
	std::vector<double> angles;	    
	for (unsigned int i = 0; i < dim_; i++) {
	    angles.push_back(state->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);
	}
    
	return isValid(angles);
}

bool MotionValidator::isValid(const std::vector<double> &s1) const {	
	std::vector<double> joint_angles;
	for (size_t i = 0; i < dim_; i++) {
		joint_angles.push_back(s1[i]);
	}
	
	if (!satisfiesConstraints(joint_angles) || collidesDiscrete(joint_angles)) {		
		return false;
	}
	
	
	return true;
}

bool MotionValidator::collidesDiscrete(const std::vector<double> &state) const{
	std::vector<double> joint_angles;
	for (size_t i = 0; i < dim_; i++) {
		joint_angles.push_back(state[i]);
	}
	
	std::vector<std::shared_ptr<fcl::CollisionObject>> collision_objects;
	robot_->createRobotCollisionObjects(joint_angles, collision_objects);    
	for (size_t i = 0; i < obstacles_.size(); i++) {
	    if (!obstacles_[i]->getTerrain()->isTraversable()) {        	
	        if (obstacles_[i]->in_collision(collision_objects)) {        		
	        	return true;
	        }
	    }
	}    
	return false; 
	
}

bool MotionValidator::collidesContinuous(const std::vector<double> &state1,
            		                     const std::vector<double> &state2) const {
	std::vector<std::shared_ptr<fcl::CollisionObject>> collision_objects_start;    	
	robot_->createRobotCollisionObjects(state1, collision_objects_start);
	std::vector<std::shared_ptr<fcl::CollisionObject>> collision_objects_goal;
    robot_->createRobotCollisionObjects(state2, collision_objects_goal);
	for (size_t i = 0; i < obstacles_.size(); i++) {
	    if (!obstacles_[i]->isTraversable()) {
	        for (size_t j = 0; j < collision_objects_start.size(); j++) {                	
	            if (obstacles_[i]->in_collision(collision_objects_start[j], collision_objects_goal[j])) {                		
	                return true;
	            }
	        }
	    }
	}
	
	return false;
}

void MotionValidator::setObstacles(std::vector<std::shared_ptr<Obstacle> > &obstacles) {
    obstacles_.clear();
    for (size_t i = 0; i < obstacles.size(); i++) {       
        obstacles_.push_back(obstacles[i]);
    }    
}

void MotionValidator::setContinuousCollisionCheck(bool continuous_collision_check) {
	continuous_collision_ = continuous_collision_check;
}

}

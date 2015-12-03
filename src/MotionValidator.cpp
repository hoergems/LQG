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
	/**for (size_t i = 0; i < s1.size(); i++) {		
		if ((fabs((s2[i] - s1[i]) / delta_t_)) > max_joint_velocity_ + 0.00001) {
			return false;
		}		
	}*/
	/**for (size_t i = 0; i < s1.size(); i++) {		
	    if ((fabs((s2[i] - s1[i]) / 0.033333)) > 4.0 + 0.00001) {	    	
			return false;
	    }		
	}*/
	std::vector<OBB> manipulator_collision_structures_goal;
	robot_->createRobotCollisionStructures(s2, manipulator_collision_structures_goal);
	/**std::vector<OBB> manipulator_collision_structures_goal = utils_.createManipulatorCollisionStructures(s2,
                                                                                                         link_dimensions_, 
                                                                                                         kinematics_);*/
    for (size_t i = 0; i < obstacles_.size(); i++) {        
        if (!obstacles_[i]->isTraversable()) {
        	if (obstacles_[i]->in_collision(manipulator_collision_structures_goal)) {        		
        		return false;
        	}
        }        
    }
    
    if (continuous_collision) {
    	std::vector<fcl::CollisionObject> manipulator_collision_objects_start;
    	std::vector<fcl::CollisionObject> manipulator_collision_objects_goal;
    	robot_->createRobotCollisionObjects(s1, manipulator_collision_objects_start);
    	robot_->createRobotCollisionObjects(s2, manipulator_collision_objects_goal);
        /**std::vector<fcl::CollisionObject> manipulator_collision_objects_start = utils_.createManipulatorCollisionObjects(s1, 
                                                                                                                         link_dimensions_,
                                                                                                                         kinematics_);
        std::vector<fcl::CollisionObject> manipulator_collision_objects_goal = utils_.createManipulatorCollisionObjects(s2, 
                                                                                                                        link_dimensions_,
                                                                                                                        kinematics_);*/
        for (size_t i = 0; i < obstacles_.size(); i++) {
            if (!obstacles_[i]->isTraversable()) {
                for (size_t j = 0; j < manipulator_collision_objects_start.size(); j++) {                	
                	if (obstacles_[i]->in_collision(manipulator_collision_objects_start[j], manipulator_collision_objects_goal[j])) {                		
                		return false;
                	}
                }
            }
        } 
    }     
    
    return true;
}

/** Check if a motion between two states is valid. This assumes that state s1 is valid */
bool MotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {
	/**if (boost::dynamic_pointer_cast<ManipulatorGoalRegion>(goal_region_)->isSatisfied(s2)) {
		cout << "WER TEHERE" << endl;		
	}*/
    std::vector<double> angles1;
    std::vector<double> angles2;    
    for (unsigned int i = 0; i < dim_; i++) {
        angles1.push_back(s1->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);        
        angles2.push_back(s2->as<ompl::base::RealVectorStateSpace::StateType>()->values[i]);
    }    
    return checkMotion(angles1, angles2, continuous_collision_);
}

/** Check if a motion between two states is valid. This assumes that state s1 is valid */
bool MotionValidator::checkMotion(const ompl::base::State *s1, 
                                  const ompl::base::State *s2, 
                                  std::pair< ompl::base::State *, double > &/*lastValid*/) const {
    return checkMotion(s1, s2);
}

bool MotionValidator::isValid(const std::vector<double> &s1) const {
	std::vector<double> joint_angles;	
	for (size_t i = 0; i < dim_; i++) {
		joint_angles.push_back(s1[i]);
	}	
	std::vector<OBB> manipulator_collision_structures;
	
	robot_->createRobotCollisionStructures(joint_angles, manipulator_collision_structures);	
    /**std::vector<OBB> manipulator_collision_structures = utils_.createManipulatorCollisionStructures(joint_angles, 
                                                                                                    link_dimensions_,
                                                                                                    kinematics_);*/
    for (size_t i = 0; i < obstacles_.size(); i++) {
        if (!obstacles_[i]->getTerrain()->isTraversable()) {        	
        	if (obstacles_[i]->in_collision(manipulator_collision_structures)) {        		
        		return false;
        	}
        }
    }    
    return true;    
}

void MotionValidator::setObstacles(std::vector<std::shared_ptr<Obstacle> > &obstacles) {
    obstacles_.clear();
    for (size_t i = 0; i < obstacles.size(); i++) {       
        obstacles_.push_back(obstacles[i]);
    }    
}

}

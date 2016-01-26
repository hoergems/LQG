#ifndef MAN_MOTION_VALIDATOR_HPP_
#define MAN_MOTION_VALIDATOR_HPP_

#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "Obstacle.hpp"
#include "fcl/BVH/BVH_model.h"
#include "fcl/BV/BV.h"
#include "fcl/collision_object.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/shape/geometric_shapes_utility.h"
#include "robot.hpp"

#include <iostream>
#include <mutex>


namespace shared {

    class MotionValidator: public ompl::base::MotionValidator {
        public:
            MotionValidator(const ompl::base::SpaceInformationPtr &si,
            		        boost::shared_ptr<shared::Robot> &robot,
                            bool continuous_collision,
                            bool dynamics);
            ~MotionValidator() = default;

            /** Check if a motion between two states is valid. This assumes that state s1 is valid */
            bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const;

            bool checkMotion(const std::vector<double> &s1, 
                             const std::vector<double> &s2,
                             const bool &continuous_collision) const; 
 
            /** Check if a motion between two states is valid. This assumes that state s1 is valid */
            bool checkMotion(const ompl::base::State *s1, 
                             const ompl::base::State *s2, 
                             std::pair< ompl::base::State *, double > &/*lastValid*/) const;
            
            bool satisfiesConstraints(const std::vector<double> &s1) const;
            
            bool isValid(const ompl::base::State *state) const;
                             
            bool isValid(const std::vector<double> &s1) const;
                                         
            void setObstacles(std::vector<std::shared_ptr<Obstacle>> &obstacles);
            
            void setContinuousCollisionCheck(bool continuous_collision_check);
            
            bool collidesDiscrete(const std::vector<double> &state) const;
            
            bool collidesContinuous(const std::vector<double> &state1,
            		                const std::vector<double> &state2) const;
            
        private:
            const ompl::base::SpaceInformationPtr si_;
            
            boost::shared_ptr<shared::Robot> robot_;
            
            std::mutex mtx;    

            /** The obstacles in the environment */
            std::vector<std::shared_ptr<Obstacle> > obstacles_;
            
            bool continuous_collision_;
            
            unsigned int dim_;             

            //void set_link_aabbs() const;
    };
}

#endif

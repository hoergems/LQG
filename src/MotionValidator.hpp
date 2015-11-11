#ifndef MAN_MOTION_VALIDATOR_HPP_
#define MAN_MOTION_VALIDATOR_HPP_

#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "Obstacle.hpp"
#include "utils.hpp"
#include "fcl/BVH/BVH_model.h"
#include "fcl/BV/BV.h"
#include "fcl/collision_object.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/shape/geometric_shapes_utility.h"
#include "Kinematics.hpp"

#include <iostream>
#include <mutex>


namespace shared {

    class MotionValidator: public ompl::base::MotionValidator {
        public:
            MotionValidator(const ompl::base::SpaceInformationPtr &si,
                            std::shared_ptr<Kinematics> kinematics,                            
                            std::vector<std::shared_ptr<Obstacle> > obstacles,
                            double delta_t,
                            bool continuous_collision);
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
                             
            bool isValid(const std::vector<double> &s1) const;
                             
            void setObstacles(std::vector<std::shared_ptr<Obstacle>> &obstacles);

            void setLinkDimensions(std::vector<std::vector<double>> &link_dimensions);           

        private:
            const ompl::base::SpaceInformationPtr si_;
            
            std::mutex mtx;
        
            /** Forward kinematics */
            std::shared_ptr<Kinematics> kinematics_;

            /** The obstacles in the environment */
            std::vector<std::shared_ptr<Obstacle> > obstacles_;
            
            double delta_t_;
            
            bool continuous_collision_;
            
            //std::vector<fcl::AABB> link_aabbs_;

            utils::Utils utils_;

            std::vector<std::vector<double>> link_dimensions_;

            //void set_link_aabbs() const;
    };
}

#endif

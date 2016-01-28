/** @file ManipulatorState.hpp
 *
 * Defines the RockSampleState class, which represents a state of the RockSample problem.
 */
#ifndef SPHERE_OBSTACLE_HPP_
#define SPHERE_OBSTACLE_HPP_
#include "Obstacle.hpp"
#include <vector>
#include <unistd.h>
#include <memory>
#include "Terrain.hpp"
#include "fcl/BV/BV.h"
#include "fcl/collision_object.h"
#include "fcl/collision_data.h"
#include "fcl/continuous_collision.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/shape/geometric_shapes_utility.h"

namespace shared {

class SphereObstacle: public Obstacle  {
    public:    
        SphereObstacle(std::string name, 
        		       double pos_x, 
        		       double pos_y, 
        		       double pos_z, 
        		       double radius, 
        		       const Terrain &terrain);
                
        virtual void createCollisionObject() override;

    private:
        double pos_x_;
        double pos_y_;
        double pos_z_;
        double radius_;
};

}

#endif /* OBSTACLE_HPP_ */
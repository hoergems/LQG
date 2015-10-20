/** @file ManipulatorState.hpp
 *
 * Defines the RockSampleState class, which represents a state of the RockSample problem.
 */
#ifndef OBSTACLE_HPP_
#define OBSTACLE_HPP_

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
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
//#include "utils.hpp"

#include <mutex>

namespace shared {

class Obstacle {
    public:
    
        Obstacle(double pos_x, double pos_y, double pos_z, double size_x, double size_y, double size_z, const Terrain &terrain);
        ~Obstacle() = default;

        /** Check of a point lies inside the obstacles */
        bool in_collision(const std::vector<std::shared_ptr<Obstacle> > &other_obstacles);
        
        bool in_collision(std::vector<fcl::OBB> &other_collision_structures);
        
        bool in_collision(const fcl::CollisionObject &collision_object_start, const fcl::CollisionObject &collision_object_goal);
        
        bool in_collision(std::vector<double> &point);
        
        bool in_collision_discrete(boost::python::list &ns);

        bool in_collision_continuous(boost::python::list &ns);

        std::shared_ptr<Terrain> getTerrain() const;
        
        std::shared_ptr<fcl::OBB> getCollisionStructure() const; 
        
        std::vector<double> getDimensions() const; 

        bool isTraversable();

    private:   
    
        double pos_x_;
        double pos_y_;
        double pos_z_;
        double size_x_;
        double size_y_;
        double size_z_;
        
        fcl::OBB collision_structure_;
        
        fcl::OBB collision_structure_expanded_;
        
        std::shared_ptr<fcl::CollisionObject> collision_object_ptr_;
                
        const shared::Terrain terrain_;
        
        void createCollisionStructure();
        
        void createCollisionObject();
};

}

#endif /* OBSTACLE_HPP_ */

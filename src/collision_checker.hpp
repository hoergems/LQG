#ifndef COLLISION_CHECKER_HPP_
#define COLLISION_CHECKER_HPP_
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include "fcl/broadphase/broadphase.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#include "fcl/collision_data.h"
#include "fcl/collision.h"
#include "Obstacle.hpp"

namespace shared {
    struct CollisionData {
        CollisionData()  {
            done = false;
        }  
        fcl::CollisionRequest request;  
        fcl::CollisionResult result;  
        bool done;
    };
    
    bool defaultCollisionFunction(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata_) {
        CollisionData* cdata = static_cast<CollisionData*>(cdata_);
        const fcl::CollisionRequest& request = cdata->request;
        fcl::CollisionResult& result = cdata->result;
        if(cdata->done) return true;
        fcl::collide(o1, o2, request, result);
        if(!request.enable_cost && (result.isCollision()) && (result.numContacts() >= request.num_max_contacts))
            cdata->done = true;
        return cdata->done;
    };


	class CollisionChecker {
	public:
		CollisionChecker();
		
		/**
		 * Set the obstacles that make up the terrain
		 */
		void setObstacles(std::vector<std::shared_ptr<Obstacle> > &obstacles);
		
		/**
		 * Python wrapper for setObstacles
		 */
		void setObstaclesPy(boost::python::list &ns);
		
		/**
		 * Check of the robot collision objects collide with the environment 
		 */
		bool inCollisionDiscrete(std::vector<std::shared_ptr<fcl::CollisionObject>> &robot_collision_objects);
		
		/**
		 * A python wrapper for inCollisionDiscrete
		 */
		bool inCollisionDiscretePy(boost::python::list &ns); 
		
		/**
		 * Check if a robot in motion collides with the environment
		 */
		bool inCollisionContinuous(std::shared_ptr<fcl::CollisionObject> &robot_collision_object_start, 
				std::shared_ptr<fcl::CollisionObject> &robot_collision_object_goal);
		
	private:
		fcl::BroadPhaseCollisionManager* obstacle_collision_manager_;
		
	};
}

#endif
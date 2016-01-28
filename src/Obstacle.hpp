/** @file ManipulatorState.hpp
 *
 * Defines the RockSampleState class, which represents a state of the RockSample problem.
 */
#ifndef OBSTACLE_HPP_
#define OBSTACLE_HPP_
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <boost/python/wrapper.hpp>
#include <vector>
#include <unistd.h>
#include <memory>
#include "Terrain.hpp"
#include "fcl/BV/BV.h"
#include "fcl/collision_object.h"
#include "fcl/collision_data.h"
#include "fcl/collision.h"
#include "fcl/continuous_collision.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/shape/geometric_shapes_utility.h"
//#include "utils.hpp"

#include <mutex>

using std::cout;
using std::endl;

namespace shared {

class Obstacle {
    public:
    
        Obstacle(std::string name, const Terrain &terrain);
        ~Obstacle() = default;
        
        //void set_dimensions(std::vector<double> &position, std::vector<double>)

        /** 
         * Checks if this obstacle collides with another obstacle 
         * */
        bool in_collision(const std::vector<std::shared_ptr<Obstacle> > &other_obstacles) const;
        
        //bool in_collision(std::vector<fcl::AABB> &other_collision_structures) const;
        
        /**
         * Checks if this obstacle collides with a CollisionObject 
         */
        bool in_collision(std::vector<std::shared_ptr<fcl::CollisionObject>> &other_collision_objects) const; 
        
        /**
         * Checks if the obstacle collides with another moving collision object.
         * The motion of of the other collision object is determined by a start
         * and goal transformation
         */
        bool in_collision(std::shared_ptr<fcl::CollisionObject> &collision_object_start, 
        		std::shared_ptr<fcl::CollisionObject>  &collision_object_goal) const;
        
        /**
         * Checks if a point lies withing this obstacle
         */
        bool in_collision(std::vector<double> &point);
        
        virtual bool in_collision_point(std::vector<double> &point);
        
        /**
         * Python interface for discrete collision check
         */
        virtual bool in_collision_discrete(boost::python::list &ns);

        /**
         * Python interface for continuous collision check
         */
        virtual bool in_collision_continuous(boost::python::list &ns); 

        /**
         * Get the terrain this obstacle consists of
         */
        std::shared_ptr<Terrain> getTerrain() const;
        
        /**
         * Gets the external force (proportional to the end effector velocity)
         * the underlying obstacles induces on the end effector.
         */
        virtual double getExternalForce();
        
        /**
         * Get the underlying collision object
         */
        std::shared_ptr<fcl::CollisionObject> getCollisionObject() const;

        /**
         * Determines if the obstacle is traversable
         */
        bool isTraversable() const;
                 
        /**
         * Create the underlying collision object
         */
        virtual void createCollisionObject() = 0;
        
        /**
         * Get the obstacle name
         */
        virtual std::string getName();
        
        /**
         * Set the obstacle's standard color
         */
        virtual void setStandardColor(std::vector<double> &diffuseColor,
        		                      std::vector<double> &ambientColor);
        
        /**
         * Get the standard diffuse color
         */
        virtual std::vector<double> getStandardDiffuseColor();
        
        /**
         * Get the standard ambient color
         */
        virtual std::vector<double> getStandardAmbientColor();

    protected:
        std::string name_;
        
        std::vector<double> diffuse_color_;
        
        std::vector<double> ambient_color_;
        
        std::shared_ptr<fcl::CollisionObject> collision_object_ptr_;
                
        const shared::Terrain terrain_;
};

/**
 * A Python wrapper to handle polymorphism
 */
struct ObstacleWrapper: Obstacle, boost::python::wrapper<Obstacle> {
public:
	ObstacleWrapper(std::string name, const Terrain &terrain):
		Obstacle(name, terrain) {		
	}
	
	void createCollisionObject() {
		this->get_override("createCollisionObject")();
	}
	
	bool in_collision_discrete(boost::python::list &ns) {		
		this->get_override("in_collision_discrete")(ns);
	}
	
	bool in_collision_continuous(boost::python::list &ns) {
		this->get_override("in_collision_continuous")(ns);
	}
	
	bool in_collision_point(std::vector<double> &point) {
		this->get_override("in_collision_point")(point);
	}
	
	double getExternalForce() {
		this->get_override("getExternalForce")();
	}
	
	std::string getName() {
		this->get_override("getName")();
	}
	
	std::vector<double> getStandardDiffuseColor() {
		this->get_override("getStandardDiffuseColor")();
	}
	
	std::vector<double> getStandardAmbientColor() {
		this->get_override("getStandardAmbientColor")();
	}
	
};

}

#endif /* OBSTACLE_HPP_ */

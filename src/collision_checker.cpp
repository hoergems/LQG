#include "collision_checker.hpp"

namespace shared {

CollisionChecker::CollisionChecker():	
	obstacle_collision_manager_(new fcl::DynamicAABBTreeCollisionManager()){
	
}

void CollisionChecker::setObstacles(std::vector<std::shared_ptr<Obstacle> > &obstacles) {
	for (auto &o: obstacles) {
		//obstacles_collision_objects_.push_back(o->getCollisionObject());
		obstacle_collision_manager_->registerObject(o->getCollisionObject().get());
	}
	obstacle_collision_manager_->setup();
}

void CollisionChecker::setObstaclesPy(boost::python::list &ns) {
	std::vector<std::shared_ptr<Obstacle> > obstacles;
	for (size_t i = 0; i < len(ns); i++) {
	    std::shared_ptr<ObstacleWrapper> obst_wrapper = boost::python::extract<std::shared_ptr<ObstacleWrapper>>(ns[i]);
	    obstacles.push_back(std::static_pointer_cast<shared::Obstacle>(obst_wrapper));
	    //obstacles_.push_back(std::make_shared<Obstacle>(boost::python::extract<Obstacle>(ns[i])));
	}
	
	setObstacles(obstacles);
}

bool CollisionChecker::inCollisionDiscrete(std::vector<std::shared_ptr<fcl::CollisionObject>> &robot_collision_objects) {
	fcl::BroadPhaseCollisionManager* robot_collision_manager = new fcl::DynamicAABBTreeCollisionManager();
	for (auto &k: robot_collision_objects) {
		robot_collision_manager->registerObject(k.get());
	}
	robot_collision_manager->setup();
	CollisionData collision_data;
	obstacle_collision_manager_->collide(robot_collision_manager, &collision_data, shared::defaultCollisionFunction);
	return collision_data.result.isCollision();	
}

bool CollisionChecker::inCollisionDiscretePy(boost::python::list &ns) {
	std::vector<std::shared_ptr<fcl::CollisionObject>> robot_collision_objects;
	for (int i = 0; i < len(ns); ++i)
	{
	    robot_collision_objects.push_back(boost::python::extract<std::shared_ptr<fcl::CollisionObject>>(ns[i]));
	}
	
	return inCollisionDiscrete(robot_collision_objects);
}

bool CollisionChecker::inCollisionContinuous(std::shared_ptr<fcl::CollisionObject> &robot_collision_object_start,
		std::shared_ptr<fcl::CollisionObject> &robot_collision_object_goal) {
	return false;
}

BOOST_PYTHON_MODULE(libcollision_checker) { 
	using namespace boost::python;
	class_<CollisionChecker>("CollisionChecker", init<>())
			.def("setObstacles", &CollisionChecker::setObstaclesPy)
			.def("inCollisionDiscrete", &CollisionChecker::inCollisionDiscretePy)
    ;
}

}
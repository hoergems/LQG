#include "SphereObstacle.hpp"
#include <iostream> 

using std::cout;
using std::endl;

using namespace fcl;

namespace shared {

SphereObstacle::SphereObstacle(std::string name, 
		                       double pos_x, 
		                       double pos_y, 
		                       double pos_z, 
		                       double radius, 
		                       const Terrain &terrain):
	Obstacle(name, terrain),
	pos_x_(pos_x),
	pos_y_(pos_y),
	pos_z_(pos_z),
	radius_(radius){
	createCollisionObject();
}

void SphereObstacle::createCollisionObject() {	
	Sphere* sphere = new Sphere(radius_);
	Vec3f trans(pos_x_, pos_y_, pos_z_);
	Matrix3f rot(1.0, 0.0, 0.0,
		         0.0, 1.0, 0.0,
		         0.0, 0.0, 1.0);
	Transform3f rotate_translate(rot, trans);
	collision_object_ptr_ = std::make_shared<fcl::CollisionObject>(fcl::CollisionObject(boost::shared_ptr<CollisionGeometry>(sphere), 
				                                                                        rotate_translate));
	
}

}
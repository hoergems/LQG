#include "BoxObstacle.hpp"
#include <iostream> 

using std::cout;
using std::endl;

using std::min;
using std::max;

using namespace fcl;
using namespace boost::python;

namespace shared {

BoxObstacle::BoxObstacle(std::string name,
		                 double pos_x, 
		                 double pos_y, 
		                 double pos_z, 
		                 double size_x, 
		                 double size_y, 
		                 double size_z, 
		                 const Terrain &terrain):
	Obstacle(name, terrain),	
    pos_x_(pos_x),
    pos_y_(pos_y),
    pos_z_(pos_z),
    size_x_(size_x),
    size_y_(size_y),
    size_z_(size_z) {
    createCollisionObject();    
}

void BoxObstacle::createCollisionObject() {	
	fcl::AABB box_aabb(fcl::Vec3f(pos_x_ - size_x_,
			                      pos_y_ - size_y_,
								  pos_z_ - size_z_),
			           fcl::Vec3f(pos_x_ + size_x_,
			                      pos_y_ + size_y_,
								  pos_z_ + size_z_));
	fcl::Box* box = new fcl::Box();
	fcl::Transform3f box_tf;
	fcl::Transform3f trans;	
	fcl::constructBox(box_aabb, trans, *box, box_tf);	
	collision_object_ptr_ = std::make_shared<fcl::CollisionObject>(boost::shared_ptr<CollisionGeometry>(box),
			                                                       box_tf);	
	/**cout << "size_x_" << size_x_ << endl;
	cout << "size_y_" << size_y_ << endl;
	cout << "size_z_" << size_z_ << endl;
	
	Box* box = new Box(size_x_, size_y_, size_z_);
	Vec3f trans(pos_x_, pos_y_, pos_z_);
	Matrix3f rot(1.0, 0.0, 0.0,
	             0.0, 1.0, 0.0,
	             0.0, 0.0, 1.0);
	Transform3f rotate_translate(rot, trans);
	collision_object_ptr_ = std::make_shared<fcl::CollisionObject>(fcl::CollisionObject(boost::shared_ptr<CollisionGeometry>(box), 
			                                                                            rotate_translate));*/
	
    /**Box* box = new Box();
    Transform3f box_tf;
    Matrix3f rot(1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0);
    Vec3f trans(0.0, 0.0, 0.0);
    Transform3f rotate_transform(rot, trans);    
    constructBox(collision_structure_, rotate_transform, *box, box_tf);    
    collision_object_ptr_ = std::make_shared<fcl::CollisionObject>(fcl::CollisionObject(boost::shared_ptr<CollisionGeometry>(box), box_tf));*/
}

}

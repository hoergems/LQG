
#include "Obstacle.hpp"
#include <iostream> 

using std::cout;
using std::endl;

using std::min;
using std::max;

using namespace fcl;
using namespace boost::python;

namespace shared {

Obstacle::Obstacle(double pos_x, double pos_y, double pos_z, double size_x, double size_y, double size_z, const Terrain &terrain): 
    pos_x_(pos_x),
    pos_y_(pos_y),
    pos_z_(pos_z),
    size_x_(size_x),
    size_y_(size_y),
    size_z_(size_z),
    collision_structure_(),    
    collision_object_ptr_(),    
    terrain_(terrain) {    
    createCollisionStructure();    
    createCollisionObject();    
}

bool Obstacle::in_collision(const std::vector<std::shared_ptr<Obstacle> > &other_obstacles) {        
    for (size_t i = 0; i < other_obstacles.size(); i++) {
        if (collision_structure_.overlap(*(other_obstacles[i]->getCollisionStructure()))) {
            return true;
        }
    }
    
    return false;
}

bool Obstacle::in_collision(std::vector<OBB> &other_collision_structures) {	
    for (size_t i = 0; i < other_collision_structures.size(); i++) {        
        if (collision_structure_.overlap(other_collision_structures[i])) {            
            return true;
        }
    }
    
    return false;
}

bool Obstacle::in_collision(std::vector<double> &point) {
    Vec3f p_vec(point[0], point[1], point[2]);    
    return collision_structure_.contain(p_vec);
}

bool Obstacle::in_collision_discrete(boost::python::list &ns) {
    std::vector<OBB> other_collision_structures;
    for (int i = 0; i < len(ns); ++i)
    {
        other_collision_structures.push_back(boost::python::extract<OBB>(ns[i]));
    }
    
    return in_collision(other_collision_structures);
}

bool Obstacle::in_collision_continuous(boost::python::list &ns) {

    fcl::CollisionObject collision_object_start = boost::python::extract<fcl::CollisionObject>(ns[0]);
    fcl::CollisionObject collision_object_goal = boost::python::extract<fcl::CollisionObject>(ns[1]);
    return in_collision(collision_object_start, collision_object_goal);
}

bool Obstacle::in_collision(const fcl::CollisionObject &collision_object_start, const fcl::CollisionObject &collision_object_goal) { 	
    fcl::ContinuousCollisionRequest request;
    fcl::ContinuousCollisionResult result;    
    fcl::continuousCollide(&collision_object_start, 
                           collision_object_goal.getTransform(), 
                           collision_object_ptr_.get(),
                           collision_object_ptr_->getTransform(),
                           request,
                           result);
    return result.is_collide;
}

void Obstacle::createCollisionObject() {
    Box* box = new Box();
    Transform3f box_tf;
    Matrix3f rot(1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0);
    Vec3f trans(0.0, 0.0, 0.0);
    Transform3f rotate_transform(rot, trans);    
    constructBox(collision_structure_, rotate_transform, *box, box_tf);    
    collision_object_ptr_ = std::make_shared<fcl::CollisionObject>(fcl::CollisionObject(boost::shared_ptr<CollisionGeometry>(box), box_tf));
}

void Obstacle::createCollisionStructure() {    
    Vec3f p1(pos_x_ - size_x_, 
             pos_y_ - size_y_, 
             pos_z_ - size_z_);
    Vec3f p2(pos_x_ + size_x_, 
             pos_y_ + size_y_, 
             pos_z_ + size_z_);
    AABB collision_structure(p1, p2);
    Matrix3f rot(1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0);
    Vec3f trans(0.0, 0.0, 0.0);
    Transform3f rotate_transform(rot, trans);
    convertBV(collision_structure, rotate_transform, collision_structure_);    
}

std::shared_ptr<Terrain> Obstacle::getTerrain() const {
    return std::make_shared<Terrain>(terrain_);
}

std::shared_ptr<OBB> Obstacle::getCollisionStructure() const {
    return std::make_shared<OBB>(collision_structure_);
}

std::vector<double> Obstacle::getDimensions() const {
    std::vector<double> dimensions({pos_x_, pos_y_, pos_z_, size_x_, size_y_, size_z_});
    return dimensions;
}

bool Obstacle::isTraversable() {
    return terrain_.isTraversable();
}

BOOST_PYTHON_MODULE(obstacle)
{   
    #include "Terrain.hpp"
    bool (Obstacle::*in_collision_d)(boost::python::list&) = &Obstacle::in_collision_discrete;
    bool (Obstacle::*in_collision_c)(boost::python::list&) = &Obstacle::in_collision_continuous;
    
    class_<Terrain>("Terrain", init<const std::string, const double, const double, const bool>())
         .def("getTraversalCost", &Terrain::getTraversalCost)
         .def("getName", &Terrain::getName)
         .def("getVelocityDamping", &Terrain::getVelocityDamping)
         .def("isTraversable", &Terrain::isTraversable)    
    ;
    
    class_<Obstacle>("Obstacle", init<double, double, double, double, double, double, Terrain>())
         .def("getDimensions", &Obstacle::getDimensions)
         .def("inCollisionDiscrete", in_collision_d)
         .def("inCollisionContinuous", in_collision_c)
         .def("isTraversable", &Obstacle::isTraversable)         
    ;
}

}

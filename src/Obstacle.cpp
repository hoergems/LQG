#include "Obstacle.hpp"
#include <iostream> 

using std::cout;
using std::endl;

using std::min;
using std::max;

using namespace fcl;
using namespace boost::python;

namespace shared {

template<class T>
struct VecToList
{
    static PyObject* convert(const std::vector<T>& vec)
    {
        boost::python::list* l = new boost::python::list();
        for(size_t i = 0; i < vec.size(); i++)
            (*l).append(vec[i]);

        return l->ptr();
    }
};

Obstacle::Obstacle(std::string name, const Terrain &terrain):
		collision_object_ptr_(nullptr),
		terrain_(terrain),
		name_(name),
		diffuse_color_(),
		ambient_color_(){
	
}

std::shared_ptr<Terrain> Obstacle::getTerrain() const {
    return std::make_shared<Terrain>(terrain_);
}

void Obstacle::setStandardColor(std::vector<double> &diffuseColor,
        		                std::vector<double> &ambientColor) {
	diffuse_color_.clear();
	ambient_color_.clear();
	
	if (diffuseColor.size() == 0) {
		diffuse_color_.push_back(0.0);
		diffuse_color_.push_back(0.0);
		diffuse_color_.push_back(0.0);
	}
	
	if (ambientColor.size() == 0) {
		ambient_color_.push_back(0.0);
		ambient_color_.push_back(0.0);
		ambient_color_.push_back(0.0);
	}
	
	for (size_t i = 0; i < diffuseColor.size(); i++) {
		diffuse_color_.push_back(diffuseColor[i]);
	}
	
	for (size_t i = 0; i < ambientColor.size(); i++) {
		ambient_color_.push_back(ambientColor[i]);
	}
}

bool Obstacle::isTraversable() const{
    return terrain_.isTraversable();
}

double Obstacle::getExternalForce() {
	return terrain_.getVelocityDamping();
}

bool Obstacle::in_collision(std::vector<double> &point) {
    Vec3f p_vec(point[0], point[1], point[2]);
    return collision_object_ptr_->getAABB().contain(p_vec);    
}

bool Obstacle::in_collision_point(std::vector<double> &point) {
    Vec3f p_vec(point[0], point[1], point[2]);
    return collision_object_ptr_->getAABB().contain(p_vec);    
}

bool Obstacle::in_collision(std::vector<std::shared_ptr<fcl::CollisionObject>> &other_collision_objects) const {	
	for (size_t i = 0; i < other_collision_objects.size(); i++) {
		fcl::CollisionRequest request;		
		fcl::CollisionResult result;
		fcl::collide(other_collision_objects[i].get(), 
				     collision_object_ptr_.get(),
					 request,
					 result);
		if (result.isCollision()) {			
			return true;
		}
		/**fcl::AABB overlap;
		if (collision_object_ptr_->getAABB().overlap(other_collision_objects[i]->getAABB(), overlap)) {
			cout << "overlap center: ";
			cout << overlap.center()[0] << ", ";
			cout << overlap.center()[1] << ", ";
			cout << overlap.center()[2] << endl;
			return true;
		}*/
	}
	
	return false;
}

bool Obstacle::in_collision(const std::vector<std::shared_ptr<Obstacle> > &other_obstacles) const{        
    for (size_t i = 0; i < other_obstacles.size(); i++) {
        if (collision_object_ptr_->getAABB().overlap(other_obstacles[i]->getCollisionObject()->getAABB())) {
            return true;
        }
    }
    
    return false;
}

bool Obstacle::in_collision(std::shared_ptr<fcl::CollisionObject> &collision_object_start, 
		std::shared_ptr<fcl::CollisionObject> &collision_object_goal) const {
	fcl::ContinuousCollisionRequest request(10,
	    		                            0.0001,
	    		                            CCDM_LINEAR,
	    		                            GST_LIBCCD,
	    		                            CCDC_NAIVE);
    fcl::ContinuousCollisionResult result;    
    fcl::continuousCollide(collision_object_start.get(), 
                           collision_object_goal->getTransform(), 
                           collision_object_ptr_.get(),
                           collision_object_ptr_->getTransform(),
                           request,
                           result);    
    return result.is_collide;    
}

bool Obstacle::in_collision_discrete(boost::python::list &ns) {
    std::vector<std::shared_ptr<fcl::CollisionObject>> other_collision_objects;
    for (int i = 0; i < len(ns); ++i)
    {
        other_collision_objects.push_back(boost::python::extract<std::shared_ptr<fcl::CollisionObject>>(ns[i]));
    }
    
    return in_collision(other_collision_objects);
}

bool Obstacle::in_collision_continuous(boost::python::list &ns) {

    std::shared_ptr<fcl::CollisionObject> collision_object_start = 
    		boost::python::extract<std::shared_ptr<fcl::CollisionObject>>(ns[0]);
    std::shared_ptr<fcl::CollisionObject> collision_object_goal = 
    		boost::python::extract<std::shared_ptr<fcl::CollisionObject>>(ns[1]);    
    return in_collision(collision_object_start, collision_object_goal);
}

std::shared_ptr<fcl::CollisionObject> Obstacle::getCollisionObject() const {
	return collision_object_ptr_;
}

std::string Obstacle::getName() {	
	return name_;
}

std::vector<double> Obstacle::getStandardDiffuseColor() {	
	return diffuse_color_;
}

std::vector<double> Obstacle::getStandardAmbientColor() {
	return ambient_color_;
}

BOOST_PYTHON_MODULE(libobstacle)
{   
    #include "Terrain.hpp"
    bool (ObstacleWrapper::*in_collision_d)(boost::python::list&) = &ObstacleWrapper::in_collision_discrete;
    bool (ObstacleWrapper::*in_collision_c)(boost::python::list&) = &ObstacleWrapper::in_collision_continuous;
    bool (ObstacleWrapper::*in_collision_p)(std::vector<double>&) = &ObstacleWrapper::in_collision_point;
    
    to_python_converter<std::vector<std::shared_ptr<ObstacleWrapper>, std::allocator<std::shared_ptr<ObstacleWrapper>> >, 
    	                    VecToList<std::shared_ptr<ObstacleWrapper>> >();
    
    register_ptr_to_python<std::shared_ptr<ObstacleWrapper>>();
    
    class_<Terrain>("Terrain", init<const std::string, const double, const double, const bool>())
         .def("getTraversalCost", &Terrain::getTraversalCost)
         .def("getName", &Terrain::getName)
         .def("getVelocityDamping", &Terrain::getVelocityDamping)
         .def("isTraversable", &Terrain::isTraversable)    
    ;
    
    class_<ObstacleWrapper, boost::noncopyable>("Obstacle", init<std::string, Terrain>())         
         .def("inCollisionDiscrete", in_collision_d)
         .def("inCollisionContinuous", in_collision_c)
		 .def("inCollisionPoint", in_collision_p)
         .def("isTraversable", &ObstacleWrapper::isTraversable)
		 .def("getExternalForce", &ObstacleWrapper::getExternalForce)
		 .def("createCollisionObject", boost::python::pure_virtual(&ObstacleWrapper::createCollisionObject))
		 .def("getName", &ObstacleWrapper::getName)
		 .def("getStandardDiffuseColor", &ObstacleWrapper::getStandardDiffuseColor)
		 .def("getStandardAmbientColor", &ObstacleWrapper::getStandardAmbientColor)
    ;
}

}

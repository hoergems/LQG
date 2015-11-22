#include "Area.hpp"

using std::cout;
using std::endl;

namespace shared {

Area::Area(double pos_x,
		   double pos_y,
		   double pos_z,
		   double size_x,
		   double size_y,
		   double size_z):
	location_({pos_x, pos_y, pos_z}),
	dimension_({size_x, size_y, size_z}),
	radius_(0),
	shape_(AreaShape::BOX)
{
	
}

Area::Area(double pos_x,
           double pos_y,
   	       double pos_z,
           double radius) :
    location_({pos_x, pos_y, pos_z}),
	dimension_(),
	radius_(radius),
	shape_(AreaShape::SPHERE)
{
	
}

Area::AreaShape Area::getAreaShape() const{
	return shape_;
}

std::vector<double> Area::getLocation() const{
	return location_;
}

std::vector<double> Area::getDimension() const {
	return dimension_;
}

double Area::getRadius() const {
	return radius_;
}

BOOST_PYTHON_MODULE(libarea)
{   
	using namespace boost::python;
	
	enum_<Area::AreaShape>("AreaShape")
	        .value("SHPERE", Area::SPHERE)
	        .value("BOX", Area::BOX)
	        .export_values()
	        ;
	
    class_<Area>("Area", init<double, double, double, double>())
         .def("getAreaShape", &Area::getAreaShape)
		 .def("getLocation", &Area::getLocation)
		 .def("getDimension", &Area::getDimension)
		 .def("getRadius", &Area::getRadius)
    ;
}

}
#ifndef UTILS_HPP_
#define UTILS_HPP_

#include <memory>
#include "Kinematics.hpp"
#include "Obstacle.hpp"
#include "Terrain.hpp"
#include <iostream> 
#include <fstream>
#include <boost/python.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include "fcl/BV/BV.h" 
#include "fcl/collision_object.h"
#include "fcl/shape/geometric_shapes.h"
#include "fcl/shape/geometric_shapes_utility.h"
#include <iostream>
#include <iosfwd> 
#include <string>
#include <sstream>

using boost::property_tree::ptree;

namespace utils {

double euclideanDistance(std::vector<double> &vec1, std::vector<double> &vec2);

class Utils {      
    public:
		void loadObstaclesXML(std::vector<std::shared_ptr<shared::Obstacle> > *obst, 
				      std::string &obstacles_path);
				           
		void loadTerrains(std::vector<std::shared_ptr<shared::Terrain> > *terrains,
				  std::string &terrains_path);

                std::vector<std::vector<double>> loadGoalStates();

                std::vector<std::vector<double>> getLinkDimensions(const std::string &model_path);
				          
		std::vector<fcl::OBB> createManipulatorCollisionStructures(const std::vector<double> &joint_angles,
                                                                           const std::vector<std::vector<double>> &link_dimensions, 
                                                                           const std::shared_ptr<shared::Kinematics> &kinematics) const;

                std::vector<fcl::CollisionObject> createManipulatorCollisionObjects(const std::vector<double> &joint_angles,
                                                                                    const std::vector<std::vector<double>> &link_dimensions,
                                                                                    const std::shared_ptr<shared::Kinematics> &kinematics) const;

        bool serializeStatePath(std::vector<std::vector<double>> state_path);
};

}

#endif

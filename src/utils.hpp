#ifndef UTILS_HPP_
#define UTILS_HPP_
#include <assert.h>
#include <memory>
#include "Obstacle.hpp"
#include "BoxObstacle.hpp"
#include "SphereObstacle.hpp"
#include "Terrain.hpp"
#include <iostream> 
#include <fstream>
#include <boost/python.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <tinyxml.h>
#include <iostream>
#include <iosfwd> 
#include <string>
#include <sstream>

using boost::property_tree::ptree;

namespace utils {

double euclideanDistance(std::vector<double> &vec1, std::vector<double> &vec2);

class Utils {      
    public:
	   std::vector<std::shared_ptr<shared::ObstacleWrapper>> loadObstaclesXMLPy(std::string obstacles_file);
	
		void loadObstaclesXML(std::string &obstacles_file,
                              std::vector<std::shared_ptr<shared::Obstacle> > &obst);
		
		void loadGoalAreaPy(std::string env_file, std::vector<double> &goal_area);
		
		void loadGoalArea(std::string &env_file, std::vector<double> &goal_area);
				           
		void loadTerrains(std::vector<std::shared_ptr<shared::Terrain> > *terrains,
				          std::string &terrains_path);

        std::vector<std::vector<double>> loadGoalStates();        

        bool serializeStatePath(std::vector<std::vector<double>> state_path);
};

}

#endif

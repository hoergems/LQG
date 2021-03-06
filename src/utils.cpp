#include "utils.hpp"

#include <sys/types.h>
#include <dirent.h>
#include <errno.h>

using std::cout;
using std::endl;

namespace utils {

struct TerrainStruct {
    std::string name;
    double velocityDamping;
    double traversalCost;    
    bool traversable;
};

struct ObstacleStruct {
	std::string name;
	std::string type;
    double x;
    double y;
    double z;
    std::vector<double> extends;
    
    // Diffuse color
    std::vector<double> d_color;
    
    // Ambient color
    std::vector<double> a_color;
    TerrainStruct terrain;     
};

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

double euclideanDistance(const std::vector<double> &vec1, const std::vector<double> &vec2) {
    double sum = 0.0;
    for (size_t i = 0; i < vec1.size(); i++) {
        sum += pow(vec2[i] - vec1[i], 2);
    }
    
    return sqrt(sum);
}

std::vector<std::vector<double>> Utils::loadGoalStates() {
    std::vector<std::vector<double>> gs;
        std::ifstream file;
        try {
            file.open("goalstates.txt");
        }
        catch (std::ios_base::failure& e) {
            std::cerr << e.what() << '\n';
            sleep(5);
        }
        
        double dub_val;
        std::string line;
               
        while (std::getline(file, line))
        {                      
            std::istringstream sin(line);
            std::vector<double> angles; 
            while (sin >> dub_val) {
                angles.push_back(dub_val);                
            }
            
            gs.push_back(angles);            
        }
        file.clear();
        file.seekg(0, file.beg);
        file.close();
        return gs;   

}

bool Utils::checkProblemFeasibility(std::vector<std::shared_ptr<shared::Obstacle>> &obstacles,
       		                 boost::shared_ptr<shared::Robot> &robot,
       		                 std::vector<double> &start_state,
       		                 std::vector<std::vector<double>> &goal_states,
       		                 std::vector<double> &ee_goal_position,
       		                 double &ee_goal_threshold,
       		                 double &simulation_step_size,
       		                 double &control_duration,       		                 
       		                 double &planning_velocity) {
	std::string planner = "RRT";
	std::shared_ptr<shared::DynamicPathPlanner> path_planner = 
			std::make_shared<shared::DynamicPathPlanner>(robot, 
			                                             false);	
	path_planner->setupMotionValidator(true);
	path_planner->setup(simulation_step_size, control_duration, planner);
	std::string control_sampler = "discrete";
	path_planner->setControlSampler(control_sampler);
	std::vector<int> num_control_samples({1});
	path_planner->setNumControlSamples(num_control_samples);
	path_planner->setRRTGoalBias(0.05);
	std::vector<int> min_max_control_duration({1, 4});
	path_planner->setMinMaxControlDuration(min_max_control_duration);
	path_planner->addIntermediateStates(true);
	path_planner->setObstacles(obstacles);
	path_planner->setGoalStates(goal_states, ee_goal_position, ee_goal_threshold);
	cout << "plan path" << endl;
	std::vector<std::vector<double>> path = path_planner->solve(start_state, 5.0);
	if (path.size() > 0) {
		std::shared_ptr<shared::PathPlanner> path_planner_kin = 
				std::make_shared<shared::PathPlanner>(robot,
						                              control_duration,
						                              true,
						                              planning_velocity,
						                              1.0,
						                              false,
						                              false,
						                              "RRTConnect");
		path_planner_kin->setGoalStates(goal_states, ee_goal_position, ee_goal_threshold);
		path_planner_kin->setup();
		path_planner_kin->setObstacles(obstacles);
		std::vector<std::vector<double> > solution_path;
		solution_path = path_planner_kin->solve(start_state, 0.1);
		if (solution_path.size() > 0) {
			return true;
		}
		else {
			cout << "No deterministic linear solution" << endl;
		}
	}
	cout << "return false" << endl;
	return false;
}

std::shared_ptr<shared::Obstacle> Utils::generateBoxObstacle(std::string name,
                                                             double x_pos,
                                                             double y_pos,
		                                                     double z_pos,
		                                                     double x_size,
		                                                     double y_size,
		                                                     double z_size) {
	shared::Terrain terrain(name + "_terrain",
			                0.0,
			                0.0,
			                false);
	std::shared_ptr<shared::Obstacle> obst = std::make_shared<shared::BoxObstacle>(name,
			                                                                       x_pos,
			                                                                       y_pos,
			                                                                       z_pos,
			                                                                       x_size,
			                                                                       y_size,
			                                                                       z_size,
			                                                                       terrain);
	std::vector<double> d_color({0.5, 0.0, 0.3, 1.0});
    std::vector<double> a_color({0.25, 0.0, 0.15, 1.0});
    obst->setStandardColor(d_color, a_color);
    return obst;
	
}

std::vector<std::shared_ptr<shared::ObstacleWrapper>> Utils::generateObstacle(std::string name,
		                                                                      double x_pos,
			                                                                  double y_pos,
																			  double z_pos,
																			  double x_size,
																			  double y_size,
																			  double z_size) {
	std::vector<std::shared_ptr<shared::ObstacleWrapper>> obstacles;
	std::vector<std::shared_ptr<shared::Obstacle> > obst;
	
	shared::Terrain terrain(name + "_terrain",
		                    0.0,
		                    0.0,
		                    false);
	
	obst.push_back(std::make_shared<shared::BoxObstacle>(name,
		    			                  x_pos,
	                                      y_pos,
	                                      z_pos,
					                      x_size,
						                  y_size,
							              z_size,
							              terrain));
	std::vector<double> d_color({0.5, 0.0, 0.3, 1.0});
	std::vector<double> a_color({0.25, 0.0, 0.15, 1.0});
	obst[obst.size() - 1]->setStandardColor(d_color, a_color);
	
	for (size_t i = 0; i < obst.size(); i++) {
		obstacles.push_back(std::static_pointer_cast<shared::ObstacleWrapper>(obst[i]));
	}
	
	return obstacles;
}

std::vector<std::shared_ptr<shared::ObstacleWrapper>> Utils::loadObstaclesXMLPy(std::string obstacles_file) {
	std::vector<std::shared_ptr<shared::ObstacleWrapper>> obstacles;	
	std::vector<std::shared_ptr<shared::Obstacle> > obst;
	loadObstaclesXML(obstacles_file, obst);
	for (size_t i = 0; i < obst.size(); i++) {
		obstacles.push_back(std::static_pointer_cast<shared::ObstacleWrapper>(obst[i]));
	}
	/**for (auto &k: obst) {
		obstacles.push_back(k);
	}*/
	
	return obstacles;
	
}

bool Utils::file_exists(std::string &filename) {
	return boost::filesystem::exists(filename);
}

void Utils::loadObstaclesXML(std::string &obstacles_file,
		                     std::vector<std::shared_ptr<shared::Obstacle> > &obst) {
	if (!file_exists(obstacles_file)) {
		cout << "Utils: ERROR: Environment file '" << obstacles_file << "' doesn't exist" << endl;		
		assert(false);		
	}
	
	std::vector<ObstacleStruct> obstacles;	
	TiXmlDocument xml_doc;	
    xml_doc.LoadFile(obstacles_file);    
	TiXmlElement *env_xml = xml_doc.FirstChildElement("Environment");
	for (TiXmlElement* obst_xml = env_xml->FirstChildElement("KinBody"); obst_xml; obst_xml = obst_xml->NextSiblingElement("KinBody"))
	{	
		std::string name(obst_xml->Attribute("name"));
		TiXmlElement *body_xml = obst_xml->FirstChildElement("Body");
		std::string enable_str(body_xml->Attribute("enable"));		
		if (enable_str == "true") {			
			TiXmlElement *body_xml = obst_xml->FirstChildElement("Body");			
			if (body_xml) {
				TiXmlElement *geom_xml = body_xml->FirstChildElement("Geom");
				TiXmlElement *terrain_xml = body_xml->FirstChildElement("Terrain");
				//Can play with different shapes here in the future
				if (geom_xml) {
					std::string type(geom_xml->Attribute("type"));					
					TiXmlElement *trans_xml = geom_xml->FirstChildElement("Translation");
					if (trans_xml) {
						obstacles.push_back(ObstacleStruct());
						const char* xyz_str = trans_xml->GetText();
						std::vector<std::string> pieces;
						std::vector<double> xyz_vec;
						boost::split( pieces, xyz_str, boost::is_any_of(" "));
				        for (unsigned int i = 0; i < pieces.size(); ++i) {
					       if (pieces[i] != "") { 
					          xyz_vec.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
		                   }
			            }
				        
				        TiXmlElement *dcolor_xml = geom_xml->FirstChildElement("diffuseColor");
				        if (dcolor_xml) {
				        	const char* color_string = dcolor_xml->GetText();				        	
				        	std::vector<std::string> pieces;
				        	std::vector<double> color_vec;
				        	boost::split(pieces, color_string, boost::is_any_of(" "));
				            for (unsigned i = 0; i < pieces.size(); i++) {				            	
				        		color_vec.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
				        	}
				            
				            obstacles[obstacles.size() - 1].d_color.push_back(color_vec[0]);
				            obstacles[obstacles.size() - 1].d_color.push_back(color_vec[1]);
				            obstacles[obstacles.size() - 1].d_color.push_back(color_vec[2]);
				            
				        }
				        
				        TiXmlElement *acolor_xml = geom_xml->FirstChildElement("ambientColor");
				        if (acolor_xml) {
				            const char* color_string = acolor_xml->GetText();
				        	std::vector<std::string> pieces;
				            std::vector<double> color_vec;
				            boost::split(pieces, color_string, boost::is_any_of(" "));
				        	for (unsigned i = 0; i < pieces.size(); i++) {
				        		color_vec.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
				        	}
				        				            
				        	obstacles[obstacles.size() - 1].a_color.push_back(color_vec[0]);
				            obstacles[obstacles.size() - 1].a_color.push_back(color_vec[1]);
				        	obstacles[obstacles.size() - 1].a_color.push_back(color_vec[2]);				        				            
				        }
				        
				        obstacles[obstacles.size() - 1].name = name;
				        obstacles[obstacles.size() - 1].type = type;
				        obstacles[obstacles.size() - 1].x = xyz_vec[0];
				        obstacles[obstacles.size() - 1].y = xyz_vec[1];
				        obstacles[obstacles.size() - 1].z = xyz_vec[2];
					    if (type == "box") {					    	
						    TiXmlElement *ext_xml = geom_xml->FirstChildElement("extents");
						    if (ext_xml) {
						        const char* ext_str = ext_xml->GetText();						        
						        std::vector<double> extends_vec;
						        pieces.clear();
						        boost::split( pieces, ext_str, boost::is_any_of(" "));
						        for (unsigned int i = 0; i < pieces.size(); ++i) {
						            if (pieces[i] != "") {
						                extends_vec.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
						            }
						        }
						        
						        obstacles[obstacles.size() - 1].extends.push_back(extends_vec[0]);
						        obstacles[obstacles.size() - 1].extends.push_back(extends_vec[1]);
						        obstacles[obstacles.size() - 1].extends.push_back(extends_vec[2]);						        
						    }
					    }
					    else if (type == "sphere") {
					    	TiXmlElement *rad_xml = geom_xml->FirstChildElement("Radius");
					    	if (rad_xml) {					    		
					    		obstacles[obstacles.size() - 1].extends.push_back(boost::lexical_cast<double>(rad_xml->GetText()));					    		
					    	}
					    }
					}
					
				}
				
				if (terrain_xml) {					
					TerrainStruct terrain;
					std::string terrain_name(terrain_xml->Attribute("name"));					
					TiXmlElement *damping_xml = terrain_xml->FirstChildElement("Damping");
					double damping = boost::lexical_cast<double>(damping_xml->GetText());
					TiXmlElement *cost_xml = terrain_xml->FirstChildElement("Cost");
					double cost = boost::lexical_cast<double>(cost_xml->GetText());
					TiXmlElement *traversable_xml = terrain_xml->FirstChildElement("Traversable");
					bool traversable = false;					
					if (boost::lexical_cast<std::string>(traversable_xml->GetText()) == "true") {						
						traversable = true;
					}
					terrain.name = terrain_name;
					terrain.velocityDamping = damping;
					terrain.traversalCost = cost;
					terrain.traversable = traversable;
					obstacles[obstacles.size() - 1].terrain = terrain;					
				}
			}
		}
	}
	
	for (size_t i = 0; i < obstacles.size(); i++) {
	    shared::Terrain terrain(obstacles[i].terrain.name,
	                            obstacles[i].terrain.traversalCost,
	                            obstacles[i].terrain.velocityDamping,
	                            obstacles[i].terrain.traversable);
	    if (obstacles[i].type == "box") {
	    	obst.push_back(std::make_shared<shared::BoxObstacle>(obstacles[i].name,
	    			                                             obstacles[i].x,
                                                                 obstacles[i].y,
                                                                 obstacles[i].z,
														         obstacles[i].extends[0],
														         obstacles[i].extends[1],
														         obstacles[i].extends[2],
														         terrain));
	    }
	    
	    else if (obstacles[i].type == "sphere") {
	    	obst.push_back(std::make_shared<shared::SphereObstacle>(obstacles[i].name,
	    			                                                obstacles[i].x,
                                                                    obstacles[i].y,
                                                                    obstacles[i].z,
                                                                    obstacles[i].extends[0],
					                                                terrain));
	    }
	    else {
	    	assert(false && "Utils: ERROR: Obstacle has an unknown type!");
	    }
	    
	    obst[obst.size() - 1]->setStandardColor(obstacles[i].d_color, obstacles[i].a_color);
	}
}

void Utils::loadGoalAreaPy(std::string env_file, std::vector<double> &goal_area) {
	loadGoalArea(env_file, goal_area);
}

void Utils::loadGoalArea(std::string &env_file, std::vector<double> &goal_area) {
	if (!file_exists(env_file)) {
		cout << "Utils: ERROR: Environment file '" << env_file << "' doesn't exist" << endl;		
		assert(false);		
	}
	TiXmlDocument xml_doc;	
	xml_doc.LoadFile(env_file);    
    TiXmlElement *env_xml = xml_doc.FirstChildElement("Environment");
	for (TiXmlElement* obst_xml = env_xml->FirstChildElement("KinBody"); obst_xml; obst_xml = obst_xml->NextSiblingElement("KinBody"))
	{
		std::string name(obst_xml->Attribute("name"));
		if (name == "GoalArea") { 
			TiXmlElement *body_xml = obst_xml->FirstChildElement("Body");			
			if (body_xml) {
				TiXmlElement *geom_xml = body_xml->FirstChildElement("Geom");
				if (geom_xml) {
					TiXmlElement *trans_xml = geom_xml->FirstChildElement("Translation");
					if (trans_xml) {
						const char* xyz_str = trans_xml->GetText();
						std::vector<std::string> pieces;						
						boost::split( pieces, xyz_str, boost::is_any_of(" "));
						for (unsigned int i = 0; i < pieces.size(); ++i) {
						    if (pieces[i] != "") { 
						        goal_area.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
							}
						}
					}
					TiXmlElement *radius_xml = geom_xml->FirstChildElement("Radius");
					if (radius_xml) {
						const char* rad_str = radius_xml->GetText();
						goal_area.push_back(boost::lexical_cast<double>(radius_xml->GetText()));
					}
				}				
			}
		}
	}
}

void Utils::loadTerrains(std::vector<std::shared_ptr<shared::Terrain> > *terrains,
                         std::string &terrains_path) {    
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(terrains_path.c_str())) == NULL) {
        cout << "Error(" << errno << ") opening " << terrains_path << endl;
    }

    while ((dirp = readdir(dp)) != NULL) {
        std::string filename = std::string(dirp->d_name);
        if (!filename.find(".") == 0 && 
            filename.find("~") > filename.length()) {
            std::string terrainName;
            double trajectoryCost;
            double velocityDamping; 
            bool traversable;           
            std::string line;
            std::ifstream fin; 
            fin.open(terrains_path + "/" + filename);
            if (fin.is_open()) {
                while (getline(fin,line)) {
                    if (line.find("name") != std::string::npos) {
                        std::string namestring = "name: ";
                        line.erase(line.find("name"), namestring.length());
                        terrainName = line;                                              
                    }
                    else if (line.find("traversalCost") != std::string::npos) {
                        std::string coststring = "traversalCost: ";
                        line.erase(line.find("traversalCost"), coststring.length());
                        trajectoryCost = atof(line.c_str());
                    }
                    else if (line.find("velocityDamping") != std::string::npos) {
                        std::string velocitystring = "velocityDamping: ";
                        line.erase(line.find("velocityDamping"), velocitystring.length());
                        velocityDamping = atof(line.c_str());
                    }
                    else if (line.find("traversable") != std::string::npos) {
                        std::string traversablestring = "traversable: ";
                        line.erase(line.find("traversable"), traversablestring.length());
                        traversable = false;
                        if (line.find("true") != std::string::npos) {
                            traversable = true;
                        }
                    }
                }
                fin.close();                
            }
            
            terrains->push_back(std::make_shared<shared::Terrain>(terrainName, trajectoryCost, velocityDamping, traversable));
        }
        
    }
    closedir(dp);
}

bool Utils::serializeStatePath(std::vector<std::vector<double>> state_path) {
    std::stringstream ss;
    std::string file("../../python/state_path1.txt");
    for (size_t i = 0; i < state_path.size(); i++) {
        for (size_t j = 0; j < state_path[i].size(); j++) {
            ss << state_path[i][j] << " ";
        }
        ss << " \n";
    }

    std::ofstream out(file.c_str());
    if(out.fail())
    {
        out.close();
        return false;
    }
    out << ss.str();
    out.close();
}

BOOST_PYTHON_MODULE(libutil) {
    // An established convention for using boost.python.
    using namespace boost::python;
    
    typedef std::vector<std::vector<double> > vec_vec;
    
    boost::python::type_info info= boost::python::type_id<std::vector<double>>();
    const boost::python::converter::registration* reg_double = boost::python::converter::registry::query(info);
    if (reg_double == NULL || (*reg_double).m_to_python == NULL)  {
        class_<std::vector<double> > ("v_double")
        	.def(vector_indexing_suite<std::vector<double> >());
    }
        
    info = boost::python::type_id<std::vector<int>>();
    const boost::python::converter::registration* reg_int = boost::python::converter::registry::query(info);
    if (reg_int == NULL || (*reg_int).m_to_python == NULL)  {    
        class_<std::vector<int> > ("v_int")
        	.def(vector_indexing_suite<std::vector<int> >());
    }
        
    info = boost::python::type_id<std::vector<std::vector<double>>>();
    const boost::python::converter::registration* reg_v2double = boost::python::converter::registry::query(info);
    if (reg_v2double == NULL || (*reg_v2double).m_to_python == NULL)  {  
        class_<std::vector<std::vector<double> > > ("v2_double")
        	.def(vector_indexing_suite<std::vector<std::vector<double> > >());
    }
        
    info = boost::python::type_id<std::vector<std::vector<int>>>();
    const boost::python::converter::registration* reg_v2int = boost::python::converter::registry::query(info);
    if (reg_v2int == NULL || (*reg_v2int).m_to_python == NULL)  {    
        class_<std::vector<std::vector<int> > > ("v2_int")
        	.def(vector_indexing_suite<std::vector<std::vector<int> > >());
    }
    
    info = boost::python::type_id<std::vector<std::shared_ptr<shared::ObstacleWrapper>>>();
    const boost::python::converter::registration* reg_vobst = boost::python::converter::registry::query(info);
    if (reg_vobst == NULL || (*reg_vobst).m_to_python == NULL)  { 
        class_<std::vector<std::shared_ptr<shared::ObstacleWrapper>> > ("v_obstacle")
            .def(vector_indexing_suite<std::vector<std::shared_ptr<shared::ObstacleWrapper>> >());
        to_python_converter<std::vector<std::shared_ptr<shared::ObstacleWrapper>, std::allocator<std::shared_ptr<shared::ObstacleWrapper>> >, 
            VecToList<std::shared_ptr<shared::ObstacleWrapper>> >();
        register_ptr_to_python<std::shared_ptr<shared::ObstacleWrapper>>();
    }
    
    class_<Utils>("Utils", init<>())
    		.def("loadObstaclesXML", &Utils::loadObstaclesXMLPy)
			.def("loadGoalArea", &Utils::loadGoalAreaPy)
			.def("generateObstacle", &Utils::generateObstacle)
    ;
}

}

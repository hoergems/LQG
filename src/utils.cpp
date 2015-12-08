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
    double x;
    double y;
    double z;
    double size_x;
    double size_y;
    double size_z;
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

double euclideanDistance(std::vector<double> &vec1, std::vector<double> &vec2) {
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

void Utils::loadObstaclesXML(std::string &obstacles_file,
		                     std::vector<std::shared_ptr<shared::Obstacle> > &obst) {
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
					TiXmlElement *trans_xml = geom_xml->FirstChildElement("Translation");
					TiXmlElement *ext_xml = geom_xml->FirstChildElement("extents");
					if (trans_xml && ext_xml) {
						obstacles.push_back(ObstacleStruct());
						const char* xyz_str = trans_xml->GetText();
						const char* ext_str = ext_xml->GetText();
						std::vector<std::string> pieces;
						std::vector<double> xyz_vec;
						std::vector<double> extends_vec;
						boost::split( pieces, xyz_str, boost::is_any_of(" "));
						for (unsigned int i = 0; i < pieces.size(); ++i) {
							if (pieces[i] != "") { 
								xyz_vec.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
							}
						}
						
						pieces.clear();
						boost::split( pieces, ext_str, boost::is_any_of(" "));
						for (unsigned int i = 0; i < pieces.size(); ++i) {
							if (pieces[i] != "") {
								extends_vec.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
							}
						}
						
						obstacles[obstacles.size() - 1].x = xyz_vec[0];
						obstacles[obstacles.size() - 1].y = xyz_vec[1];
						obstacles[obstacles.size() - 1].z = xyz_vec[2];
						obstacles[obstacles.size() - 1].size_x = extends_vec[0];
						obstacles[obstacles.size() - 1].size_y = extends_vec[1];
						obstacles[obstacles.size() - 1].size_z = extends_vec[2];
																			 
																			 
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
					if (traversable_xml->GetText() == "true") {
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
	    obst.push_back(std::make_shared<shared::Obstacle>(obstacles[i].x,
	                                                      obstacles[i].y,
	                                                      obstacles[i].z,
	                                                      obstacles[i].size_x,
	                                                      obstacles[i].size_y,
	                                                      obstacles[i].size_z,
	                                                      terrain));
	       
	}
}

void Utils::loadGoalArea(std::string &env_file, std::vector<double> &goal_area) {
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
    
    class_<std::vector<std::vector<double> > > ("v2_double")
         .def(vector_indexing_suite<std::vector<std::vector<double> > >());
    
    class_<std::vector<std::vector<int> > > ("v2_int")
         .def(vector_indexing_suite<std::vector<std::vector<int> > >());
         
    class_<std::vector<double> > ("v_double")
         .def(vector_indexing_suite<std::vector<double> >());
         
    class_<std::vector<int> > ("v_int")
         .def(vector_indexing_suite<std::vector<int> >());
    
    class_<Utils>("Utils")
         
         
    ;
}

}

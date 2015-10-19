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

std::vector<std::vector<double>> Utils::getLinkDimensions(const std::string &model_file) {    
    /**DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(model_path.c_str())) == NULL) {
        cout << "Error(" << errno << ") opening " << model_path << endl;
    }*/
    std::vector<std::vector<double>> link_dimensions;
    
    
    
    ptree pt;
    std::ifstream fin;   
    fin.open(model_file);
    read_xml(fin, pt);
    BOOST_FOREACH( ptree::value_type &v, pt.get_child("Robot") ) {
        if (v.first == "KinBody") { 
            cout << "Got Kin Body" << endl;
            boost::property_tree::ptree subtree = (boost::property_tree::ptree)v.second;
            BOOST_FOREACH(boost::property_tree::ptree::value_type &vs, subtree) {
                if (vs.first == "Body") {
                    if (vs.second.get_child("<xmlattr>.type").data() == "dynamic") {
                        std::vector<double> dims;                    
                        boost::property_tree::ptree subtree2 = (boost::property_tree::ptree)vs.second;
                        BOOST_FOREACH(boost::property_tree::ptree::value_type &vss, subtree2) {
                            if (vss.first == "Geom") {
                                std::string extents = vss.second.get<std::string>("Extents");
                                std::istringstream size_s(extents);
                                double d;
                                while (size_s >> d) {                                            
                                    dims.push_back(d * 2.0);
                                }
                            }
                        }
                        link_dimensions.push_back(dims);
                    }
                }
            }
        }
    }
    
    return link_dimensions;
}

void Utils::loadObstaclesXML(std::vector<std::shared_ptr<shared::Obstacle> > *obst, 
                             std::string &obstacles_path) {
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(obstacles_path.c_str())) == NULL) {
        cout << "Error(" << errno << ") opening " << obstacles_path << endl;
    }
    std::vector<ObstacleStruct> obstacles;
    while ((dirp = readdir(dp)) != NULL) {
        std::string filename = std::string(dirp->d_name);
        if (!filename.find(".") == 0 && 
            filename.find("~") > filename.length()) {                              
                ptree pt;
                std::ifstream fin;   
                fin.open(obstacles_path + "/" + filename);
                read_xml(fin, pt);
                BOOST_FOREACH( ptree::value_type &v, pt.get_child("Environment") ) {
                    if(v.first == "KinBody") {
                        obstacles.push_back(ObstacleStruct());
                        std::vector<double> position;
                        std::vector<double> size;                                            
                        boost::property_tree::ptree subtree = (boost::property_tree::ptree)v.second;
                        BOOST_FOREACH(boost::property_tree::ptree::value_type &vs, subtree) {
                            if (vs.first == "Body") {                               
                                boost::property_tree::ptree subtree2 = (boost::property_tree::ptree)vs.second;
                                BOOST_FOREACH(boost::property_tree::ptree::value_type &vss, subtree2) {                                    
                                    if (vss.first == "Geom") {                                          
                                        std::string trans = vss.second.get<std::string>("Translation");
                                        std::string size_string = vss.second.get<std::string>("extents");                                        
                                        std::istringstream trans_s(trans);
                                        std::istringstream size_s(size_string);
                                        double d;
                                        while (trans_s >> d) {                                            
                                            position.push_back(d);
                                        }
                                        while (size_s >> d) {                                            
                                            size.push_back(d);
                                        }
                                        obstacles[obstacles.size() - 1].x = position[0];
                                        obstacles[obstacles.size() - 1].y = position[1];
                                        obstacles[obstacles.size() - 1].z = position[2];
                                        obstacles[obstacles.size() - 1].size_x = size[0];
                                        obstacles[obstacles.size() - 1].size_y = size[1];
                                        obstacles[obstacles.size() - 1].size_z = size[2];
                                    }
                                    else if (vss.first == "Terrain") {                                        
                                        TerrainStruct terrain;
                                        terrain.name = vss.second.get_child("<xmlattr>.name").data();
                                        terrain.velocityDamping = vss.second.get<double>("Damping");
                                        terrain.traversalCost = vss.second.get<double>("Cost");                                        
                                        terrain.traversable = vss.second.get<bool>("Traversable");                                                                
                                        obstacles[obstacles.size() - 1].terrain = terrain;
                                        
                                    }
                                }   
                            }
                        }
                    }
                }
             }
         
    }
    for (size_t i = 0; i < obstacles.size(); i++) {
        shared::Terrain terrain(obstacles[i].terrain.name,
                                obstacles[i].terrain.traversalCost,
                                obstacles[i].terrain.velocityDamping,
                                obstacles[i].terrain.traversable);
        obst->push_back(std::make_shared<shared::Obstacle>(obstacles[i].x,
                                                           obstacles[i].y,
                                                           obstacles[i].z,
                                                           obstacles[i].size_x,
                                                           obstacles[i].size_y,
                                                           obstacles[i].size_z,
                                                           terrain));
       std::vector<double> dimensions = obst->at(i)->getDimensions();
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



std::vector<fcl::OBB> Utils::createManipulatorCollisionStructures(const std::vector<double> &joint_angles,
                                                                  const std::vector<std::vector<double>> &link_dimensions,
                                                                  const std::shared_ptr<shared::Kinematics> &kinematics) const{
    std::vector<fcl::AABB> link_aabbs;
    for (size_t i = 0; i < link_dimensions.size(); i++) {
        link_aabbs.push_back(fcl::AABB(fcl::Vec3f(0.0, -link_dimensions[i][1] / 2.0, -link_dimensions[i][2] / 2.0),
                                       fcl::Vec3f(link_dimensions[i][0], link_dimensions[i][1] / 2.0, link_dimensions[i][2] / 2.0)));
    }
    //fcl::AABB link_aabb(fcl::Vec3f(0.0, -0.0025, -0.0025), fcl::Vec3f(1.0, 0.0025, 0.0025));
    std::vector<fcl::OBB> collision_structures;
    int n = 0;
    for (size_t i = 0; i < joint_angles.size(); i++) {
        const std::pair<fcl::Vec3f, fcl::Matrix3f> pose_link_n = kinematics->getPoseOfLinkN(joint_angles, n);
        fcl::OBB obb;
        fcl::convertBV(link_aabbs[i], fcl::Transform3f(pose_link_n.second, pose_link_n.first), obb);
        collision_structures.push_back(obb);
        n++;
    }
    
    return collision_structures;
}

std::vector<fcl::CollisionObject> Utils::createManipulatorCollisionObjects(const std::vector<double> &joint_angles,
                                                                           const std::vector<std::vector<double>> &link_dimensions,
                                                                           const std::shared_ptr<shared::Kinematics> &kinematics) const {
    std::vector<fcl::CollisionObject> vec;
    std::vector<fcl::AABB> link_aabbs;
    for (size_t i = 0; i < link_dimensions.size(); i++) {
        link_aabbs.push_back(fcl::AABB(fcl::Vec3f(0.0, -link_dimensions[i][1] / 2.0, -link_dimensions[i][2] / 2.0),
                                       fcl::Vec3f(link_dimensions[i][0], link_dimensions[i][1] / 2.0, link_dimensions[i][2] / 2.0)));
    }
    
    //fcl::AABB link_aabb(fcl::Vec3f(0.0, -0.0025, -0.0025), fcl::Vec3f(1.0, 0.0025, 0.0025));
    int n = 0;
    for (size_t i = 0; i < joint_angles.size(); i++) {
        const std::pair<fcl::Vec3f, fcl::Matrix3f> pose_link_n = kinematics->getPoseOfLinkN(joint_angles, n);
        fcl::Box* box = new fcl::Box();
        fcl::Transform3f box_tf;
        fcl::Transform3f trans(pose_link_n.second, pose_link_n.first);        
        fcl::constructBox(link_aabbs[i], trans, *box, box_tf);        
        vec.push_back(fcl::CollisionObject(boost::shared_ptr<fcl::CollisionGeometry>(box), box_tf));
        n++;
    }
    return vec;
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

BOOST_PYTHON_MODULE(util) {
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
    
    class_<fcl::OBB>("OBB");
    class_<fcl::CollisionObject>("CollisionObject", init<const boost::shared_ptr<fcl::CollisionGeometry>, const fcl::Transform3f>());
    to_python_converter<std::vector<fcl::OBB, std::allocator<fcl::OBB> >, VecToList<fcl::OBB> >();
    to_python_converter<std::vector<fcl::CollisionObject, std::allocator<fcl::CollisionObject> >, VecToList<fcl::CollisionObject> >();
    class_<Utils>("Utils")
         .def("createManipulatorCollisionStructures", &Utils::createManipulatorCollisionStructures)
         .def("createManipulatorCollisionObjects", &Utils::createManipulatorCollisionObjects)
         .def("getLinkDimensions", &Utils::getLinkDimensions)         
         
    ;
}

}

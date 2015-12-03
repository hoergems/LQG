#include "robot.hpp"

using std::cout;
using std::endl;

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

bool Robot::initJoints(TiXmlElement *robot_xml) {
	for (TiXmlElement* joint_xml = robot_xml->FirstChildElement("joint"); joint_xml; joint_xml = joint_xml->NextSiblingElement("joint"))
	{
		// Joint Names
		std::string joint_name(joint_xml->Attribute("name"));
		joint_names_.push_back(joint_name);
		
		// Joint origin
		std::vector<double> origin = process_origin_(joint_xml);
		joint_origins_.push_back(origin);
		
		// Joint axes
		std::vector<int> joint_axis;
		TiXmlElement *axis_xml = joint_xml->FirstChildElement("axis");
		if (axis_xml) {
			const char* xyz_str = axis_xml->Attribute("xyz");
			std::vector<std::string> pieces;					
			boost::split( pieces, xyz_str, boost::is_any_of(" "));
			for (unsigned int i = 0; i < pieces.size(); ++i) { 
			    if (pieces[i] != "") { 
			    	try {
			    		joint_axis.push_back(boost::lexical_cast<int>(pieces[i].c_str()));
			    	}
			    	catch (boost::bad_lexical_cast &e) {
			    								    									
			        }
			    }
			}
			
			joint_axes_.push_back(joint_axis);
		}
		else {
			std::vector<int> ax({0, 0, 0});
			joint_axes_.push_back(ax);
		}
		
		// Joint limits
		TiXmlElement *limit_xml = joint_xml->FirstChildElement("limit");
		double torque_limit = 0.0;
		double lower_limit = 0.0;
		double upper_limit = 0.0;
		double velocity_limit = 0.0;
		if (limit_xml) {
			try {
				std::string effort_str(limit_xml->Attribute("effort"));
			    torque_limit = boost::lexical_cast<double>(effort_str.c_str());
			    
			    std::string lower_str(limit_xml->Attribute("lower"));
			    lower_limit = boost::lexical_cast<double>(lower_str.c_str());
			    
			    std::string upper_str(limit_xml->Attribute("upper"));
			    upper_limit = boost::lexical_cast<double>(upper_str.c_str());
			    
			    std::string vel_str(limit_xml->Attribute("velocity"));
			    velocity_limit = boost::lexical_cast<double>(vel_str.c_str());
			}
			catch (boost::bad_lexical_cast &e) {
						    								    									
		    }
		}
		
		joint_torque_limits_.push_back(torque_limit);
		lower_joint_limits_.push_back(lower_limit);
		upper_joint_limits_.push_back(upper_limit);
		joint_velocity_limits_.push_back(velocity_limit);
		
		// Joint types
		std::string joint_type(joint_xml->Attribute("type"));
		joint_types_.push_back(joint_type);
		if (joint_type == "revolute") {
			active_joints_.push_back(joint_name);
			active_joint_origins_.push_back(origin);
			active_joint_axes_.push_back(joint_axis);
			active_joint_torque_limits_.push_back(torque_limit);
			
			active_lower_joint_limits_.push_back(lower_limit);
			active_upper_joint_limits_.push_back(upper_limit);
			active_joint_velocity_limits_.push_back(velocity_limit);			
		}		
	}
}

bool Robot::initLinks(TiXmlElement *robot_xml) {
	for (TiXmlElement* link_xml = robot_xml->FirstChildElement("link"); link_xml; link_xml = link_xml->NextSiblingElement("link"))
	{
		//Link names
		std::string link_name(link_xml->Attribute("name"));
		link_names_.push_back(link_name);
		
		//Link dimensions
		std::vector<double> link_dimension;
		TiXmlElement *coll_xml = link_xml->FirstChildElement("collision");
		if (coll_xml) {
			active_link_names_.push_back(link_name);
			TiXmlElement *geom_xml = coll_xml->FirstChildElement("geometry");			
			TiXmlElement *dim_xml = geom_xml->FirstChildElement("box");			
		    const char* xyz_str = dim_xml->Attribute("size");		    
		    std::vector<std::string> pieces;					
		    boost::split( pieces, xyz_str, boost::is_any_of(" "));
		    for (unsigned int i = 0; i < pieces.size(); ++i) { 
			    if (pieces[i] != "") {
				    try {
					    link_dimension.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
				    }
				    catch (boost::bad_lexical_cast &e) {
							    									
				    }
			    }
		    }
		    if (link_dimension.size() != 3) {
		    	std::vector<double> ld({0.0, 0.0, 0.0});
		    	link_dimensions_.push_back(ld);
		    }
		    else {
		    	link_dimensions_.push_back(link_dimension);
		    	active_link_dimensions_.push_back(link_dimension);
		    }
		    
		    std::vector<double> link_origin = process_origin_(coll_xml);
		    link_origins_.push_back(link_origin);
		      
		    link_origins_.push_back(link_origin);		    
		}
		else {
			std::vector<double> ld({0.0, 0.0, 0.0});
			link_dimensions_.push_back(ld);
		}
		
		//Link inertia
		TiXmlElement *ine = link_xml->FirstChildElement("inertial");
		
		if (ine) {
			
			// Link masses
			active_links_.push_back(link_name);			
			TiXmlElement *mass_xml = ine->FirstChildElement("mass");
			double mass = 0.0;
			if (mass_xml) {
				try {
					if (mass_xml->Attribute("value")) {
						mass = boost::lexical_cast<double>(mass_xml->Attribute("value"));
					}
				}
				catch (boost::bad_lexical_cast &e) {
					
				}
			}
			link_masses_.push_back(mass);			
			
			//Inertia origins
			std::vector<double> inertia_origin = process_origin_(ine);
			link_inertia_origins_.push_back(inertia_origin);
			
			//Inertia matrix
			std::vector<double> inertia_vals;
			double ixx = 0.0;
			double ixy = 0.0;
			double ixz = 0.0;
			double iyy = 0.0;
			double iyz = 0.0;
			double izz = 0.0;
			TiXmlElement *matr_xml = ine->FirstChildElement("inertia");
			if (matr_xml) {				
				try {
				    if (matr_xml->Attribute("ixx")) {
					    ixx = boost::lexical_cast<double>(matr_xml->Attribute("ixx"));					    
				    }
				    if (matr_xml->Attribute("ixy")) {
				        ixy = boost::lexical_cast<double>(matr_xml->Attribute("ixy"));
				    }
				    if (matr_xml->Attribute("ixz")) {
				    	ixz = boost::lexical_cast<double>(matr_xml->Attribute("ixz"));
				    }
				    if (matr_xml->Attribute("iyy")) {
				    	iyy = boost::lexical_cast<double>(matr_xml->Attribute("iyy"));
				    }
				    if (matr_xml->Attribute("iyz")) {
				    	iyz = boost::lexical_cast<double>(matr_xml->Attribute("iyz"));
				    }
				    if (matr_xml->Attribute("izz")) {
				    	izz = boost::lexical_cast<double>(matr_xml->Attribute("izz"));
				    }
				}
				catch (boost::bad_lexical_cast &e) {
									
				}
			}
				
			inertia_vals.push_back(ixx);
			inertia_vals.push_back(ixy);
			inertia_vals.push_back(ixz);
			inertia_vals.push_back(iyy);
			inertia_vals.push_back(iyz);
			inertia_vals.push_back(izz);
			link_inertia_matrices_.push_back(inertia_vals);
		}
		else {			
			link_masses_.push_back(0.0);
			
			std::vector<double> origin({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
			link_inertia_origins_.push_back(origin);
			
			std::vector<double> inert({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
			link_inertia_matrices_.push_back(inert);			
		}
	}
	
	return true;
}

std::vector<double> Robot::process_origin_(TiXmlElement *xml) {
	TiXmlElement *origin_xml = xml->FirstChildElement("origin");
	std::vector<double> origin;
	if (origin_xml) {				
		if (origin_xml->Attribute("xyz")) {
			const char* xyz_str = origin_xml->Attribute("xyz");
			const char* rpy_str = origin_xml->Attribute("rpy");
			std::vector<std::string> pieces;					
			boost::split( pieces, xyz_str, boost::is_any_of(" "));
			for (unsigned int i = 0; i < pieces.size(); ++i){
				if (pieces[i] != "") {
					try {
						origin.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
					}
					catch (boost::bad_lexical_cast &e) {
								
					}
				}						
			}
			
			pieces.clear();
			boost::split( pieces, rpy_str, boost::is_any_of(" "));
			for (unsigned int i = 0; i < pieces.size(); ++i){
				if (pieces[i] != "") {
					try {
						origin.push_back(boost::lexical_cast<double>(pieces[i].c_str()));
					}
					catch (boost::bad_lexical_cast &e) {
											
					}
				}						
			}
		}				
	}
	if (origin.size() == 6) {
		return origin;
	}
	else {
		std::vector<double> orig({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
		return orig;
	}
}

Robot::Robot(std::string robot_file):
	robot_file_(robot_file),	
	link_names_(),
	active_link_names_(),
	joint_names_(),
	joint_origins_(),
	link_origins_(),
	active_joint_origins_(),
	active_links_(),
	active_joints_(),
	joint_axes_(),
	active_joint_axes_(),
	joint_torque_limits_(),
	lower_joint_limits_(),
	upper_joint_limits_(),
	joint_velocity_limits_(),
	active_joint_velocity_limits_(),
	active_joint_torque_limits_(),	
	active_lower_joint_limits_(),
	active_upper_joint_limits_(),
	link_masses_(),
	link_inertia_origins_(),
	robot_state_(),
	propagator_(new Propagator()),
	kinematics_(new Kinematics()){	
	
	TiXmlDocument xml_doc;
	xml_doc.LoadFile(robot_file);
	
	TiXmlElement *robot_xml = xml_doc.FirstChildElement("robot");
	initLinks(robot_xml);
	initJoints(robot_xml);
	
	propagator_->setup(active_lower_joint_limits_,
			           active_upper_joint_limits_,
			           active_joint_velocity_limits_);
	
	kinematics_->setJointOrigins(active_joint_origins_);
	kinematics_->setLinkDimensions(active_link_dimensions_);
}

void Robot::test() {
	cout << "HELLLO IN ROBOT" << endl;
}

std::vector<fcl::OBB> Robot::createRobotCollisionStructuresPy(const std::vector<double> &joint_angles) {
	std::vector<fcl::OBB> collision_structures;
	createRobotCollisionStructures(joint_angles, collision_structures);
	return collision_structures;
}

void Robot::createRobotCollisionStructures(const std::vector<double> &joint_angles, std::vector<fcl::OBB> &collision_structures) {
	std::vector<fcl::AABB> link_aabbs;	
	for (size_t i = 0; i < active_link_dimensions_.size(); i++) {
	    link_aabbs.push_back(fcl::AABB(fcl::Vec3f(0.0, -active_link_dimensions_[i][1] / 2.0, -active_link_dimensions_[i][2] / 2.0),
	                                   fcl::Vec3f(active_link_dimensions_[i][0], active_link_dimensions_[i][1] / 2.0, active_link_dimensions_[i][2] / 2.0)));
	}
	
	int n = 0;	
	for (size_t i = 0; i < joint_angles.size(); i++) {		
	    const std::pair<fcl::Vec3f, fcl::Matrix3f> pose_link_n = kinematics_->getPoseOfLinkN(joint_angles, n);
	    fcl::OBB obb;	    
	    fcl::convertBV(link_aabbs[i], fcl::Transform3f(pose_link_n.second, pose_link_n.first), obb);	    
	    collision_structures.push_back(obb);
	    n++;
	 }
}

std::vector<fcl::CollisionObject> Robot::createRobotCollisionObjectsPy(const std::vector<double> &joint_angles) {
	std::vector<fcl::CollisionObject> collision_objects;
	createRobotCollisionObjects(joint_angles, collision_objects);
	return collision_objects;
}

void Robot::createRobotCollisionObjects(const std::vector<double> &joint_angles, std::vector<fcl::CollisionObject> &collision_objects) {
    std::vector<fcl::CollisionObject> vec;
    std::vector<fcl::AABB> link_aabbs;
    for (size_t i = 0; i < active_link_dimensions_.size(); i++) {
        link_aabbs.push_back(fcl::AABB(fcl::Vec3f(0.0, -active_link_dimensions_[i][1] / 2.0, -active_link_dimensions_[i][2] / 2.0),
                                       fcl::Vec3f(active_link_dimensions_[i][0], active_link_dimensions_[i][1] / 2.0, active_link_dimensions_[i][2] / 2.0)));
    }
    
    //fcl::AABB link_aabb(fcl::Vec3f(0.0, -0.0025, -0.0025), fcl::Vec3f(1.0, 0.0025, 0.0025));
    int n = 0;
    for (size_t i = 0; i < joint_angles.size(); i++) {
        const std::pair<fcl::Vec3f, fcl::Matrix3f> pose_link_n = kinematics_->getPoseOfLinkN(joint_angles, n);
        fcl::Box* box = new fcl::Box();
        fcl::Transform3f box_tf;
        fcl::Transform3f trans(pose_link_n.second, pose_link_n.first);        
        fcl::constructBox(link_aabbs[i], trans, *box, box_tf);        
        collision_objects.push_back(fcl::CollisionObject(boost::shared_ptr<fcl::CollisionGeometry>(box), box_tf));
        n++;
    }
    
}

void Robot::getPositionOfLinkN(const std::vector<double> &joint_angles, const int &n, std::vector<double> &position) {
	kinematics_->getPositionOfLinkN(joint_angles, n, position);
}
    	    
void Robot::getEndEffectorPosition(const std::vector<double> &joint_angles, std::vector<double> &end_effector_position) {
	kinematics_->getEndEffectorPosition(joint_angles, end_effector_position);
}

void Robot::setupViewer(std::string model_file, std::string environment_file) {
	propagator_->setup_viewer(model_file, environment_file);
}

void Robot::updateViewerValues(const std::vector<double> &current_joint_values,
                               const std::vector<double> &current_joint_velocities) {
	propagator_->update_robot_values(current_joint_values, current_joint_velocities, nullptr);
}

void Robot::propagate(std::vector<double> &current_state,
		              std::vector<double> &control_input,
		              std::vector<double> &control_error,
		              double simulation_step_size,
		              double duration,
		              std::vector<double> &result) {
	std::vector<double> current_joint_values;
	std::vector<double> current_joint_velocities;
	
	for (size_t i = 0; i < current_state.size() / 2; i++) {
		current_joint_values.push_back(current_state[i]);
		current_joint_velocities.push_back(current_state[i + current_state.size() / 2]);
	}
	
	
	propagator_->propagate_nonlinear(current_joint_values,
			                         current_joint_velocities,
			                         control_input,
			                         control_error,
			                         simulation_step_size,
			                         duration,
			                         result);
}

void Robot::setState(std::vector<double> &joint_values, std::vector<double> &joint_velocities) {
	robot_state_.joint_values = joint_values;
	robot_state_.joint_velocities = joint_velocities;
}

void Robot::getState(std::vector<double> &state) {
	for (auto &s: robot_state_.joint_values) {
		state.push_back(s);
	}
	
	for (auto &s: robot_state_.joint_velocities) {
		state.push_back(s);
	}
}

unsigned int Robot::get_link_index(std::string &link_name) {	
	for (size_t i = 0; i < link_names_.size(); i++) {
		if (link_name == link_names_[i]) {
			return i;
		}
	}
	
	return 0;
}

void Robot::getLinkNames(std::vector<std::string> &link_names) {
	for (auto &name: link_names_) {
		link_names.push_back(name);
	}	
}

void Robot::getJointNames(std::vector<std::string> &joint_names) {
	for (auto &name: joint_names_) {
		joint_names.push_back(name);
	}	
}

void Robot::getLinkMasses(std::vector<std::string> &link, std::vector<double> &link_masses) {
	int index = 0;
	for (size_t i = 0; i < link.size(); i++) {
		index = get_link_index(link[i]);
		link_masses.push_back(link_masses_[index]);
	}	
}

void Robot::getLinkInertias(std::vector<std::string> &link, std::vector<std::vector<double>> &inertias) {
	double index = 0;
	for (size_t i = 0; i < link.size(); i++) {
		index = get_link_index(link[i]);
		inertias.push_back(link_inertia_matrices_[index]);
	}
}

void Robot::getActiveLinkDimensions(std::vector<std::vector<double>> &dimensions) {
	for (auto &k: active_link_dimensions_) {
		dimensions.push_back(k);
	}
	/**for (size_t i = 0; i < link_names_.size(); i++) {
		for (size_t j = 0; j < active_link_names_.size(); j ++) {
			if (link_names_[i] == active_link_names_[j]) {
				dimensions.push_back(link_dimensions_[i]);
			}
		}
	}*/	
}

void Robot::getLinkDimension(std::vector<std::string> &link, std::vector<std::vector<double>> &dimension) {
	cout << "HELLO" << endl;
	int index = 0;
	for (size_t i = 0; i < link.size(); i++) {
		cout << "link: " << link[i] << endl;		
		index = get_link_index(link[i]);
		cout << "index: " << index << endl;
		cout << "dim: ";
		for (auto &k: link_dimensions_[index]) {
			cout << k << ",";
		}
		cout << endl;
		dimension.push_back(link_dimensions_[index]);
	}
}

void Robot::getLinkPose(std::vector<std::string> &link, std::vector<std::vector<double>> &pose) {
	int index = 0;
	for (size_t i = 0; i < link.size(); i++) {
		index = get_link_index(link[i]);		
		pose.push_back(link_origins_[index]);
	}
}

void Robot::getLinkInertialPose(std::vector<std::string> &link, std::vector<std::vector<double>> &pose) {
	int index = 0;
	for (size_t i = 0; i < link.size(); i++) {
		index = get_link_index(link[i]);
		pose.push_back(link_inertia_origins_[index]);
	}
}

void Robot::getActiveJoints(std::vector<std::string> &joints) {
	for (auto &joint: active_joints_) {
		joints.push_back(joint);
	}
}

void Robot::getJointLowerPositionLimits(std::vector<std::string> &joints, std::vector<double> &joint_limits) {
	int index = 0;
	for (size_t i = 0; i < joints.size(); i++) {
		for (size_t j = 0; j < joint_names_.size(); j++) {
			if (joints[i] == joint_names_[j]) {
				joint_limits.push_back(lower_joint_limits_[j]);
			}
		}
	}
}

void Robot::getJointUpperPositionLimits(std::vector<std::string> &joints, std::vector<double> &joint_limits) {
	int index = 0;
	for (size_t i = 0; i < joints.size(); i++) {
		for (size_t j = 0; j < joint_names_.size(); j++) {
			if (joints[i] == joint_names_[j]) {
				joint_limits.push_back(upper_joint_limits_[j]);
			}
		}
	}
}

void Robot::getJointVelocityLimits(std::vector<std::string> &joints, std::vector<double> &joint_limits) {
	int index = 0;
	for (size_t i = 0; i < joints.size(); i++) {
		for (size_t j = 0; j < joint_names_.size(); j++) {
			if (joints[i] == joint_names_[j]) {
				joint_limits.push_back(joint_velocity_limits_[j]);
			}
		}
	}
}

void Robot::getJointTorqueLimits(std::vector<std::string> &joints, std::vector<double> &joint_limits) {
	int index = 0;
	for (size_t i = 0; i < joints.size(); i++) {
		for (size_t j = 0; j < joint_names_.size(); j++) {
			if (joints[i] == joint_names_[j]) {
				joint_limits.push_back(joint_torque_limits_[j]);
			}
		}
	}
}

void Robot::getJointType(std::vector<std::string> &joint, std::vector<std::string> &type) {
	int index = 0;
	for (size_t i = 0; i < joint.size(); i++) {
		for (size_t j = 0; j < joint_names_.size(); j++) {
			if (joint[i] == joint_names_[j]) {
				type.push_back(joint_types_[j]);
			}
		}
	}	
}

void Robot::getJointOrigin(std::vector<std::string> &joints, std::vector<std::vector<double>> &origins) {
	for (size_t i = 0; i < joints.size(); i++) {
		for (size_t j = 0; j < joint_names_.size(); j++) {
			if (joints[i] == joint_names_[j]) {
				origins.push_back(joint_origins_[j]);
			}
		}
	}
}

void Robot::getJointAxis(std::vector<std::string> &joints, std::vector<std::vector<int>> &axis) {
	for (size_t i = 0; i < joints.size(); i++) {
		for (size_t j = 0; j < joint_names_.size(); j++) {
			if (joints[i] == joint_names_[j]) { 
				axis.push_back(joint_axes_[j]);
			}
		}
	}
}

int Robot::getDOF() {
	return active_joints_.size();
}

BOOST_PYTHON_MODULE(librobot) {
    using namespace boost::python;
    
    class_<std::vector<double> > ("v_double")
             .def(vector_indexing_suite<std::vector<double> >());
    
    class_<std::vector<int> > ("v_int")
             .def(vector_indexing_suite<std::vector<int> >());
    
    class_<std::vector<std::vector<double> > > ("v2_double")
             .def(vector_indexing_suite<std::vector<std::vector<double> > >());
    
    class_<std::vector<std::vector<int> > > ("v2_int")
             .def(vector_indexing_suite<std::vector<std::vector<int> > >());
    
    class_<std::vector<std::string> > ("v_string")
                 .def(vector_indexing_suite<std::vector<std::string> >());   
    
    
    class_<fcl::OBB>("OBB");
    class_<fcl::CollisionObject>("CollisionObject", init<const boost::shared_ptr<fcl::CollisionGeometry>, const fcl::Transform3f>());
    to_python_converter<std::vector<fcl::OBB, std::allocator<fcl::OBB> >, VecToList<fcl::OBB> >();
    to_python_converter<std::vector<fcl::CollisionObject, std::allocator<fcl::CollisionObject> >, VecToList<fcl::CollisionObject> >();
    
    class_<Robot, boost::shared_ptr<Robot>>("Robot", init<std::string>())
                        .def("getLinkNames", &Robot::getLinkNames)
                        .def("getLinkDimension", &Robot::getLinkDimension)
                        .def("getActiveLinkDimensions", &Robot::getActiveLinkDimensions)
                        .def("getLinkMasses", &Robot::getLinkMasses)
                        .def("getLinkPose", &Robot::getLinkPose)
                        .def("getLinkInertialPose", &Robot::getLinkInertialPose)
                        .def("getLinkInertias", &Robot::getLinkInertias)
                        .def("getJointNames", &Robot::getJointNames)
                        .def("getActiveJoints", &Robot::getActiveJoints)
                        .def("getJointType", &Robot::getJointType)
                        .def("getJointOrigin", &Robot::getJointOrigin)
                        .def("getJointAxis", &Robot::getJointAxis)
                        .def("propagate", &Robot::propagate)
                        .def("createRobotCollisionStructures", &Robot::createRobotCollisionStructuresPy)
                        .def("createRobotCollisionObjects", &Robot::createRobotCollisionObjectsPy)
                        .def("getEndEffectorPosition", &Robot::getEndEffectorPosition)
                        .def("setupViewer", &Robot::setupViewer)
                        .def("updateViewerValues", &Robot::updateViewerValues)
                        .def("test", &Robot::test)
                        .def("getDOF", &Robot::getDOF)
                        //.def("setup", &Integrate::setup)                        
    ;
}

}
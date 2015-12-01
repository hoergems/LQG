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
		    }
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
	}
	
	return true;
}

std::vector<double> Robot::process_origin_(TiXmlElement *xml) {
	TiXmlElement *origin_xml = xml->FirstChildElement("origin");
	std::vector<double> origin;
	if (origin_xml) {				
		if (origin_xml->Attribute("xyz")) {
			const char* xyz_str = origin_xml->Attribute("xyz");
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
		}				
	}
	if (origin.size() == 3) {
		return origin;
	}
	else {
		std::vector<double> orig({0.0, 0.0, 0.0});
		return orig;
	}
}

Robot::Robot(std::string robot_file):
	robot_file_(robot_file),
	model_(new urdf::Model()),
	joints_(),
	link_names_(),
	active_links_(),
	link_masses_(),
	link_inertia_origins_(){	
	
	TiXmlDocument xml_doc;
	xml_doc.LoadFile(robot_file);
	
	TiXmlElement *robot_xml = xml_doc.FirstChildElement("robot");
	initLinks(robot_xml);	
	
	sleep(10);
	
	if (!model_->initFile(robot_file)){
		cout << "Failed to parse urdf file" << endl;
	}
	else {
		cout << "Successfully parsed urdf file" << endl;
	}
	
	std::vector<boost::shared_ptr<urdf::Link>> links;
	boost::shared_ptr<const urdf::Link> root = model_->getRoot();	
	while (root->child_links.size() != 0) {
		joints_.push_back(root->child_joints[0]);
		root = root->child_links[0];
	}	
}

void Robot::getLinkNames(std::vector<std::string> &link_names) {
	boost::shared_ptr<const urdf::Link> root = model_->getRoot();
	link_names.push_back(root->name);
	while (root->child_links.size() != 0) {
		root = root->child_links[0];
		link_names.push_back(root->name);
	}	
}

void Robot::getJointNames(std::vector<std::string> &joint_names) {
	boost::shared_ptr<const urdf::Link> root = model_->getRoot();
	boost::shared_ptr<urdf::Joint> joint;
	while (root->child_joints.size() != 0) {
		joint = root->child_joints[0];
		root = root->child_links[0];
		joint_names.push_back(joint->name);
	}	
}

void Robot::getLinkMasses(std::vector<std::string> &link, std::vector<double> &link_masses) {
	std::vector<boost::shared_ptr<urdf::Link>> links;	
	model_->getLinks(links);	
	for (size_t i = 0; i < link.size(); i++) {
		for (size_t j = 0; j < links.size(); j ++) {
			if (link[i] == links[j]->name) {
				if (links[j]->inertial != nullptr) {			
					link_masses.push_back(links[j]->inertial->mass);
				}
				else{
					link_masses.push_back(0.0);
				}
			}
		}
	}
}

void Robot::getLinkInertias(std::vector<std::string> &link, std::vector<std::vector<double>> &inertias) {
	std::vector<boost::shared_ptr<urdf::Link>> links;
	model_->getLinks(links);
	for (size_t i = 0; i < link.size(); i++) {
		for (size_t j = 0; j < links.size(); j ++) {
			if (links[j]->name == link[i]) {				
				if (links[i]->inertial != nullptr) {					
					std::vector<double> inert;
					inert.push_back(links[i]->inertial->ixx);
					inert.push_back(links[i]->inertial->ixy);
					inert.push_back(links[i]->inertial->ixz);
					inert.push_back(links[i]->inertial->iyy);
					inert.push_back(links[i]->inertial->iyz);
					inert.push_back(links[i]->inertial->izz);
					inertias.push_back(inert);
				}
				else {
					std::vector<double> inert;
					inert.push_back(0.0);
				    inert.push_back(0.0);
					inert.push_back(0.0);
					inert.push_back(0.0);
					inert.push_back(0.0);
					inert.push_back(0.0);
					inertias.push_back(inert);
				}
			}
		}
	}
}

void Robot::getActiveLinkDimensions(std::vector<std::vector<double>> &dimensions) {
	std::vector<boost::shared_ptr<urdf::Link>> links;
	model_->getLinks(links);
	for (auto &link: links) {
		if (link->collision != nullptr) {
			std::vector<double> dim;
			boost::shared_ptr<urdf::Box> box = boost::static_pointer_cast<urdf::Box>(link->collision->geometry);
			dim.push_back(box->dim.x);
			dim.push_back(box->dim.y);
			dim.push_back(box->dim.z);
			dimensions.push_back(dim);		    
		}
	}
}

void Robot::getLinkDimension(std::vector<std::string> &link, std::vector<std::vector<double>> &dimension) {
	std::vector<boost::shared_ptr<urdf::Link>> links;
	model_->getLinks(links);
	for (size_t i = 0; i < link.size(); i++) {
		for (size_t j = 0; j < links.size(); j ++) {
			if (links[j]->name == link[i]) {
				if (links[j]->collision != nullptr) {
					std::vector<double> dim;
					boost::shared_ptr<urdf::Box> box = boost::static_pointer_cast<urdf::Box>(links[j]->collision->geometry);
					dim.push_back(box->dim.x);
					dim.push_back(box->dim.y);
					dim.push_back(box->dim.z);
					dimension.push_back(dim);
				}
				else {
					std::vector<double> dim;
					dimension.push_back(dim);
				}
			}
		}
	}
}

void Robot::getLinkPose(std::vector<std::string> &link, std::vector<std::vector<double>> &pose) {
	std::vector<boost::shared_ptr<urdf::Link>> links;
	model_->getLinks(links);
	for (size_t i = 0; i < link.size(); i++) {
		for (size_t j = 0; j < links.size(); j ++) {
			if (links[j]->name == link[i]) {
				if (links[j]->collision != nullptr) {
					double roll, pitch, yaw;
					std::vector<double> pse; 
					pse.push_back(links[j]->collision->origin.position.x);
					pse.push_back(links[j]->collision->origin.position.y);
					pse.push_back(links[j]->collision->origin.position.z);
					
					links[j]->collision->origin.rotation.getRPY(roll, pitch, yaw);
					pse.push_back(roll);
					pse.push_back(pitch);
					pse.push_back(yaw);					
					
					pose.push_back(pse);
				}
				else {
					std::vector<double> pse; 
					pose.push_back(pse);
				}
			}
		}
	}
}

void Robot::getLinkInertialPose(std::vector<std::string> &link, std::vector<std::vector<double>> &pose) {
	std::vector<boost::shared_ptr<urdf::Link>> links;
	model_->getLinks(links);
	for (size_t i = 0; i < link.size(); i++) {
		for (size_t j = 0; j < links.size(); j ++) {
			if (links[j]->name == link[i]) {
				if (links[j]->inertial != nullptr) {
					double roll, pitch, yaw;
					std::vector<double> pse;
					pse.push_back(links[i]->inertial->origin.position.x);
					pse.push_back(links[i]->inertial->origin.position.y);
					pse.push_back(links[i]->inertial->origin.position.z);
					
					links[j]->inertial->origin.rotation.getRPY(roll, pitch, yaw);
					pse.push_back(roll);
					pse.push_back(pitch);
					pse.push_back(yaw);
					pose.push_back(pse);
				}
				else {
					std::vector<double> pse;
					for (size_t k = 0; k < 6; k++) {
						pse.push_back(0.0);
					}
					pose.push_back(pse);
				}
			}
		}
	}
}

void Robot::getActiveJoints(std::vector<std::string> &joints) {
	for (auto &joint: joints_) {
		if (joint->type != urdf::Joint::FIXED) {
			joints.push_back(joint->name);
		}
	}
}

void Robot::getJointType(std::vector<std::string> &joint, std::vector<std::string> &type) {
	for (size_t i = 0; i < joint.size(); i++) {
		for (size_t j = 0; j < joints_.size(); j++) {
			if (joint[i] == joints_[j]->name) {
				if (joints_[j]->type == urdf::Joint::FIXED) {
					type.push_back("fixed");
				}
				else if (joints_[j]->type == urdf::Joint::REVOLUTE) {
					type.push_back("revolute");
				}
			}
		}
	}	
}

void Robot::getJointOrigin(std::vector<std::string> &joints, std::vector<std::vector<double>> &origins) {
	for (size_t i = 0; i < joints.size(); i++) {
		for (size_t j = 0; j < joints_.size(); j++) {
			if (joints[i] == joints_[j]->name) {
				std::vector<double> orig;
				orig.push_back(joints_[j]->parent_to_joint_origin_transform.position.x);
				orig.push_back(joints_[j]->parent_to_joint_origin_transform.position.y);
				orig.push_back(joints_[j]->parent_to_joint_origin_transform.position.z);
				
				double roll, pitch, yaw;
				joints_[j]->parent_to_joint_origin_transform.rotation.getRPY(roll, pitch, yaw);
				orig.push_back(roll);
				orig.push_back(pitch);
				orig.push_back(yaw);
				origins.push_back(orig);
			}
		}
	}
}

void Robot::getJointAxis(std::vector<std::string> &joints, std::vector<std::vector<int>> &axis) {
	for (size_t i = 0; i < joints.size(); i++) {
		for (size_t j = 0; j < joints_.size(); j++) {
			if (joints[i] == joints_[j]->name) {
				std::vector<int> ax;
				ax.push_back(joints_[j]->axis.x);
				ax.push_back(joints_[j]->axis.y);
				ax.push_back(joints_[j]->axis.z);
				axis.push_back(ax);
			}
		}
	}
}

BOOST_PYTHON_MODULE(librobot) {
    using namespace boost::python;
    
    class_<std::vector<double> > ("v_double")
             .def(vector_indexing_suite<std::vector<double> >());
    
    class_<std::vector<std::vector<double> > > ("v2_double")
             .def(vector_indexing_suite<std::vector<std::vector<double> > >());
    
    class_<std::vector<std::string> > ("v_string")
                 .def(vector_indexing_suite<std::vector<std::string> >());
    
    class_<Robot>("Robot", init<std::string>())
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
                        //.def("setup", &Integrate::setup)                        
    ;
}

}
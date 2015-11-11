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

Robot::Robot(std::string robot_file):
	robot_file_(robot_file),
	model_(new urdf::Model()),
	joints_(){	
	
	if (!model_->initFile(robot_file)){
		cout << "Failed to parse urdf file" << endl;
	}
	else {
		cout << "Successfully parsed urdf file" << endl;
	}
	
	std::vector<boost::shared_ptr<urdf::Link>> links;
	model_->getLinks(links);
	for (size_t i = 0; i < links.size(); i++) {
		if (links[i]->child_joints.size() != 0) {			
			joints_.push_back(links[i]->child_joints[0]);
		}		
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

void Robot::getJointAxis(std::vector<std::string> &joints, std::vector<std::vector<double>> &axis) {
	for (size_t i = 0; i < joints.size(); i++) {
		for (size_t j = 0; j < joints_.size(); j++) {
			if (joints[i] == joints_[j]->name) {
				std::vector<double> ax;
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
                        .def("getLinkMasses", &Robot::getLinkMasses)
                        .def("getLinkPose", &Robot::getLinkPose)
                        .def("getLinkInertialPose", &Robot::getLinkInertialPose)
                        .def("getLinkInertias", &Robot::getLinkInertias)
                        .def("getJointNames", &Robot::getJointNames)
                        .def("getJointType", &Robot::getJointType)
                        .def("getJointOrigin", &Robot::getJointOrigin)
                        .def("getJointAxis", &Robot::getJointAxis)
                        //.def("setup", &Integrate::setup)                        
    ;
}

}
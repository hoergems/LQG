#include "viewer_interface.hpp"
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

using std::cout;
using std::endl;

namespace shared {

ViewerInterface::ViewerInterface ():
	viewer_setup_(false),
	env_(),
	robot_(nullptr),
	urdf_loader_(),
	viewer_(new shared::RaveViewer()),
	particle_plot_limit_(50){
}

bool ViewerInterface::setupViewer(std::string model_file,
                                  std::string environment_file) {
	if (viewer_setup_) {
		return false;
    }
	
	model_file_ = model_file;	
	OpenRAVE::RaveInitialize(true);	
	OpenRAVE::RaveSetDebugLevel(OpenRAVE::Level_Error);
	env_ = OpenRAVE::RaveCreateEnvironment();
	env_->SetPhysicsEngine(nullptr);
	env_->SetCollisionChecker(nullptr);
	env_->StopSimulation();
	cout << "loading " << environment_file << endl;
	env_->Load(environment_file);	
	cout << "loaded environment" << endl;	
	OpenRAVE::KinBodyPtr robot_ptr = urdf_loader_->load(model_file, env_);
	cout << "loaded robot" << endl;
	env_->Add(robot_ptr, true);	
	std::vector<OpenRAVE::KinBodyPtr> bodies;
	env_->GetBodies(bodies);
	env_->StopSimulation();	
	for (auto &k: bodies) {
		cout << k->GetName() << endl;
	}
	
	OpenRAVE::RobotBasePtr robot = getRobot();	
	const std::vector<OpenRAVE::KinBody::LinkPtr> links(robot->GetLinks());	
	for (size_t i = 0; i < links.size(); i++) {			
		if (links[i]->GetName() == "world") {
			links[i]->SetStatic(true);
		}
		else if (links[i]->GetName() == "end_effector") {
			links[i]->Enable(false);
		}
	}
	
	
	//shared::RaveViewer viewer;
	viewer_->testView(env_);
	cout << "Initialized viewer" << endl;
	viewer_setup_ = true;
	return true;
	
}

void ViewerInterface::setViewerSize(int x, int y) {
	viewer_->setViewerSize(x, y);
}

void ViewerInterface::setBackgroundColor(double &r, double &g, double &b) {
	viewer_->setBackgroundColor(r, g, b);
}

void ViewerInterface::setCameraTransform(std::vector<double> &rot, std::vector<double> &trans) {
	viewer_->setCameraTransform(rot, trans);
}

void ViewerInterface::removePermanentParticles() {
	std::vector<OpenRAVE::KinBodyPtr> bodies;
    env_->GetBodies(bodies);
    // Remove the particle bodies from the scene
    std::string particle_string = "permanent_";
    for (auto &body: bodies) {		
    	if (body->GetName().find(particle_string) != std::string::npos) {			
    		env_->Remove(body);
    	}		
    }
}

void ViewerInterface::addPermanentParticles(const std::vector<std::vector<double>> &particle_joint_values,
		                                    const std::vector<std::vector<double>> &particle_colors) {	
	double num_plot = particle_joint_values.size();
	for (size_t i = 0; i < num_plot; i++) {
	    OpenRAVE::KinBodyPtr robot_ptr = urdf_loader_->load(model_file_, env_);
		std::string name = "permanent_robot_";
		name.append(std::to_string(i));
		robot_ptr->SetName(name);
	    env_->Add(robot_ptr, true);
	    std::vector<OpenRAVE::dReal> joint_vals;
	    for (auto &k: particle_joint_values[i]) {
	        joint_vals.push_back(k);
	    }
	        
	    joint_vals.push_back(0);
	        
	    robot_ptr->SetDOFValues(joint_vals);
	        
	    const std::vector<OpenRAVE::KinBody::LinkPtr> links = robot_ptr->GetLinks();
	    for (auto &link: links) {
	        const std::vector<OpenRAVE::KinBody::Link::GeometryPtr> link_geometries = link->GetGeometries();
	        for (auto &geometry: link_geometries) {
	            if (geometry->IsVisible()) {
	        	    OpenRAVE::Vector color(particle_colors[i][0], 
	        					           particle_colors[i][1], 
										   particle_colors[i][2], 
										   particle_colors[i][3]);
	        		geometry->SetDiffuseColor(color);
					geometry->SetAmbientColor(color);
	        		geometry->SetTransparency(0.75);
	        	}
	        }
	    }
	}
	
}

void ViewerInterface::setParticlePlotLimit(unsigned int particle_plot_limit) {
	particle_plot_limit_ = particle_plot_limit;
}

void ViewerInterface::updateRobotValues(const std::vector<double> &current_joint_values,
		                                const std::vector<double> &current_joint_velocities,
										const std::vector<std::vector<double>> &particle_joint_values,
										const std::vector<std::vector<double>> &particle_colors,
								        OpenRAVE::RobotBasePtr robot=nullptr) {	
	OpenRAVE::RobotBasePtr robot_to_use(nullptr);
	
	std::vector<OpenRAVE::KinBodyPtr> bodies;
	env_->GetBodies(bodies);
	std::string particle_string = "particle";
		
	// Remove the particle bodies from the scene	
	for (auto &body: bodies) {		
		if (body->GetName().find(particle_string) != std::string::npos) {			
			env_->Remove(body);
		}		
	}
			
	if (robot == nullptr) {
		robot_to_use = getRobot();
	}
	else {
		robot_to_use = robot;
	}
	if (robot_to_use == nullptr) {
		cout << "Propagator: Error: Environment or robot has not been initialized or passed as argument. Can't propagate the state" << endl;
		return;	
	}
	
	std::vector<OpenRAVE::KinBody::LinkPtr> links = robot_to_use->GetLinks();
	std::vector<OpenRAVE::KinBody::JointPtr> joints = robot_to_use->GetJoints();	
	std::vector<OpenRAVE::KinBody::LinkInfo> link_infos;
	std::vector<OpenRAVE::KinBody::JointInfo> joint_infos;	
	for (auto &link: links) {
		link_infos.push_back(link->GetInfo());
	}
	
	for (auto &joint: joints) {
		joint_infos.push_back(joint->GetInfo());
	}
	
	std::vector<OpenRAVE::dReal> newJointValues;
	for (size_t i = 0; i < current_joint_values.size(); i++) {		
		if (current_joint_values[i] < -M_PI) {
			newJointValues.push_back(2.0 * M_PI + current_joint_values[i]);
		}
		else if (current_joint_values[i] > M_PI) {
			newJointValues.push_back(-2.0 * M_PI + current_joint_values[i]);
		}
		else {
			newJointValues.push_back(current_joint_values[i]);
		}
	}	
		
	newJointValues.push_back(0);	
	robot_to_use->SetDOFValues(newJointValues);
	size_t num_plot = particle_plot_limit_;	
	if (particle_joint_values.size() < num_plot) {
		num_plot = particle_joint_values.size();
	}
	
	// Here we plot non-permanent particles
	for (size_t i = 0; i < num_plot; i++) {
		OpenRAVE::KinBodyPtr robot_ptr = urdf_loader_->load(model_file_, env_);
		std::string name = "particle_robot_";
		name.append(std::to_string(i));
		robot_ptr->SetName(name);
        env_->Add(robot_ptr, true);
        std::vector<OpenRAVE::dReal> joint_vals;
        for (auto &k: particle_joint_values[i]) {
        	joint_vals.push_back(k);
        }
        
        joint_vals.push_back(0);        
        robot_ptr->SetDOFValues(joint_vals);        
        const std::vector<OpenRAVE::KinBody::LinkPtr> links = robot_ptr->GetLinks();
        for (auto &link: links) {
        	const std::vector<OpenRAVE::KinBody::Link::GeometryPtr> link_geometries = link->GetGeometries();
        	for (auto &geometry: link_geometries) {
        		if (geometry->IsVisible()) {
        			OpenRAVE::Vector color(particle_colors[i][0], 
        					               particle_colors[i][1], 
										   particle_colors[i][2], 
										   particle_colors[i][3]);
        			geometry->SetDiffuseColor(color);
					geometry->SetAmbientColor(color);
        			geometry->SetTransparency(particle_colors[i][3]);
        		}
        	}
        }
	}
}

OpenRAVE::RobotBasePtr ViewerInterface::getRobot() {
    std::vector<OpenRAVE::KinBodyPtr> bodies;
    env_->GetBodies(bodies);
    for (auto &body: bodies) {
    	if (body->GetDOF() > 0) {
    		OpenRAVE::RobotBasePtr robot = boost::static_pointer_cast<OpenRAVE::RobotBase>(body);
    		return robot;
    	}    	
    }   
}

}
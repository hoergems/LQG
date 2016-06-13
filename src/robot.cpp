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
		
		// Joint dynamics
		TiXmlElement *dyn_xml = joint_xml->FirstChildElement("dynamics");
		double damping = 0.0;
		if (dyn_xml) {
			std::string damping_str(dyn_xml->Attribute("damping"));
			damping = boost::lexical_cast<double>(damping_str.c_str());
		}
		
		joint_dampings_.push_back(damping);
		
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
		
		shared::Link link;		
		link.active = false;
		//Link names
		std::string link_name(link_xml->Attribute("name"));
		link_names_.push_back(link_name);
		link.name = link_name;
		
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
		    	for (auto &k: ld) {
		    		link.link_dimensions.push_back(k);
		    	}		    	
		    }
		    else {
		    	link_dimensions_.push_back(link_dimension);
		    	active_link_dimensions_.push_back(link_dimension);
		    	for (auto &k: link_dimension) {
		    		link.link_dimensions.push_back(k);
		    	}		    	
		    }
		    
		    std::vector<double> link_origin = process_origin_(coll_xml);
		    link_origins_.push_back(link_origin);		      
		    for (auto &k: link_origin) {
		    	link.origin.push_back(k);
		    }		    
		}
		else {
			std::vector<double> ld({0.0, 0.0, 0.0});
			link_dimensions_.push_back(ld);
			for (auto &k: ld) {
				link.link_dimensions.push_back(k);
			}
			
		}
		
		//Link inertia
		TiXmlElement *ine = link_xml->FirstChildElement("inertial");
		
		if (ine) {
			
			// Link masses
			active_links_.push_back(link_name);
			link.active = true;
			
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
			link.mass = mass;
			
			//Inertia origins
			std::vector<double> inertia_origin = process_origin_(ine);
			link_inertia_origins_.push_back(inertia_origin);
			for (auto &k: inertia_origin) {
				link.inertia_origin.push_back(k);
			}
			
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
			link.inertials.push_back(ixx);
			link.inertials.push_back(ixy);
			link.inertials.push_back(ixz);
			link.inertials.push_back(iyy);
			link.inertials.push_back(iyz);
			link.inertials.push_back(izz);
			
			link_inertia_matrices_.push_back(inertia_vals);
		}
		else {			
			link_masses_.push_back(0.0);
			link.mass = 0.0;
			
			std::vector<double> origin({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
			link_inertia_origins_.push_back(origin);
			for (auto &k: origin) {
				link.inertia_origin.push_back(k);
			}
			
			std::vector<double> inert({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
			link_inertia_matrices_.push_back(inert);
			for (auto &k: inert) {
				link.inertials.push_back(k);
			}
		}
		
		links_.push_back(link);
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
	links_(),
	joints_(),
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
	enforce_constraints_(false),
	propagator_(new Propagator()),
	kinematics_(new Kinematics()),
	viewer_(nullptr){
	
#ifdef USE_URDF
	viewer_ = std::make_shared<shared::ViewerInterface>();
#endif
	TiXmlDocument xml_doc;
	xml_doc.LoadFile(robot_file);	
	TiXmlElement *robot_xml = xml_doc.FirstChildElement("robot");
	initLinks(robot_xml);
	initJoints(robot_xml);
	
	propagator_->setup(active_lower_joint_limits_,
			           active_upper_joint_limits_,
			           active_joint_velocity_limits_,
					   enforce_constraints_);
	kinematics_->setJointOrigins(joint_origins_);	
	kinematics_->setLinkDimensions(active_link_dimensions_);
	propagator_->getIntegrator()->setJointDamping(joint_dampings_);
}

void Robot::quatFromRPY(double &roll, double &pitch, double &yaw, std::vector<double> &quat) {
	double phi, the, psi;
	 
    phi = roll / 2.0;
	the = pitch / 2.0;
	psi = yaw / 2.0;
    double x = sin(phi) * cos(the) * cos(psi) - cos(phi) * sin(the) * sin(psi);
	double y = cos(phi) * sin(the) * cos(psi) + sin(phi) * cos(the) * sin(psi);
	double z = cos(phi) * cos(the) * sin(psi) - sin(phi) * sin(the) * cos(psi);
	double w = cos(phi) * cos(the) * cos(psi) + sin(phi) * sin(the) * sin(psi);
	
	double s = sqrt(x * x +
	                y * y +
	                z * z +
	                w * w);
	
	if (s == 0.0) {
	    x = 0.0;
	    y = 0.0;
	    z = 0.0;
	    w = 1.0;
	}
	else {
	    x /= s;
	    y /= s;
	    z /= s;
	    w /= s;
	}
	
	quat.push_back(x);
	quat.push_back(y);
	quat.push_back(z);
	quat.push_back(w);
}

bool Robot::constraintsEnforced() {
	return enforce_constraints_;
}

void Robot::enforceConstraints(bool enforce) {
	enforce_constraints_ = enforce;
	propagator_->enforce_constraints(enforce_constraints_);
}

void Robot::test() {
	cout << "HELLLO IN ROBOT" << endl;
}

std::vector<std::shared_ptr<fcl::CollisionObject>> Robot::createRobotCollisionObjectsPy(const std::vector<double> &joint_angles) {
	std::vector<std::shared_ptr<fcl::CollisionObject>> collision_objects;	
	createRobotCollisionObjects(joint_angles, collision_objects);	
	return collision_objects;
}

std::vector<std::shared_ptr<fcl::CollisionObject>> 
Robot::createEndEffectorCollisionObjectPy(const std::vector<double> &joint_angles) {
	std::vector<std::shared_ptr<fcl::CollisionObject>> collision_objects;
	createEndEffectorCollisionObject(joint_angles, collision_objects);
	return collision_objects;
}

void Robot::initCollisionObjects() {
	// Init the link collision objects
	std::vector<fcl::AABB> link_aabbs;
	for (size_t i = 0; i < active_link_dimensions_.size(); i++) {
	    /**link_aabbs.push_back(fcl::AABB(fcl::Vec3f(0.0, 
	        		                              -active_link_dimensions_[i][1] / 2.0, 
											      -active_link_dimensions_[i][2] / 2.0),
	                                   fcl::Vec3f(active_link_dimensions_[i][0], 
	                                    		  active_link_dimensions_[i][1] / 2.0, 
											      active_link_dimensions_[i][2] / 2.0)));*/
		link_aabbs.push_back(fcl::AABB(fcl::Vec3f(-active_link_dimensions_[i][0] / 2.0, 
			        		                      -active_link_dimensions_[i][1] / 2.0, 
											      -active_link_dimensions_[i][2] / 2.0),
			                           fcl::Vec3f(active_link_dimensions_[i][0] / 2.0, 
			                                      active_link_dimensions_[i][1] / 2.0, 
												  active_link_dimensions_[i][2] / 2.0)));
	}
	for (size_t i = 0; i < active_link_dimensions_.size(); i++) {
		fcl::Box* box = new fcl::Box();
		fcl::Transform3f box_tf;
		fcl::Transform3f trans;
		fcl::constructBox(link_aabbs[i], trans, *box, box_tf);
		collision_objects_.push_back(std::make_shared<fcl::CollisionObject>(boost::shared_ptr<fcl::CollisionGeometry>(box), box_tf));
	}
	
	// Init the end-effector collision object
	fcl::AABB aabb(fcl::Vec3f(0.0,
				              -active_link_dimensions_[active_link_dimensions_.size() - 1][1] / 2.0, 
			                  -active_link_dimensions_[active_link_dimensions_.size() - 1][2] / 2.0),
				   fcl::Vec3f(0.001,
						      active_link_dimensions_[active_link_dimensions_.size() - 1][1] / 2.0, 
						      active_link_dimensions_[active_link_dimensions_.size() - 1][2] / 2.0));
	fcl::Box* box = new fcl::Box();
    fcl::Transform3f box_tf;
    fcl::Transform3f trans;
    fcl::constructBox(aabb, trans, *box, box_tf);
    collision_objects_.push_back(std::make_shared<fcl::CollisionObject>(boost::shared_ptr<fcl::CollisionGeometry>(box), box_tf));
}

void Robot::createRobotCollisionObjects(const std::vector<double> &joint_angles, 
		                                std::vector<std::shared_ptr<fcl::CollisionObject>> &collision_objects) {
	Eigen::MatrixXd res = Eigen::MatrixXd::Identity(4, 4);
	res(0, 3) = joint_origins_[0][0];
	res(1, 3) = joint_origins_[0][1];
	res(2, 3) = joint_origins_[0][2];
	
	for (size_t i = 0; i < joint_angles.size(); i++) {		
		//const std::pair<fcl::Vec3f, fcl::Matrix3f> pose_link_n = kinematics_->getPoseOfLinkN(joint_angles, i);
		res = kinematics_->getPoseOfLinkN(joint_angles[i], res, i);	
		
		fcl::Matrix3f trans_matrix(res(0,0), res(0,1), res(0,2),
				                   res(1,0), res(1,1), res(1,2),
								   res(2,0), res(2,1), res(2,2));
		fcl::Vec3f trans_vec(res(0,3), res(1,3), res(2,3));
		
		fcl::Transform3f trans(trans_matrix, trans_vec); 
		
		
		fcl::AABB link_aabb(fcl::Vec3f(0.0, 
					        		   -active_link_dimensions_[i][1] / 2.0, 
				                       -active_link_dimensions_[i][2] / 2.0),
					        fcl::Vec3f(active_link_dimensions_[i][0], 
					                   active_link_dimensions_[i][1] / 2.0, 
			                           active_link_dimensions_[i][2] / 2.0));
		
		fcl::Box* box = new fcl::Box();  
		fcl::Transform3f box_tf;		
		fcl::constructBox(link_aabb, trans, *box, box_tf);		
		std::shared_ptr<fcl::CollisionObject> coll_obj = 
				std::make_shared<fcl::CollisionObject>(boost::shared_ptr<fcl::CollisionGeometry>(box), box_tf);
		//fcl::CollisionObject coll_obj(geom, box_tf);
		
		collision_objects.push_back(coll_obj);		
		
	    /**const std::pair<fcl::Vec3f, fcl::Matrix3f> pose_link_n = kinematics_->getPoseOfLinkN(joint_angles, i);
	    cout << "pose trans" << i << ": " << pose_link_n.first << endl;
	    cout << "pose rot" << i << ": " << pose_link_n.second << endl;
	    fcl::Transform3f trans(pose_link_n.second, pose_link_n.first); 
	    fcl::Vec3f center(0.5, 0.0, 0.0);
	    //fcl::Transform3f trans_res = trans * fcl::Transform3f(collision_objects_[i]->getAABB().center());
	    fcl::Transform3f trans_res = trans * fcl::Transform3f(center);
	    cout << "trans " << i << ": " << trans_res.getTranslation() << endl;
	    fcl::Vec3f null_vec = fcl::Vec3f(0.0, 0.0, 0.0);
	    fcl::Vec3f res_vec = trans_res.transform(null_vec);
	    cout << "res vec " << i << ": " << res_vec<< endl;	    
	    collision_objects_[i]->setTransform(trans_res);
	    //collision_objects_[i]->computeAABB();
	    cout << "min " << collision_objects_[i]->getAABB().min_ << endl;
	    cout << "max " << collision_objects_[i]->getAABB().max_ << endl;
	    cout << endl;
        collision_objects.push_back(collision_objects_[i]);*/
	}	
}

void Robot::createEndEffectorCollisionObject(const std::vector<double> &joint_angles,
    	    		std::vector<std::shared_ptr<fcl::CollisionObject>> &collision_objects) {	
	const std::pair<fcl::Vec3f, fcl::Matrix3f> pose_ee = kinematics_->getPoseOfLinkN(joint_angles, active_link_dimensions_.size());
	fcl::Transform3f trans(pose_ee.second, pose_ee.first);
	fcl::Transform3f trans_res = trans * fcl::Transform3f(collision_objects_[collision_objects_.size() - 1]->getAABB().center());
	collision_objects_[collision_objects_.size() - 1]->setTransform(trans_res);
	collision_objects.push_back(collision_objects_[collision_objects_.size() - 1]);
	/**fcl::AABB aabb(fcl::Vec3f(0.0,
			                  -active_link_dimensions_[active_link_dimensions_.size() - 1][1] / 2.0, 
		                      -active_link_dimensions_[active_link_dimensions_.size() - 1][2] / 2.0),
			       fcl::Vec3f(0.01,
					          active_link_dimensions_[active_link_dimensions_.size() - 1][1] / 2.0, 
					          active_link_dimensions_[active_link_dimensions_.size() - 1][2] / 2.0));
	const std::pair<fcl::Vec3f, fcl::Matrix3f> pose_ee = kinematics_->getPoseOfLinkN(joint_angles, 3);
	fcl::Box* box = new fcl::Box();
	fcl::Transform3f box_tf;
	fcl::Transform3f trans(pose_ee.second, pose_ee.first);
	fcl::constructBox(aabb, trans, *box, box_tf);	
	collision_objects.push_back(std::make_shared<fcl::CollisionObject>(boost::shared_ptr<fcl::CollisionGeometry>(box), box_tf));*/	
}

void Robot::getPositionOfLinkN(const std::vector<double> &joint_angles, const int &n, std::vector<double> &position) {
	kinematics_->getPositionOfLinkN(joint_angles, n, position);
}
    	    
void Robot::getEndEffectorPosition(const std::vector<double> &joint_angles, std::vector<double> &end_effector_position) {
	kinematics_->getEndEffectorPosition(joint_angles, end_effector_position);
}


/****************************************
 * Viewer functions
 */
#ifdef USE_URDF
void Robot::setupViewer(std::string model_file, std::string environment_file) {	
	viewer_->setupViewer(model_file, environment_file);	
}

void Robot::addPermanentViewerParticles(const std::vector<std::vector<double>> &particle_joint_values,
							            const std::vector<std::vector<double>> &particle_colors) {
	assert(particle_joint_values.size() == particle_colors.size() &&  
		   "Number of particles must be the same as number of colours!");
	viewer_->addPermanentParticles(particle_joint_values,
			                       particle_colors);	
}

void Robot::removePermanentViewerParticles() {
	viewer_->removePermanentParticles();
}

void Robot::setParticlePlotLimit(unsigned int particle_plot_limit) {
	viewer_->setParticlePlotLimit(particle_plot_limit);
}

void Robot::updateViewerValues(const std::vector<double> &current_joint_values,
                               const std::vector<double> &current_joint_velocities,
							   const std::vector<std::vector<double>> &particle_joint_values,
							   const std::vector<std::vector<double>> &particle_colors) {		
	assert(particle_joint_values.size() == particle_colors.size() &&  
		   "Number of particles must be the same as number of colours!");
	// particle_color = {r, g, b, a}
	viewer_->updateRobotValues(current_joint_values, 
			                   current_joint_velocities,							  
							   particle_joint_values,
							   particle_colors,
							   nullptr);
}

void Robot::setViewerSize(int x, int y) {
	viewer_->setViewerSize(x, y);
}

void Robot::setViewerBackgroundColor(double r, double g, double b) {
	viewer_->setBackgroundColor(r, g, b);
}

void Robot::setViewerCameraTransform(std::vector<double> &rot, std::vector<double> &trans) {
	viewer_->setCameraTransform(rot, trans);
}

void Robot::setObstacleColor(std::string obstacle_name, 
 		                     std::vector<double> &diffuse_color, 
                             std::vector<double> &ambient_color) {
	viewer_->setObstacleColor(obstacle_name, diffuse_color, ambient_color);
}

void Robot::drawBox(std::string name, std::vector<double> &dimensions) {
	viewer_->addObstacle(name, dimensions);
}

void Robot::removeBox(std::string name) {
	viewer_->removeObstacle(name);
}

#endif

void Robot::addBoxObstacles(std::vector<std::shared_ptr<shared::Obstacle>> &obstacles) {
#ifdef USE_URDF
	for (size_t i = 0; i < obstacles.size(); i++) {
		std::vector<double> dims;		
		std::shared_ptr<shared::Obstacle> o = obstacles[i];
		std::shared_ptr<shared::BoxObstacle> o_box = std::static_pointer_cast<shared::BoxObstacle>(o);
		dims.push_back(o_box->pos_x_);
		dims.push_back(o_box->pos_y_);
		dims.push_back(o_box->pos_z_);
		dims.push_back(o_box->size_x_);
		dims.push_back(o_box->size_y_);
		dims.push_back(o_box->size_z_);
			
		std::string name = o_box->getName();
		viewer_->addObstacle(name,
					         dims);
	}
#endif
}

void Robot::addObstacles(std::vector<std::shared_ptr<shared::ObstacleWrapper>> &obstacles) {
#ifdef USE_URDF
	for (size_t i = 0; i < obstacles.size(); i++) {
		std::vector<double> dims;		
		std::shared_ptr<shared::Obstacle> o = std::static_pointer_cast<shared::Obstacle>(obstacles[i]);
		std::shared_ptr<shared::BoxObstacle> o_box = std::static_pointer_cast<shared::BoxObstacle>(o);
		dims.push_back(o_box->pos_x_);
		dims.push_back(o_box->pos_y_);
		dims.push_back(o_box->pos_z_);
		dims.push_back(o_box->size_x_);
		dims.push_back(o_box->size_y_);
		dims.push_back(o_box->size_z_);
		
		std::string name = o_box->getName();
		viewer_->addObstacle(name,
				             dims);
	}
#endif
}

void Robot::removeObstacles() {
#ifdef USE_URDF
	viewer_->removeObstacles();
#endif
}

bool Robot::propagate_linear(std::vector<double> &current_state,
    	    		         std::vector<double> &control_input,
    	    		         std::vector<double> &control_error,
    	    		         double duration,
    	    		         std::vector<double> &result) {
	return propagator_->propagate_linear(current_state,
			                             control_input,
			                             control_error,
			                             duration,
			                             result);	
}

void Robot::setGravityConstant(double gravity_constant) {
	propagator_->getIntegrator()->setGravityConstant(gravity_constant);
	//rbdl_interface_->setGravity(gravity_constant);
}

void Robot::setExternalForce(double f_x, 
		                     double f_y, 
							 double f_z,
							 double f_roll,
							 double f_pitch,
							 double f_yaw) {
	propagator_->getIntegrator()->setExternalForce(f_x, f_y, f_z, f_roll, f_pitch, f_yaw);	
}

void Robot::setAccelerationLimit(double accelerationLimit) {
	propagator_->getIntegrator()->setAccelerationLimit(accelerationLimit);
}

void Robot::getEndEffectorJacobian(const std::vector<double> &joint_angles, 
    	    		               std::vector<std::vector<double>> &ee_jacobian) {
	
	//std::vector<double> state;
	std::vector<double> state2;
	//for (auto &k: joint_angles) {
	//	state.push_back(k);		
	//}
	
	if (joint_angles.size() > getDOF()) {
	    for (size_t i = 0; i < joint_angles.size() / 2; i++) {
		    state2.push_back(joint_angles[i]);
	    }
	}
	else {
		for (size_t i = 0; i < joint_angles.size(); i++) {
		    state2.push_back(joint_angles[i]);
		}
	} 
	
	
	/**MatrixXd jacobian1(6, getDOF());
	propagator_->getIntegrator()->getRBDLInterface()->getEEJacobian(state, jacobian1);
	cout << "rbdl: " << endl;
	cout << jacobian1 << endl;*/
	/**MatrixXd jacobian2 = propagator_->get_ee_jacobian(state);
	cout << "prop: " << endl;
	cout << jacobian2 << endl;*/
	MatrixXd jacobian(6, getDOF());	
	kinematics_->getEEJacobian(state2, jacobian);
	//cout << "kin: " << endl;
	//cout << jacobian << endl;
	
	//MatrixXd jacobian = propagator_->get_ee_jacobian(state);	
	for (size_t i = 0; i < jacobian.rows(); i++) {
		std::vector<double> row;
		for (size_t j = 0; j < jacobian.cols(); j++) {			
			row.push_back(jacobian(i, j));
		}		
		ee_jacobian.push_back(row);
	}
}

void Robot::getEndEffectorVelocity(std::vector<double> &state,
    	    		               std::vector<double> &ee_velocity) {
	MatrixXd j = propagator_->get_ee_jacobian(state);
	MatrixXd vel(state.size() / 2, 1);
	for (size_t i = 0; i < state.size() / 2; i++) {
		vel(i, 0) = state[i + state.size() / 2];
	}
	
	MatrixXd res = j * vel;
	ee_velocity.clear();
	for (size_t i = 0; i < 6; i++) { 
		ee_velocity.push_back(res(i, 0));
	}	
}

bool Robot::propagate(std::vector<double> &current_state,
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
	
	
	return propagator_->propagate_nonlinear(current_joint_values,
			                                current_joint_velocities,
			                                control_input,
			                                control_error,
			                                simulation_step_size,
			                                duration,
			                                result);
}

bool Robot::propagate_first_order(std::vector<double> &current_state,
                                  std::vector<double> &control_input,
                                  std::vector<double> &control_error,
		                          std::vector<double> &nominal_state,
		                          std::vector<double> &nominal_control,
                                  double simulation_step_size,
                                  double duration,
                                  std::vector<double> &result) {
	std::vector<double> current_joint_values;
	std::vector<double> current_joint_velocities;
		
	for (size_t i = 0; i < current_state.size() / 2; i++) {
		current_joint_values.push_back(current_state[i]);
		current_joint_velocities.push_back(current_state[i + current_state.size() / 2]);
	}
	
	return propagator_->propagate_nonlinear_first_order(current_joint_values,
				                                         current_joint_velocities,
				                                         control_input,
				                                         control_error,
														 nominal_state,
														 nominal_control,
				                                         simulation_step_size,
				                                         duration,
				                                         result);
}

bool Robot::propagate_second_order(std::vector<double> &current_state,
        std::vector<double> &control_input,
        std::vector<double> &control_error,
		std::vector<double> &nominal_state,
		std::vector<double> &nominal_control,
        double simulation_step_size,
        double duration,
        std::vector<double> &result) {
	std::vector<double> current_joint_values;
	std::vector<double> current_joint_velocities;
		
	for (size_t i = 0; i < current_state.size() / 2; i++) {
		current_joint_values.push_back(current_state[i]);
		current_joint_velocities.push_back(current_state[i + current_state.size() / 2]);
	}
	
	return propagator_->propagate_nonlinear_second_order(current_joint_values,
				                                         current_joint_velocities,
				                                         control_input,
				                                         control_error,
														 nominal_state,
														 nominal_control,
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
}

void Robot::getLinkDimension(std::vector<std::string> &link, std::vector<std::vector<double>> &dimension) {	
	int index = 0;
	for (size_t i = 0; i < link.size(); i++) {				
		index = get_link_index(link[i]);
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

void Robot::getJointDamping(std::vector<std::string> &joints, std::vector<double> &damping) {
	int index = 0;
	for (size_t i = 0; i < joints.size(); i++) {
	    for (size_t j = 0; j < joint_names_.size(); j++) {
		    if (joints[i] == joint_names_[j]) {
				damping.push_back(joint_dampings_[j]);
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

std::vector<double> Robot::getProcessMatrices(std::vector<double> &x, 
                                              std::vector<double> &rho, 
				                              double t_e) {
	return propagator_->getIntegrator()->getProcessMatricesVec(x, rho, t_e);
}

BOOST_PYTHON_MODULE(librobot) {
    using namespace boost::python;
    
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
    
    info = boost::python::type_id<std::vector<std::string>>();
    const boost::python::converter::registration* reg_vstring = boost::python::converter::registry::query(info);
    if (reg_vstring == NULL || (*reg_vstring).m_to_python == NULL)  {    	
    	class_<std::vector<std::string> > ("v_string")
    	    .def(vector_indexing_suite<std::vector<std::string> >());
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
    	
    class_<fcl::OBB>("OBB");
    class_<fcl::CollisionObject>("CollisionObject", init<const boost::shared_ptr<fcl::CollisionGeometry>, const fcl::Transform3f>());
    to_python_converter<std::vector<fcl::OBB, std::allocator<fcl::OBB> >, VecToList<fcl::OBB> >();
    to_python_converter<std::vector<fcl::CollisionObject, std::allocator<fcl::CollisionObject> >, VecToList<fcl::CollisionObject> >();
    to_python_converter<std::vector<std::shared_ptr<fcl::CollisionObject>, std::allocator<std::shared_ptr<fcl::CollisionObject>> >, 
        VecToList<std::shared_ptr<fcl::CollisionObject>> >();
    register_ptr_to_python<std::shared_ptr<fcl::CollisionObject>>();
    
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
						.def("getJointDamping", &Robot::getJointDamping)
                        .def("getJointOrigin", &Robot::getJointOrigin)
                        .def("getJointAxis", &Robot::getJointAxis)
                        .def("propagate", &Robot::propagate)
						.def("propagate_first_order", &Robot::propagate_first_order)
						.def("propagate_second_order", &Robot::propagate_second_order)
                        //.def("createRobotCollisionStructures", &Robot::createRobotCollisionStructuresPy)
                        .def("createRobotCollisionObjects", &Robot::createRobotCollisionObjectsPy)
						.def("createEndEffectorCollisionObject", &Robot::createEndEffectorCollisionObjectPy)
                        .def("getEndEffectorPosition", &Robot::getEndEffectorPosition)						
                        .def("test", &Robot::test)
                        .def("getDOF", &Robot::getDOF)
						.def("getJointLowerPositionLimits", &Robot::getJointLowerPositionLimits)
						.def("getJointUpperPositionLimits", &Robot::getJointUpperPositionLimits)
						.def("getJointVelocityLimits", &Robot::getJointVelocityLimits)
						.def("getJointTorqueLimits", &Robot::getJointTorqueLimits)
						.def("enforceConstraints", &Robot::enforceConstraints)
						.def("constraintsEnforced", &Robot::constraintsEnforced)
						.def("setGravityConstant", &Robot::setGravityConstant)
						.def("setExternalForce", &Robot::setExternalForce)
						.def("setAccelerationLimit", &Robot::setAccelerationLimit)
						.def("getEndEffectorVelocity", &Robot::getEndEffectorVelocity)
						.def("getProcessMatrices", &Robot::getProcessMatrices)						
						.def("getEndEffectorJacobian", &Robot::getEndEffectorJacobian)
						.def("addObstacles", &Robot::addObstacles)
						.def("removeObstacles", &Robot::removeObstacles)
#ifdef USE_URDF
						.def("setupViewer", &Robot::setupViewer)						
						.def("updateViewerValues", &Robot::updateViewerValues)
						.def("setViewerSize", &Robot::setViewerSize)
						.def("setViewerBackgroundColor", &Robot::setViewerBackgroundColor)
					    .def("setViewerCameraTransform", &Robot::setViewerCameraTransform)
					    .def("addPermanentViewerParticles", &Robot::addPermanentViewerParticles)
					    .def("removePermanentViewerParticles", &Robot::removePermanentViewerParticles)					    
						.def("setObstacleColor", &Robot::setObstacleColor)
						.def("setParticlePlotLimit", &Robot::setParticlePlotLimit)
						.def("drawBox", &Robot::drawBox)
						.def("removeBox", &Robot::removeBox)
#endif
                        //.def("setup", &Integrate::setup)                        
    ;
}

}

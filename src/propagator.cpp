#include "propagator.hpp"

using std::cout;
using std::endl;

namespace shared {

Propagator::Propagator():
	damper_(nullptr),
	env_(nullptr),
	robot_(nullptr),
	show_viewer_(false),
	integrator_(new Integrate()){	
}

void Propagator::setup(double coulomb, 
		               double viscous,
		               std::vector<double> &jointsLowerPositionLimit,
		               std::vector<double> &jointsUpperPositionLimit,
		               std::vector<double> &jointsLowerVelLimit,
		               std::vector<double> &jointsUpperVelLimit) {
	damper_ = std::make_shared<TorqueDamper>(coulomb, viscous);
	for (size_t i = 0; i < jointsLowerPositionLimit.size(); i++) {
		jointsLowerPositionLimit_.push_back(jointsLowerPositionLimit[i]);
		jointsUpperPositionLimit_.push_back(jointsUpperPositionLimit[i]);
		jointsLowerVelocityLimit_.push_back(jointsLowerVelLimit[i]);
		jointsUpperVelocityLimit_.push_back(jointsUpperVelLimit[i]);
	}
}

bool Propagator::setup_py(std::string model_file,
		                  std::string environment_file,
		                  double coulomb, 
                          double viscous,
                          bool show_viewer) {
	damper_ = std::make_shared<TorqueDamper>(coulomb, viscous);
	
    OpenRAVE::RaveInitialize(true);    
    env_ = OpenRAVE::RaveCreateEnvironment();
    env_->Load(environment_file);
    
    const std::string module_str("or_urdf_plugin");
    if(!OpenRAVE::RaveLoadPlugin(module_str)) {
    	cout << "Failed to load the or_urdf_plugin." << endl;
    	return false;
    }
    	    
    OpenRAVE::ModuleBasePtr urdf_module = OpenRAVE::RaveCreateModule(env_, "URDF");
    const std::string cmdargs("");
    env_->AddModule(urdf_module, cmdargs);
    std::stringstream sinput, sout;
    sinput << "load " << model_file;
    if (!urdf_module->SendCommand(sout,sinput)) {
    	cout << "Failed to load URDF model" << endl;
    	return false;
    }
    	
    std::vector<OpenRAVE::KinBodyPtr> bodies;
    env_->GetBodies(bodies);
    env_->StopSimulation();
    
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
    
    
    
    /***** Create the physics engine *****/
    createPhysicsEngine(env_);
    
    /**
     * Set the joint limits
     */
    const std::vector<OpenRAVE::KinBody::JointPtr> joints(robot->GetJoints());
    for (size_t i = 0; i < joints.size(); i++) {
    	std::vector<OpenRAVE::dReal> lower_limit;
    	std::vector<OpenRAVE::dReal> upper_limit;        
    	joints[i]->GetLimits(lower_limit, upper_limit);
    	
    	jointsLowerPositionLimit_.push_back(lower_limit[0]);
    	jointsUpperPositionLimit_.push_back(upper_limit[0]);
    	
    	jointsLowerVelocityLimit_.push_back(-joints[i]->GetMaxVel());
    	jointsUpperVelocityLimit_.push_back(joints[i]->GetMaxVel());
    }
    
    if (show_viewer) {
    	show_viewer_ = true;
    	shared::RaveViewer viewer;
    	viewer.testView(env_);
    }
    return true;
}

void Propagator::createPhysicsEngine(OpenRAVE::EnvironmentBasePtr env) {
	const std::string engine = "ode";
	OpenRAVE::PhysicsEngineBasePtr physics_engine_ = OpenRAVE::RaveCreatePhysicsEngine(env, engine);
	    	    
	const OpenRAVE::Vector gravity({0.0, 0.0, -9.81});    
	physics_engine_->SetGravity(gravity);
	env->SetPhysicsEngine(physics_engine_);
}

OpenRAVE::RobotBasePtr Propagator::getRobot() {
    std::vector<OpenRAVE::KinBodyPtr> bodies;
    env_->GetBodies(bodies);
    for (auto &body: bodies) {
    	if (body->GetDOF() > 0) {
    		OpenRAVE::RobotBasePtr robot = boost::static_pointer_cast<OpenRAVE::RobotBase>(body);
    		return robot;
    	}    	
    }   
}


void Propagator::update_robot_values(const std::vector<double> &current_joint_values,
		                             const std::vector<double> &current_joint_velocities,									 
									 OpenRAVE::RobotBasePtr robot=nullptr) {
	OpenRAVE::RobotBasePtr robot_to_use(nullptr);
			
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
	
	std::vector<OpenRAVE::dReal> newJointValues;
		
	std::vector<OpenRAVE::dReal> newJointVelocities;
	for (size_t i = 0; i < current_joint_values.size(); i++) {
		newJointValues.push_back(current_joint_values[i]);		
	}
		
	for (size_t i = 0; i < current_joint_velocities.size(); i++) {
		newJointVelocities.push_back(current_joint_velocities[i]);		
	}
	
	newJointValues.push_back(0);
	newJointVelocities.push_back(0);
	
	robot_to_use->SetDOFValues(newJointValues);
    robot_to_use->SetDOFVelocities(newJointVelocities);	
}

void Propagator::propagate_linear(const std::vector<double> &x_dash,
                                  const std::vector<double> &u_dash,
								  const std::vector<double> &x_star_current,
								  const std::vector<double> &u_star_current,
								  const std::vector<double> &x_star_next,								  
                                  const std::vector<double> &control_error_vec,	   		                 
                                  const double duration,
                                  std::vector<double> &result,
                                  OpenRAVE::EnvironmentBasePtr environment=nullptr,
                                  OpenRAVE::RobotBasePtr robot=nullptr) {
	
	std::vector<double> x_dash_vec;
	std::vector<double> u_dash_vec;
	std::vector<double> x_star_vec;
	std::vector<double> u_star_vec;
	
	for (auto &k: x_dash) {
		x_dash_vec.push_back(k);
	}
	for (auto &k: u_dash) {
		u_dash_vec.push_back(k);
	}	
	for (auto &k: x_star_current) {
		x_star_vec.push_back(k);
	}
	for (auto &k: u_star_current) {
		u_star_vec.push_back(k);
	}
	
	VectorXd x_dash_eigen_vec(x_dash_vec.size());
	VectorXd u_dash_eigen_vec(u_dash_vec.size());
	VectorXd zeta_eigen_vec(control_error_vec.size());
	VectorXd x_star_next_eigen_vec(x_star_next.size());
	
	for (size_t i = 0; i < x_dash_vec.size(); i++) {
		x_dash_eigen_vec[i] = x_dash_vec[i];
	}
	for (size_t i = 0; i < u_dash_vec.size(); i++) {
		u_dash_eigen_vec[i] = u_dash_vec[i];
	}
	for (size_t i = 0; i < zeta_eigen_vec.size(); i++) {
		zeta_eigen_vec[i] = control_error_vec[i];
	}
	for (size_t i = 0; i < x_star_next.size(); i++) {
		x_star_next_eigen_vec[i] = x_star_next[i];
	}
	
	std::vector<MatrixXd> ABV;
	VectorXd x_dash_next_eigen_vec(x_dash_vec.size());
	VectorXd res_eigen_vec(x_dash_vec.size());
	integrator_->getProcessMatrices(x_star_vec, u_star_vec, duration, ABV);	
		
	x_dash_next_eigen_vec = ABV[0] * x_dash_eigen_vec + ABV[1] * u_dash_eigen_vec + ABV[2] * zeta_eigen_vec;
	res_eigen_vec = x_dash_next_eigen_vec + x_star_next_eigen_vec;
    
	for (size_t i = 0; i < res_eigen_vec.size(); i++) {
		result.push_back(res_eigen_vec[i]);
	}
	
}
	
void Propagator::propagate_nonlinear(const std::vector<double> &current_joint_values,
				                     const std::vector<double> &current_joint_velocities,
				                     std::vector<double> &control,
				                     std::vector<double> &control_error_vec,
				                     const double simulation_step_size,
				                     const double duration,
				                     std::vector<double> &result,				                     
				                     OpenRAVE::EnvironmentBasePtr environment=nullptr,
				                     OpenRAVE::RobotBasePtr robot=nullptr) {
	OpenRAVE::EnvironmentBasePtr env_to_use(nullptr);
	OpenRAVE::RobotBasePtr robot_to_use(nullptr);
	if (environment == nullptr) {
		env_to_use = env_;
	}
	else {
		env_to_use = environment;
	}
	
	if (robot == nullptr) {
		robot_to_use = getRobot();
	}
	else {
		robot_to_use = robot;
	}
	if (env_to_use == nullptr || robot_to_use == nullptr) {
		cout << "Propagator: Error: Environment or robot have not been initialized or passed as argument. Can't propagate the state" << endl;
		return;	
	}
	
	/**for (size_t i = 0; i < links.size(); i++) {
		cout << links[i]->GetName() << ": " << endl;
		cout << links[i]->GetLocalInertia() << endl;		
		cout << links[i]->GetLocalMassFrame() << endl;
		cout << "=================" << endl;
		cout << "global com: " << links[i]->GetGlobalCOM() << endl;
		cout << "mass: " << links[i]->GetInfo()._mass << endl;
		cout << "static: " << links[i]->GetInfo()._bStatic << endl;
		cout << "enabled: " << links[i]->GetInfo()._bIsEnabled << endl;
		cout << "=================" << endl;		
	}
	sleep(10);*/
	
	std::vector<double> current_vel;
	const std::vector<OpenRAVE::KinBody::JointPtr> joints(robot_to_use->GetJoints());
	std::vector<OpenRAVE::dReal> input_torques(control);
	std::vector<OpenRAVE::dReal> input_torques2(control);
	std::vector<OpenRAVE::dReal> damped_torques(control);
	std::vector<OpenRAVE::dReal> torque_error(control_error_vec);
	const std::vector<OpenRAVE::dReal> currentJointValues(current_joint_values);
	const std::vector<OpenRAVE::dReal> currentJointVelocities(current_joint_velocities);
	robot_to_use->SetDOFValues(currentJointValues);
	robot_to_use->SetDOFVelocities(currentJointVelocities);	
	int num_steps = duration / simulation_step_size;
	
	/**boost::timer t;	
	for (size_t i = 0; i < input_torques.size(); i++) {
		input_torques2[i] = 1.0;
	}
	robot_to_use->GetDOFVelocities(current_vel);
	damper_->damp_torques(current_vel,
		    		      damped_torques);
	
	
	for (unsigned int i = 0; i < num_steps; i++) {
	    robot_to_use->GetDOFVelocities(current_vel);
	    damper_->damp_torques(current_vel,
	    		    		  damped_torques);		
	    for (size_t k = 0; k < joints.size(); k++) {
	        input_torques[k] = input_torques2[k];// + damped_torques[k]; //+ torque_error[k];
	        
	        const std::vector<OpenRAVE::dReal> torques({input_torques[k]});	        
	        joints[k]->AddTorque(torques);
	    }
	    
	    
	    env_to_use->StepSimulation(simulation_step_size);
	    //env_to_use->StopSimulation();
	    if (show_viewer_) {
	    	usleep(1000000 * simulation_step_size);
	    }	    
	}
	cout << "elapsed1 " << t.elapsed() << endl;*/ 
	
	std::vector<OpenRAVE::dReal> newJointValues;
	std::vector<OpenRAVE::dReal> newJointVelocities;	
	//robot_to_use->GetDOFValues(newJointValues);
	//robot_to_use->GetDOFVelocities(newJointVelocities);
	
	std::vector<double> state;
	
	for (size_t i = 0; i < current_joint_values.size(); i++) {
		state.push_back(current_joint_values[i]);
	}
	for (size_t i = 0; i < current_joint_values.size(); i++) {
		state.push_back(current_joint_velocities[i]);
	}
	
	/**cout << "state: ";
	for (auto &k: state) {
		cout << k << ", ";
	}
	cout << endl;
	
	cout << "control: ";
	for (auto &k: input_torques2) {
		cout << k << ", ";
	}
	cout << endl;
	
	cout << "control error: ";
	for (auto &k: control_error_vec) {
		cout << k << ", ";
	}
	cout << endl;*/
	
	std::vector<double> integration_result;
	std::vector<double> inte_times({0.0, duration, simulation_step_size});	
	integrator_->do_integration(state, input_torques2, control_error_vec, inte_times, integration_result);
	
	/**cout << "res1: ";
	for (auto &k: newJointValues) {
		cout << k << ", ";
	}
	
	for (auto &k: newJointVelocities) {
		cout << k << ", ";
	}
	
	cout << endl;*/
	
	
	
	
	for (size_t i = 0; i < integration_result.size() / 2; i++) {
		newJointValues.push_back(integration_result[i]);		
	}
	
	for (size_t i = integration_result.size() / 2; i < integration_result.size(); i++) {
		newJointVelocities.push_back(integration_result[i]);		
	}
	
	//Enforce position and velocity limits
	for (unsigned int i = 0; i < newJointValues.size(); i++) {
	    if (newJointValues[i] < jointsLowerPositionLimit_[i]) {
		    newJointValues[i] = jointsLowerPositionLimit_[i];
		    newJointVelocities[i] = 0;
		}
		else if (newJointValues[i] > jointsUpperPositionLimit_[i]) {
		    newJointValues[i] = jointsUpperPositionLimit_[i];
		    newJointVelocities[i] = 0;
		}

		if (newJointVelocities[i] < jointsLowerVelocityLimit_[i]) {
		    newJointVelocities[i] = jointsLowerVelocityLimit_[i];
		}
		else if (newJointVelocities[i] > jointsUpperVelocityLimit_[i]) {
	        newJointVelocities[i] = jointsUpperVelocityLimit_[i];
		}
		        
	}
	
	for (size_t i = 0; i < joints.size(); i++) {
		result.push_back(newJointValues[i]);
	}
	
	for (size_t i = 0; i < joints.size(); i++) {
		result.push_back(newJointVelocities[i]);
	}
	
	if (show_viewer_) {
		newJointValues.push_back(0);
		newJointVelocities.push_back(0);
	    robot_to_use->SetDOFValues(newJointValues);
	    robot_to_use->SetDOFVelocities(newJointVelocities);
	}
	
	//if (show_viewer_) {
    //    usleep(1000000 * duration);
    //}
}

BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(propagate_nonlinear_overload, propagate_nonlinear, 7, 9);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(propagate_linear_overload, propagate_linear, 8, 10);
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(update_robot_values_overload, update_robot_values, 2, 3);

BOOST_PYTHON_MODULE(libpropagator) {
    using namespace boost::python;
    
    
   
    class_<Propagator>("Propagator", init<>())
							   .def("setup", &Propagator::setup_py)
							   .def("propagate", &Propagator::propagate_nonlinear, propagate_nonlinear_overload())
							   .def("propagateLinear", &Propagator::propagate_linear, propagate_linear_overload())
							   .def("updateRobotValues", &Propagator::update_robot_values, update_robot_values_overload())
		                       
										            		 
                        //.def("doIntegration", &Integrate::do_integration)                        
                        //.def("getResult", &Integrate::getResult)
    ;
}


}
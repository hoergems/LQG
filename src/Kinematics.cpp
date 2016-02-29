#include "Kinematics.hpp"

using std::cout;
using std::endl;


namespace shared {

Kinematics::Kinematics():
    links_(),
	joint_origins_(),
	joint_axis_(){
}

void Kinematics::setJointOrigins(std::vector<std::vector<double>> &joint_origins) {	
	for (auto &o: joint_origins) {
		joint_origins_.push_back(o);		
	}
}

/**void Kinematics::setJointAxis(std::vector<std::vector<int>> &axis) {	
	for (auto &o: axis) {
		for (auto &k: o) {
			cout << k << ", ";
		}
		cout << endl;
		joint_axis_.push_back(o);		
	}
}*/

void Kinematics::setLinkDimensions(std::vector<std::vector<double>> &link_dimensions) {
	for (auto &k: link_dimensions) {
		links_.push_back(k);
	}
}

void Kinematics::getPositionOfLinkN(const std::vector<double> &joint_angles, const int &n, std::vector<double> &position) const {
    std::pair<fcl::Vec3f, fcl::Matrix3f> link_n_pose = getPoseOfLinkN(joint_angles, n);
    position.push_back(link_n_pose.first[0]);
    position.push_back(link_n_pose.first[1]);
    position.push_back(link_n_pose.first[2]);
}

/* Gets the end effector position for a given set of joint angles */    
void Kinematics::getEndEffectorPosition(const std::vector<double> &joint_angles, std::vector<double> &end_effector_position) const {
    int n = joint_angles.size();
    std::pair<fcl::Vec3f, fcl::Matrix3f> ee_pose = getPoseOfLinkN(joint_angles, n);
    end_effector_position.push_back(ee_pose.first[0]);
    end_effector_position.push_back(ee_pose.first[1]);
    end_effector_position.push_back(ee_pose.first[2]);
}  

Eigen::MatrixXd Kinematics::getEndEffectorPose(const std::vector<double> &joint_angles, bool &eigen) {	
	Eigen::MatrixXd m(4, 4);
	Eigen::MatrixXd res = Eigen::MatrixXd::Identity(4, 4);
	res(0, 3) = joint_origins_[0][0];
	res(1, 3) = joint_origins_[0][1];
	res(2, 3) = joint_origins_[0][2];	
	for (size_t i = 0; i < joint_angles.size() + 1; i++) {
		if (i == joint_angles.size()) {
			res = getPoseOfLinkN(0.0, res, i);
		}
		else {
			res = getPoseOfLinkN(joint_angles[i], res, i);
		}
		
	}
	
	return res;
}

Eigen::MatrixXd Kinematics::getPoseOfLinkN(const double &joint_angle, 
		                                   Eigen::MatrixXd &current_transform, 
										   size_t &n) const {
	Eigen::MatrixXd new_trans;
	if (n == 0) {
		new_trans = getTransformationMatr(joint_angle, 0.0, 0.0, 0.0);		
	}
	else if (n == joint_origins_.size()) {
		new_trans = getTransformationMatr(0.0, 0.0, links_[n-1][0], 0.0);
	}
	else {		
		new_trans = getTransformationMatrRot(0.0, 0.0, links_[n-1][0], joint_origins_[n][3], joint_angle);		
	}
	
	return current_transform * new_trans;	
}

std::pair<fcl::Vec3f, fcl::Matrix3f> Kinematics::getPoseOfLinkN(const std::vector<double> &joint_angles, const int &n) const {    
   Eigen::MatrixXd res = Eigen::MatrixXd::Identity(4, 4);
   Eigen::MatrixXd init_trans(4, 4);   
   init_trans << 1.0, 0.0, 0.0, joint_origins_[0][0], 
		         0.0, 1.0, 0.0, joint_origins_[0][1],
				 0.0, 0.0, 1.0, joint_origins_[0][2],
				 0.0, 0.0, 0.0, 1.0;
   std::vector<Eigen::MatrixXd> transformations;
   transformations.push_back(init_trans);
   
   for (unsigned int i = 0; i < n; i++) {	   
       transformations.push_back(getTransformationMatr(joint_angles[i], 0.0, links_[i][0], joint_origins_[i + 1][3]));	      
   } 
      
   if (n != 0) {
   	   transformations.push_back(getTransformationMatr(joint_angles[n], 0.0, 0.0, 0.0));
   }
   else {
       transformations.push_back(getTransformationMatr(joint_angles[0], 0.0, 0.0, 0.0));
   }
      
   for (int i = transformations.size() - 1; i >= 0; i--) {	   
   	   res = transformations[i] * res;	  
   }
   
   fcl::Vec3f r_vec = fcl::Vec3f(res(0, 3), res(1, 3), res(2, 3));
   fcl::Matrix3f r_matr = fcl::Matrix3f(res(0, 0), res(0, 1), res(0, 2), 
                                        res(1, 0), res(1, 1), res(1, 2), 
                                        res(2, 0), res(2, 1), res(2, 2));   
   auto p = std::make_pair(r_vec, r_matr);   
   return p;
}

std::pair<fcl::Vec3f, fcl::Matrix3f> Kinematics::getEndEffectorPose(const std::vector<double> &joint_angles) const {
	int n = joint_angles.size(); 
    return getPoseOfLinkN(joint_angles, n);
}

void Kinematics::getEEJacobian(const std::vector<double> &joint_angles, Eigen::MatrixXd &jacobian) {
	auto ee_pose = getEndEffectorPose(joint_angles);	
	Eigen::Vector3d o_e;
	o_e << ee_pose.first[0], ee_pose.first[1], ee_pose.first[2];	
	std::vector<std::pair<fcl::Vec3f, fcl::Matrix3f>> link_poses;
	for (size_t i = 0; i < joint_angles.size(); i++) {
		auto pose = getPoseOfLinkN(joint_angles, i);
		Eigen::VectorXd column_vector(6);
		Eigen::Vector3d o_i;
		o_i << pose.first[0], pose.first[1], pose.first[2];		
		Eigen::Vector3d z_i;
		z_i << pose.second(0, 2), pose.second(1, 2), pose.second(2, 2);		
		Eigen::Vector3d upper = z_i.cross(o_e - o_i);
		column_vector << upper, z_i;
		jacobian.col(i) = column_vector;
	}
}

Eigen::MatrixXd Kinematics::transform(double x, double y, double z, double roll, double pitch, double yaw) const{
	Eigen::MatrixXd trans(4, 4);
	trans << 1.0, 0.0, 0.0, x,
			 0.0, 1.0, 0.0, y,
			 0.0, 0.0, 1.0, z,
			 0.0, 0.0, 0.0, 1.0;
	
	Eigen::MatrixXd ro(4, 4);
	ro << 1.0, 0.0, 0.0, 0.0,
		 0.0, cos(roll), -sin(roll), 0.0,
		 0.0, sin(roll), cos(roll), 0.0,
		 0.0, 0.0, 0.0, 1.0;
	
	Eigen::MatrixXd pi(4, 4);
	pi << cos(pitch), 0.0, sin(pitch), 0.0,
		 0.0, 1.0, 0.0, 0.0,
		 -sin(pitch), 0.0, cos(pitch), 0.0,
		 0.0, 0.0, 0.0, 1.0;
	
	Eigen::MatrixXd ya(4, 4);
	ya << cos(yaw), -sin(yaw), 0.0, 0.0,
		  sin(yaw), cos(yaw), 0.0, 0.0,
		  0.0, 0.0, 1.0, 0.0,
		  0.0, 0.0, 0.0, 1.0;
	
	Eigen::MatrixXd res = ro * pi * ya * trans;
	return res;
}

Eigen::MatrixXd Kinematics::getTransformationMatr(double sigma_n, double d_n, double a_n, double alpha_n) const {
    Eigen::MatrixXd b(4,4);    
    b << cos(sigma_n), -sin(sigma_n) * cos(alpha_n), sin(sigma_n) * sin(alpha_n), a_n * cos(sigma_n),
         sin(sigma_n), cos(sigma_n) * cos(alpha_n), -cos(sigma_n) * sin(alpha_n), a_n * sin(sigma_n),
         0.0, sin(alpha_n), cos(alpha_n), d_n,
         0.0, 0.0, 0.0, 1.0;
    return b;
}

Eigen::MatrixXd Kinematics::getTransformationMatrRot(double sigma_n, double d_n, double a_n, double alpha_n, double theta_n) const{
	Eigen::MatrixXd b(4,4);
	b << -sin(sigma_n)*sin(theta_n)*cos(alpha_n) + cos(sigma_n)*cos(theta_n), -sin(sigma_n)*cos(alpha_n)*cos(theta_n) - sin(theta_n)*cos(sigma_n),  sin(alpha_n)*sin(sigma_n), a_n*cos(sigma_n),
		 sin(sigma_n)*cos(theta_n) + sin(theta_n)*cos(alpha_n)*cos(sigma_n), -sin(sigma_n)*sin(theta_n) + cos(alpha_n)*cos(sigma_n)*cos(theta_n), -sin(alpha_n)*cos(sigma_n), a_n*sin(sigma_n),
		 sin(alpha_n)*sin(theta_n), sin(alpha_n)*cos(theta_n), cos(alpha_n), d_n,
		 0.0, 0.0, 0.0, 1.0;
	return b;
}
 
}

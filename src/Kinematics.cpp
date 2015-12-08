#include "Kinematics.hpp"

using std::cout;
using std::endl;


namespace shared {

Kinematics::Kinematics():
    links_(),    
    rotation_offsets_(),
	joint_origins_(){
}

void Kinematics::setJointOrigins(std::vector<std::vector<double>> &joint_origins) {
	for (auto &o: joint_origins) {
		joint_origins_.push_back(o);
		rotation_offsets_.push_back(o[3]);
	}
}

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
    
    //return std::vector<double>({link_n_pose.first[0], link_n_pose.first[1], link_n_pose.first[2]});
}

/* Gets the end effector position for a given set of joint angles */    
void Kinematics::getEndEffectorPosition(const std::vector<double> &joint_angles, std::vector<double> &end_effector_position) const {
    int n = 3;
    std::pair<fcl::Vec3f, fcl::Matrix3f> ee_pose = getPoseOfLinkN(joint_angles, n);
    end_effector_position.push_back(ee_pose.first[0]);
    end_effector_position.push_back(ee_pose.first[1]);
    end_effector_position.push_back(ee_pose.first[2]);
    
    //return std::vector<double>({ee_pose.first[0], ee_pose.first[1], ee_pose.first[2]});
}  

std::pair<fcl::Vec3f, fcl::Matrix3f> Kinematics::getPoseOfLinkN(const std::vector<double> &joint_angles, const int &n) const {
   Eigen::MatrixXd res(4, 4);
   Eigen::MatrixXd init_trans(4, 4);
   init_trans << 1.0, 0.0, 0.0, joint_origins_[0][0], 
		         0.0, 1.0, 0.0, joint_origins_[0][1],
				 0.0, 0.0, 1.0, joint_origins_[0][2],
				 0.0, 0.0, 0.0, 1.0;
   if (n == 0) {
       res = init_trans * getTransformationMatr(joint_angles[0], 0.0, 0.0, rotation_offsets_[0]);             
   }
   else if (n == 1) {
       Eigen::MatrixXd a = getTransformationMatr(joint_angles[0], 0.0, links_[0][0], rotation_offsets_[1]);
       Eigen::MatrixXd b = getTransformationMatr(joint_angles[1], 0.0, 0.0, 0.0);
       res = init_trans * (a * b);
   }
   else if (n == 2) {       
       Eigen::MatrixXd a = getTransformationMatr(joint_angles[0], 0.0, links_[0][0], rotation_offsets_[1]);
       Eigen::MatrixXd b = getTransformationMatr(joint_angles[1], 0.0, links_[1][0], rotation_offsets_[2]);
       Eigen::MatrixXd c = getTransformationMatr(joint_angles[2], 0.0, 0.0, 0.0);       
       res = init_trans * (a * (b * c));       
   }
   else if (n == 3) {       
       Eigen::MatrixXd a = getTransformationMatr(joint_angles[0], 0.0, links_[0][0], rotation_offsets_[1]);
       Eigen::MatrixXd b = getTransformationMatr(joint_angles[1], 0.0, links_[1][0], rotation_offsets_[2]);
       Eigen::MatrixXd c = getTransformationMatr(joint_angles[2], 0.0, links_[2][0], 0.0);      
       res = init_trans * (a * (b * c));
   } 
   
   fcl::Vec3f r_vec = fcl::Vec3f(res(0, 3), res(1, 3), res(2, 3));
   fcl::Matrix3f r_matr = fcl::Matrix3f(res(0, 0), res(0, 1), res(0, 2), 
                                        res(1, 0), res(1, 1), res(1, 2), 
                                        res(2, 0), res(2, 1), res(2, 2));
   auto p = std::make_pair(r_vec, r_matr);
   return p;
}

std::pair<fcl::Vec3f, fcl::Matrix3f> Kinematics::getEndEffectorPose(const std::vector<double> &joint_angles) const {
    int n = 3;
    return getPoseOfLinkN(joint_angles, n);
}

Eigen::MatrixXd Kinematics::getTransformationMatr(double sigma_n, double d_n, double a_n, double alpha_n) const {
    Eigen::MatrixXd b(4,4);    
    b << cos(sigma_n), -sin(sigma_n) * cos(alpha_n), sin(sigma_n) * sin(alpha_n), a_n * cos(sigma_n),
         sin(sigma_n), cos(sigma_n) * cos(alpha_n), -cos(sigma_n) * sin(alpha_n), a_n * sin(sigma_n),
         0.0, sin(alpha_n), cos(alpha_n), d_n,
         0.0, 0.0, 0.0, 1.0;
    return b;
}

/**bool Kinematics::setLinksAndAxisCalled() {
    return setLinksAndAxisCalled_;
}*/



/**BOOST_PYTHON_MODULE(libkinematics)
{
    using namespace boost::python;
    
    void (Kinematics::*setParams_d)(boost::python::list&) = &Kinematics::setParamsPy;
    
    class_<Kinematics, std::shared_ptr<Kinematics>>("Kinematics")
        .def("getPositionOfLinkN", &Kinematics::getPositionOfLinkN)
        .def("getEndEffectorPosition", &Kinematics::getEndEffectorPosition)         
		.def("setParams", setParams_d)
    ;
}*/
 
}

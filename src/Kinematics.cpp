#include "Kinematics.hpp"

using std::cout;
using std::endl;


namespace shared {

Kinematics::Kinematics():
    links_(),    
    rotation_offsets_() {
}

void Kinematics::setLinksAndAxis(std::vector<std::vector<double>> links, std::vector<std::vector<int>> axis) {
    for (size_t i = 0; i < links.size(); i++) {
        links_.push_back(links[i]);
    }

    for (size_t i = 0; i < axis.size(); i++) {
        rotation_offsets_.push_back(0.0);
        if (axis[i][1] == 1) {
           rotation_offsets_[i] = -M_PI / 2.0;
        }
        else if (axis[i][1] == 1) {
           rotation_offsets_[i] = M_PI / 2.0;
        }       
    }
}

std::vector<double> Kinematics::getPositionOfLinkN(const std::vector<double> &joint_angles, int &n) const {
    std::pair<fcl::Vec3f, fcl::Matrix3f> link_n_pose = getPoseOfLinkN(joint_angles, n);
    return std::vector<double>({link_n_pose.first[0], link_n_pose.first[1], link_n_pose.first[2]});
}

/* Gets the end effector position for a given set of joint angles */    
std::vector<double> Kinematics::getEndEffectorPosition(const std::vector<double> &joint_angles) const {
    int n = 3;
    std::pair<fcl::Vec3f, fcl::Matrix3f> ee_pose = getPoseOfLinkN(joint_angles, n);
    return std::vector<double>({ee_pose.first[0], ee_pose.first[1], ee_pose.first[2]});
}  

std::pair<fcl::Vec3f, fcl::Matrix3f> Kinematics::getPoseOfLinkN(const std::vector<double> &joint_angles, int &n) const {
   Eigen::MatrixXd res(4, 4);
   if (n == 0) {
       res = getTransformationMatr(joint_angles[0], 0.0, 0.0, rotation_offsets_[0]);             
   }
   else if (n == 1) {
       Eigen::MatrixXd a = getTransformationMatr(joint_angles[0], 0.0, links_[0][0], rotation_offsets_[1]);
       Eigen::MatrixXd b = getTransformationMatr(joint_angles[1], 0.0, 0.0, 0.0);
       res = a*b;
   }
   else if (n == 2) {       
       Eigen::MatrixXd a = getTransformationMatr(joint_angles[0], 0.0, links_[0][0], rotation_offsets_[1]);
       Eigen::MatrixXd b = getTransformationMatr(joint_angles[1], 0.0, links_[1][0], rotation_offsets_[2]);
       Eigen::MatrixXd c = getTransformationMatr(joint_angles[2], 0.0, 0.0, 0.0);       
       res = a*(b*c);       
   }
   else if (n == 3) {       
       Eigen::MatrixXd a = getTransformationMatr(joint_angles[0], 0.0, links_[0][0], rotation_offsets_[1]);
       Eigen::MatrixXd b = getTransformationMatr(joint_angles[1], 0.0, links_[1][0], rotation_offsets_[2]);
       Eigen::MatrixXd c = getTransformationMatr(joint_angles[2], 0.0, links_[2][0], 0.0);      
       res = a*(b*c);
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



BOOST_PYTHON_MODULE(kin)
{
    using namespace boost::python;
    
    typedef std::vector<std::vector<double> > double_trouble;
    
    class_<Kinematics, std::shared_ptr<Kinematics>>("Kinematics")
        .def("getPositionOfLinkN", &Kinematics::getPositionOfLinkN)
        .def("getEndEffectorPosition", &Kinematics::getEndEffectorPosition)
        .def("setLinksAndAxis", &Kinematics::setLinksAndAxis)       
    ;
}
 
}

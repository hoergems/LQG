#ifndef KINEMATICS_HPP_
#define KINEMATICS_HPP_

#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include <iostream>
#include <vector>
#include <algorithm>
//#include "/usr/local/include/fcl/math/matrix_3f.h"
#include "fcl/math/matrix_3f.h"
#include <Eigen/Dense>
#include <boost/timer.hpp>

namespace shared {

    class Kinematics {
        public:
            Kinematics();
            
            //void setLinksAndAxis(std::vector<std::vector<double>> links, std::vector<std::vector<int>> axis); 
            
            void getPositionOfLinkN(const std::vector<double> &joint_angles, const int &n, std::vector<double> &position) const;
            
            void getEndEffectorPosition(const std::vector<double> &joint_angles, std::vector<double> &end_effector_position) const;
            
            std::pair<fcl::Vec3f, fcl::Matrix3f> getPoseOfLinkN(const std::vector<double> &joint_angles, const int &n) const;
            
            Eigen::MatrixXd getPoseOfLinkN(const double &joint_angle, 
            		                       Eigen::MatrixXd &current_transform, 
										   size_t &n) const;
            
            Eigen::MatrixXd getEndEffectorPose(const std::vector<double> &joint_angles, bool &eigen);
            
            std::pair<fcl::Vec3f, fcl::Matrix3f> getEndEffectorPose(const std::vector<double> &joint_angles) const;
            
            void setJointOrigins(std::vector<std::vector<double>> &joint_origins);
            //bool setLinksAndAxisCalled();
            
            //void setJointAxis(std::vector<std::vector<int>> &axis);
            
            void setLinkDimensions(std::vector<std::vector<double>> &link_dimensions);
            
            void getEEJacobian(const std::vector<double> &joint_angles, Eigen::MatrixXd &jacobian);
            
        private:
            std::vector<std::vector<double>> joint_origins_;
            
            std::vector<std::vector<int>> joint_axis_;
            
            std::vector<std::vector<double>> links_;
            
            Eigen::MatrixXd getTransformationMatr(double sigma_n, double d_n, double a_n, double alpha_n) const;
            
            Eigen::MatrixXd getTransformationMatrRot(double sigma_n, double d_n, double a_n, double alpha_n, double theta_n) const;
            
            Eigen::MatrixXd transform(double x, double y, double z, double roll, double pitch, double yaw) const;
            
            
    };



}

#endif

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

namespace shared {

    class Kinematics {
        public:
            Kinematics();
            
            //void setLinksAndAxis(std::vector<std::vector<double>> links, std::vector<std::vector<int>> axis); 
            
            void getPositionOfLinkN(const std::vector<double> &joint_angles, const int &n, std::vector<double> &position) const;
            
            void getEndEffectorPosition(const std::vector<double> &joint_angles, std::vector<double> &end_effector_position) const;
            
            std::pair<fcl::Vec3f, fcl::Matrix3f> getPoseOfLinkN(const std::vector<double> &joint_angles, const int &n) const;
            
            std::pair<fcl::Vec3f, fcl::Matrix3f> getEndEffectorPose(const std::vector<double> &joint_angles) const;
            
            void setJointOrigins(std::vector<std::vector<double>> &joint_origins);
            //bool setLinksAndAxisCalled();
            
            void setLinkDimensions(std::vector<std::vector<double>> &link_dimensions);
            
            
            
        private:
            std::vector<std::vector<double>> joint_origins_;
            
            std::vector<std::vector<double>> links_;
            
            Eigen::MatrixXd getTransformationMatr(double sigma_n, double d_n, double a_n, double alpha_n) const;
            
            
            
    };



}

#endif

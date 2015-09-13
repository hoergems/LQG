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
            
            void setLinksAndAxis(std::vector<std::vector<double>> links, std::vector<std::vector<int>> axis); 
            
            std::vector<double> getPositionOfLinkN(const std::vector<double> &joint_angles, const int &n) const;
            
            std::vector<double> getEndEffectorPosition(const std::vector<double> &joint_angles) const;
            
            std::pair<fcl::Vec3f, fcl::Matrix3f> getPoseOfLinkN(const std::vector<double> &joint_angles, const int &n) const;
            
            std::pair<fcl::Vec3f, fcl::Matrix3f> getEndEffectorPose(const std::vector<double> &joint_angles) const;

            bool setLinksAndAxisCalled();
            
            
            
        private:        
            std::vector<std::vector<double>> links_;
            
            std::vector<double> rotation_offsets_;

            bool setLinksAndAxisCalled_;
            
            Eigen::MatrixXd getTransformationMatr(double sigma_n, double d_n, double a_n, double alpha_n) const;
            
            
            
    };



}

#endif

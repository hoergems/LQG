#ifndef RAVE_VIEWER_HPP_
#define RAVE_VIEWER_HPP_
#include <iostream>
#include <openrave-core.h>
#include <vector>
#include <cstring>
#include <sstream>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

namespace shared {
    class RaveViewer {
        public:
    	    RaveViewer();
            void testView(OpenRAVE::EnvironmentBasePtr &penv);
            void setViewer(OpenRAVE::EnvironmentBasePtr &penv, const std::string &viewername);
            void setViewerSize(int x, int y); 
            void setBackgroundColor(double &r, double &g, double &b);            
            void setCameraTransform(std::vector<double> &rot, std::vector<double> &trans);
            
        private:
            int size_x_;
            int size_y_;
            
            /**
             * The background colors
             */
            double r_;
            double g_;
            double b_;
            
            /**
             * The camera transform
             */
            std::vector<double> rot_;
            std::vector<double> trans_;
            double focal_distance_;
            

    };

}

#endif

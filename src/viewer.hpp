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

            void SetViewer(OpenRAVE::EnvironmentBasePtr &penv, const std::string &viewername);

    };

}

#endif

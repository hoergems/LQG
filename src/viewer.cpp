#include "viewer.hpp"

using std::cout;
using std::endl;

namespace shared {

RaveViewer::RaveViewer():
	size_x_(500),
	size_y_(500),
	r_(1.0),
	g_(1.0),
	b_(1.0),
	rot_({1.0, 0.0, 0.0, 0.0}),
	trans_({0.0, 0.0, 0.0}),
	focal_distance_(0.0){

	
}

void RaveViewer::testView(OpenRAVE::EnvironmentBasePtr &penv) {    
    std::string viewername = "qtcoin";
    boost::thread thviewer(boost::bind(&RaveViewer::setViewer, this, penv, viewername));
    //thviewer.join();
}

void RaveViewer::setViewer(OpenRAVE::EnvironmentBasePtr &penv, const std::string &viewername)
{
    OpenRAVE::ViewerBasePtr viewer = OpenRAVE::RaveCreateViewer(penv, viewername);
    viewer->SetName("viewer");    
    viewer->SetSize(size_x_, size_y_);
    OpenRAVE::RaveVector<float> color(r_, g_, b_);
    viewer->SetBkgndColor(color);
    OpenRAVE::Vector trans(trans_[0], trans_[1], trans_[2]);
    OpenRAVE::Vector rot(rot_[0], rot_[1], rot_[2]);
    OpenRAVE::RaveTransform<double> transform(rot, trans);
    //viewer->SetCamera(transform);    
    BOOST_ASSERT(!!viewer);
    // attach it to the environment:
    penv->Add(viewer);
    // finally call the viewer's infinite loop (this is why a separate thread is needed)
    bool showgui = true;
    viewer->main(showgui);
}

void RaveViewer::setViewerSize(int x, int y) {
	size_x_ = x;
	size_y_ = y;
}

void RaveViewer::setBackgroundColor(double &r, double &g, double &b) {
	r_ = r;
	g_ = g;
	b_ = b;
}

void RaveViewer::setCameraTransform(std::vector<double> &rot, std::vector<double> &trans) {	
	for (size_t i = 0; i < rot.size(); i++) {
		rot_[i] = rot[i];
	}	
	
	for (size_t i = 0; i < trans.size(); i++) {
		trans_[i] = trans[i];
	}
}

}

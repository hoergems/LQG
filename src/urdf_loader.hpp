#ifndef URDF_LOADER_HPP_
#define URDF_LOADER_HPP_

#include <openrave-core.h>
#include <openrave/environment.h>
#include <boost/bind.hpp>
#include <urdf/model.h>


namespace shared {

    class URDFLoader
	{
	    public:
    	    URDFLoader();
    	
    		OpenRAVE::KinBodyPtr load(std::string model_file, OpenRAVE::EnvironmentBasePtr &env);
    	
    	    void ParseURDF(urdf::Model &model, 
    	    		       std::vector<OpenRAVE::KinBody::LinkInfoPtr> &link_infos,
    	                   std::vector<OpenRAVE::KinBody::JointInfoPtr> &joint_infos);
    	    
	    private:
    	    OpenRAVE::Transform URDFPoseToRaveTransform(const urdf::Pose &pose);
    	    
    	    OpenRAVE::Vector URDFRotationToRaveVector(const urdf::Rotation &rotation);
    	    
    	    OpenRAVE::Vector URDFColorToRaveVector(const urdf::Color &color);
    	    
    	    OpenRAVE::Vector URDFVectorToRaveVector(const urdf::Vector3 &vector);
    	    
    	    std::pair<OpenRAVE::KinBody::JointType, bool> URDFJointTypeToRaveJointType(int type);
    	    
	};

}

#endif
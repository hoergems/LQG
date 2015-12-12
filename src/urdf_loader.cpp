#include "urdf_loader.hpp"
//#include "boostfs_helpers.h"

#include <boost/algorithm/string/predicate.hpp>
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>

using std::cout;
using std::endl;

namespace shared {

template <class T>
std::vector<boost::shared_ptr<T const> > MakeConst(
        std::vector<boost::shared_ptr<T> > const &vconst)
{
    std::vector<boost::shared_ptr<T const> > v;
    v.reserve(vconst.size());

    BOOST_FOREACH (boost::shared_ptr<T> const &x, vconst) {
        v.push_back(x);
    }
    return v;
}

URDFLoader::URDFLoader() {
	
}

/** Converts from URDF 3D vector to OpenRAVE 3D vector. */
OpenRAVE::Vector URDFLoader::URDFVectorToRaveVector(const urdf::Vector3 &vector)
{
    return OpenRAVE::Vector(vector.x, vector.y, vector.z);
}

/** Converts from URDF 3D rotation to OpenRAVE 3D vector. */
OpenRAVE::Vector URDFLoader::URDFRotationToRaveVector(const urdf::Rotation &rotation)
{
    return OpenRAVE::Vector(rotation.w, rotation.x, rotation.y, rotation.z);
}

OpenRAVE::Vector URDFLoader::URDFColorToRaveVector(const urdf::Color &color)
{
    return OpenRAVE::Vector(color.r, color.g, color.b, color.a);
}

OpenRAVE::Transform URDFLoader::URDFPoseToRaveTransform(const urdf::Pose &pose)
{
    return OpenRAVE::Transform(URDFRotationToRaveVector(pose.rotation),
                               URDFVectorToRaveVector(pose.position));
}

std::pair<OpenRAVE::KinBody::JointType, bool> URDFLoader::URDFJointTypeToRaveJointType(int type)
{
    switch(type) {
    case urdf::Joint::REVOLUTE:
        return std::make_pair(OpenRAVE::KinBody::JointRevolute, true);

    case urdf::Joint::PRISMATIC:
        return std::make_pair(OpenRAVE::KinBody::JointSlider, true);

    case urdf::Joint::FIXED:
        return std::make_pair(OpenRAVE::KinBody::JointHinge, false);

    case urdf::Joint::CONTINUOUS:
        return std::make_pair(OpenRAVE::KinBody::JointHinge, true);

    // TODO: Fill the rest of these joint types in!
    case urdf::Joint::PLANAR:
        throw std::runtime_error("Planar joints are not supported.");

    case urdf::Joint::FLOATING:
        throw std::runtime_error("Floating joints are not supported.");

    case urdf::Joint::UNKNOWN:
        throw std::runtime_error("Unknown joints are not supported.");

    default:
        throw std::runtime_error(boost::str(
            boost::format("Unkonwn type of joint %d.") % type));
    }
}

void URDFLoader::ParseURDF(
        urdf::Model &model,
        std::vector<OpenRAVE::KinBody::LinkInfoPtr> &link_infos,
        std::vector<OpenRAVE::KinBody::JointInfoPtr> &joint_infos)
  {
    // Populate list of links from URDF model. We'll force the root link to be
    // first.
    std::vector<boost::shared_ptr<urdf::Link> > link_vector;
    model.getLinks(link_vector);

    std::list<boost::shared_ptr<urdf::Link const> > link_list;
    std::set<std::string> finished_links;

    link_list.insert(link_list.begin(), model.getRoot());
    BOOST_FOREACH (boost::shared_ptr<urdf::Link> link, link_vector) {
        if (link != model.getRoot()) {
            link_list.insert(link_list.end(), link);
        }
    }

    // TODO: prevent infinite loops here
    // Iterate through all links, allowing deferred evaluation (putting links
    // back on the list) if their parents do not exist yet
    boost::shared_ptr<urdf::Link const> link_ptr;

    while (!link_list.empty()) {
      // Get next element in list
      link_ptr = link_list.front();
      link_list.pop_front();

      OpenRAVE::KinBody::LinkInfoPtr link_info
            = boost::make_shared<OpenRAVE::KinBody::LinkInfo>();

      // TODO: Set "type" to "dynamic".
      link_info->_name = link_ptr->name;
      
      // Set inertial parameters
      boost::shared_ptr<urdf::Inertial> inertial = link_ptr->inertial;
      if (inertial) {
        // XXX: We should also specify the off-diagonal terms (ixy, iyz, ixz)
        // of the inertia tensor. We can do this in KinBody XML files, but I
        // cannot figure out how to do so through this API.
        link_info->_mass = inertial->mass;
        link_info->_tMassFrame = URDFPoseToRaveTransform(inertial->origin);
        link_info->_vinertiamoments = OpenRAVE::Vector(
                inertial->ixx, inertial->iyy, inertial->izz);
      }

      // Set local transformation to be same as parent joint
      boost::shared_ptr<urdf::Joint> parent_joint = link_ptr->parent_joint;
      while (parent_joint) {
        link_info->_t = URDFPoseToRaveTransform(
                parent_joint->parent_to_joint_origin_transform) * link_info->_t;
        boost::shared_ptr<urdf::Link const> parent_link
                = model.getLink(parent_joint->parent_link_name);
        parent_joint = parent_link->parent_joint;
      }
      
      // Set information for collision geometry
      BOOST_FOREACH (boost::shared_ptr<urdf::Collision> collision, link_ptr->collision_array) {
        OpenRAVE::KinBody::GeometryInfoPtr geom_info
                = boost::make_shared<OpenRAVE::KinBody::GeometryInfo>();

        // TODO: Shouldn't we apply this transform?
        geom_info->_t = URDFPoseToRaveTransform(collision->origin);
        geom_info->_bVisible = false;
        geom_info->_bModifiable = false;

        switch (collision->geometry->type) {
        /**case urdf::Geometry::MESH: {
            urdf::Mesh const &mesh
                    = dynamic_cast<const urdf::Mesh &>(*collision->geometry);
            geom_info->_filenamecollision = resolveURI(mesh.filename);
            geom_info->_type = OpenRAVE::GT_TriMesh;

            // This doesn't seem to do anything, but we'll set it anyway.
            geom_info->_vCollisionScale = URDFVectorToRaveVector(mesh.scale);

            boost::shared_ptr<OpenRAVE::TriMesh> trimesh
                    = boost::make_shared<OpenRAVE::TriMesh>();
            trimesh = GetEnv()->ReadTrimeshURI(trimesh,
                                               geom_info->_filenamecollision);
            if (trimesh) {
                // The _vCollisionScale property does nothing, so we have to
                // manually scale the mesh.
                BOOST_FOREACH(OpenRAVE::Vector &vertex, trimesh->vertices) {
                    vertex *= geom_info->_vCollisionScale;
                }

                geom_info->_meshcollision = *trimesh;
            } else {
                RAVELOG_WARN("Link[%s]: Failed loading collision mesh %s\n",
                             link_ptr->name.c_str(), geom_info->_filenamecollision.c_str());
            }
            break;
        }*/

        case urdf::Geometry::SPHERE: {
            RAVELOG_WARN("Creating sphere!\n");
            urdf::Sphere const &sphere
                    = dynamic_cast<const urdf::Sphere &>(*collision->geometry);
            geom_info->_vGeomData = sphere.radius * OpenRAVE::Vector(1, 1, 1);
            geom_info->_type = OpenRAVE::GT_Sphere;
            break;
        }

        case urdf::Geometry::BOX: {
            urdf::Box const &box
                    = dynamic_cast<const urdf::Box &>(*collision->geometry);
            geom_info->_vGeomData = 0.5 * OpenRAVE::Vector(box.dim.x, box.dim.y,
                                                           box.dim.z);
            geom_info->_type = OpenRAVE::GT_Box;
            break;
        }

        case urdf::Geometry::CYLINDER: {
            urdf::Cylinder const &cylinder
                    = dynamic_cast<const urdf::Cylinder &>(*collision->geometry);
            geom_info->_vGeomData = OpenRAVE::Vector(cylinder.radius,
                                                     cylinder.length, 0);
            geom_info->_type = OpenRAVE::GT_Cylinder;
            break;
        }
        }

        link_info->_vgeometryinfos.push_back(geom_info);
      }

      // Add the render geometry. We can't create a link with no collision
      // geometry, so we'll instead create a zero-radius sphere with the
      // desired render mesh.
      // TODO: Why does GT_None crash OpenRAVE?
      boost::shared_ptr<urdf::Visual> visual = link_ptr->visual;
      if (visual) {
        OpenRAVE::KinBody::GeometryInfoPtr geom_info
            = boost::make_shared<OpenRAVE::KinBody::GeometryInfo>();
        geom_info->_t = URDFPoseToRaveTransform(visual->origin);
        geom_info->_type = OpenRAVE::GT_Sphere;
        geom_info->_vGeomData = OpenRAVE::Vector(0.0, 0.0, 0.0);
        geom_info->_bModifiable = false;
        geom_info->_bVisible = true;

        switch (visual->geometry->type) {
        /**case urdf::Geometry::MESH: {
            const urdf::Mesh &mesh = dynamic_cast<const urdf::Mesh&>(*visual->geometry);
            geom_info->_filenamerender = resolveURI(mesh.filename);
            geom_info->_vRenderScale = URDFVectorToRaveVector(mesh.scale);

            // If a material color is specified, use it.
            boost::shared_ptr<urdf::Material> material = visual->material;            
            if (material) {            	
                geom_info->_vDiffuseColor = URDFColorToRaveVector(material->color);
                geom_info->_vAmbientColor = URDFColorToRaveVector(material->color);
            }

            // Add the render-only geometry to the standard geometry group for
            // backwards compatability with QtCoin.
            link_info->_vgeometryinfos.push_back(geom_info);

            // Create a group dedicated to visual geometry for or_rviz.
            OpenRAVE::KinBody::GeometryInfoPtr geom_info_clone
                = boost::make_shared<OpenRAVE::KinBody::GeometryInfo>(*geom_info);
            link_info->_mapExtraGeometries["visual"].push_back(geom_info_clone);
            break;
        }*/

        case urdf::Geometry::BOX: {
            urdf::Box const &box
                    = dynamic_cast<const urdf::Box &>(*visual->geometry);
            geom_info->_vGeomData = 0.5 * OpenRAVE::Vector(box.dim.x, box.dim.y,
                                                           box.dim.z);
            geom_info->_type = OpenRAVE::GT_Box;
            // If a material color is specified, use it.
            boost::shared_ptr<urdf::Material> material = visual->material;            
            if (material) {            	
                geom_info->_vDiffuseColor = URDFColorToRaveVector(material->color);
                geom_info->_vAmbientColor = URDFColorToRaveVector(material->color);
            }
            link_info->_vgeometryinfos.push_back(geom_info);
            break;
        }

        default:
            RAVELOG_WARN("Link[%s]: Only trimeshes are supported for visual geometry.\n", link_ptr->name.c_str());
        }
      }

      // Verify that the "visual" and "spheres" groups always exist. Recall 
      // that accessing an element with operator[] creates it using the default
      // no-arg constructor if it does not already exist.
      link_info->_mapExtraGeometries["visual"];
      link_info->_mapExtraGeometries["spheres"];
      link_infos.push_back(link_info);
    }

    // Populate vector of joints
    std::string joint_name; 
    boost::shared_ptr<urdf::Joint> joint_ptr;

    // Parse the joint properties
    std::vector<boost::shared_ptr<urdf::Joint> > ordered_joints;
    BOOST_FOREACH(boost::tie(joint_name, joint_ptr), model.joints_) {
        ordered_joints.push_back(joint_ptr);
    }

    BOOST_FOREACH(boost::shared_ptr<urdf::Joint> joint_ptr, ordered_joints) {
      OpenRAVE::KinBody::JointInfoPtr joint_info = boost::make_shared<OpenRAVE::KinBody::JointInfo>();
      joint_info->_name = joint_ptr->name;
      joint_info->_linkname0 = joint_ptr->parent_link_name;
      joint_info->_linkname1 = joint_ptr->child_link_name;
      joint_info->_vanchor = URDFVectorToRaveVector(joint_ptr->parent_to_joint_origin_transform.position);

      int const urdf_joint_type = joint_ptr->type;

      // Set the joint type. Some URDF joints correspond to disabled OpenRAVE
      // joints, so we'll appropriately set the corresponding IsActive flag.
      OpenRAVE::KinBody::JointType joint_type;
      bool enabled;
      boost::tie(joint_type, enabled) = URDFJointTypeToRaveJointType(urdf_joint_type);
      joint_info->_type = joint_type;
      joint_info->_bIsActive = enabled;
      joint_info->_bIsCircular[0] = (urdf_joint_type == urdf::Joint::CONTINUOUS);

      // URDF only supports linear mimic joints with a constant offset. We map
      // that into the correct position (index 0) and velocity (index 1)
      // equations for OpenRAVE.
      boost::shared_ptr<urdf::JointMimic> mimic = joint_ptr->mimic;
      if (mimic) {
        joint_info->_vmimic[0] = boost::make_shared<OpenRAVE::KinBody::MimicInfo>();
        joint_info->_vmimic[0]->_equations[0] = boost::str(boost::format("%s*%0.6f+%0.6f")
                % mimic->joint_name % mimic->multiplier % mimic->offset);
        joint_info->_vmimic[0]->_equations[1] = boost::str(boost::format("|%s %0.6f")
                % mimic->joint_name % mimic->multiplier);
      }

      // Configure joint axis. Add an arbitrary axis if the joint is disabled.
      urdf::Vector3 joint_axis;
      if (enabled) {
        joint_axis = joint_ptr->parent_to_joint_origin_transform.rotation * joint_ptr->axis;
      } else {
        joint_axis = urdf::Vector3(1, 0, 0);
      }
      joint_info->_vaxes[0] = URDFVectorToRaveVector(joint_axis);
      
      // Configure joint limits.
      boost::shared_ptr<urdf::JointLimits> limits = joint_ptr->limits;
      if (limits) {
          // TODO: What about acceleration?
          joint_info->_vlowerlimit[0] = limits->lower;
          joint_info->_vupperlimit[0] = limits->upper;
          joint_info->_vmaxvel[0] = limits->velocity;
          joint_info->_vmaxtorque[0] = limits->effort;
      }
      // Fixed joints are just revolute joints with zero limits.
      else if (!enabled) {
          joint_info->_vlowerlimit[0] = 0;
          joint_info->_vupperlimit[0] = 0;
      }
      // Default to +/- 2*PI. This is the same default used by OpenRAVE for
      // revolute joints.
      else {
          joint_info->_vlowerlimit[0] = -M_PI;
          joint_info->_vupperlimit[0] =  M_PI;
      }

      // Force continuous joints to have +/- PI limits. Otherwise, the internal
      // _vcircularlowerlimit and _vcircularupperlimit values will be set to
      // zero. This causes OpenRAVE::utils::NormalizeCircularAngle to crash.
      if (urdf_joint_type == urdf::Joint::CONTINUOUS) {
          joint_info->_vlowerlimit[0] = -M_PI;
          joint_info->_vupperlimit[0] =  M_PI;
      }

      joint_infos.push_back(joint_info);
    }
  }

OpenRAVE::KinBodyPtr URDFLoader::load(std::string model_file,
		                              OpenRAVE::EnvironmentBasePtr &env) {	
	std::string name;

	// Load the URDF file.
	urdf::Model urdf_model;
	if (!urdf_model.initFile(model_file)) {
	    throw OpenRAVE::openrave_exception("Failed to open URDF file.");
	}

	std::vector<OpenRAVE::KinBody::LinkInfoPtr> link_infos;
	std::vector<OpenRAVE::KinBody::JointInfoPtr> joint_infos;
	ParseURDF(urdf_model, link_infos, joint_infos);

	
	// It's just a URDF file, so create a KinBody.
	        
	std::vector<OpenRAVE::KinBody::LinkInfoConstPtr> link_infos_const
	                = MakeConst(link_infos);
	std::vector<OpenRAVE::KinBody::JointInfoConstPtr> joint_infos_const
	                = MakeConst(joint_infos);

	//body = OpenRAVE::RaveCreateKinBody(GetEnv(), "");
	OpenRAVE::KinBodyPtr body = OpenRAVE::RaveCreateKinBody(env, "");
	body->Init(link_infos_const, joint_infos_const);
	        

    body->SetName(urdf_model.getName());
	
	         
	return body;
	
}

}
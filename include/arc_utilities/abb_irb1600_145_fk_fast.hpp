#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <map>
#include <Eigen/Geometry>

#ifndef ABB_IRB1600_145_FK_FAST_HPP
#define ABB_IRB1600_145_FK_FAST_HPP

namespace ABB_IRB1600_145_FK_FAST
{
    const size_t ABB_IRB1600_145_NUM_ACTIVE_JOINTS = 6;
    const size_t ABB_IRB1600_145_NUM_LINKS = 8;

    const std::string ABB_IRB1600_145_ACTIVE_JOINT_1_NAME = "joint_1";
    const std::string ABB_IRB1600_145_ACTIVE_JOINT_2_NAME = "joint_2";
    const std::string ABB_IRB1600_145_ACTIVE_JOINT_3_NAME = "joint_3";
    const std::string ABB_IRB1600_145_ACTIVE_JOINT_4_NAME = "joint_4";
    const std::string ABB_IRB1600_145_ACTIVE_JOINT_5_NAME = "joint_5";
    const std::string ABB_IRB1600_145_ACTIVE_JOINT_6_NAME = "joint_6";

    typedef std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>> VectorAffine3d;

    inline Eigen::Affine3d Get_base_joint1_LinkJointTransform(const double joint_val)
    {
        Eigen::Translation3d pre_joint_translation(0.0, 0.0, 0.1245);
        Eigen::Quaterniond pre_joint_rotation(1.0, 0.0, 0.0, 0.0);
        Eigen::Affine3d pre_joint_transform = pre_joint_translation * pre_joint_rotation;
        Eigen::Translation3d joint_translation(0.0, 0.0, 0.0);
        Eigen::Quaterniond joint_rotation(Eigen::AngleAxisd(joint_val, Eigen::Vector3d::UnitZ()));
        Eigen::Affine3d joint_transform = joint_translation * joint_rotation;
        return (pre_joint_transform * joint_transform);
    }

    inline Eigen::Affine3d Get_link_1_joint_2_LinkJointTransform(const double joint_val)
    {
        Eigen::Translation3d pre_joint_translation(0.15, -0.1395, 0.362);
        Eigen::Quaterniond pre_joint_rotation(1.0, 0.0, 0.0, 0.0);
        Eigen::Affine3d pre_joint_transform = pre_joint_translation * pre_joint_rotation;
        Eigen::Translation3d joint_translation(0.0, 0.0, 0.0);
        Eigen::Quaterniond joint_rotation(Eigen::AngleAxisd(joint_val, Eigen::Vector3d::UnitY()));
        Eigen::Affine3d joint_transform = joint_translation * joint_rotation;
        return (pre_joint_transform * joint_transform);
    }

    inline Eigen::Affine3d Get_link_2_joint_3_LinkJointTransform(const double joint_val)
    {
        Eigen::Translation3d pre_joint_translation(0.0, 0.028, 0.7);
        Eigen::Quaterniond pre_joint_rotation(1.0, 0.0, 0.0, 0.0);
        Eigen::Affine3d pre_joint_transform = pre_joint_translation * pre_joint_rotation;
        Eigen::Translation3d joint_translation(0.0, 0.0, 0.0);
        Eigen::Quaterniond joint_rotation(Eigen::AngleAxisd(joint_val, Eigen::Vector3d::UnitY()));
        Eigen::Affine3d joint_transform = joint_translation * joint_rotation;
        return (pre_joint_transform * joint_transform);
    }

    inline Eigen::Affine3d Get_link_3_joint_4_LinkJointTransform(const double joint_val)
    {
        Eigen::Translation3d pre_joint_translation(0.314, 0.107, 0.0);
        Eigen::Quaterniond pre_joint_rotation(1.0, 0.0, 0.0, 0.0);
        Eigen::Affine3d pre_joint_transform = pre_joint_translation * pre_joint_rotation;
        Eigen::Translation3d joint_translation(0.0, 0.0, 0.0);
        Eigen::Quaterniond joint_rotation(Eigen::AngleAxisd(joint_val, Eigen::Vector3d::UnitX()));
        Eigen::Affine3d joint_transform = joint_translation * joint_rotation;
        return (pre_joint_transform * joint_transform);
    }

    inline Eigen::Affine3d Get_link_4_joint_5_LinkJointTransform(const double joint_val)
    {
        Eigen::Translation3d pre_joint_translation(0.286, 0.0, 0.0);
        Eigen::Quaterniond pre_joint_rotation(1.0, 0.0, 0.0, 0.0);
        Eigen::Affine3d pre_joint_transform = pre_joint_translation * pre_joint_rotation;
        Eigen::Translation3d joint_translation(0.0, 0.0, 0.0);
        Eigen::Quaterniond joint_rotation(Eigen::AngleAxisd(joint_val, Eigen::Vector3d::UnitY()));
        Eigen::Affine3d joint_transform = joint_translation * joint_rotation;
        return (pre_joint_transform * joint_transform);
    }

    inline Eigen::Affine3d Get_link_5_joint_6_LinkJointTransform(const double joint_val)
    {
        Eigen::Translation3d pre_joint_translation(0.0, 0.0, 0.0);
        Eigen::Quaterniond pre_joint_rotation(1.0, 0.0, 0.0, 0.0);
        Eigen::Affine3d pre_joint_transform = pre_joint_translation * pre_joint_rotation;
        Eigen::Translation3d joint_translation(0.0, 0.0, 0.0);
        Eigen::Quaterniond joint_rotation(Eigen::AngleAxisd(joint_val, Eigen::Vector3d::UnitX()));
        Eigen::Affine3d joint_transform = joint_translation * joint_rotation;
        return (pre_joint_transform * joint_transform);
    }

    inline Eigen::Affine3d Get_Fixed_link_6_joint_tool_LinkJointTransform(void)
    {
        Eigen::Translation3d pre_joint_translation(0.065, 0.0, 0.0);
        Eigen::Quaterniond pre_joint_rotation(0.7071067811865476, 0.0, 0.7071067811865476, 0.0);
        Eigen::Affine3d pre_joint_transform = pre_joint_translation * pre_joint_rotation;
        return pre_joint_transform;
    }

    inline VectorAffine3d GetLinkTransforms(const std::vector<double>& configuration)
    {
        assert(configuration.size() == ABB_IRB1600_145_NUM_ACTIVE_JOINTS);
        VectorAffine3d link_transforms(ABB_IRB1600_145_NUM_LINKS);
        link_transforms[0] = Eigen::Affine3d::Identity();
        link_transforms[1] = link_transforms[0] * Get_base_joint1_LinkJointTransform(configuration[0]);
        link_transforms[2] = link_transforms[1] * Get_link_1_joint_2_LinkJointTransform(configuration[1]);
        link_transforms[3] = link_transforms[2] * Get_link_2_joint_3_LinkJointTransform(configuration[2]);
        link_transforms[4] = link_transforms[3] * Get_link_3_joint_4_LinkJointTransform(configuration[3]);
        link_transforms[5] = link_transforms[4] * Get_link_4_joint_5_LinkJointTransform(configuration[4]);
        link_transforms[6] = link_transforms[5] * Get_link_5_joint_6_LinkJointTransform(configuration[5]);
        link_transforms[7] = link_transforms[6] * Get_Fixed_link_6_joint_tool_LinkJointTransform();
        return link_transforms;
    }

    inline VectorAffine3d GetLinkTransforms(std::map<std::string, double> configuration)
    {
        std::vector<double> configuration_vector(ABB_IRB1600_145_NUM_ACTIVE_JOINTS);
        configuration_vector[0] = configuration[ABB_IRB1600_145_ACTIVE_JOINT_1_NAME];
        configuration_vector[1] = configuration[ABB_IRB1600_145_ACTIVE_JOINT_2_NAME];
        configuration_vector[2] = configuration[ABB_IRB1600_145_ACTIVE_JOINT_3_NAME];
        configuration_vector[3] = configuration[ABB_IRB1600_145_ACTIVE_JOINT_4_NAME];
        configuration_vector[4] = configuration[ABB_IRB1600_145_ACTIVE_JOINT_5_NAME];
        configuration_vector[5] = configuration[ABB_IRB1600_145_ACTIVE_JOINT_6_NAME];
        return GetLinkTransforms(configuration_vector);
    }
}

#endif // ABB_IRB1600_145_FK_FAST_HPP

#include <stdio.h>
#include <stdlib.h>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <arc_utilities/abb_irb1600_145_fk_fast.hpp>

int main(int argc, char** argv)
{
    printf("%d arguments\n", argc);
    for (int idx = 0; idx < argc; idx++)
    {
        printf("Argument %d: %s\n", idx, argv[idx]);
    }
    std::cout << "Testing PrettyPrints..." << std::endl;
    std::cout << PrettyPrint::PrettyPrint(Eigen::Affine3d::Identity()) << std::endl;
    std::cout << PrettyPrint::PrettyPrint(Eigen::Vector3d(0.0, 0.0, 0.0)) << std::endl;
    std::cout << PrettyPrint::PrettyPrint(std::vector<bool>{true, false, true, false}) << std::endl;
    std::cout << "...done" << std::endl;
    std::vector<double> base_config = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    EigenHelpers::VectorAffine3d link_transforms = ABB_IRB1600_145_FK_FAST::GetLinkTransforms(base_config);
    std::cout << "Link transforms:\n" << PrettyPrint::PrettyPrint(link_transforms, false, "\n") << std::endl;
    // Test Vector3d averaging
    Eigen::Vector3d testvec1(-1.0, -1.0, -1.0);
    Eigen::Vector3d testvec2(-1.0, -1.0, 1.0);
    Eigen::Vector3d testvec3(-1.0, 1.0, -1.0);
    Eigen::Vector3d testvec4(-1.0, 1.0, 1.0);
    Eigen::Vector3d testvec5(1.0, -1.0, -1.0);
    Eigen::Vector3d testvec6(1.0, -1.0, 1.0);
    Eigen::Vector3d testvec7(1.0, 1.0, -1.0);
    Eigen::Vector3d testvec8(1.0, 1.0, 1.0);
    EigenHelpers::VectorVector3d testvecs = {testvec1, testvec2, testvec3, testvec4, testvec5, testvec6, testvec7, testvec8};
    std::cout << "Individual vectors: " << PrettyPrint::PrettyPrint(testvecs) << std::endl;
    Eigen::Vector3d averagevec = EigenHelpers::AverageEigenVector3d(testvecs);
    std::cout << "Average vector: " << PrettyPrint::PrettyPrint(averagevec) << std::endl;
    return 0;
}

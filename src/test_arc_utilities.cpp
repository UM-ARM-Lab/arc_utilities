#include <stdio.h>
#include <stdlib.h>
#include <random>
#include <chrono>
#include <arc_utilities/arc_helpers.hpp>
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
    EigenHelpers::VectorVector3d testvecs(8, Eigen::Vector3d::Zero());
    testvecs[0] = Eigen::Vector3d(-1.0, -1.0, -1.0);
    testvecs[1] = Eigen::Vector3d(-1.0, -1.0, 1.0);
    testvecs[2] = Eigen::Vector3d(-1.0, 1.0, -1.0);
    testvecs[3] = Eigen::Vector3d(-1.0, 1.0, 1.0);
    testvecs[4] = Eigen::Vector3d(1.0, -1.0, -1.0);
    testvecs[5] = Eigen::Vector3d(1.0, -1.0, 1.0);
    testvecs[6] = Eigen::Vector3d(1.0, 1.0, -1.0);
    testvecs[7] = Eigen::Vector3d(1.0, 1.0, 1.0);
    std::cout << "Individual vectors: " << PrettyPrint::PrettyPrint(testvecs) << std::endl;
    Eigen::Vector3d averagevec = EigenHelpers::AverageEigenVector3d(testvecs);
    std::cout << "Average vector: " << PrettyPrint::PrettyPrint(averagevec) << std::endl;
    auto seed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::mt19937_64 prng(seed);
    arc_helpers::TruncatedNormalDistribution dist(0.0, 1.0, -5.0, 5.0);
    std::vector<double> test_trunc_normals(100000, 0.0);
    for (size_t idx = 0; idx < test_trunc_normals.size(); idx++)
    {
        test_trunc_normals[idx] = dist(prng);
    }
    std::cout << "Truncated normal test:\n" << PrettyPrint::PrettyPrint(test_trunc_normals, false, ",") << std::endl;
    return 0;
}

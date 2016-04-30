#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <functional>
#include <arc_utilities/arc_helpers.hpp>
#include <Eigen/Geometry>

#ifndef SIMPLE_HIERARCHICAL_CLUSTERING_HPP
#define SIMPLE_HIERARCHICAL_CLUSTERING_HPP

namespace simple_hierarchical_clustering
{
    class SimpleHierarchicalClustering
    {
    private:

        SimpleHierarchicalClustering() {}

        static std::pair<std::pair<std::pair<bool, int64_t>, std::pair<bool, int64_t>>, double> GetClosestPair(const std::vector<uint8_t>& datapoint_mask, const Eigen::MatrixXd& distance_matrix, const std::vector<std::vector<int64_t>>& clusters)
        {
            // Compute distances between unclustered points <-> unclustered points, unclustered_points <-> clusters, and clusters <-> clusters
            // Compute the minimum unclustered point <-> unclustered point / unclustered_point <-> cluster distance
            double min_distance = INFINITY;
            std::pair<int64_t, std::pair<bool, int64_t>> min_element_pair(-1, std::pair<bool, int64_t>(false, -1));
            for (size_t idx = 0; idx < datapoint_mask.size(); idx++)
            {
                // Make sure we aren't in a cluster already
                if (datapoint_mask[idx] == 0)
                {
                    // Compute the minimum unclustered point <-> unclustered point distance
                    double min_point_point_distance = INFINITY;
                    int64_t min_point_index = -1;
                    for (size_t jdx = 0; jdx < datapoint_mask.size(); jdx++)
                    {
                        // Make sure the other point isn't us, and isn't already in a cluster
                        if ((idx != jdx) && (datapoint_mask[jdx] == 0))
                        {
                            const double& current_distance = distance_matrix(idx, jdx);
                            // Update the clkosest point
                            if (current_distance < min_point_point_distance)
                            {
                                min_point_point_distance = current_distance;
                                min_point_index = jdx;
                            }
                        }
                    }
                    // Compute the minimum unclustered point <-> cluster distance
                    double min_point_cluster_distance = INFINITY;
                    int64_t min_cluster_index = -1;
                    for (size_t cdx = 0; cdx < clusters.size(); cdx++)
                    {
                        // We only work with clusters that aren't empty
                        if (clusters[cdx].size() > 0)
                        {
                            // Compute the distance to the current cluster
                            double current_distance = 0.0;
                            for (size_t cpdx = 0; cpdx < clusters[cdx].size(); cpdx++)
                            {
                                const int64_t& current_cluster_point_index = clusters[cdx][cpdx];
                                const double& new_distance = distance_matrix(idx, current_cluster_point_index);
                                if (new_distance > current_distance)
                                {
                                    current_distance = new_distance;
                                }
                            }
                            // Update the closest cluster
                            if (current_distance < min_point_cluster_distance)
                            {
                                min_point_cluster_distance = current_distance;
                                min_cluster_index = cdx;
                            }
                        }
                    }
                    // Update the closest index
                    if (min_point_point_distance < min_distance)
                    {
                        min_distance = min_point_point_distance;
                        min_element_pair.first = idx;
                        min_element_pair.second.first = false;
                        min_element_pair.second.second = min_point_index;
                    }
                    if (min_point_cluster_distance < min_distance)
                    {
                        min_distance = min_point_cluster_distance;
                        min_element_pair.first = idx;
                        min_element_pair.second.first = true;
                        min_element_pair.second.second = min_cluster_index;
                    }
                }
            }
            // Compute the minimum cluster <-> cluster distance
            double min_cluster_cluster_distance = INFINITY;
            std::pair<int64_t, int64_t> min_cluster_pair(-1, -1);
            for (size_t fcdx = 0; fcdx < clusters.size(); fcdx++)
            {
                const std::vector<int64_t>& first_cluster = clusters[fcdx];
                // Don't evaluate empty clusters
                if (first_cluster.size() > 0)
                {
                    for (size_t scdx = 0; scdx < clusters.size(); scdx++)
                    {
                        // Don't compare against ourself
                        if (fcdx != scdx)
                        {
                            const std::vector<int64_t>& second_cluster = clusters[scdx];
                            // Don't evaluate empty clusters
                            if (second_cluster.size() > 0)
                            {
                                // Compute the cluster <-> cluster distance
                                double max_point_point_distance = 0.0;
                                // Find the maximum-pointwise distance between clusters
                                for (size_t fcpx = 0; fcpx < first_cluster.size(); fcpx++)
                                {
                                    for (size_t scpx = 0; scpx < second_cluster.size(); scpx++)
                                    {
                                        const int64_t& fcp_index = first_cluster[fcpx];
                                        const int64_t& scp_index = second_cluster[scpx];
                                        const double& new_distance = distance_matrix(fcp_index, scp_index);
                                        if (new_distance > max_point_point_distance)
                                        {
                                            max_point_point_distance = new_distance;
                                        }
                                    }
                                }
                                double cluster_cluster_distance = max_point_point_distance;
                                if (cluster_cluster_distance < min_cluster_cluster_distance)
                                {
                                    min_cluster_cluster_distance = cluster_cluster_distance;
                                    min_cluster_pair.first = fcdx;
                                    min_cluster_pair.second = scdx;
                                }
                            }
                        }
                    }
                }
            }
            // Return the minimum-distance pair
            if (min_distance <= min_cluster_cluster_distance)
            {
                // Set the indices
                std::pair<bool, int64_t> first_index(false, min_element_pair.first);
                std::pair<bool, int64_t> second_index = min_element_pair.second;
                std::pair<std::pair<bool, int64_t>, std::pair<bool, int64_t>> indices(first_index, second_index);
                std::pair<std::pair<std::pair<bool, int64_t>, std::pair<bool, int64_t>>, double> minimum_pair(indices, min_distance);
                return minimum_pair;
            }
            // A cluster <-> cluster pair is closest
            else
            {
                // Set the indices
                std::pair<bool, int64_t> first_index(true, min_cluster_pair.first);
                std::pair<bool, int64_t> second_index(true, min_cluster_pair.second);
                std::pair<std::pair<bool, int64_t>, std::pair<bool, int64_t>> indices(first_index, second_index);
                std::pair<std::pair<std::pair<bool, int64_t>, std::pair<bool, int64_t>>, double> minimum_pair(indices, min_cluster_cluster_distance);
                return minimum_pair;
            }
        }

    public:

        template<typename Datatype, typename Allocator=std::allocator<Datatype>>
        static std::pair<std::vector<std::vector<Datatype, Allocator>>, double> Cluster(const std::vector<Datatype, Allocator>& data, const std::function<double(const Datatype&, const Datatype&)>& distance_fn, const double max_cluster_distance)
        {
            const Eigen::MatrixXd distance_matrix = arc_helpers::BuildDistanceMatrix(data, distance_fn);
            return Cluster(data, distance_matrix, max_cluster_distance);
        }

        template<typename Datatype, typename Allocator=std::allocator<Datatype>>
        static std::pair<std::vector<std::vector<Datatype, Allocator>>, double> Cluster(const std::vector<Datatype, Allocator>& data, const Eigen::MatrixXd& distance_matrix, const double max_cluster_distance)
        {
            assert((size_t)distance_matrix.rows() == data.size());
            assert((size_t)distance_matrix.cols() == data.size());
            std::vector<uint8_t> datapoint_mask(data.size(), 0u);
            std::vector<std::vector<int64_t>> cluster_indices;
            double closest_distance = 0.0;
            bool complete = false;
            while (!complete)
            {
                // Get closest pair of elements (an element can be a cluster or single data value!)
                std::pair<std::pair<std::pair<bool, int64_t>, std::pair<bool, int64_t>>, double> closest_element_pair = GetClosestPair(datapoint_mask, distance_matrix, cluster_indices);
                closest_distance = closest_element_pair.second;
                //std::cout << "Element pair: " << PrettyPrint::PrettyPrint(closest_element_pair, true) << std::endl;
                if (closest_element_pair.second <= max_cluster_distance)
                {
                    // If both elements are points, create a new cluster
                    if ((closest_element_pair.first.first.first == false) && (closest_element_pair.first.second.first == false))
                    {
                        //std::cout << "New point-point cluster" << std::endl;
                        int64_t first_element_index = closest_element_pair.first.first.second;
                        assert(first_element_index >= 0);
                        int64_t second_element_index = closest_element_pair.first.second.second;
                        assert(second_element_index >= 0);
                        // Add a cluster
                        cluster_indices.push_back(std::vector<int64_t>{first_element_index, second_element_index});
                        // Mask out the indices
                        datapoint_mask[first_element_index] += 1;
                        datapoint_mask[second_element_index] += 1;
                    }
                    // If both elements are clusters, merge the clusters
                    else if ((closest_element_pair.first.first.first == true) && (closest_element_pair.first.second.first == true))
                    {
                        //std::cout << "Combining clusters" << std::endl;
                        // Get the cluster indices
                        int64_t first_cluster_index = closest_element_pair.first.first.second;
                        assert(first_cluster_index >= 0);
                        int64_t second_cluster_index = closest_element_pair.first.second.second;
                        assert(second_cluster_index >= 0);
                        // Merge the second cluster into the first
                        cluster_indices[first_cluster_index].insert(cluster_indices[first_cluster_index].end(), cluster_indices[second_cluster_index].begin(), cluster_indices[second_cluster_index].end());
                        // Empty the second cluster (we don't remove, because this triggers move)
                        cluster_indices[second_cluster_index].clear();
                    }
                    // If one of the elements is a cluster and the other is a point, add the point to the existing cluster
                    else
                    {
                        //std::cout << "Adding to an existing cluster" << std::endl;
                        int64_t cluster_index = -1;
                        int64_t element_index = -1;
                        if (closest_element_pair.first.first.first)
                        {
                            cluster_index = closest_element_pair.first.first.second;
                            element_index = closest_element_pair.first.second.second;
                        }
                        else if (closest_element_pair.first.second.first)
                        {
                            cluster_index = closest_element_pair.first.second.second;
                            element_index = closest_element_pair.first.first.second;
                        }
                        else
                        {
                            assert(false);
                        }
                        assert(cluster_index >= 0);
                        assert(element_index >= 0);
                        // Add the element to the cluster
                        cluster_indices[cluster_index].push_back(element_index);
                        // Mask out the element index
                        datapoint_mask[element_index] += 1;
                    }
                }
                else
                {
                    complete = true;
                }
            }
            // Extract the actual cluster data
            std::vector<std::vector<Datatype, Allocator>> clusters;
            for (size_t idx = 0; idx < cluster_indices.size(); idx++)
            {
                const std::vector<int64_t>& current_cluster = cluster_indices[idx];
                // Ignore empty clusters
                if (current_cluster.size() > 0)
                {
                    std::vector<Datatype, Allocator> new_cluster;
                    for (size_t cdx = 0; cdx < current_cluster.size(); cdx++)
                    {
                        int64_t index = current_cluster[cdx];
                        new_cluster.push_back(data[index]);
                    }
                    clusters.push_back(new_cluster);
                }
            }
            return std::pair<std::vector<std::vector<Datatype, Allocator>>, double>(clusters, closest_distance);
        }
    };
}
#endif // SIMPLE_HIERARCHICAL_CLUSTERING_HPP

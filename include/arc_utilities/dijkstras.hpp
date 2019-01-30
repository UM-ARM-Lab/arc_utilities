#ifndef DIJKSTRAS_HPP
#define DIJKSTRAS_HPP

#include <random>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <vector>
#include <Eigen/Geometry>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/serialization.hpp>

namespace arc_dijkstras
{
    enum EDGE_VALIDITY
    {
        VALID, INVALID, UNKNOWN
    };

    /***********************************
     **    Graph Edge
     **********************************/
    class GraphEdge
    {
    protected:

        int64_t from_index_;
        int64_t to_index_;
        double weight_;
        EDGE_VALIDITY edge_validity_;

    public:

        static uint64_t Serialize(const GraphEdge& edge, std::vector<uint8_t>& buffer)
        {
            return edge.serializeSelf(buffer);
        }

        static std::pair<GraphEdge, uint64_t>
        Deserialize(const std::vector<uint8_t>& buffer, const uint64_t current)
        {
            GraphEdge temp_edge;
            const uint64_t bytes_read = temp_edge.deserializeSelf(buffer, current);
            return std::make_pair(temp_edge, bytes_read);
        }

        GraphEdge(const int64_t from_index, const int64_t to_index, const double weight)
            : from_index_(from_index), to_index_(to_index), weight_(weight),
              edge_validity_(EDGE_VALIDITY::UNKNOWN)
        {}

        GraphEdge()
            : from_index_(-1), to_index_(-1), weight_(0.0),
              edge_validity_(EDGE_VALIDITY::UNKNOWN)
        {}

        uint64_t serializeSelf(std::vector<uint8_t>& buffer) const
        {
            const uint64_t start_buffer_size = buffer.size();
            arc_utilities::SerializeFixedSizePOD<int64_t>(from_index_, buffer);
            arc_utilities::SerializeFixedSizePOD<int64_t>(to_index_, buffer);
            arc_utilities::SerializeFixedSizePOD<double>(weight_, buffer);
            arc_utilities::SerializeFixedSizePOD<EDGE_VALIDITY>(edge_validity_, buffer);
            // Figure out how many bytes were written
            const uint64_t end_buffer_size = buffer.size();
            const uint64_t bytes_written = end_buffer_size - start_buffer_size;
            return bytes_written;
        }

        uint64_t deserializeSelf(const std::vector<uint8_t>& buffer, const uint64_t current)
        {
            using namespace arc_utilities;
            uint64_t current_position = current;
            const std::pair<int64_t, uint64_t> deserialized_from_index =
                DeserializeFixedSizePOD<int64_t>(buffer, current_position);
            from_index_ = deserialized_from_index.first;
            current_position += deserialized_from_index.second;
            const std::pair<int64_t, uint64_t> deserialized_to_index =
                DeserializeFixedSizePOD<int64_t>(buffer, current_position);
            to_index_ = deserialized_to_index.first;
            current_position += deserialized_to_index.second;
            const std::pair<double, uint64_t> deserialized_weight =
                DeserializeFixedSizePOD<double>(buffer, current_position);
            weight_ = deserialized_weight.first;
            current_position += deserialized_weight.second;
            const std::pair<EDGE_VALIDITY, uint64_t> deserialized_validity =
                DeserializeFixedSizePOD<EDGE_VALIDITY>(buffer, current_position);
            edge_validity_ = deserialized_validity.first;
            current_position += deserialized_validity.second;
            
            // Figure out how many bytes were read
            const uint64_t bytes_read = current_position - current;
            return bytes_read;
        }

        bool operator==(const GraphEdge& other) const
        {
            return (from_index_ == other.getFromIndex() &&
                    to_index_ == other.getToIndex() &&
                    weight_ == other.getWeight() &&
                    edge_validity_ == other.getValidity());
        }

        std::string print() const
        {
            return std::string("(" + std::to_string(from_index_) + "->" +
                               std::to_string(to_index_) + ") : " + std::to_string(weight_) );
        }

        int64_t getFromIndex() const
        {
            return from_index_;
        }

        int64_t getToIndex() const
        {
            return to_index_;
        }

        double getWeight() const
        {
            return weight_;
        }

        void setFromIndex(const int64_t new_from_index)
        {
            from_index_ = new_from_index;
        }

        void setToIndex(const int64_t new_to_index)
        {
            to_index_ = new_to_index;
        }

        void setWeight(const double new_weight)
        {
            weight_ = new_weight;
        }

        EDGE_VALIDITY getValidity() const
        {
            return edge_validity_;
        }

        void setValidity(const EDGE_VALIDITY new_validity)
        {
            edge_validity_ = new_validity;
        }
    };

    inline std::ostream& operator<< (std::ostream& stream, const GraphEdge& edge)
    {
        stream << edge.print();
        return stream;
    }


    /***********************************
     **    Graph Node
     **********************************/
    template<typename NodeValueType, typename Allocator = std::allocator<NodeValueType>>
    class GraphNode
    {
    protected:

        NodeValueType value_;
        double distance_;
        std::vector<GraphEdge> in_edges_;
        std::vector<GraphEdge> out_edges_;

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        static uint64_t Serialize(
                const GraphNode<NodeValueType, Allocator>& node,
                std::vector<uint8_t>& buffer,
                const std::function<uint64_t(const NodeValueType&, std::vector<uint8_t>&)>& value_serializer)
        {
            return node.serializeSelf(buffer, value_serializer);
        }

        static std::pair<GraphNode<NodeValueType, Allocator>, uint64_t> Deserialize(
                const std::vector<uint8_t>& buffer,
                const uint64_t current,
                const std::function<std::pair<NodeValueType, uint64_t>(const std::vector<uint8_t>&,
                                                                       const uint64_t)>& value_deserializer)
        {
            GraphNode<NodeValueType, Allocator> temp_node;
            const uint64_t bytes_read = temp_node.deserializeSelf(buffer, current, value_deserializer);
            return std::make_pair(temp_node, bytes_read);
        }

        GraphNode(
                const NodeValueType& value,
                const double distance,
                const std::vector<GraphEdge>& new_in_edges,
                const std::vector<GraphEdge>& new_out_edges)
            : value_(value)
            , distance_(distance)
            , in_edges_(new_in_edges)
            , out_edges_(new_out_edges)
        {}

        explicit GraphNode(const NodeValueType& value)
            : value_(value)
            , distance_(std::numeric_limits<double>::infinity())
        {}

        GraphNode()
            : distance_(std::numeric_limits<double>::infinity())
        {}

        uint64_t serializeSelf(
                std::vector<uint8_t>& buffer,
                const std::function<uint64_t(const NodeValueType&,
                                             std::vector<uint8_t>&)>& value_serializer) const
        {
            const uint64_t start_buffer_size = buffer.size();
            // Serialize the value
            value_serializer(value_, buffer);
            // Serialize the distance
            arc_utilities::SerializeFixedSizePOD<double>(distance_, buffer);
            // Serialize the in edges
            arc_utilities::SerializeVector<GraphEdge>(in_edges_, buffer, GraphEdge::Serialize);
            // Serialize the in edges
            arc_utilities::SerializeVector<GraphEdge>(out_edges_, buffer, GraphEdge::Serialize);
            // Figure out how many bytes were written
            const uint64_t end_buffer_size = buffer.size();
            const uint64_t bytes_written = end_buffer_size - start_buffer_size;
            return bytes_written;
        }

        uint64_t deserializeSelf(
                const std::vector<uint8_t>& buffer,
                const uint64_t current,
                const std::function<std::pair<NodeValueType, uint64_t>(const std::vector<uint8_t>&,
                                                                       const uint64_t)>& value_deserializer)
        {
            using namespace arc_utilities;
            uint64_t current_position = current;
            // Deserialize the value
            const std::pair<NodeValueType, uint64_t> value_deserialized =
                value_deserializer(buffer, current_position);
            value_ = value_deserialized.first;
            current_position += value_deserialized.second;
            // Deserialize the distace
            const std::pair<double, uint64_t> distance_deserialized =
                DeserializeFixedSizePOD<double>(buffer, current_position);
            distance_ = distance_deserialized.first;
            current_position += distance_deserialized.second;
            // Deserialize the in edges
            const std::pair<std::vector<GraphEdge>, uint64_t> in_edges_deserialized =
                DeserializeVector<GraphEdge>(buffer, current_position, GraphEdge::Deserialize);
            in_edges_ = in_edges_deserialized.first;
            current_position += in_edges_deserialized.second;
            // Deserialize the out edges
            const std::pair<std::vector<GraphEdge>, uint64_t> out_edges_deserialized =
                DeserializeVector<GraphEdge>(buffer, current_position, GraphEdge::Deserialize);
            out_edges_ = out_edges_deserialized.first;
            current_position += out_edges_deserialized.second;
            // Figure out how many bytes were read
            const uint64_t bytes_read = current_position - current;
            return bytes_read;
        }

        std::string print() const
        {
            std::ostringstream strm;
            strm << "Node : " << distance_ << " In Edges : ";
            if (in_edges_.size() > 0)
            {
                strm << in_edges_[0].print();
                for (size_t idx = 1; idx < in_edges_.size(); idx++)
                {
                    strm << ", " << in_edges_[idx].print();
                }
            }
            strm << " Out Edges : ";
            if (out_edges_.size() > 0)
            {
                strm << out_edges_[0].print();
                for (size_t idx = 1; idx < out_edges_.size(); idx++)
                {
                    strm << ", " << out_edges_[idx].print();
                }
            }
            return strm.str();
        }

        const NodeValueType& getValue() const
        {
            return value_;
        }

        NodeValueType& getValue()
        {
            return value_;
        }

        void addInEdge(const GraphEdge& new_in_edge)
        {
            in_edges_.push_back(new_in_edge);
        }

        void addOutEdge(const GraphEdge& new_out_edge)
        {
            out_edges_.push_back(new_out_edge);
        }

        void addEdgePair(const GraphEdge& new_in_edge, const GraphEdge& new_out_edge)
        {
            addInEdge(new_in_edge);
            addOutEdge(new_out_edge);
        }

        double getDistance() const
        {
            return distance_;
        }

        void setDistance(const double distance)
        {
            distance_ = distance;
        }

        GraphEdge& getEdgeTo(const int64_t other_node_ind)
        {
            for(auto &e: out_edges_)
            {
                if(e.getToIndex() == other_node_ind)
                {
                    return e;
                }
            }
            throw std::invalid_argument("Invalid node index, no edge exists");
        }

        const std::vector<GraphEdge>& getInEdges() const
        {
            return in_edges_;
        }

        std::vector<GraphEdge>& getInEdges()
        {
            return in_edges_;
        }

        const std::vector<GraphEdge>& getOutEdges() const
        {
            return out_edges_;
        }

        std::vector<GraphEdge>& getOutEdges()
        {
            return out_edges_;
        }

        void setInEdges(const std::vector<GraphEdge>& new_in_edges)
        {
            in_edges_ = new_in_edges;
        }

        void setOutEdges(const std::vector<GraphEdge>& new_out_edges)
        {
            out_edges_ = new_out_edges;
        }
    };


    /***********************************
     **    Graph
     **********************************/
    template<typename NodeValueType, typename Allocator = std::allocator<NodeValueType>>
    class Graph
    {
    protected:

        std::vector<GraphNode<NodeValueType, Allocator>> nodes_;

        size_t markConnectedComponentUndirected(
                const int64_t starting_node_idx,
                std::vector<uint32_t>& components,
                const uint32_t component_id) const
        {
            // When we push into the queue, we mark as connected, so we don't need a separate
            // "queued" tracker to
            // avoid duplication, unlike what is used in CollisionMapGrid::MarkConnectedComponent
            std::queue<int64_t> working_queue;
            working_queue.push(starting_node_idx);
            size_t num_marked = 1;

            while (working_queue.size() > 0)
            {
                const auto next_node_idx = working_queue.front();
                working_queue.pop();

                const auto& out_edges = nodes_[next_node_idx].getOutEdges();
                for (const auto& edge : out_edges)
                {
                    const auto neighbour_idx = edge.getToIndex();
                    if (components[neighbour_idx] == 0)
                    {
                        components[neighbour_idx] = component_id;
                        ++num_marked;
                        working_queue.push(neighbour_idx);
                    }
                }
            }

            return num_marked;
        }

    public:

        static uint64_t Serialize(
                const Graph<NodeValueType,
                Allocator>& graph, std::vector<uint8_t>& buffer,
                const std::function<uint64_t(const NodeValueType&, std::vector<uint8_t>&)>& value_serializer)
        {
            return graph.serializeSelf(buffer, value_serializer);
        }

        static std::pair<Graph<NodeValueType, Allocator>, uint64_t> Deserialize(
                const std::vector<uint8_t>& buffer, const uint64_t current,
                const std::function<std::pair<NodeValueType, uint64_t>(const std::vector<uint8_t>&,
                                                                       const uint64_t)>& value_deserializer)
        {
            Graph<NodeValueType, Allocator> temp_graph;
            const uint64_t bytes_read = temp_graph.deserializeSelf(buffer, current, value_deserializer);
            return std::make_pair(temp_graph, bytes_read);
        }

        Graph(const std::vector<GraphNode<NodeValueType, Allocator>>& nodes)
        {
            if (CheckGraphLinkage(nodes))
            {
                nodes_ = nodes;
            }
            else
            {
                throw std::invalid_argument("Invalid graph linkage");
            }
        }

        Graph(const size_t expected_size)
        {
            nodes_.reserve(expected_size);
        }

        Graph()
        {}


        uint64_t serializeSelf(
            std::vector<uint8_t>& buffer,
            const std::function<uint64_t(const NodeValueType&, std::vector<uint8_t>&)>& value_serializer) const
        {
            const uint64_t start_buffer_size = buffer.size();
            const auto graph_state_serializer = std::bind(GraphNode<NodeValueType, Allocator>::Serialize,
                                                          std::placeholders::_1, std::placeholders::_2,
                                                          value_serializer);
            arc_utilities::SerializeVector<GraphNode<NodeValueType, Allocator>>(nodes_, buffer,
                                                                                graph_state_serializer);
            // Figure out how many bytes were written
            const uint64_t end_buffer_size = buffer.size();
            const uint64_t bytes_written = end_buffer_size - start_buffer_size;
            return bytes_written;
        }

        uint64_t deserializeSelf(
            const std::vector<uint8_t>& buffer,
            const uint64_t current,
            const std::function<std::pair<NodeValueType, uint64_t>(const std::vector<uint8_t>&,
                                                                   const uint64_t)>& value_deserializer)
        {
            using namespace arc_utilities;
            const auto graph_state_deserializer = std::bind(GraphNode<NodeValueType,
                                                            Allocator>::Deserialize, std::placeholders::_1,
                                                            std::placeholders::_2, value_deserializer);
            const auto deserialized_nodes =
                DeserializeVector<GraphNode<NodeValueType, Allocator>>(buffer, current,
                                                                       graph_state_deserializer);
            nodes_ = deserialized_nodes.first;
            return deserialized_nodes.second;
        }

        std::string print() const
        {
            std::ostringstream strm;
            strm << "Graph - Nodes : ";
            if (nodes_.size() > 0)
            {
                strm << nodes_[0].print();
                for (size_t idx = 1; idx < nodes_.size(); idx++)
                {
                    strm << "\n" << nodes_[idx].print();
                }
            }
            return strm.str();
        }

        void shrinkToFit()
        {
            nodes_.shrink_to_fit();
        }

        bool indexInRange(const int64_t index) const
        {
            return index >= 0 && index < (int64_t)(nodes_.size());
        }

        bool checkGraphLinkage() const
        {
            return CheckGraphLinkage(getNodes());
        }

        GraphEdge& getEdge(const int64_t node_ind_1, const int64_t node_ind_2)
        {
            return getNode(node_ind_1).getEdgeTo(node_ind_2);
        }

        GraphEdge& getReverseEdge(const GraphEdge &e)
        {
            return getNode(e.getToIndex()).getEdgeTo(e.getFromIndex());
        }

        static bool CheckGraphLinkage(const Graph<NodeValueType, Allocator>& graph)
        {
            return CheckGraphLinkage(graph.getNodes());
        }

        static bool CheckGraphLinkage(const std::vector<GraphNode<NodeValueType, Allocator>>& nodes)
        {
            // Go through every node and make sure the edges are valid
            for (size_t idx = 0; idx < nodes.size(); idx++)
            {
                const GraphNode<NodeValueType, Allocator>& current_node = nodes[idx];
                // Check the in edges first
                const std::vector<GraphEdge>& in_edges = current_node.getInEdges();
                for (size_t in_edge_idx = 0; in_edge_idx < in_edges.size(); in_edge_idx++)
                {
                    const GraphEdge& current_edge = in_edges[in_edge_idx];
                    // Check from index to make sure it's in bounds
                    const int64_t from_index = current_edge.getFromIndex();
                    if (from_index < 0 || from_index >= (int64_t)nodes.size())
                    {
                        return false;
                    }
                    // Check to index to make sure it matches our own index
                    const int64_t to_index = current_edge.getToIndex();
                    if (to_index != (int64_t)idx)
                    {
                        return false;
                    }
                    // Check edge validity (edges to ourself are not allowed)
                    if (from_index == to_index)
                    {
                        return false;
                    }
                    // Check to make sure that the from index node is linked to us
                    const GraphNode<NodeValueType, Allocator>& from_node = nodes[(size_t)from_index];
                    const std::vector<GraphEdge>& from_node_out_edges = from_node.getOutEdges();
                    bool from_node_connection_valid = false;
                    // Make sure at least one out edge of the from index node corresponds to the current node
                    for (size_t from_node_out_edge_idx = 0;
                         from_node_out_edge_idx < from_node_out_edges.size();
                         from_node_out_edge_idx++)
                    {
                        const GraphEdge& current_from_node_out_edge =
                            from_node_out_edges[from_node_out_edge_idx];
                        if (current_from_node_out_edge.getToIndex() == (int64_t)idx)
                        {
                            from_node_connection_valid = true;
                        }
                    }
                    if (from_node_connection_valid == false)
                    {
                        return false;
                    }
                }
                // Check the out edges second
                const std::vector<GraphEdge>& out_edges = current_node.getOutEdges();
                for (size_t out_edge_idx = 0; out_edge_idx < out_edges.size(); out_edge_idx++)
                {
                    const GraphEdge& current_edge = out_edges[out_edge_idx];
                    // Check from index to make sure it matches our own index
                    const int64_t from_index = current_edge.getFromIndex();
                    if (from_index != (int64_t)idx)
                    {
                        return false;
                    }
                    // Check to index to make sure it's in bounds
                    const int64_t to_index = current_edge.getToIndex();
                    if (to_index < 0 || to_index >= (int64_t)nodes.size())
                    {
                        return false;
                    }
                    // Check edge validity (edges to ourself are not allowed)
                    if (from_index == to_index)
                    {
                        return false;
                    }
                    // Check to make sure that the to index node is linked to us
                    const GraphNode<NodeValueType, Allocator>& to_node = nodes[(size_t)to_index];
                    const std::vector<GraphEdge>& to_node_in_edges = to_node.getInEdges();
                    bool to_node_connection_valid = false;
                    // Make sure at least one in edge of the to index node corresponds to the current node
                    for (size_t to_node_in_edge_idx = 0;
                         to_node_in_edge_idx < to_node_in_edges.size();
                         to_node_in_edge_idx++)
                    {
                        const GraphEdge& current_to_node_in_edge = to_node_in_edges[to_node_in_edge_idx];
                        if (current_to_node_in_edge.getFromIndex() == (int64_t)idx)
                        {
                            to_node_connection_valid = true;
                        }
                    }
                    if (to_node_connection_valid == false)
                    {
                        return false;
                    }
                }
            }
            return true;
        }

        const std::vector<GraphNode<NodeValueType, Allocator>>& getNodes() const
        {
            return nodes_;
        }

        std::vector<GraphNode<NodeValueType, Allocator>>& getNodes()
        {
            return nodes_;
        }

        const GraphNode<NodeValueType, Allocator>& getNode(const int64_t index) const
        {
            return nodes_.at((size_t)index);
        }

        GraphNode<NodeValueType, Allocator>& getNode(const int64_t index)
        {
            return nodes_.at((size_t)index);
        }

        int64_t addNode(const GraphNode<NodeValueType, Allocator>& new_node)
        {
            nodes_.push_back(new_node);
            return (int64_t)(nodes_.size() - 1);
        }

        int64_t addNode(const NodeValueType& new_value)
        {
            nodes_.push_back(GraphNode<NodeValueType, Allocator>(new_value));
            return (int64_t)(nodes_.size() - 1);
        }

        GraphEdge& addEdgeBetweenNodes(const int64_t from_index, const int64_t to_index,
                                       const double edge_weight)
        {
            // We retrieve the nodes first, since retrieval performs bounds checks first
            GraphNode<NodeValueType, Allocator>& from_node = getNode(from_index);
            GraphNode<NodeValueType, Allocator>& to_node = getNode(to_index);
            if (from_index == to_index)
            {
                throw std::invalid_argument("Invalid circular edge from==to not allowed");
            }
            const GraphEdge new_edge(from_index, to_index, edge_weight);
            from_node.addOutEdge(new_edge);
            to_node.addInEdge(new_edge);
            return from_node.getEdgeTo(to_index);
        }

        std::pair<const GraphEdge, const GraphEdge>
        addEdgesBetweenNodes(const int64_t first_index, const int64_t second_index, const double edge_weight)
        {
            GraphEdge& e1 = addEdgeBetweenNodes(first_index, second_index, edge_weight);
            GraphEdge& e2 = addEdgeBetweenNodes(second_index, first_index, edge_weight);
            return std::make_pair(e1, e2);
        }

        /**
         * @brief getConnectedComponentsUndirected
         * @return A vector of the component ids for each node, and the total number of components
         */
        std::pair<std::vector<uint32_t>, uint32_t> getConnectedComponentsUndirected() const
        {
            size_t total_num_marked = 0;
            auto connected_components = std::make_pair(std::vector<uint32_t>(nodes_.size(), 0), 0u);

            for (size_t node_idx = 0; node_idx < nodes_.size() && total_num_marked < nodes_.size(); ++node_idx)
            {
                // If we have not yet marked this node, then mark it and anything it can reach
                if (connected_components.first[node_idx] == 0)
                {
                    connected_components.second++;
                    size_t num_marked = markConnectedComponentUndirected(node_idx,
                                                                         connected_components.first,
                                                                         connected_components.second);
                    total_num_marked += num_marked;
                }
            }
            return connected_components;
        }
    };

    template<typename NodeValueType, typename Allocator = std::allocator<NodeValueType>>
    class SimpleDijkstrasAlgorithm
    {
    protected:

        class CompareIndexFn
        {
            public:
            constexpr bool operator()(const std::pair<int64_t, double>& lhs,
                                      const std::pair<int64_t, double>& rhs) const
            {
                return lhs.second > rhs.second;
            }
        };

        SimpleDijkstrasAlgorithm()
        {}

    public:

        typedef std::pair<Graph<NodeValueType, Allocator>, std::pair<std::vector<int64_t>,
                                                                     std::vector<double>>> DijkstrasResult;

        static DijkstrasResult PerformDijkstrasAlgorithm(
                const Graph<NodeValueType, Allocator>& graph,
                const int64_t start_index)
        {
            if ((start_index < 0) || (start_index >= (int64_t)graph.getNodes().size()))
            {
                throw std::invalid_argument("Start index out of range");
            }
            Graph<NodeValueType, Allocator> working_copy = graph;
            // Setup
            std::vector<int64_t> previous_index_map(working_copy.getNodes().size(), -1);
            std::vector<double> distances(working_copy.getNodes().size(),
                                          std::numeric_limits<double>::infinity());
            std::priority_queue<std::pair<int64_t, double>,
                                std::vector<std::pair<int64_t, double>>,
                                CompareIndexFn> queue;
            std::unordered_map<int64_t, uint32_t> explored(graph.getNodes().size());
            for (size_t idx = 0; idx < working_copy.getNodes().size(); idx++)
            {
                working_copy.getNode((int64_t)idx).setDistance(std::numeric_limits<double>::infinity());
                queue.push(std::make_pair((int64_t)idx, std::numeric_limits<double>::infinity()));
            }
            working_copy.getNode(start_index).setDistance(0.0);
            previous_index_map[(size_t)start_index] = start_index;
            distances[(size_t)start_index] = 0.0;
            queue.push(std::make_pair(start_index, 0.0));
            while (queue.size() > 0)
            {
                const std::pair<int64_t, double> top_node = queue.top();
                const int64_t& top_node_index = top_node.first;
                const double& top_node_distance = top_node.second;
                queue.pop();
                if (explored[top_node.first] > 0)
                {
                    // We've already been here
                    continue;
                }
                else
                {
                    // Note that we've been here
                    explored[top_node.first] = 1;
                    // Get our neighbors
                    const std::vector<GraphEdge>& neighbor_edges =
                        working_copy.getNode(top_node_index).getInEdges();
                    // Go through our neighbors
                    for (size_t neighbor_idx = 0; neighbor_idx < neighbor_edges.size(); neighbor_idx++)
                    {
                        const int64_t neighbor_index = neighbor_edges[neighbor_idx].getFromIndex();
                        const double neighbor_edge_weight = neighbor_edges[neighbor_idx].getWeight();
                        const double new_neighbor_distance = top_node_distance + neighbor_edge_weight;
                        // Check against the neighbor
                        const double stored_neighbor_distance =
                            working_copy.getNode(neighbor_index).getDistance();
                        if (new_neighbor_distance < stored_neighbor_distance)
                        {
                            // We've found a better way to get to this node
                            // Check if it's already been explored
                            if (explored[neighbor_index] > 0)
                            {
                                // If it's already been explored, we just update it in place
                                working_copy.getNode(neighbor_index).setDistance(new_neighbor_distance);
                            }
                            else
                            {
                                // If it hasn't been explored, we need to update it and add it to the queue
                                working_copy.getNode(neighbor_index).setDistance(new_neighbor_distance);
                                queue.push(std::make_pair(neighbor_index, new_neighbor_distance));
                            }
                            // Update that we're the best previous node
                            previous_index_map[(size_t)neighbor_index] = top_node_index;
                            distances[(size_t)neighbor_index] = new_neighbor_distance;
                        }
                        else
                        {
                            // Do nothing
                            continue;
                        }
                    }
                }
            }
            return std::make_pair(working_copy, std::make_pair(previous_index_map, distances));
        }

        static uint64_t SerializeDijstrasResult(
                const DijkstrasResult& result,
                std::vector<uint8_t>& buffer,
                const std::function<uint64_t(const NodeValueType&, std::vector<uint8_t>&)>& value_serializer)
        {
            const uint64_t start_buffer_size = buffer.size();
            // Serialize the graph
            result.first.SerializeSelf(buffer, value_serializer);
            // Serialize the previous index
            const auto index_serializer = std::bind(arc_utilities::SerializeFixedSizePOD<int64_t>,
                                                    std::placeholders::_1, std::placeholders::_2);
            SerializeVector(result.second.first, index_serializer);
            // Serialze the distances
            const auto distance_serializer = std::bind(arc_utilities::SerializeFixedSizePOD<double>,
                                                       std::placeholders::_1, std::placeholders::_2);
            SerializeVector(result.second.second, distance_serializer);
            // Figure out how many bytes were written
            const uint64_t end_buffer_size = buffer.size();
            const uint64_t bytes_written = end_buffer_size - start_buffer_size;
            return bytes_written;
        }

        static std::pair<DijkstrasResult, uint64_t> DijstrasResult(
                const std::vector<uint8_t>& buffer,
                const uint64_t current,
                const std::function<std::pair<NodeValueType, uint64_t>(const std::vector<uint8_t>&,
                                                                       const uint64_t)>& value_deserializer)
        {
            using namespace arc_utilities;
            uint64_t current_position = current;
            // Deserialize the graph itself
            std::pair<DijkstrasResult, uint64_t> deserialized;
            const auto graph_deserialized =
                Graph<NodeValueType, Allocator>::Deserialize(buffer, current_position, value_deserializer);
            deserialized.first.first = graph_deserialized.first;
            current_position += graph_deserialized.second;
            // Deserialize the previous index
            const auto index_deserializer = std::bind(DeserializeFixedSizePOD<int64_t>,
                                                      std::placeholders::_1, std::placeholders::_2);
            const auto prev_index_deserialized = DeserializeVector<int64_t>(buffer, current_position,
                                                                            index_deserializer);
            deserialized.first.second.first = prev_index_deserialized.first;
            current_position += prev_index_deserialized.second;
            // Deserialize the distances
            const auto distance_deserializer = std::bind(DeserializeFixedSizePOD<double>,
                                                         std::placeholders::_1, std::placeholders::_2);
            const auto distance_deserialized = DeserializeVector<double>(buffer, current_position,
                                                                         distance_deserializer);
            deserialized.first.second.second = distance_deserialized.first;
            current_position += distance_deserialized.second;
            // Figure out how many bytes were read
            deserialized.second = current_position - current;
            return deserialized;
        }
    };

    template<typename NodeValueType, typename Allocator = std::allocator<NodeValueType>>
    class SimpleGraphAstar
    {
    protected:

        SimpleGraphAstar() {}

    public:

        static arc_helpers::AstarResult PerformLazyAstar(
                const Graph<NodeValueType, Allocator>& graph,
                const int64_t start_index,
                const int64_t goal_index,
                const std::function<bool(const Graph<NodeValueType, Allocator>&,
                                         const GraphEdge&)>& edge_validity_check_fn,
                const std::function<double(const Graph<NodeValueType, Allocator>&,
                                           const GraphEdge&)>& distance_fn,
                const std::function<double(const NodeValueType&, const NodeValueType&)>& heuristic_fn,
                const bool limit_pqueue_duplicates)
        {
            using namespace arc_helpers;
            // Enforced sanity checks
            if ((start_index < 0) || (start_index >= (int64_t)graph.getNodes().size()))
            {
                throw std::invalid_argument("Start index out of range");
            }
            if ((goal_index < 0) || (goal_index >= (int64_t)graph.getNodes().size()))
            {
                throw std::invalid_argument("Goal index out of range");
            }
            if (start_index == goal_index)
            {
                throw std::invalid_argument("Start and goal indices must be different");
            }
            // Make helper function
            const auto heuristic_function = [&] (const int64_t node_index)
            {
                return heuristic_fn(graph.getNode(node_index).getValue(), graph.getNode(goal_index).getValue());
            };
            // Setup
            std::priority_queue<AstarPQueueElement,
                                std::vector<AstarPQueueElement>,
                                CompareAstarPQueueElementFn> queue;
            
            // Optional map to reduce the number of duplicate items added to the pqueue
            // Key is the node index in the provided graph
            // Value is cost-to-come
            std::unordered_map<int64_t, double> queue_members_map;
            
            // Key is the node index in the provided graph
            // Value is a pair<backpointer, cost-to-come>
            // backpointer is the parent index in the provided graph
            std::unordered_map<int64_t, std::pair<int64_t, double>> explored;
            
            // Initialize
            queue.push(AstarPQueueElement(start_index, -1, 0.0, heuristic_function(start_index)));
            if (limit_pqueue_duplicates)
            {
                queue_members_map[start_index] = 0.0;
            }
            
            // Search
            while (queue.size() > 0)
            {
                // Get the top of the priority queue
                const arc_helpers::AstarPQueueElement n = queue.top();
                queue.pop();

                if (n.id() == goal_index)
                {
                    // Solution found
                    explored[n.id()] = std::make_pair(n.backpointer(), n.costToCome());
                    break;
                }

                if (limit_pqueue_duplicates)
                {
                    queue_members_map.erase(n.id());
                }
                
                if (explored.count(n.id()) && n.costToCome() >= explored[n.id()].second)
                {
                    continue;
                }
                
                // Add to the explored list
                explored[n.id()] = std::make_pair(n.backpointer(), n.costToCome());
                
                
                // Explore and add the children
                for(const GraphEdge& current_out_edge: graph.getNode(n.id()).getOutEdges())
                {
                    // Get the next potential child node
                    const int64_t child_id = current_out_edge.getToIndex();

                    if (!edge_validity_check_fn(graph, current_out_edge))
                    {
                        continue;
                    }
                    
                    // Compute the cost-to-come for the new child
                    const double child_cost_to_come = n.costToCome() + distance_fn(graph, current_out_edge);

                    if(explored.count(child_id) &&
                       child_cost_to_come >= explored[child_id].second)
                    {
                        continue;
                    }

                    if (limit_pqueue_duplicates && queue_members_map.count(child_id) &&
                        child_cost_to_come >= queue_members_map[child_id])
                    {
                        continue;
                    }
                    
                    const double child_value = child_cost_to_come + heuristic_function(child_id);
                    queue.push(AstarPQueueElement(child_id, n.id(), child_cost_to_come, child_value));
                }
            }
            return ExtractAstarResult(explored, start_index, goal_index);
        }

        static arc_helpers::AstarResult PerformLazyAstar(
                const Graph<NodeValueType, Allocator>& graph,
                const int64_t start_index,
                const int64_t goal_index,
                const std::function<bool(const NodeValueType&, const NodeValueType&)>& edge_validity_check_fn,
                const std::function<double(const NodeValueType&, const NodeValueType&)>& distance_fn,
                const std::function<double(const NodeValueType&, const NodeValueType&)>& heuristic_fn,
                const bool limit_pqueue_duplicates)
        {
            const auto edge_validity_check_function = [&] (const Graph<NodeValueType,
                                                           Allocator>& search_graph, const GraphEdge& edge)
            {
                return edge_validity_check_fn(search_graph.getNode(edge.getFromIndex()).getValue(),
                                              search_graph.getNode(edge.getToIndex()).getValue());
            };
            const auto distance_function = [&] (const Graph<NodeValueType, Allocator>& search_graph,
                                                const GraphEdge& edge)
            {
                return distance_fn(search_graph.getNode(edge.getFromIndex()).getValue(),
                                   search_graph.getNode(edge.getToIndex()).getValue());
            };
            return PerformLazyAstar(graph, start_index, goal_index, edge_validity_check_function,
                                    distance_function, heuristic_fn, limit_pqueue_duplicates);
        }

        static arc_helpers::AstarResult PerformAstar(
                const Graph<NodeValueType, Allocator>& graph,
                const int64_t start_index,
                const int64_t goal_index,
                const std::function<double(const NodeValueType&, const NodeValueType&)>& heuristic_fn,
                const bool limit_pqueue_duplicates)
        {
            const auto edge_validity_check_function = [&] (const Graph<NodeValueType,
                                                           Allocator>& search_graph, const GraphEdge& edge)
            {
                UNUSED(search_graph);
                if(edge.getValidity() == EDGE_VALIDITY::INVALID)
                {
                    return false;
                }

                return edge.getWeight() < std::numeric_limits<double>::infinity();
            };
            const auto distance_function = [&] (const Graph<NodeValueType, Allocator>& search_graph, 
                                                const GraphEdge& edge)
            {
                UNUSED(search_graph);
                return edge.getWeight();
            };
            return PerformLazyAstar(graph, start_index, goal_index, edge_validity_check_function,
                                    distance_function, heuristic_fn, limit_pqueue_duplicates);
        }
    };

    template<typename NodeValueType, typename Allocator = std::allocator<NodeValueType>>
    class GraphRandomWalk
    {
    protected:
        GraphRandomWalk() {}

    public:

        template <typename Generator>
        static std::vector<int64_t> PerformRandomWalk(
                const Graph<NodeValueType, Allocator>& graph,
                const int64_t start_index,
                const int64_t goal_index,
                Generator& generator)
        {
            std::uniform_int_distribution<int64_t> uniform_int_distribution;

            std::vector<int64_t> path(1, start_index);

            while (path.back() != goal_index)
            {
                // Collect data from the current node
                const int64_t curr_index = path.back();
                const auto& out_edges = graph.getNode(curr_index).getOutEdges();
                const auto num_edges = out_edges.size();

                // Determine which node to step to next
                std::uniform_int_distribution<int64_t>::param_type params(0, (int64_t)(num_edges - 1));
                uniform_int_distribution.param(params);
                const int64_t next_step = uniform_int_distribution(generator);
                const auto next_index = out_edges.at(next_step).getToIndex();

                // If the next index is somewhere we've been already, then "trim" the loop off
                const auto it = std::find(path.begin(), path.end(), next_index);
                path.erase(it, path.end());

                // (Re)add the new index to the path
                path.push_back(next_index);
            }

            return path;
        }
    };
}

#endif // DIJKSTRAS_HPP

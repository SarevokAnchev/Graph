#pragma once

#include <vector>
#include <string>
#include <functional>

#include "../../src/Node.h"

class Graph
{
private:
    std::vector<Node> nodes;
    std::vector<std::pair<std::pair<uint32_t, uint32_t>, float>> edges;

public:
    Graph() = default;

    uint32_t add_node(float);

    void set_node(uint32_t, float);
    float get_node(uint32_t) const;
    void remove_edge(uint32_t, uint32_t);

    inline size_t nnodes() const noexcept { return nodes.size(); }

    void set_edge(uint32_t, uint32_t, float);
    float get_edge(uint32_t, uint32_t) const;
    std::vector<std::pair<std::pair<uint32_t, uint32_t>, float>> get_edges() const { return edges; }

    void disconnect_node(uint32_t);

    [[nodiscard]] std::vector<uint32_t> shortest_path(uint32_t, uint32_t) const;

    [[nodiscard]] uint32_t farthest_node(uint32_t) const;

    [[nodiscard]] std::vector<std::vector<uint32_t>> connected_components() const noexcept;

    [[nodiscard]] Graph min_spanning_tree() const;

    [[nodiscard]] Graph subgraph(const std::vector<uint32_t>& selection) const;

    [[nodiscard]] std::vector<float> distances_to(uint32_t) const;

    [[nodiscard]] std::vector<uint32_t> felzenszwalb_segmentation(double scale, uint32_t& n_segments) const;

    [[nodiscard]] std::vector<int> kmeans_clustering(uint32_t n_segments, double compactness) const;

    [[nodiscard]] std::vector<double> laplace(unsigned int order, std::vector<float> weights) const;

    [[nodiscard]] std::vector<float> apply(std::function<float(const Node&, const Graph&)> kernel) const;

    [[nodiscard]] double density() const;
};


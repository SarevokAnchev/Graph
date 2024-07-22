#include <Graph/Graph.h>

#include <limits>
#include <stdexcept>
#include <algorithm>
#include <utility>
#include <cmath>
#include <list>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include "Node.h"

uint32_t Graph::add_node(float weight)
{
    uint32_t node_id = nodes.size();
    Node new_node(node_id, weight);
    nodes.push_back(new_node);
    return node_id;
}

void Graph::set_node(uint32_t id, float weight)
{
    for (auto& n : nodes) {
        if (n.get_id() == id) {
            n.set_weight(weight);
            return;
        }
    }
}

float Graph::get_node(uint32_t id) const
{
    for (size_t i = 0; i < nodes.size(); i++) {
        if (nodes[i].get_id() == id) {
            return nodes[i].get_weight();
        }
    }
    throw std::invalid_argument("Node does not exist.");
}

void Graph::remove_edge(uint32_t n1, uint32_t n2)
{
    int idx = -1;
    for (int i = 0; i < edges.size(); i++) {
        if (edges[i].first.first == n1 && edges[i].first.second == n2
            || edges[i].first.first == n2 && edges[i].first.second == n1) {
            idx = i;
            break;
        }
    }
    if (idx >= 0) {
        edges.erase(edges.begin() + idx);
        nodes[n1].remove_neighbor(n2);
        nodes[n2].remove_neighbor(n1);
    }
}

void Graph::set_edge(uint32_t n1, uint32_t n2, float weight)
{
    // FIXME : takes too long (too many checks)
    uint32_t n1_idx = 0;
    bool n1_found = false;
    uint32_t n2_idx = 0;
    bool n2_found = false;
    for (size_t i = 0; i < nodes.size(); i++) {
        if (nodes[i].get_id() == n1) {
            n1_idx = i;
            n1_found = true;
        }
        if (nodes[i].get_id() == n2) {
            n2_idx = i;
            n2_found = true;
        }
        if (n1_found && n2_found) break;
    }
    if (!n1_found)
        n1_idx = add_node(1.);
    if (!n2_found)
        n2_idx = add_node(1.);

    // check if edge already exists
    bool edge_exists = false;
    for (const auto & i : nodes[n1_idx].get_neighbors()) {
        if (i.first == n2_idx) {
            edge_exists = true;
            break;
        }
    }

    if (edge_exists) {
        // find edge and update weight
        for (auto & edge : edges) {
            if (edge.first.first == n1_idx && edge.first.second == n2_idx
                || edge.first.first == n2_idx && edge.first.second == n1_idx) {
                edge.second = weight;
                break;
            }
        }
    }
    else {
        // create and add edge
        edges.emplace_back(std::make_pair(n1_idx, n2_idx), weight);
    }
    // update nodes neighbors
    nodes[n1_idx].add_neighbor(n2_idx, weight);
    nodes[n2_idx].add_neighbor(n1_idx, weight);
}

float Graph::get_edge(uint32_t n1, uint32_t n2) const
{
    for (const auto& e: edges) {
        if (e.first.first == n1 && e.first.second == n2) {
            return e.second;
        }
    }
    throw std::invalid_argument("Edge does not exist.");
}

void Graph::disconnect_node(uint32_t node_id)
{
    // find edges
    std::vector<unsigned int> to_remove;
    std::vector<uint32_t> neighbors;
    for (unsigned int i = 0; i < edges.size(); i++) {
        if (edges[i].first.first == node_id) {
            to_remove.push_back(i);
            neighbors.push_back(edges[i].first.second);
        }
        else if (edges[i].first.second == node_id) {
            to_remove.push_back(i);
            neighbors.push_back(edges[i].first.first);
        }
    }
    // remove neighbors
    for (auto n : neighbors) {
        try {
            nodes[n].remove_neighbor(node_id);
        }
        catch (std::exception& e) {}
    }
    nodes[node_id].clear_neighbors();
    // remove edges
    for (auto it = to_remove.rbegin(); it < to_remove.rend(); it++) {
        if (*it != edges.size() - 1) {
            std::swap(edges[*it], edges.back());
        }
        edges.pop_back();
    }
}

std::vector<uint32_t> Graph::shortest_path(uint32_t start_node, uint32_t stop_node) const
{
    std::vector<float> distances(nodes.size(), std::numeric_limits<float>::infinity());
    std::vector<bool> explored(nodes.size(), false);
    std::vector<uint32_t> predecessors(nodes.size(), 0);

    explored[start_node] = true;

    auto& start_neighbors = nodes[start_node].get_neighbors();
    for (const auto & start_neighbor : start_neighbors) {
        distances[start_neighbor.first] = start_neighbor.second;
        predecessors[start_neighbor.first] = start_node;
    }

    while (true) {
        auto min_dist = distances[0];
        uint32_t closest_point = 0;
        for (int i = 1; i < distances.size(); i++) {
            if (distances[i] < min_dist) {
                closest_point = i;
                min_dist = distances[i];
            }
        }
        if (min_dist == std::numeric_limits<float>::infinity())
            throw std::invalid_argument("No existing path between given nodes.");

        distances[closest_point] = std::numeric_limits<float>::infinity();
        explored[closest_point] = true;
        if (closest_point == stop_node) // found the path
            break;

        auto& new_node_neighbors = nodes[closest_point].get_neighbors();
        for (const auto & new_node_neighbor : new_node_neighbors) {
            auto node = new_node_neighbor.first;
            if (!explored[node]) {
                auto dist = new_node_neighbor.second;
                float new_dist = min_dist + dist;
                if (new_dist < distances[node]) {
                    distances[node] = new_dist;
                    predecessors[node] = closest_point;
                }
            }
        }
    }

    std::vector<uint32_t> ret_path;
    auto cur_node = stop_node;
    while (cur_node != start_node) {
        ret_path.push_back(cur_node);
        cur_node = predecessors[cur_node];
    }
    ret_path.push_back(start_node);
    std::reverse(ret_path.begin(), ret_path.end());
    return ret_path;
}

/// Uses breadth first search to search for the farthest node from a given starting point
/// @param start_node starting node
/// @return farthest node
uint32_t Graph::farthest_node(uint32_t start_node) const
{
    std::vector<float> distances(nodes.size(), std::numeric_limits<float>::infinity());
    std::vector<bool> explored(nodes.size(), false);
    uint32_t last_explored = start_node;

    explored[start_node] = true;

    auto& start_neighbors = nodes[start_node].get_neighbors();
    for (const auto & start_neighbor : start_neighbors) {
        distances[start_neighbor.first] = start_neighbor.second;
    }

    while (true) {
        auto min_dist = distances[0];
        uint32_t closest_point = 0;
        for (int i = 1; i < distances.size(); i++) {
            if (distances[i] < min_dist) {
                closest_point = i;
                min_dist = distances[i];
            }
        }
        if (min_dist == std::numeric_limits<float>::infinity())
            return last_explored;

        last_explored = closest_point;
        distances[closest_point] = std::numeric_limits<float>::infinity();
        explored[closest_point] = true;

        auto& new_node_neighbors = nodes[closest_point].get_neighbors();
        for (const auto & new_node_neighbor : new_node_neighbors) {
            auto node = new_node_neighbor.first;
            if (!explored[node]) {
                auto dist = new_node_neighbor.second;
                float new_dist = min_dist + dist;
                if (new_dist < distances[node]) {
                    distances[node] = new_dist;
                }
            }
        }
    }
}


std::vector<std::vector<uint32_t>> Graph::connected_components() const noexcept
{
    std::vector<bool> explored(nodes.size(), false);
    std::vector<std::vector<uint32_t>> components;
    auto n_unexplored = nodes.size();
    while (n_unexplored > 0) {
        std::vector<uint32_t> component;
        std::vector<float> distances(nodes.size(), std::numeric_limits<float>::infinity());
        float next_dist = 0.;
        uint32_t next_node = 0;
        for (uint32_t i = 0; i < explored.size(); i++) {
            if (!explored[i]) {
                next_node = i;
                break;
            }
        }
        while (next_dist < std::numeric_limits<float>::infinity()) {
            explored[next_node] = true;
            distances[next_node] = std::numeric_limits<float>::infinity();
            n_unexplored--;
            component.push_back(next_node);
            auto& neighbors = nodes[next_node].get_neighbors();
            for (auto it = neighbors.begin(); it < neighbors.end(); it++) {
                if (!explored[it->first] && it->second < distances[it->first]) {
                    distances[it->first] = it->second;
                }
            }
            next_dist = std::numeric_limits<float>::infinity();
            for (auto it = distances.begin(); it < distances.end(); it++) {
                if (*it < std::numeric_limits<float>::infinity()) {
                    next_node = it - distances.begin();
                    next_dist = *it;
                    break;
                }
            }
        }
        components.push_back(component);
    }
    return components;
}

Graph Graph::min_spanning_tree() const
{
    Graph bench;
    for (const auto& n : nodes) {
        bench.add_node(n.get_weight());
    }
    std::vector<std::pair<uint32_t, double>> neighbors(nnodes(), std::make_pair(0,std::numeric_limits<double>::infinity()));
    for (const auto& n : nodes[0].get_neighbors()) {
        neighbors[n.first].first = 0;
        neighbors[n.first].second = n.second;
    }
    int added_nodes = 1;
    while (added_nodes < nnodes()) {
        auto min_dist = std::numeric_limits<double>::infinity();
        uint32_t next = 0;
        for (size_t i = 0; i < neighbors.size(); i++) {
            if (!bench.nodes[i].get_neighbors().empty()) {
                continue;
            }
            const auto& n = neighbors[i];
            if (n.second < min_dist) {
                min_dist = n.second;
                next = i;
            }
        }
        if (min_dist == std::numeric_limits<double>::infinity()) {
            throw std::invalid_argument("The graph has multiple components.");
        }
        bench.set_edge(neighbors[next].first, next, (float)neighbors[next].second);
        for (const auto& n : nodes[next].get_neighbors()) {
            if (n.second < neighbors[n.first].second) {
                neighbors[n.first].first = next;
                neighbors[n.first].second = n.second;
            }
        }
        added_nodes++;
    }
    return bench;
}

/// Returns a subgraph containing a selection of nodes and the edges that exist between these nodes.
/// The order of the ids provided in selection is preserved in the resulting graph, with ids starting from 0.
/// @param selection the ids of the selected nodes
/// @return the subgraph containing the selected nodes and edges
Graph Graph::subgraph(const std::vector<uint32_t>& selection) const
{
    Graph g;
    std::vector<bool> selected(nnodes(), false);
    std::vector<uint32_t> new_ids(nnodes(), std::numeric_limits<uint32_t>::infinity());
    for (const auto& n: selection) {
        selected[n] = true;
        new_ids[n] = g.nnodes();
        g.add_node(nodes[n].get_weight());
    }
    for (const auto& e: edges) {
        if (selected[e.first.first] && selected[e.first.second]) {
            g.set_edge(new_ids[e.first.first], new_ids[e.first.second], e.second);
        }
    }
    return g;
}

/// Computes the distance of any node to the given node in the graph
/// @param start_node id of the starting node
/// @return distance of every node to the starting node
std::vector<float> Graph::distances_to(uint32_t start_node) const
{
    std::vector<float> distances(nodes.size(), std::numeric_limits<float>::infinity());
    std::vector<bool> explored(nodes.size(), false);

    distances[start_node] = 0.;
    explored[start_node] = true;

    auto& start_neighbors = nodes[start_node].get_neighbors();
    for (const auto & start_neighbor : start_neighbors) {
        distances[start_neighbor.first] = start_neighbor.second;
    }

    while (true) {
        auto min_dist = std::numeric_limits<float>::infinity();
        uint32_t closest_point = 0;
        for (int i = 0; i < distances.size(); i++) {
            if (!explored[i] && distances[i] < min_dist) {
                closest_point = i;
                min_dist = distances[i];
            }
        }
        if (min_dist == std::numeric_limits<float>::infinity())
            return distances;

        explored[closest_point] = true;

        auto& new_node_neighbors = nodes[closest_point].get_neighbors();
        for (const auto & new_node_neighbor : new_node_neighbors) {
            auto node = new_node_neighbor.first;
            if (!explored[node]) {
                auto dist = new_node_neighbor.second;
                float new_dist = min_dist + dist;
                if (new_dist < distances[node]) {
                    distances[node] = new_dist;
                }
            }
        }
    }
}

/// computes a sgementation of the graph using Felzenszwalb algorithm
/// @param scale parameter controlling the tolerance for merging superpixels
/// @param n_segments output parameter for the number of segments computed
/// @return label for each node of the graph
std::vector<uint32_t> Graph::felzenszwalb_segmentation(double scale, uint32_t& n_segments) const
{
    std::vector<uint32_t> segments(nnodes());
    for (int i = 0; i < segments.size(); i++) {
        segments[i] = i;
    }
    std::vector<uint32_t> segment_size(nnodes(), 1);
    std::vector<float> max_edges(nnodes(), std::numeric_limits<float>::infinity());
    std::vector<std::pair<std::pair<uint32_t, uint32_t>, float>> sorted_edges(edges.size());
    std::copy(edges.begin(), edges.end(), sorted_edges.begin());
    std::sort(sorted_edges.begin(), sorted_edges.end(), [](const auto &e1, const auto &e2) {
        return e1.second < e2.second;
    });
    for (const auto & e : sorted_edges) {
        const auto g1 = segments[e.first.first];
        const auto g2 = segments[e.first.second];
        if (g1 == g2) continue;
        // compute minimum internal difference
        const auto int1 = max_edges[e.first.first];
        const auto int2 = max_edges[e.first.second];
        const auto m_int = std::min(int1 + scale/segment_size[e.first.first], int2 + scale/segment_size[e.first.second]);
        const auto weight = e.second;
        if (weight < m_int) {
            // merge the segments
            for (int n = 0; n < segments.size(); n++) {
                if (segments[n] == g2) {
                    segments[n] = g1;
                }
                if (segments[n] == g1) {
                    max_edges[n] = weight;
                }
            }
        }
    }
    std::vector<uint32_t> classes;
    for (auto& s : segments) {
        auto seg = std::find(classes.begin(), classes.end(), s);
        if (seg == classes.end()) {
            classes.emplace_back(s);
            s = classes.size() - 1;
        }
        else {
            s = seg - classes.begin();
        }
    }
    n_segments = classes.size();
    return segments;
}

/// segments the graph wrt the weights of its nodes, a compactness parameter enables balancing of the
/// intra-component variability of node weights with distance from the origin node.
/// The origin nodes are chosen regularly according to their ids.
/// @param n_segments number of desired components
/// @param compactness factor by which the edge weights will be multiplied in computation of distance to a component.
/// @return label for each node of the graph
std::vector<int> Graph::kmeans_clustering(uint32_t n_segments, double compactness) const
{
    // WIP : kmeans graph clustering
    std::vector<float> origin_values(n_segments);
    std::vector<int> origin_nodes(n_segments);
    std::vector<int> segments(nnodes(), -1);

    std::vector<std::vector<float>> edge_distance_to_origin(nnodes(), std::vector<float>(n_segments, std::numeric_limits<float>::infinity()));
    std::vector<std::list<std::pair<uint32_t, float>>> neighbors(n_segments);
    std::vector<double> average_values(n_segments, 0);

    for (int i = 0; i < n_segments; i++) {
        auto id = i*nnodes()/n_segments;
        segments[id] = i;
        origin_nodes[i] = id;
    }

    int iterations = 10;

    for (int it = 0; it  < iterations; it++) {

        float shortest_dist = std::numeric_limits<float>::infinity();
        int comp = -1;
        int closest = -1;

        // initialization with new origin nodes
        for (int i = 0; i < n_segments; i++) {
            auto node = nodes[origin_nodes[i]];
            origin_values[i] = node.get_weight();
            for (const auto& n : node.get_neighbors()) {
                // if node already in a segment
                if (segments[n.first] >= 0) continue;

                edge_distance_to_origin[n.first][i] = n.second;
                float distance = std::abs(node.get_weight() - nodes[n.first].get_weight()) + (float)compactness*n.second;
                if (distance < shortest_dist) {
                    shortest_dist = distance;
                    comp = i;
                    closest = n.first;
                }
                auto neighbors_iterator = neighbors[i].begin();
                for (neighbors_iterator = neighbors[i].begin(); neighbors_iterator != neighbors[i].end() && neighbors_iterator->second < distance; ++neighbors_iterator) {}
                neighbors[i].emplace(neighbors_iterator, std::make_pair(n.first, distance));
            }
        }

        while (shortest_dist < std::numeric_limits<float>::infinity()) {
            // add the closest point to its segment
            segments[closest] = comp;

            // remove the added point from the neighbors lists
            for (int c = 0; c < n_segments; ++c) {
                for (auto it = neighbors[c].begin(); it != neighbors[c].end();) {
                    if (it->first == closest) {
                        it = neighbors[c].erase(it);
                    }
                    else {
                        ++it;
                    }
                }
            }

            // add the neighbors of the added point to the list
            auto node = nodes[closest];
            for (const auto& n : node.get_neighbors()) {
                // if node already in a segment
                if (segments[n.first] >= 0) continue;

                edge_distance_to_origin[n.first][comp] = std::min(n.second + edge_distance_to_origin[closest][comp], edge_distance_to_origin[n.first][comp]);
                float distance = std::abs(origin_values[comp] - nodes[n.first].get_weight()) + (float)compactness*edge_distance_to_origin[n.first][comp];
                auto it = neighbors[comp].begin();
                for (it = neighbors[comp].begin(); it != neighbors[comp].end() && it->second < distance; ++it) {}
                neighbors[comp].emplace(it, std::make_pair(n.first, distance));
            }

            // find the closest point
            shortest_dist = std::numeric_limits<float>::infinity();
            comp = -1;
            closest = -1;
            for (int c = 0; c < n_segments; ++c) {
                if (!neighbors[c].empty() && neighbors[c].cbegin()->second < shortest_dist) {
                    shortest_dist = neighbors[c].cbegin()->second;
                    comp = c;
                    closest = neighbors[c].cbegin()->first;
                }
            }
        }

        // prepare next iteration
        if (it + 1 < iterations) {
            for (int i = 0; i < n_segments; i++) {
                double average = 0;
                int n_elems = 0;
                for (int j = 0; j < segments.size(); j++) {
                    if (segments[j] == i) {
                        n_elems++;
                        average += nodes[j].get_weight();
                    }
                }
                average = average/n_elems;
                auto min_dist = std::numeric_limits<double>::infinity();
                int new_origin_node = 0;
                for (int j = 0; j < segments.size(); j++) {
                    if (segments[j] == i) {
                        auto dist = std::abs(average - nodes[j].get_weight());
                        if (dist < min_dist) {
                            min_dist = dist;
                            new_origin_node = j;
                        }
                        segments[j] = -1;

                        // reset exploration data
                        std::fill(edge_distance_to_origin[j].begin(), edge_distance_to_origin[j].end(), std::numeric_limits<float>::infinity());
                    }
                }
                origin_nodes[i] = new_origin_node;
                origin_values[i] = nodes[new_origin_node].get_weight();
                average_values[i] = origin_values[i];
                segments[new_origin_node] = i;
                neighbors[i].clear();
            }
        }
    }
    return segments;
}

std::vector<double> Graph::laplace(unsigned int order, std::vector<float> weights) const
{
    if (order == 0) {
        std::vector<double> res(nodes.size());
        for (int i = 0; i < nodes.size(); i++) {
            res[i] = weights[0]*nodes[i].get_weight();
        }
        return res;
    }
    Eigen::MatrixXd L(nodes.size(), nodes.size());
    L.fill(0);
    Eigen::VectorXd x(nodes.size());
    for (int i = 0; i < nodes.size(); i++) {
        const auto& n = nodes[i];
        x(i) = n.get_weight();
        const auto& neighbors = n.get_neighbors();
        double edges = 0.;
        for (const auto& n: neighbors) {
            edges += n.second;
            L(i, n.first) = -n.second;
            L(n.first, i) = -n.second;
        }
        L(i, i) = edges;
    }
    // normalize L
    auto eigenvalues= L.eigenvalues();
    double max_eigenvalue = eigenvalues(0, 0).real();
    for (int i = 0; i < eigenvalues.rows(); i++) {
        if (max_eigenvalue < eigenvalues(i, 0).real()) {
            max_eigenvalue = eigenvalues(i, 0).real();
        }
    }
    Eigen::MatrixXd id(nodes.size(), nodes.size());
    id.setIdentity();
    L = 2*L/max_eigenvalue - id;

    Eigen::MatrixXd p = weights[0]*L;
    for (int i = 0; i < order; i++) {
        Eigen::MatrixXd mat(L.pow(i+1));
        p = p + weights[i+1]*mat;
    }
    x = p*x;
    std::vector<double> res(nodes.size());
    for (int i = 0; i < nodes.size(); i++) {
        res[i] = x(i);
    }
    return res;
}

std::vector<float> Graph::apply(std::function<float(const Node&, const Graph&)> kernel) const
{
    std::vector<float> ret(nnodes());
    for (int i = 0; i < nnodes(); i++) {
        ret[i] = kernel(nodes[i], *this);
    }
    return ret;
}

double Graph::density() const
{
    if (nnodes() <= 1) return 0;
    return 2.*edges.size()/(nnodes()*(nnodes() - 1.));
}

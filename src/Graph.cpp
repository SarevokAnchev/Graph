#include <Graph/Graph.h>

#include <limits>
#include <stdexcept>
#include <algorithm>
#include <utility>

#include "Node.h"

uint32_t Graph::set_node(const std::string& name, float weight)
{
    for (uint32_t i = 0; i < names.size(); i++) {
        if (name == names[i]) {
            nodes[i].set_weight(weight);
            return i;
        }
    }
    uint32_t node_id = names.size();
    names.emplace_back(name);
    Node new_node(node_id, weight);
    nodes.push_back(new_node);
    return node_id;
}

void Graph::set_edge(const std::string& n1, const std::string& n2, float weight)
{
    // FIXME : takes too long (too many checks)
    uint32_t n1_idx = 0;
    bool n1_found = false;
    uint32_t n2_idx = 0;
    bool n2_found = false;
    for (uint32_t i = 0; i < names.size(); i++) {
        if (names[i] == n1) {
            n1_idx = i;
            n1_found = true;
        }
        if (names[i] == n2) {
            n2_idx = i;
            n2_found = true;
        }
        if (n1_found && n2_found) break;
    }
    if (!n1_found)
        n1_idx = set_node(n1, 1.);
    if (!n2_found)
        n2_idx = set_node(n2, 1.);

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

void Graph::disconnect_node(const std::string& name)
{
    uint32_t node_id;
    try {
        node_id = get_node_id(name);
    }
    catch (std::exception& e) {
        throw;
    }
    disconnect_node(node_id);
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
        uint32_t closest_point = std::min_element(distances.begin(), distances.end()) - distances.begin();
        auto min_dist = distances[closest_point];
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

std::vector<std::string> Graph::shortest_path(const std::string& n_start, const std::string& n_stop) const
{
    uint32_t start_node, stop_node;
    try {
        start_node = get_node_id(n_start);
        stop_node = get_node_id(n_stop);
    }
    catch (std::invalid_argument& e) {
        throw;
    }

    auto id_path = shortest_path(start_node, stop_node);

    std::vector<std::string> ret_path;
    for (auto it = id_path.begin(); it < id_path.end(); it++)
        ret_path.push_back(names[*it]);
    return ret_path;
}

uint32_t Graph::get_node_id(const std::string& node) const
{
    for (uint32_t i = 0; i < names.size(); i++) {
        if (names[i] == node) return i;
    }
    throw std::invalid_argument("Node name not in graph.");
}

std::vector<std::vector<std::string>> Graph::connected_components_by_names() const noexcept
{
    auto uint_comps = connected_components_by_ids();
    std::vector<std::vector<std::string>> components;
    for (auto & uint_comp : uint_comps) {
        std::vector<std::string> comp;
        for (auto it = uint_comp.begin(); it < uint_comp.end(); it++) {
            comp.push_back(names[*it]);
        }
        components.push_back(comp);
    }
    return components;
}

std::vector<std::vector<uint32_t>> Graph::connected_components_by_ids() const noexcept
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

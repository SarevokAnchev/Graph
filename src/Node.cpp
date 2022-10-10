#include "Node.h"

#include <stdexcept>

Node::Node(uint32_t n_id, float n_weight)
{
    id = n_id;
    weight = n_weight;
}

void Node::add_neighbor(uint32_t n_id, float e_weight) noexcept
{
    for (auto & neighbor : neighbors) {
        if (neighbor.first == n_id) {
            neighbor.second = e_weight;
            return;
        }
    }
    neighbors.emplace_back(n_id, e_weight);
}

void Node::remove_neighbor(uint32_t node)
{
    unsigned int to_remove = 0;
    for (to_remove = 0; to_remove < neighbors.size(); to_remove++) {
        if (neighbors[to_remove].first == node) break;
    }
    if (to_remove == neighbors.size()) throw std::invalid_argument("Neighbor not found.");
    if (to_remove == neighbors.size() - 1) {
        neighbors.pop_back();
    }
    else {
        neighbors[to_remove] = *(neighbors.end() - 1);
        neighbors.pop_back();
    }
}



#pragma once

#include <vector>
#include <cstdint>

class Node
{
private:
    float weight;
    uint32_t id;
    std::vector<std::pair<uint32_t, float>> neighbors;

public:
    Node(uint32_t, float);

    [[nodiscard]] inline const std::vector<std::pair<uint32_t, float>>& get_neighbors() const { return neighbors; }

    [[nodiscard]] inline uint32_t n_neighbors() const { return neighbors.size(); }

    void add_neighbor(uint32_t, float) noexcept;

    void remove_neighbor(uint32_t);

    inline void clear_neighbors() noexcept { neighbors.clear(); }

    inline void set_weight(float new_weight) noexcept { weight = new_weight; }

    [[nodiscard]] inline float get_weight() const noexcept { return weight; }

    [[nodiscard]] inline uint32_t get_id() const noexcept { return id; }
};


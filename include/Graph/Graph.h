#pragma once

#include <vector>
#include <string>

#include "../../src/Node.h"

class Graph
{
private:
    std::vector<std::string> names;
    std::vector<Node> nodes;
    std::vector<std::pair<std::pair<uint32_t, uint32_t>, float>> edges;

public:
    Graph() = default;

    uint32_t set_node(const std::string&, float);

    void set_edge(const std::string&, const std::string&, float);

    void disconnect_node(const std::string&);

    void disconnect_node(uint32_t);

    [[nodiscard]] std::vector<std::string> shortest_path(const std::string&, const std::string&) const;

    [[nodiscard]] std::vector<uint32_t> shortest_path(uint32_t, uint32_t) const;

    [[nodiscard]] uint32_t get_node_id(const std::string&) const;

    [[nodiscard]] inline std::string get_node_name(uint32_t id) const { return names[id]; }

    [[nodiscard]] std::vector<std::vector<std::string>> connected_components_by_names() const noexcept;

    [[nodiscard]] std::vector<std::vector<uint32_t>> connected_components_by_ids() const noexcept;

    [[nodiscard]] inline const std::vector<std::string>& get_names() const noexcept { return names; }
};


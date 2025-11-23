#include "Graph.hpp"
#include <iostream>
#include <vector>
#include <tuple>

int main() {
    Graph g;
    
    std::vector<std::string> vertices1 = { "1", "2", "3", "4", "5", "6" };
    for (size_t i = 0; i < vertices1.size(); i++) {
        g.addVertex(vertices1[i]);
    }
    
    std::vector<std::tuple<std::string, std::string, unsigned long>> edges1 = {
        {"1", "2", 7}, {"1", "3", 9}, {"1", "6", 14}, {"2", "3", 10},
        {"2", "4", 15}, {"3", "4", 11}, {"3", "6", 2}, {"4", "5", 6},
        {"5", "6", 9}
    };
    
    for (size_t i = 0; i < edges1.size(); i++) {
        g.addEdge(std::get<0>(edges1[i]), std::get<1>(edges1[i]), std::get<2>(edges1[i]));
    }
    
    std::vector<std::string> path;
    unsigned long distance = g.shortestPath("1", "5", path);
    
    std::cout << "Shortest distance from 1 to 5: " << distance << std::endl;
    std::cout << "Path: ";
    for (size_t i = 0; i < path.size(); i++) {
        std::cout << path[i];
        if (i < path.size() - 1) {
            std::cout << " -> ";
        }
    }
    std::cout << std::endl;
    
    return 0;
}


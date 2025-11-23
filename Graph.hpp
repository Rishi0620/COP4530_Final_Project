#ifndef GRAPH_H
#define GRAPH_H
#include "GraphBase.hpp"
#include <string>
#include <vector>
#include <map>

struct Edge {
    std::string destination;
    unsigned long weight;
    Edge(std::string dest, unsigned long w) : destination(dest), weight(w) {}
};

struct Vertex {
    std::string label;
    std::vector<Edge> edges;
    Vertex(std::string lbl) : label(lbl) {}
};

struct PriorityQueueNode {
    std::string vertex;
    unsigned long distance;
    std::string previous;
    PriorityQueueNode(std::string v, unsigned long d, std::string p) : vertex(v), distance(d), previous(p) {}
};

class PriorityQueue {
private:
    std::vector<PriorityQueueNode> heap;
    // This function maintains heap property by moving a node up the heap
    void heapifyUp(int index);
    // This function maintains heap property by moving a node down the heap
    void heapifyDown(int index);
    // This function returns the parent index of a given index
    int getParent(int index);
    // This function returns the left child index of a given index
    int getLeftChild(int index);
    // This function returns the right child index of a given index
    int getRightChild(int index);
public:
    // This function adds a node to the priority queue
    void push(PriorityQueueNode node);
    // This function removes and returns the node with minimum distance
    PriorityQueueNode pop();
    // This function checks if the priority queue is empty
    bool isEmpty();
    // This function updates the distance of a vertex in the priority queue
    void updateDistance(std::string vertex, unsigned long newDistance, std::string previous);
    // This function checks if a vertex exists in the priority queue
    bool contains(std::string vertex);
    // This function finds and returns a node with the given vertex label
    PriorityQueueNode find(std::string vertex);
};

class Graph : public GraphBase {
private:
    std::map<std::string, Vertex*> vertices;
    // This function deletes all vertices and clears the graph
    void cleanup();
public:
    // This function constructs an empty graph
    Graph();
    // This function destructs the graph and frees all memory
    ~Graph();
    // This function adds a vertex with the given label to the graph
    void addVertex(std::string label) override;
    // This function removes a vertex with the given label from the graph
    void removeVertex(std::string label) override;
    // This function adds an edge between two vertices with the given weight
    void addEdge(std::string label1, std::string label2, unsigned long weight) override;
    // This function removes an edge between two vertices
    void removeEdge(std::string label1, std::string label2) override;
    // This function calculates the shortest path between two vertices using Dijkstra's algorithm
    unsigned long shortestPath(std::string startLabel, std::string endLabel, std::vector<std::string> &path) override;
};

#endif


#include "Graph.hpp"
#include <algorithm>
#include <climits>
#include <iostream>

// This function constructs an empty graph
Graph::Graph() {
}

// This function destructs the graph and frees all memory
Graph::~Graph() {
    cleanup();
}

// This function deletes all vertices and clears the graph
void Graph::cleanup() {
    for (auto it = vertices.begin(); it != vertices.end(); ++it) {
        delete it->second;
    }
    vertices.clear();
}

// This function adds a vertex with the given label to the graph
void Graph::addVertex(std::string label) {
    if (vertices.find(label) == vertices.end()) {
        vertices[label] = new Vertex(label);
    }
}

// This function removes a vertex with the given label from the graph
void Graph::removeVertex(std::string label) {
    if (vertices.find(label) != vertices.end()) {
        Vertex* vertexToRemove = vertices[label];
        
        for (auto it = vertices.begin(); it != vertices.end(); ++it) {
            if (it->first != label) {
                Vertex* v = it->second;
                for (auto edgeIt = v->edges.begin(); edgeIt != v->edges.end();) {
                    if (edgeIt->destination == label) {
                        edgeIt = v->edges.erase(edgeIt);
                    } else {
                        ++edgeIt;
                    }
                }
            }
        }
        
        delete vertexToRemove;
        vertices.erase(label);
    }
}

// This function adds an edge between two vertices with the given weight
void Graph::addEdge(std::string label1, std::string label2, unsigned long weight) {
    if (label1 == label2) {
        return;
    }
    
    if (vertices.find(label1) == vertices.end() || vertices.find(label2) == vertices.end()) {
        return;
    }
    
    Vertex* v1 = vertices[label1];
    Vertex* v2 = vertices[label2];
    
    for (size_t i = 0; i < v1->edges.size(); i++) {
        if (v1->edges[i].destination == label2) {
            return;
        }
    }
    
    v1->edges.push_back(Edge(label2, weight));
    v2->edges.push_back(Edge(label1, weight));
}

// This function removes an edge between two vertices
void Graph::removeEdge(std::string label1, std::string label2) {
    if (vertices.find(label1) == vertices.end() || vertices.find(label2) == vertices.end()) {
        return;
    }
    
    Vertex* v1 = vertices[label1];
    Vertex* v2 = vertices[label2];
    
    for (auto it = v1->edges.begin(); it != v1->edges.end();) {
        if (it->destination == label2) {
            it = v1->edges.erase(it);
            break;
        } else {
            ++it;
        }
    }
    
    for (auto it = v2->edges.begin(); it != v2->edges.end();) {
        if (it->destination == label1) {
            it = v2->edges.erase(it);
            break;
        } else {
            ++it;
        }
    }
}

// This function calculates the shortest path between two vertices using Dijkstra's algorithm
unsigned long Graph::shortestPath(std::string startLabel, std::string endLabel, std::vector<std::string> &path) {
    path.clear();
    
    if (vertices.find(startLabel) == vertices.end() || vertices.find(endLabel) == vertices.end()) {
        return 0;
    }
    
    if (startLabel == endLabel) {
        path.push_back(startLabel);
        return 0;
    }
    
    std::map<std::string, unsigned long> distances;
    std::map<std::string, std::string> previous;
    PriorityQueue pq;
    
    for (auto it = vertices.begin(); it != vertices.end(); ++it) {
        std::string vertexLabel = it->first;
        unsigned long dist = (vertexLabel == startLabel) ? 0 : ULONG_MAX;
        distances[vertexLabel] = dist;
        previous[vertexLabel] = "";
        pq.push(PriorityQueueNode(vertexLabel, dist, ""));
    }
    
    while (!pq.isEmpty()) {
        PriorityQueueNode current = pq.pop();
        std::string currentLabel = current.vertex;
        
        if (currentLabel == endLabel) {
            break;
        }
        
        if (distances[currentLabel] == ULONG_MAX) {
            break;
        }
        
        Vertex* currentVertex = vertices[currentLabel];
        for (size_t i = 0; i < currentVertex->edges.size(); i++) {
            std::string neighborLabel = currentVertex->edges[i].destination;
            unsigned long edgeWeight = currentVertex->edges[i].weight;
            unsigned long newDistance = distances[currentLabel] + edgeWeight;
            
            if (newDistance < distances[neighborLabel]) {
                distances[neighborLabel] = newDistance;
                previous[neighborLabel] = currentLabel;
                pq.updateDistance(neighborLabel, newDistance, currentLabel);
            }
        }
    }
    
    if (distances[endLabel] == ULONG_MAX) {
        return 0;
    }
    
    std::string current = endLabel;
    while (current != "") {
        path.insert(path.begin(), current);
        current = previous[current];
    }
    
    return distances[endLabel];
}

// This function maintains heap property by moving a node up the heap
void PriorityQueue::heapifyUp(int index) {
    if (index == 0) {
        return;
    }
    
    int parentIndex = getParent(index);
    if (heap[index].distance < heap[parentIndex].distance) {
        std::swap(heap[index], heap[parentIndex]);
        heapifyUp(parentIndex);
    }
}

// This function maintains heap property by moving a node down the heap
void PriorityQueue::heapifyDown(int index) {
    int leftChild = getLeftChild(index);
    int rightChild = getRightChild(index);
    int smallest = index;
    
    if (leftChild < (int)heap.size() && heap[leftChild].distance < heap[smallest].distance) {
        smallest = leftChild;
    }
    
    if (rightChild < (int)heap.size() && heap[rightChild].distance < heap[smallest].distance) {
        smallest = rightChild;
    }
    
    if (smallest != index) {
        std::swap(heap[index], heap[smallest]);
        heapifyDown(smallest);
    }
}

// This function returns the parent index of a given index
int PriorityQueue::getParent(int index) {
    return (index - 1) / 2;
}

// This function returns the left child index of a given index
int PriorityQueue::getLeftChild(int index) {
    return 2 * index + 1;
}

// This function returns the right child index of a given index
int PriorityQueue::getRightChild(int index) {
    return 2 * index + 2;
}

// This function adds a node to the priority queue
void PriorityQueue::push(PriorityQueueNode node) {
    heap.push_back(node);
    heapifyUp(heap.size() - 1);
}

// This function removes and returns the node with minimum distance
PriorityQueueNode PriorityQueue::pop() {
    if (heap.empty()) {
        return PriorityQueueNode("", 0, "");
    }
    
    PriorityQueueNode min = heap[0];
    heap[0] = heap[heap.size() - 1];
    heap.pop_back();
    
    if (!heap.empty()) {
        heapifyDown(0);
    }
    
    return min;
}

// This function checks if the priority queue is empty
bool PriorityQueue::isEmpty() {
    return heap.empty();
}

// This function updates the distance of a vertex in the priority queue
void PriorityQueue::updateDistance(std::string vertex, unsigned long newDistance, std::string previous) {
    for (size_t i = 0; i < heap.size(); i++) {
        if (heap[i].vertex == vertex) {
            heap[i].distance = newDistance;
            heap[i].previous = previous;
            heapifyUp(i);
            return;
        }
    }
}

// This function checks if a vertex exists in the priority queue
bool PriorityQueue::contains(std::string vertex) {
    for (size_t i = 0; i < heap.size(); i++) {
        if (heap[i].vertex == vertex) {
            return true;
        }
    }
    return false;
}

// This function finds and returns a node with the given vertex label
PriorityQueueNode PriorityQueue::find(std::string vertex) {
    for (size_t i = 0; i < heap.size(); i++) {
        if (heap[i].vertex == vertex) {
            return heap[i];
        }
    }
    return PriorityQueueNode("", 0, "");
}


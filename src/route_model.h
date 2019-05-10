#pragma once

#include <limits>
#include <cmath>
#include <unordered_map>
#include "model.h"
#include <iostream>

class RouteModel : public Model {

  public:
    class Node : public Model::Node {
      public:
        float g_value = 0.f;
        float h_value = std::numeric_limits<float>::max();
        bool visited = false;
        Node *parent = nullptr;
        std::vector<Node *> neighbors;

        void FindNeighbors();
        float distance(Node) const;
        
        Node(){}
        Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {}
      
      private:
        int index;
        RouteModel * parent_model = nullptr;

        RouteModel::Node * FindNeighbor(std::vector<int>);
    };
    
    std::vector<Node> path; // This variable will eventually store the path that is found by the A* search.

    auto &SNodes() { return m_Nodes; };    
    auto &GetNodeToRoadMap() { return node_to_road; } // Used for testing functionality of node_to_road
    
    RouteModel(const std::vector<std::byte> &xml);
    Node &FindClosestNode(float, float);

  private:
    std::vector<Node> m_Nodes;
    std::unordered_map<int, std::vector<const Model::Road *> > node_to_road;
    
    void CreateNodeToRoadHashmap();

};
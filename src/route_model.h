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
        std::vector<Node *> neighbors;
        Node *parent = nullptr;
        bool visited = false;
        
        float distance(Node) const;
        void FindNeighbors();
        Node(){}
        Node(int idx, RouteModel *search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {}
      
      private:
        int index;
        RouteModel *parent_model = nullptr;

        RouteModel::Node * FindNeighbor(std::vector<int>);
    };
    
    std::vector<Node> path; // This variable will eventually store the path that is found by the A* search.

    auto &GetNodeToRoadMap() { return node_to_road; } // Used for testing functionality of node_to_road
    auto &SNodes() { return m_Nodes; };
    
    Node &FindClosestNode(float, float);
    RouteModel(const std::vector<std::byte> &xml);
  
  private:
    std::vector<Node> m_Nodes;
    std::unordered_map<int, std::vector<const Model::Road *> > node_to_road;
    
    void CreateNodeToRoadHashmap();
};
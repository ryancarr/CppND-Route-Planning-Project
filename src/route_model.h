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

        float distance(Node dest) const { return std::sqrt( std::pow(this->x - dest.x, 2)+ std::pow(this->y - dest.y, 2) ); }
        
        Node(){}
        Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {}
      
      private:
        // Add private Node variables and methods here.
        int index;
        RouteModel * parent_model = nullptr;
    };
    
    // Add public RouteModel variables and methods here.
    RouteModel(const std::vector<std::byte> &xml);
    auto &SNodes() { return m_Nodes; };
    std::vector<Node> path; // This variable will eventually store the path that is found by the A* search.

  private:
    std::vector<Node> m_Nodes;

};
#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"


class RoutePlanner {
  public:
    RoutePlanner(RouteModel &, float, float, float, float);
    float GetDistance() const { return distance; }
    void AStarSearch();

  private:
    RouteModel &m_Model;
    RouteModel::Node *start_node, *end_node;
    std::vector<RouteModel::Node *> open_list;
    float distance;

    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *);
    float CalculateHValue(const RouteModel::Node *);
    static bool Compare(const RouteModel::Node *, const RouteModel::Node *);
    RouteModel::Node *NextNode();
    void AddNeighbors(RouteModel::Node *);
    
};

#endif
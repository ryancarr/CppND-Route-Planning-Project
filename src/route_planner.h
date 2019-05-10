#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"


class RoutePlanner {
  public:
    void AStarSearch();    
    float GetDistance() const { return total_distance; }
    RoutePlanner(RouteModel &, float, float, float, float);
    

  private:
    float total_distance;
    RouteModel &m_Model;
    std::vector<RouteModel::Node *> open_list;
    RouteModel::Node *start_node, *end_node;

    void AddNeighbors(RouteModel::Node *);
    float CalculateHValue(const RouteModel::Node *);
    static bool Compare(const RouteModel::Node *, const RouteModel::Node *);
    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *);    
    RouteModel::Node *NextNode();
};
#endif
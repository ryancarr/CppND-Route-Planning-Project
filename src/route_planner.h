#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"


class RoutePlanner {
  public:
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);
    float GetDistance() const { return distance; }
    // Add public variables or methods declarations here.

  private:
    // Add private variables or methods declarations here.
    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *);
    RouteModel &m_Model;
    RouteModel::Node *start_node, *end_node;
    float distance;
};

#endif
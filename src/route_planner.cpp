#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model)
{
    m_Model = model;
    start_x *= 0.01;
    start_y *= 0.01;
    end_x   *= 0.01;
    end_y   *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

void RoutePlanner::AStarSearch()
{
    start_node->visited = true;
    open_list.emplace_back(start_node);

    RouteModel::Node *current_node = nullptr;
    while(open_list.size() > 0)
    {
        current_node = NextNode();

        if(current_node->distance(*end_node) == 0)
        {
            m_Model.path = ConstructFinalPath(current_node);
            return;            
        }
        else
        {
            AddNeighbors(current_node);
        }        
    }
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node)
{
    std::vector<RouteModel::Node> path_found;
    distance = 0.0f;

    while(current_node->parent != nullptr)
    {
        path_found.emplace_back(*current_node);
        distance += current_node->distance(*(current_node->parent));
        current_node = current_node->parent;
    }
    path_found.emplace_back(*current_node);
    distance *= m_Model.MetricScale();

    return path_found;
}

float RoutePlanner::CalculateHValue(const RouteModel::Node *node)
{
    return node->distance(*end_node);
}

RouteModel::Node *RoutePlanner::NextNode()
{
    // Sort open_list by f value
    std::sort(open_list.begin(), open_list.end(), Compare);

    RouteModel::Node *first = open_list.front();
    open_list.erase(open_list.begin());

    return first;
}

bool RoutePlanner::Compare(const RouteModel::Node *first, const RouteModel::Node *second)
{
    return first->g_value + first->h_value < second->g_value + second->h_value;
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node)
{
    current_node->FindNeighbors();

    for(auto neighbor : current_node->neighbors)
    {
        neighbor->parent  = current_node;
        neighbor->g_value = (current_node->g_value + neighbor->distance(*current_node));
        neighbor->h_value = CalculateHValue(neighbor);
        open_list.emplace_back(neighbor);
        neighbor->visited = true;
    }
}
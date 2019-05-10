#include "route_planner.h"
#include <algorithm>

/**
 * Find and add any neighbors of the current node to the open list
 * 
 * @param current_node Pointer to the current node we are evaluating
 */
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node)
{
    current_node->FindNeighbors();

    // Loop through each prospective neighbor and populate it's data
    for(auto neighbor : current_node->neighbors)
    {
        neighbor->parent  = current_node;

        // g_value is the cumulative distance from the start node to the current node
        neighbor->g_value = (current_node->g_value + neighbor->distance(*current_node));
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->visited = true;

        // Once the values have been set add the neighbor to the open list
        open_list.emplace_back(neighbor);        
    }
}

/**
 * Utilizes A* Algorithm to find the shortest path on the route
 */
void RoutePlanner::AStarSearch()
{
    start_node->visited = true;
    open_list.emplace_back(start_node);

    RouteModel::Node *current_node = nullptr;
    
    while(open_list.size() > 0)
    {
        current_node = NextNode();

        // Check if current_node is the end_node
        if(current_node->distance(*end_node) == 0)
        {
            m_Model.path = ConstructFinalPath(current_node);
            return;            
        }
        else
        {
            // Helps provide open_list with prospective nodes
            AddNeighbors(current_node);
        }        
    }
}

/**
 * Calculate remaining distance between current node and end_node
 * 
 * @param node Current node to check distance from
 * 
 * @return A float representing the distance from current_node to end_node
 */
float RoutePlanner::CalculateHValue(const RouteModel::Node *node)
{
    return node->distance(*end_node);
}

/**
 * Compare the f_value of two nodes
 * 
 * @param first  Pointer to first  Node to compare
 * @param second Pointer to second Node to compare
 * 
 * @return True if first f_value is less than second f_value
 * 
 */
bool RoutePlanner::Compare(const RouteModel::Node *first, const RouteModel::Node *second)
{
    // f_value is determined by adding g_value to h_value
    return (first->g_value + first->h_value) < (second->g_value + second->h_value);
}

/**
 * Constructs a path of nodes starting at the end
 * 
 * @param current_node Typically the end node of a route
 * 
 * @return The path of nodes from end to start
 */
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node)
{
    std::vector<RouteModel::Node> path_found;
    total_distance = 0.0f;

    // When current_node's parent is nullptr we know we've reached the start_node
    while(current_node->parent != nullptr)
    {
        path_found.emplace_back(*current_node);
        // Finds cumulative sum of distance between all nodes
        total_distance += current_node->distance(*(current_node->parent));

        // Move back one node in the route
        current_node = current_node->parent;
    }

    // Add start_node to path
    path_found.emplace_back(*current_node);

    // Convert distance to meters based on scale of map
    total_distance *= m_Model.MetricScale();

    return path_found;
}

/**
 * Find the next nearest node based on f value
 * 
 * @return Pointer to nearest node
 */
RouteModel::Node *RoutePlanner::NextNode()
{
    // Sort open_list by f value
    std::sort(open_list.begin(), open_list.end(), Compare);

    // Temporary variable to hold the position of the nearest node
    RouteModel::Node *first = open_list.front();

    // Remove node from open nodes to avoid revisiting it
    open_list.erase(open_list.begin());

    return first;
}

/**
 * Constructor for RoutePlanner
 * 
 * @param model   Model to use in this route
 * @param start_x Starting x location for route
 * @param start_y Starting y location for route
 * @param end_x   Ending x location for route
 * @param end_y   Ending y location for route
 */
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
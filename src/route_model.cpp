#include "route_model.h"
#include <iostream>

/**
 * Create a HashMap to associate nodes with their respective road
 */
void RouteModel::CreateNodeToRoadHashmap()
{
	// Loop over every road on the map
	for(auto &road : Roads())
	{
		if(road.type == Model::Road::Type::Footway) continue;

		// Store each node along a road in a hashmap
		for(auto node_idx : Ways()[road.way].nodes)
		{
			if(node_to_road.find(node_idx) == node_to_road.end()) // node_idx doesn't exist
				node_to_road[node_idx] = std::vector<const Model::Road *> {};
			
			node_to_road[node_idx].emplace_back(&road);
		}
	}
}

/**
 * Find Euclidean distance between two Nodes
 * 
 * @param dest The destination node
 * 
 * @return Float representing the distance between the x,y coordinates of both Nodes
 */
float RouteModel::Node::distance(Node dest) const
{
	return std::sqrt( std::pow(this->x - dest.x, 2) + std::pow(this->y - dest.y, 2) );
}

/**
 * Find the closest valid node to a given x and y coordinate
 * 
 * @param x Float representing a given x
 * @param y Float representing a given y
 * 
 * @return A reference to the node that best fits the x,y coordinates
 */
RouteModel::Node &RouteModel::FindClosestNode(float x, float y)
{
	Node temp_node;
	temp_node.x = x;
	temp_node.y = y;

	float min_distance = std::numeric_limits<float>::max();
	float distance;
	int closest_index;

	for(auto &road : Roads())
	{
		// Skip over any Footpaths, we only want roads
		if(road.type == Model::Road::Footway) continue;

		// Loop through the index of each node along each road
		for(auto node_index : Ways()[road.way].nodes)
		{
			distance = temp_node.distance(SNodes()[node_index]);

			if(distance < min_distance)
			{
				closest_index = node_index;
				min_distance = distance;
			}
		}
	}

	return SNodes()[closest_index];
}

/**
 * Find the nearest neighbor Node
 * 
 * @param node_indices Vector of ints containing every node_indice
 * 
 * @return Pointer to the closest node
 */
RouteModel::Node * RouteModel::Node::FindNeighbor(std::vector<int> node_indices)
{
	Node *closest_node = nullptr;
	Node node;

	for(int node_index : node_indices)
	{
		node = parent_model->SNodes()[node_index];

		if(this->distance(node) != 0 && !node.visited)
			if(closest_node == nullptr || this->distance(node) < this->distance(*closest_node))
				closest_node = &parent_model->SNodes()[node_index];
	}

	return closest_node;
}

/**
 * Finds the neighbor Nodes along all valid roads using FindNeighbor
 */
void RouteModel::Node::FindNeighbors()
{
	for(auto &road : parent_model->node_to_road[this->index])
	{
		RouteModel::Node * new_neighbor = this->FindNeighbor(parent_model->Ways()[road->way].nodes);

		if(new_neighbor) this->neighbors.emplace_back(new_neighbor);
	}
}

/**
 * Constructor for RouteModel
 * 
 * @param xml The xml file containing the map data
 */
RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) 
{
	int counter = 0;

	for(Model::Node node : this->Nodes())
	{
		m_Nodes.emplace_back(RouteModel::Node(counter, this, node));
		counter++;
	}

	CreateNodeToRoadHashmap();
}
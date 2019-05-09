#include "route_model.h"
#include <iostream>

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) 
{
	int counter = 0;

	for(Model::Node node : this->Nodes())
	{
		m_Nodes.push_back(RouteModel::Node(counter, this, node));
		counter++;
	}

	CreateNodeToRoadHashmap();
}

float RouteModel::Node::distance(Node dest) const
{
	// Find Euclidean distance between two Nodes
	return std::sqrt( std::pow(this->x - dest.x, 2) + std::pow(this->y - dest.y, 2) );
}

void RouteModel::CreateNodeToRoadHashmap()
{
	for(auto &road : Roads()) // Needs to reference the road
	{
		if(road.type == Model::Road::Type::Footway) continue;

		for(auto node_idx : Ways()[road.way].nodes)
		{
			if(node_to_road.find(node_idx) == node_to_road.end()) // node_idx doesn't exist
				node_to_road[node_idx] = std::vector<const Model::Road *> {};
			
			node_to_road[node_idx].push_back(&road);
		}
	}
}

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

void RouteModel::Node::FindNeighbors()
{
	for(auto &road : parent_model->node_to_road[this->index])
	{
		RouteModel::Node * new_neighbor = this->FindNeighbor(parent_model->Ways()[road->way].nodes);

		if(new_neighbor) this->neighbors.push_back(new_neighbor);
	}
}
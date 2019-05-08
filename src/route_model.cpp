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
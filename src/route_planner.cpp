#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes in the RoutePlanner's start_node and end_node attributes.
    m_Model = model;
    start_node = &model.FindClosestNode(start_x, start_y);
    end_node = &model.FindClosestNode(end_x, end_y);
}


// Use the distance to the end_node for the h value.
// Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// Expand the current node by adding all unvisited neighbors to the open list.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    for (auto neighbor : current_node->neighbors) {
        if (!neighbor->visited) {
            neighbor->parent = current_node;
            neighbor->h_value = CalculateHValue(neighbor);
            neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
            neighbor->visited = true;
            open_list.emplace_back(neighbor);
        }
    }
}


// Sort the open list and return the next node.

RouteModel::Node *RoutePlanner::NextNode() {
    // Sort the open_list according to the sum of the h value and g value.
    std::sort(open_list.begin(), open_list.end(), RouteModel::FCompare);

    // Create a pointer to the node in the list with the lowest sum.
    RouteModel::Node *result = open_list.back();
    
    // Remove that node from the open_list.
    open_list.pop_back();
    
    return result;
    // might be interesting to read for this part
    // http://ptgmedia.pearsoncmg.com/images/020163371x/supplements/Exception_Handling_Article.html
    // why pop_back() does not have a return value
}


// Return the final path found from A* search.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // This method iteratively follows the chain of parents of nodes until the starting node is found
    while (current_node != start_node) {
        path_found.emplace_back(*current_node);
        // For each node in the chain, add the distance from the node to its parent to the distance variable.
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }

    // Add the start node
    path_found.emplace_back(*this->start_node);
    // Reverse the vector so that the start node is the first element and the end node be the last.
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// A* Search algorithm.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // Initialize
    start_node->visited = true;
    open_list.emplace_back(start_node);

    while (open_list.size()) {
        // Use the NextNode() method to sort the open_list and return the next node.
        current_node = NextNode();
        if (current_node == end_node) {
            // When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
            // Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.
            m_Model.path = ConstructFinalPath(current_node);
            break;
        }

        // Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
        this->AddNeighbors(current_node);
    }




}
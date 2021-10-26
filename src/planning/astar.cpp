#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>


robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    
    //////////////////// A* initialize /////////////////////////////////
    // int map_width = distances.widthInCells();
    // int map_height = distances.heightInCells();

    cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
    int start_x = startCell.x;
    int start_y = startCell.y;
    int goal_x = goalCell.x;
    int goal_y = goalCell.y;

    Node* goalNode = new Node(goal_x, goal_y);
    goalNode->g_cost = 1.0E9;

    Node* startNode = new Node(start_x, start_y);
    startNode->h_cost = h_cost(startNode, goalNode);


    PriorityQueue open_list;
    open_list.push(startNode);
    std::vector<Node* > closed_list;
    std::vector<Node* > path_list;
    bool pathFound = false;
    ////////////////// initialize completed /////////
    Node* currentNode = open_list.pop();
    while (!(*currentNode == *goalNode) && 
           distances.isCellInGrid(currentNode->cell.x, currentNode->cell.y) &&
           !is_obstacle(currentNode, distances))
    {
        std::vector<Node* > neighbors;
        closed_list.push_back(currentNode);
        neighbors = expand_node(currentNode, open_list, distances, params);

        // set attributes for valid neighbors
        for (auto& neighbor : neighbors){
            int x = neighbor->cell.x;
            int y = neighbor->cell.y;
            if (!is_member_in_list(neighbor, closed_list) && 
                distances.isCellInGrid(x, y) && 
                !(distances(x, y) <= 1.5*params.minDistanceToObstacle) &&
                !is_obstacle(neighbor, distances))
            {
                if (!open_list.is_member(neighbor)){
                    neighbor->g_cost = g_cost(currentNode, neighbor, distances, params);
                    neighbor->h_cost = h_cost(neighbor, goalNode);
                    neighbor->parent = currentNode;
                    open_list.push(neighbor);
                }
                else if (neighbor->g_cost > g_cost(currentNode, neighbor, distances, params)){
                    neighbor->g_cost = g_cost(currentNode, neighbor, distances, params);
                    neighbor->parent = currentNode;
                    open_list.push(neighbor);
                }
            }
        }

        currentNode = open_list.pop();

        if ((*currentNode == *goalNode)){
            pathFound = true;
            std::cout << "find a path yeah!!!" << std::endl;
        }
    }
    
    path_list = extract_node_path(currentNode);

    
    robot_path_t path;

    if (pathFound){
        path.utime = start.utime;
        path.path = extract_pose_path(path_list, distances);
        path.path_length = path.path.size();
    }
    else{
        path.utime = start.utime;
        path.path.push_back(start);    
        path.path_length = path.path.size();
    }

    return path;
}

bool is_member_in_list(Node* n, std::vector<Node* >& list){
    for (auto& node : list){
        if (*n == *node){
            return true;
        }
    }
    return false;
}

bool is_obstacle(Node* n, const ObstacleDistanceGrid& grid){
    int x = n->cell.x;
    int y = n->cell.y;

    if (grid(x,y) == 0.0f){
        return true;
    }

    return false;
}

double h_cost(Node* from, Node* goal){
    int x = std::abs(goal->cell.x - from->cell.x);
    int y = std::abs(goal->cell.y - from->cell.y);
    double cost;

    if (x >= y){
        cost = 14*y + 10*(x-y);
    }
    else{
        cost = 14*x + 10*(y-x);
    }

    return cost;

}

double g_cost(Node* from, 
              Node* to, 
              const ObstacleDistanceGrid& distances, 
              const SearchParams& params)
{
    int x = std::abs(from->cell.x - to->cell.x);
    int y = std::abs(from->cell.y - to->cell.y);
    double cost;

    if (x == 1 && y == 1){
        cost = from->g_cost + 14;
    }
    else{
        cost = from->g_cost + 10;
    }

    return cost;
}

std::vector<Node* > expand_node(Node* node, 
                                PriorityQueue& open_list, 
                                const ObstacleDistanceGrid& distances, 
                                const SearchParams& params)
{
    const int WAYS = 8;
    int xDeltas[WAYS] = {1, -1, 0, 0, 1, 1, -1, -1};
    int yDeltas[WAYS] = {0, 0, 1, -1, 1, -1, 1, -1};
    std::vector<Node* > neighbors;
    for (int i = 0; i < WAYS; i++){
        Node* newNode = new Node(node->cell.x + xDeltas[i], node->cell.y + yDeltas[i]);
        Node* neighbor;
        if (open_list.is_member(newNode)){
            neighbor = open_list.get_member(newNode);
        }
        else{
            neighbor = newNode;
        }

        neighbors.push_back(neighbor);
    }

    return neighbors;
}

std::vector<Node* > extract_node_path(Node* node){
    Node* currentNode = node;
    std::vector<Node* > path;
    while (currentNode != NULL){
        path.push_back(currentNode);

        // std::cout << "(" << currentNode->cell.x << ", " << currentNode->cell.y << ") "
        //           << "h_cost: " << currentNode->h_cost << " g_cost: " 
        //           << currentNode->g_cost << std::endl;
        
        currentNode = currentNode->parent;
        
    }
    std::reverse(path.begin(), path.end());
    return path;
    
}

std::vector<pose_xyt_t> extract_pose_path(
        std::vector<Node* >& nodePath, 
        const ObstacleDistanceGrid& distances)
{
    std::vector<pose_xyt_t> posePath;

    // deal with x, y
    for (auto& n : nodePath){
        Point<double> globalPoint = grid_position_to_global_position(
                                     n->cell, 
                                     distances);
        pose_xyt_t currentPose;
        currentPose.x = globalPoint.x;
        currentPose.y = globalPoint.y;
        posePath.push_back(currentPose);
    }

    // deal with theta
    int posePathSize = posePath.size();
    for (int i = 0; i < posePathSize; i++){
        if (i == posePathSize-1){
            posePath[i].theta = 0.0;
            break;
        }
        double dx = posePath[i+1].x - posePath[i].x;
        double dy = posePath[i+1].y - posePath[i].y;
        posePath[i].theta = std::atan2(dy, dx);
    }

    return posePath;
}


#include <planning/astar.hpp>
//#include <planning/obstacle_distance_grid.hpp>


/*robot_path_t search_for_path(pose_xyt_t start,
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////
    
    robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);    
    path.path_length = path.path.size();
    return path;
}*/
bool is_reached(Node* node, Node* goal){
    return *node == *goal;
}

bool is_member_vector(Node* node, std::vector<Node*>& elements){
    for(auto n: elements){
        if(n == node){
            return true;
        }
    }
    return false;
}

//L2-Distance
double h_cost(Node* from, Node* goal) {
    int dx;
    int dy;
    dx = std::abs(from->cell.x - goal->cell.x);
    dy = std::abs(from->cell.y - goal->cell.y);
    return std::sqrt(dx*dx + dy*dy);
}

double g_cost(Node* from, Node* to, const ObstacleDistanceGrid& distances, const SearchParams& params){
    int dx;
    int dy;
    dx = to->cell.x - from->cell.x;
    dy = to->cell.y - from->cell.y;
    return from->g_cost+std::sqrt(dx*dx+dy*dy);
}

std::vector<Node*> expand_node(Node* node, const ObstacleDistanceGrid& distances, const SearchParams& params){
    //do an 8-way search;
    std::vector<Node*> children;
    const int xDeltas[8] = {1, 1, 1, 0, 0, -1, -1, -1};
    const int yDeltas[8] = {0, 1, -1, -1, 1, 1, -1, 0};
    for(int i=0;i<8;i++){
        int newX = node->cell.x+xDeltas[i];
        int newY = node->cell.y+yDeltas[i];
        if(not distances.isCellInGrid(newX, newY)) { continue; }
        //TODO:add some heuristic, for example obstacle

        //update new kid
        Node* adjacentNode = new Node(newX, newY);
        adjacentNode->parent = node;
        children.push_back(adjacentNode);
    }
    return children;
}

std::vector<Node*> extract_node_path(Node* node){
    std::vector<Node*> path;
    Node* n = node;
    path.push_back(n);
    while(n->parent != NULL){
        n = n->parent;
        path.push_back(n);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<pose_xyt_t> extract_pose_path(std::vector<Node*>& nodePath, const ObstacleDistanceGrid& distances){
    //convert node path to global pose_xyt;
    std::vector<pose_xyt_t> posepath_;
    for(auto n: nodePath){
        pose_xyt_t p;
        Point<double> pGlobal = grid_position_to_global_position(n->cell, distances);
        p.x = pGlobal.x;
        p.y = pGlobal.y;
        //what about theta and utime?
        posepath_.push_back(p);
    }
    return posepath_;
}

robot_path_t search_for_path(pose_xyt_t start,
                             pose_xyt_t goal,
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////

    //initialization
    //convert start, goal pose to the nodes in grid
    cell_t goalCell = global_position_to_grid_cell(Point<double>(goal.x, goal.y), distances);
    Node* goalNode = new Node(goalCell.x, goalCell.y);
    goalNode->g_cost = 1.0e16;
    goalNode->h_cost = 0.0;
    goalNode->parent = NULL;

    cell_t startCell = global_position_to_grid_cell(Point<double>(start.x, start.y), distances);
    Node* startNode = new Node(startCell.x, startCell.y);
    startNode->g_cost = 0.0;
    startNode->h_cost = h_cost(startNode, goalNode);
    startNode->parent = NULL;

    //set up a vector to store the node path of A* planning
    std::vector<Node*> nodepath_;

    if(is_reached(startNode, goalNode))
    {
        nodepath_ = extract_node_path(startNode);
    }
    else
    {
        PriorityQueue openList;
        std::vector<Node*> closedList;

        openList.push(startNode);

        bool find_path = false;
        while (not openList.empty()) {
            Node *n = openList.pop();
            std::vector<Node*> kids = expand_node(n, distances, params);
            for (auto kid_: kids) {
                // compute f of kid_
                kid_->g_cost = g_cost(n, kid_, distances, params);
                kid_->h_cost = h_cost(kid_, goalNode);

                if (is_reached(kid_, goalNode)) {
                    nodepath_ = extract_node_path(kid_);
                    find_path = true;
                    break;
                }
                if (not is_member_vector(kid_, closedList)) {
                    openList.push(kid_);
                }

            }

            closedList.push_back(n);

            if (find_path) {
                break;
            }
        }
        // nodepath_ = NULL
    }

    //get the node path
    //generate the robot path
    robot_path_t path;

    if(nodepath_.empty()){
        path.utime = start.utime;
        path.path.push_back(start);
        path.path_length = path.path.size();
        return path;
    }
    /*robot_path_t path;
    path.utime = start.utime;
    path.path.push_back(start);
    path.path_length = path.path.size();*/
    path.utime = start.utime;
    path.path = extract_pose_path(nodepath_, distances);
    path.path_length = path.path.size();
    return path;
}

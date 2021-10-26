#ifndef PLANNING_ASTAR_HPP
#define PLANNING_ASTAR_HPP

#include <lcmtypes/robot_path_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <common/point.hpp>
#include <common/grid_utils.hpp>
#include <queue>
#include <vector>
#include <algorithm>

typedef Point<int> cell_t;

struct Node{
    
    cell_t cell;
    Node* parent;
    double h_cost;
    double g_cost;

    Node(int x, int y): cell(x,y), g_cost(0.0), h_cost(0.0), parent(NULL){}
    double f_cost(void) const{return g_cost + h_cost;}
    bool operator==(const Node& rhs) const {return (cell == rhs.cell);}
};

struct CompareNode{
    bool operator() (Node* n1, Node* n2){
        if (n1->f_cost() == n2->f_cost()){
            return (n1->h_cost > n2->h_cost);
        }
        else{
            return (n1->f_cost() > n2->f_cost());
        }
    }
};

struct PriorityQueue{
    std::priority_queue<Node*, std::vector<Node*>, CompareNode> Q;
    std::vector<Node* > elements;

    //PriorityQueue(){}

    bool empty(){
        return Q.empty();
    }

    bool is_member(Node* n){
        for (auto& node : elements){
            if (*n == *node){
                return true;
            }
        }
        return false;
    }

    Node* get_member(Node* n){
        for (auto& node : elements){
            if (*n == *node){
                return node;
            }
        }

        return NULL;
    }

    void push(Node* n){
        elements.push_back(n);
        Q.push(n);
    }

    Node* pop(){
        int idx = -1;
        int elements_size = elements.size();
        Node* n = Q.top();
        Q.pop();
        for (int i = 0; i < elements_size; i++){
            if (elements[i] == n){
                idx = i;
                break;
            }
        }
        elements.erase(elements.begin() + idx);
        return n;
    }
};

class ObstacleDistanceGrid;

/**
* SearchParams defines the parameters to use when searching for a path. See associated comments for details
*/
struct SearchParams
{
    double minDistanceToObstacle;   ///< The minimum distance a robot can be from an obstacle before
                                    ///< a collision occurs
                                    
    double maxDistanceWithCost;     ///< The maximum distance from an obstacle that has an associated cost. The planned
                                    ///< path will attempt to stay at least this distance from obstacles unless it must
                                    ///< travel closer to actually find a path
                                    
    double distanceCostExponent;    ///< The exponent to apply to the distance cost, whose function is:
                                    ///<   pow(maxDistanceWithCost - cellDistance, distanceCostExponent)
                                    ///< for cellDistance > minDistanceToObstacle && cellDistance < maxDistanceWithCost
};


/**
* search_for_path uses an A* search to find a path from the start to goal poses. The search assumes a circular robot
* 
* \param    start           Starting pose of the robot
* \param    goal            Desired goal pose of the robot
* \param    distances       Distance to the nearest obstacle for each cell in the grid
* \param    params          Parameters specifying the behavior of the A* search
* \return   The path found to the goal, if one exists. If the goal is unreachable, then a path with just the initial
*   pose is returned, per the robot_path_t specification.
*/
robot_path_t search_for_path(pose_xyt_t start, 
                             pose_xyt_t goal, 
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params);


bool is_member_in_list(Node* n, std::vector<Node* >& list);
bool is_obstacle(Node* n, const ObstacleDistanceGrid& grid);
double h_cost(Node* from, Node* goal);
double g_cost(Node* from, Node* to, const ObstacleDistanceGrid& distances, const SearchParams& params);
std::vector<Node* > expand_node(Node* node, 
                                PriorityQueue& open_list, 
                                const ObstacleDistanceGrid& distances, 
                                const SearchParams& params);


// Once we got the goal, return the path from start to goal
std::vector<Node* > extract_node_path(Node* node);

// Turn the node path into poses
std::vector<pose_xyt_t> extract_pose_path(
        std::vector<Node* >& nodePath, 
        const ObstacleDistanceGrid& distances);

#endif // PLANNING_ASTAR_HPP

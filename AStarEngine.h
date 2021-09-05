//
// Created by oshanath on 2021-09-03.
//

#ifndef UNTITLED_ASTARENGINE_H
#define UNTITLED_ASTARENGINE_H

#include <list>
#include <memory>
#include <unordered_set>
#include <unordered_map>

/**
 *
 * @tparam P type of position (any feature of a node that can uniquely identify that node)
 * P must be hashable, and must override the == operator.
 *
 * @tparam C data type of the heuristic function (float, double, int ?)
 */

template <typename P, typename C>
class AStarEngine{
private:
    /**
     * Calculates the F Cost of a node (F cost = G cost + H cost)
     * Parent is the node from which we came towards a certain node
     * @param parentCost The F Cost of parent node
     * @param parentPosition The P of the parent node
     * @param currentPosition The P of the current node
     * @param end The target P
     * @return The F cost of the current node
     */
    C getFCost(C parentCost,P parentPosition,P currentPosition, P end) const{
        C GCost = parentCost + getCost(parentPosition, currentPosition);
        C HCost = getCost(currentPosition, end);
        return GCost + HCost;
    }

public:
    struct Node{
        P position;
        std::shared_ptr<Node> parent;
        C cost;
    };


    /**
     * Child class must implement the following two pure virtual functions
     */


    /**
     * @return the heuristic cost of moving from a to b
     */
    virtual C getCost(P a, P b) const = 0;

    /**
     * @return The vector of P containing all the valid (traversable) nodes from a
     */
    virtual std::vector<P> getTraversableNeighbours(P a) const = 0;

    /**
     * Calculate the shortest path from start to end using A* algorithm
     * Uses hash tables for performance
     * Uses std::shared_ptr for performance and safety
     * Program already tested for memory leaks using Valgrind
     *
     * @return A std::list containing the path from start to end
     */
    std::list<P> calculatePath(P start, P end) const{

        std::unordered_map<P, std::shared_ptr<Node>> open;
        std::unordered_set<P> closed;

        auto startNode = std::shared_ptr<Node>(new Node{start, nullptr, getCost(start, end)});
        auto current = startNode;
        open.insert(std::make_pair(start, startNode));

        while(true){

            //Find the node in open with the lowest f cost and make it the current node
            C lowestFCost = -1;
            auto currentIterator = open.end();

            for(auto i = open.begin(); i != open.end(); i++){
                if(lowestFCost == -1 || i->second->cost < lowestFCost){
                    lowestFCost =i->second->cost;
                    current = i->second;
                    currentIterator = i;
                }
            }

            if(lowestFCost == -1){
                // Path doesn't exist
                std::list<P> path;

                return path;
            }

            //Remove current from open and add to closed
            open.erase(currentIterator);
            closed.insert((P)current->position);

            if(current->position == end){
                //Path found

                std::list<P> path;
                path.push_front(current->position);

                while(current->parent != nullptr){
                    current = current->parent;
                    path.push_front(current->position);
                }

                return path;

            }

            for(auto& neighbour : getTraversableNeighbours(current->position)){

                if(closed.find(neighbour) != closed.end()){
                    continue;
                }

                auto currentNodeOpenIterator = open.find(current->position);
                C neighbourCost = getFCost(current->cost, current->position, neighbour, end);

                if(currentNodeOpenIterator != open.end()){
                    auto neighbourNode = open[neighbour];

                    if(neighbourCost < neighbourNode->cost){
                        neighbourNode->cost = neighbourCost;
                        neighbourNode->parent = current;
                    }
                }
                else{
                    open.insert(std::make_pair<P, std::shared_ptr<Node>>((P)neighbour, (std::shared_ptr<Node>)new Node{neighbour, current, neighbourCost}));
                }

            }

        }

    }
};


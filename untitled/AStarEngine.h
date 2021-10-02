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

/**
     * Since we are working with a 3D grid, each node can be uniquely identified with its position.
     * So the template parameter P will be the struct Position
     */
struct Position3D {
    int layer;
    int row;
    int col;

    bool operator==(Position3D p) const {
        return layer == p.layer && row == p.row && col == p.col;
    }
};

/**
The FNV-1a algorithm is:

hash = FNV_offset_basis
for each octetOfData to be hashed
    hash = hash xor octetOfData
hash = hash * FNV_prime
return hash
    Where the constants FNV_offset_basis and FNV_prime depend on the return hash size you want:

Hash Size
===========
32-bit
    prime: 2^24 + 2^8 + 0x93 = 16777619
offset: 2166136261
64-bit
    prime: 2^40 + 2^8 + 0xb3 = 1099511628211
offset: 14695981039346656037
128-bit
    prime: 2^88 + 2^8 + 0x3b = 309485009821345068724781371
offset: 144066263297769815596495629667062367629
256-bit
    prime: 2^168 + 2^8 + 0x63 = 374144419156711147060143317175368453031918731002211
offset: 100029257958052580907070968620625704837092796014241193945225284501741471925557
512-bit
    prime: 2^344 + 2^8 + 0x57 = 35835915874844867368919076489095108449946327955754392558399825615420669938882575126094039892345713852759
offset: 9659303129496669498009435400716310466090418745672637896108374329434462657994582932197716438449813051892206539805784495328239340083876191928701583869517785
1024-bit
    prime: 2^680 + 2^8 + 0x8d = 5016456510113118655434598811035278955030765345404790744303017523831112055108147451509157692220295382716162651878526895249385292291816524375083746691371804094271873160484737966720260389217684476157468082573
offset: 1419779506494762106872207064140321832088062279544193396087847491461758272325229673230371772250864096521202355549365628174669108571814760471015076148029755969804077320157692458563003215304957150157403644460363550505412711285966361610267868082893823963790439336411086884584107735010676915
*/

/**
 * This class inherits the AStarEngine class
 * @tparam E type of a single element of the grid (int, char ?)
 * E must override the == operator
 */
template<typename E>
class AStarEngine3D : public AStarEngine<Position3D, int> {
public:

    E traversable;

    std::vector<Position3D> getTraversableNeighbours(Position3D a) const override{
        std::vector<Position3D> neighbours;

        assert(a.layer < layers && a.row < rows && a.col < cols);
        assert(a.layer >= 0 && a.row >= 0 && a.col >= 0);

        //Z=0   --------------------------------------------------------------------------------
        //Right
        Position3D neighbour = {a.layer, a.row, a.col + 1};
        if (neighbour.col < cols && grid[neighbour.layer][neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Left
        neighbour = {a.layer, a.row, a.col - 1};
        if (neighbour.col >= 0 && grid[neighbour.layer][neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Up
        neighbour = {a.layer, a.row - 1, a.col};
        if (neighbour.row >= 0 && grid[neighbour.layer][neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Down
        neighbour = {a.layer, a.row + 1, a.col};
        if (neighbour.row < rows && grid[neighbour.layer][neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Right-Up
        neighbour = {a.layer, a.row - 1, a.col + 1};
        if (neighbour.row >= 0 && neighbour.col < cols && grid[neighbour.layer][neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Right-Down
        neighbour = {a.layer, a.row + 1, a.col + 1};
        if (neighbour.row < rows && neighbour.col < cols && grid[neighbour.layer][neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Left-Up
        neighbour = {a.layer, a.row - 1, a.col - 1};
        if (neighbour.row >= 0 && neighbour.col >= 0 && grid[neighbour.layer][neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Left-Down
        neighbour = {a.layer, a.row + 1, a.col - 1};
        if (neighbour.row < rows && neighbour.col >= 0 && grid[neighbour.layer][neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Z-   --------------------------------------------------------------------------------
        //Center
        neighbour = {a.layer - 1, a.row, a.col};
        if (neighbour.layer >= 0 && grid[neighbour.layer][neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Right
        neighbour = {a.layer - 1, a.row, a.col + 1};
        if (neighbour.layer >= 0 && neighbour.col < cols && grid[neighbour.layer][neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Left
        neighbour = {a.layer - 1, a.row, a.col - 1};
        if (neighbour.layer >= 0 && neighbour.col >= 0 && grid[neighbour.layer][neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Up
        neighbour = {a.layer - 1, a.row - 1, a.col};
        if (neighbour.layer >= 0 && neighbour.row >= 0 && grid[neighbour.layer][neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Down
        neighbour = {a.layer - 1, a.row + 1, a.col};
        if (neighbour.layer >= 0 && neighbour.row < rows && grid[neighbour.layer][neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Right-Up
        neighbour = {a.layer - 1, a.row - 1, a.col + 1};
        if (neighbour.layer >= 0 && neighbour.row >= 0 && neighbour.col< cols &&
            grid[neighbour.layer][neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Right-Down
        neighbour = {a.layer - 1, a.row + 1, a.col + 1};
        if (neighbour.layer >= 0 && neighbour.row < rows && neighbour.col < cols &&
            grid[neighbour.layer][neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Left-Up
        neighbour = {a.layer - 1, a.row - 1, a.col - 1};
        if (neighbour.layer >= 0 && neighbour.row >= 0 && neighbour.col >= 0 &&
            grid[neighbour.layer][neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Left-Down
        neighbour = {a.layer - 1, a.row + 1, a.col - 1};
        if (neighbour.layer >= 0 && neighbour.row < rows && neighbour.col >= 0 &&
            grid[neighbour.layer][neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Z+   --------------------------------------------------------------------------------
        //Center
        neighbour = {a.layer + 1, a.row, a.col};
        if (neighbour.layer < layers && grid[neighbour.layer][neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Right
        neighbour = {a.layer + 1, a.row, a.col + 1};
        if (neighbour.layer < layers && neighbour.col < cols &&
            grid[neighbour.layer][neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Left
        neighbour = {a.layer + 1, a.row, a.col - 1};
        if (neighbour.layer < layers && neighbour.col >= 0 && grid[neighbour.layer][neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Up
        neighbour = {a.layer + 1, a.row - 1, a.col};
        if (neighbour.layer < layers && neighbour.row >= 0 &&
            grid[neighbour.layer][neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Down
        neighbour = {a.layer + 1, a.row + 1, a.col};
        if (neighbour.layer < layers && neighbour.row < rows && grid[neighbour.layer][neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Right-Up
        neighbour = {a.layer + 1, a.row - 1, a.col + 1};
        if (neighbour.layer < layers && neighbour.row >= 0 && neighbour.col < cols &&
            grid[neighbour.layer][neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Right-Down
        neighbour = {a.layer + 1, a.row + 1, a.col + 1};
        if (neighbour.layer < layers && neighbour.row < rows && neighbour.col < cols &&
            grid[neighbour.layer][neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Left-Up
        neighbour = {a.layer + 1, a.row - 1, a.col - 1};
        if (neighbour.layer < layers && neighbour.row >= 0 && neighbour.col >= 0 &&
            grid[neighbour.layer][neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Left-Down
        neighbour = {a.layer + 1, a.row + 1, a.col - 1};
        if (neighbour.layer < layers && neighbour.row < rows && neighbour.col >= 0 &&
            grid[neighbour.layer][neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        return neighbours;
    }

private:
    const size_t rows;
    const size_t cols;
    const size_t layers;

    int getCost(Position3D a, Position3D b) const override{
        return std::abs(a.layer - b.layer) + std::abs(a.row - b.row) + std::abs(a.col - b.col);
    }

public:
    std::vector<std::vector<std::vector<E>>> grid;

    /**
     * Copies the grid by value since the original will be used to visualize the path.
     * Use an r-value reference and std::move  to remove the copying.
     */
    AStarEngine3D(std::vector<std::vector<std::vector<E>>> grid, E traversibleValue) : grid(std::move(grid)),
                                                                                       rows(grid[0].size()),
                                                                                       cols(grid[0][0].size()),
                                                                                       layers(grid.size()),
                                                                                       traversable(
                                                                                               traversibleValue) {}

};

/**
 * The hash object to hash Position objects
 * Uses the 64-bit variant of the fnv-1a algorithm for hashing
 */
template<>
struct std::hash<Position3D> {

    uint64_t offset = 14695981039346656037U;
    uint64_t prime = 1099511628211;

    size_t operator()(const Position3D &position) const {

        uint64_t hash = offset;

        /**
         * Directly copying bytes of a Position object to the vector for the hash function to use
         * This is safe for this particular struct
         */
        std::array<uint8_t, sizeof(Position3D)> bytes;
        std::memcpy(&bytes[0], &position, sizeof(Position3D));

        for (uint8_t byte: bytes) {
            hash ^= byte;
            hash *= prime;
        }
        return hash;

    }
};

/**
 * Since we are working with a 2D grid, each node can be uniquely identified with its position.
 * So the template parameter P will be the struct Position
 */
struct Position2D {
    int row;
    int col;

    bool operator==(Position2D p) const {
        return row == p.row && col == p.col;
    }
};

/**
The FNV-1a algorithm is:

hash = FNV_offset_basis
for each octetOfData to be hashed
        hash = hash xor octetOfData
hash = hash * FNV_prime
return hash
        Where the constants FNV_offset_basis and FNV_prime depend on the return hash size you want:

Hash Size
===========
32-bit
        prime: 2^24 + 2^8 + 0x93 = 16777619
offset: 2166136261
64-bit
        prime: 2^40 + 2^8 + 0xb3 = 1099511628211
offset: 14695981039346656037
128-bit
        prime: 2^88 + 2^8 + 0x3b = 309485009821345068724781371
offset: 144066263297769815596495629667062367629
256-bit
        prime: 2^168 + 2^8 + 0x63 = 374144419156711147060143317175368453031918731002211
offset: 100029257958052580907070968620625704837092796014241193945225284501741471925557
512-bit
        prime: 2^344 + 2^8 + 0x57 = 35835915874844867368919076489095108449946327955754392558399825615420669938882575126094039892345713852759
offset: 9659303129496669498009435400716310466090418745672637896108374329434462657994582932197716438449813051892206539805784495328239340083876191928701583869517785
1024-bit
        prime: 2^680 + 2^8 + 0x8d = 5016456510113118655434598811035278955030765345404790744303017523831112055108147451509157692220295382716162651878526895249385292291816524375083746691371804094271873160484737966720260389217684476157468082573
offset: 1419779506494762106872207064140321832088062279544193396087847491461758272325229673230371772250864096521202355549365628174669108571814760471015076148029755969804077320157692458563003215304957150157403644460363550505412711285966361610267868082893823963790439336411086884584107735010676915
*/



/**
 * This class inherits the AStarEngine class
 * @tparam E type of a single element of the grid (int, char ?)
 * E must override the == operator
 */
template<typename E>
class AStarEngine2D : public AStarEngine<Position2D, int> {
public:
    E traversable;
private:
    const size_t rows;
    const size_t cols;

    int getCost(Position2D a, Position2D b) const override{
        return std::abs(a.row - b.row) + std::abs(a.col - b.col);
    }

    std::vector<Position2D> getTraversableNeighbours(Position2D a) const override{
        std::vector<Position2D> neighbours;

        assert(a.row < rows && a.col < cols);
        assert(a.row >= 0 && a.col >= 0);

        //Right
        Position2D neighbour = {a.row + 1, a.col};
        if (neighbour.row < rows && grid[neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Left
        neighbour = {a.row - 1, a.col};
        if (neighbour.row >= 0 && grid[neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Up
        neighbour = {a.row, a.col + 1};
        if (neighbour.col < cols && grid[neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Down
        neighbour = {a.row, a.col - 1};
        if (neighbour.col >= 0 && grid[neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Right-Up
        neighbour = {a.row + 1, a.col + 1};
        if (neighbour.row < rows && neighbour.col < cols && grid[neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Right-Down
        neighbour = {a.row + 1, a.col - 1};
        if (neighbour.row < rows && neighbour.col >= 0 && grid[neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Left-Up
        neighbour = {a.row - 1, a.col + 1};
        if (neighbour.row >= 0 && neighbour.col < cols && grid[neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        //Left-Down
        neighbour = {a.row - 1, a.col - 1};
        if (neighbour.row >= 0 && neighbour.col >= 0 && grid[neighbour.row][neighbour.col] == traversable)
            neighbours.push_back(neighbour);

        return neighbours;
    }

public:
    std::vector<std::vector<E>> grid;

    /**
     * Copies the grid by value since the original will be used to visualize the path.
     * Use an r-value reference and std::move to remove the copying.
     */
    AStarEngine2D(std::vector<std::vector<E>> grid, E traversibleValue) : grid(std::move(grid)), rows(grid.size()),
                                                                          cols(grid[0].size()),
                                                                          traversable(traversibleValue) {}

};


/**
 * The hash object to hash Position objects
 * Uses the 64-bit variant of the fnv-1a algorithm for hashing
 */
template<>
struct std::hash<Position2D> {

    uint64_t offset = 14695981039346656037U;
    uint64_t prime = 1099511628211;

    size_t operator()(const Position2D &position) const {

        uint64_t hash = offset;

        /**
         * Directly copying bytes of a Position object to the vector for the hash function to use
         * This is safe for this particular struct
         */
        std::array<uint8_t, sizeof(Position2D)> bytes;
        std::memcpy(&bytes[0], &position, sizeof(Position2D));

        for (uint8_t byte: bytes) {
            hash ^= byte;
            hash *= prime;
        }
        return hash;

    }
};
#endif //UNTITLED_ASTARENGINE_H

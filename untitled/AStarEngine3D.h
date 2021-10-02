//
// Created by oshanath on 2021-09-04.
//

#ifndef UNTITLED_ASTARENGINE3D_H
#define UNTITLED_ASTARENGINE3D_H

#include "AStarEngine.h"

namespace ASE3D {

    /**
     * Since we are working with a 3D grid, each node can be uniquely identified with its position.
     * So the template parameter P will be the struct Position
     */
    struct Position {
        int layer;
        int row;
        int col;

        bool operator==(Position p) const {
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
    class AStarEngine3D : public AStarEngine<Position, int> {
    public:

        E traversable;

        std::vector<Position> getTraversableNeighbours(Position a) {
            std::vector<Position> neighbours;

            assert(a.layer < layers && a.row < rows && a.col < cols);
            assert(a.layer >= 0 && a.row >= 0 && a.col >= 0);

            //Z=0   --------------------------------------------------------------------------------
            //Right
            Position neighbour = {a.layer, a.row, a.col + 1};
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

        int getCost(Position a, Position b) {
            return std::abs(a.layer - b.layer) + std::abs(a.row - b.row) + std::abs(a.col - b.col);
        }

    public:
        std::vector<std::vector<std::vector<E>>> grid;

        /**
         * Copies the grid by value since the original will be used to visualize the path.
         * Use an r-value reference and std::move where the constructor is called to
         * remove the copying.
         */
        AStarEngine3D(std::vector<std::vector<std::vector<E>>> grid, E traversibleValue) : grid(std::move(grid)),
                                                                                           rows(grid[0].size()),
                                                                                           cols(grid[0][0].size()),
                                                                                           layers(grid.size()),
                                                                                           traversable(
                                                                                                   traversibleValue) {}

    };

}

/**
 * The hash object to hash Position objects
 * Uses the 64-bit variant of the fnv-1a algorithm for hashing
 */
template<>
struct std::hash<ASE3D::Position> {

    uint64_t offset = 14695981039346656037U;
    uint64_t prime = 1099511628211;

    size_t operator()(const ASE3D::Position &position) const {

        uint64_t hash = offset;

        /**
         * Directly copying bytes of a Position object to the vector for the hash function to use
         * This is safe for this particular struct
         */
        std::array<uint8_t, sizeof(ASE3D::Position)> bytes;
        std::memcpy(&bytes[0], &position, sizeof(ASE3D::Position));

        for (uint8_t byte: bytes) {
            hash ^= byte;
            hash *= prime;
        }
        return hash;

    }
};

#endif //UNTITLED_ASTARENGINE3D_H

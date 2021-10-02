#include <iostream>
#include <vector>
#include <cassert>
#include <cstring>
#include <functional>
#include "AStarEngine.h"

int main() {

    //The grid

    std::vector<std::vector<int>> layer0 = {
            {1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    };

    std::vector<std::vector<int>> layer1 = {
            {1, 1, 0, 1, 1, 1, 1, 1, 1, 1},
            {1, 1, 0, 1, 1, 1, 1, 1, 1, 1},
            {1, 1, 0, 1, 1, 1, 1, 1, 1, 1},
            {1, 1, 0, 1, 1, 1, 1, 1, 1, 1},
            {1, 1, 0, 1, 1, 1, 1, 1, 1, 1},
            {1, 1, 0, 1, 1, 1, 1, 1, 1, 1},
            {1, 1, 0, 1, 0, 0, 0, 0, 1, 1},
            {1, 1, 0, 1, 1, 1, 1, 0, 1, 1},
            {1, 1, 0, 0, 0, 0, 1, 0, 0, 0},
            {1, 1, 1, 1, 1, 1, 1, 1, 1, 1}
    };

    std::vector<std::vector<int>> layer2 = {
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
            {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    };

    std::vector<std::vector<std::vector<int>>> grid = {layer0, layer1, layer2};

    //Instantiate the necessary class, if the value of a cell is 1, it is traversable
    AStarEngine3D<int> engine(grid, 1);

    //Find the shortest path from 1,1 to 4,5
    auto path = engine.calculatePath({0, 0, 0}, {2, 4, 5});

    //Replace the path with 2, in the original grid for visualization
    for(auto& p : path){
        grid[p.layer][p.row][p.col] = 2;
    }

    for(auto& i : grid){
        for(auto& j : i){
            for(int k : j){
                std::cout << k << " ";
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }
}

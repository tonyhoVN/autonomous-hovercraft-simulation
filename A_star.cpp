#include <iostream>
#include <vector>
#include <queue>
#include <cmath>


// Structure to represent a node in the graph
struct Node {
    int x, y;  // Coordinates of the node
    int g, h;  // Cost values (g: cost from start, h: heuristic cost to goal)
    Node* parent;  // Parent node in the path
};

// Function to calculate the heuristic cost between two nodes (Euclidean distance)
int calculateHeuristic(Node* current, Node* goal) {
    int dx = std::abs(current->x - goal->x);
    int dy = std::abs(current->y - goal->y);
    return std::sqrt(dx * dx + dy * dy);
}

// Function to check if a node is within the bounds of the grid
bool isWithinBounds(int x, int y, int gridWidth, int gridHeight) {
    return (x >= 0 && x < gridWidth && y >= 0 && y < gridHeight);
}

// Function to check if a node is traversable (example: not blocked or obstacle)
bool isTraversable(int x, int y, std::vector<std::vector<int>>& grid) {
    return (grid[x][y] == 0);
}

// Function to find the shortest path using the A* algorithm
std::vector<Node*> findShortestPath(std::vector<std::vector<int>>& grid, Node* start, Node* goal) {
    int gridWidth = grid.size();
    int gridHeight = grid[0].size();

    // Create a priority queue for open nodes (nodes to be evaluated)
    std::priority_queue<Node*, std::vector<Node*>, 
        bool(*)(const Node*, const Node*)> openNodes([](const Node* a, const Node* b) {
            return (a->g + a->h) > (b->g + b->h);
        });

    // Create a 2D array to track visited nodes
    std::vector<std::vector<bool>> visited(gridWidth, std::vector<bool>(gridHeight, false));

    // Start node initialization
    start->g = 0;
    start->h = calculateHeuristic(start, goal);
    start->parent = nullptr;

    // Add start node to open nodes queue
    openNodes.push(start);

    // A* algorithm loop
    while (!openNodes.empty()) {
        // Get the node with the lowest cost from the open nodes queue
        Node* current = openNodes.top();
        openNodes.pop();

        // Check if the goal node has been reached
        if (current == goal) {
            // Reconstruct the path from the goal node to the start node
            std::vector<Node*> path;
            while (current != nullptr) {
                path.push_back(current);
                current = current->parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        // Mark the current node as visited
        visited[current->x][current->y] = true;

        // Generate neighboring nodes
        int dx[] = {1, 0, -1, 0};  // Possible x movements (right, down, left, up)
        int dy[] = {0, 1, 0, -1};  // Possible y movements (right, down, left, up)
        for (int i = 0; i < 4; ++i) {
            int newX = current->x + dx[i];
            int newY = current->y + dy[i];

            // Check if the neighboring node is within the bounds of the grid
            if (isWithinBounds(newX, newY, gridWidth, gridHeight)) {
                // Check if the neighboring node is traversable and not visited
                if (isTraversable(newX, newY, grid) && !visited[newX][newY]) {
                    // Calculate the cost from the start node to the neighboring node
                    int newG = current->g + 1;

                    // Check if the neighboring node is already in the open nodes queue
                    bool inOpenNodes = false;
                    for (Node* node : openNodes) {
                        if (node->x == newX && node->y == newY) {
                            inOpenNodes = true;
                            break;
                        }
                    }

                    // Create the neighboring node and calculate its cost values
                    Node* neighbor = new Node{newX, newY, newG, calculateHeuristic(current, goal), current};

                    // Add the neighboring node to the open nodes queue if not already there
                    if (!inOpenNodes) {
                        openNodes.push(neighbor);
                    }
                }
            }
        }
    }

    // No path found
    return std::vector<Node*>();
}

// Function to print the grid with the shortest path
void printGridWithShortestPath(std::vector<std::vector<int>>& grid, std::vector<Node*>& path) {
    int gridWidth = grid.size();
    int gridHeight = grid[0].size();

    // Create a copy of the grid to mark the shortest path
    std::vector<std::vector<int>> markedGrid = grid;

    // Mark the nodes in the path
    for (Node* node : path) {
        markedGrid[node->x][node->y] = 2;  // Marked path node
    }

    // Print the grid
    for (int y = 0; y < gridHeight; ++y) {
        for (int x = 0; x < gridWidth; ++x) {
            std::cout << markedGrid[x][y] << " ";
        }
        std::cout << std::endl;
    }
}

int main() {
    // Example usage
    std::vector<std::vector<int>> grid = {
        {0, 0, 1, 0, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 1, 0, 0},
        {0, 0, 1, 0, 0}
    };

    int startX = 0;
    int startY = 0;
    int goalX = 4;
    int goalY = 4;

    Node* start = new Node{startX, startY, 0, 0, nullptr};
    Node* goal = new Node{goalX, goalY, 0, 0, nullptr};

    std::vector<Node*> path = findShortestPath(grid, start, goal);
    printGridWithShortestPath(grid, path);

    // Clean up memory
    delete start;
    delete goal;
    for (Node* node : path) {
        delete node;
    }

    return 0;
}

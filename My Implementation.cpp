#include <iostream>
#include <vector>
#include <queue>
#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include <SelbaWard/Line.hpp>


struct Node
{
    int posX, posY;
    int parentIndex;
    int index;
    bool isStartNode = false;
    bool isEndNode = false;

    bool isVisited = false;
    bool isAnObstacle = false;
    bool isFinalTrack = false;
    std::vector<Node*> nodeNeighbors;
};

class World
{
private:
    int worldHeight;
    int worldWidth;
    int cellWidth;
    int startIndex = -1, endIndex = -1;
    bool ready = false;
    sf::Vector2u windowSize;
    std::vector<sw::Line> grid;
    std::vector<Node> nodes;

public:
    World(sf::RenderWindow& window)
    {
        // settings
        windowSize = window.getSize();
        cellWidth = 24;
        worldWidth = windowSize.x / cellWidth;
        worldHeight = windowSize.y / cellWidth;


        // Set the nodes settings
        nodes.resize(worldHeight * worldWidth);
        for (int x = 0; x < worldWidth; x++)
        {
            for (int y = 0; y < worldHeight; y++)
            {
                // y * width + x ( 2D --> 1D transformation)
                int i = y * worldWidth + x;

                // Position and Parent
                nodes[i].posX = x;
                nodes[i].posY = y;
                nodes[i].index = i;

                // Boundery checks before adding neighbors
                int up = (y - 1) * worldWidth + x;
                int down = (y + 1) * worldWidth + x;
                int left = y * worldWidth + (x - 1);
                int right = y * worldWidth + (x + 1);

                if (y > 0)
                    nodes[i].nodeNeighbors.push_back(&nodes[up]);
                if (y < worldHeight - 1)
                    nodes[i].nodeNeighbors.push_back(&nodes[down]);
                if (x > 0)
                    nodes[i].nodeNeighbors.push_back(&nodes[left]);
                if (x < worldWidth - 1)
                    nodes[i].nodeNeighbors.push_back(&nodes[right]);

            }
        }


        // grid
        sw::Line line;
        line.setColor(sf::Color::Black);

        // Horizontal
        for (int y = 1; y < worldHeight; y++)
        {
            line.setPoint(0, sf::Vector2f(0.0f, y * cellWidth));
            line.setPoint(1, sf::Vector2f(windowSize.x, y * cellWidth));
            grid.push_back(line);
        }
        // Vertical
        for (int x = 1; x < worldWidth; x++)
        {
            line.setPoint(0, sf::Vector2f(x * cellWidth, 0.0f));
            line.setPoint(1, sf::Vector2f(x * cellWidth, windowSize.y));
            grid.push_back(line);
        }
    }

    void update(sf::RenderWindow& window, sf::Event& event)
    {
        // Check if any updates are done to the nodes before running BFS
        updateNodes(window, event);

        // Only run BFS when both start point and point are set, and when Enter is Pressed
        if (event.type == sf::Event::KeyReleased)
        {
            if (ready && event.key.code == sf::Keyboard::Enter)
            {
                BFS(window);
            }
        }
    }

    void updateNodes(sf::RenderWindow& window, sf::Event& event)
    {
        sf::Vector2i mousePos = sf::Mouse::getPosition(window);
        // y * width + x ( 2D --> 1D transformation)
        int i = (mousePos.y / cellWidth) * worldWidth + (mousePos.x / cellWidth);

        if (mousePos.x > 0 && mousePos.y > 0 && mousePos.x < windowSize.x && mousePos.y < windowSize.y)
        {
            // Add Obstcale
            if (sf::Mouse::isButtonPressed(sf::Mouse::Left))
                nodes[i].isAnObstacle = true;

            // Remove Obstcale
            if (sf::Mouse::isButtonPressed(sf::Mouse::Right))
                nodes[i].isAnObstacle = false;

            if (event.type == sf::Event::KeyReleased)
            {
                // Add start point
                if (event.key.code == sf::Keyboard::S)
                {
                    // Switch indexes after the first one is put.
                    if (startIndex > 0)
                        nodes[startIndex].isStartNode = false;
                    nodes[i].isStartNode = true;
                    startIndex = i;
                }

                // Add end point
                if (event.key.code == sf::Keyboard::E)
                {
                    // Switch indexes after the first one is put.
                    if (endIndex > 0)
                        nodes[endIndex].isEndNode = false;
                    nodes[i].isEndNode = true;
                    endIndex = i;
                }
            }

            // Check if the start and endpoint are set
            if (startIndex >= 0 && endIndex >= 0)
                ready = true;

        }
    }


    void BFS(sf::RenderWindow& window)
    {
        int checks = 0;
        bool noSolution = false;

        // Set the first node
        std::queue<Node> testNodesQ;
        testNodesQ.push(nodes[startIndex]);
        nodes[startIndex].isVisited = true;

        Node currentNode;
        // Loop until a solution is found
        while (!noSolution)
        {
            // if we went through all the nodes then there's no solution
            if (testNodesQ.empty())
            {
                noSolution = true;
                break;
            }

            // Add current node to the queue
            currentNode = testNodesQ.front();
            // Remove the front element so we can add it's neighbors
            testNodesQ.pop();

            // If current node is the end point node then exit (solution is found)
            if (currentNode.isEndNode)
                break;

            // Add the neighbors of the current node
            for (int i = 0; i < currentNode.nodeNeighbors.size(); i++)
            {
                // If the neighbor is visited or is an obstacle then don't add to the queue
                if (!currentNode.nodeNeighbors[i]->isVisited && !currentNode.nodeNeighbors[i]->isAnObstacle)
                {
                    // Make the neighbor visited and update it's parrent index then push it to the queue
                    currentNode.nodeNeighbors[i]->isVisited = true;
                    currentNode.nodeNeighbors[i]->parentIndex = currentNode.index;
                    testNodesQ.push(*currentNode.nodeNeighbors[i]);
                }
            }
            checks++;
        }

        // Create final track (shortest path) if solution was found
        if (!noSolution)
        {
            // Backtrack from the end node to the start index.
            Node n = nodes[endIndex];
            int parent = n.parentIndex;
            while (parent != nodes[startIndex].index)
            {
                parent = n.parentIndex;
                nodes[parent].isFinalTrack = true;
                n = nodes[parent];
            }
        }

        // Print number of checks
        if (checks > 1)
            std::cout << "Number of checks: " << checks << std::endl;

    }

    void draw(sf::RenderWindow& window)
    {
        // Nodes
        sf::RectangleShape rect(sf::Vector2f(cellWidth, cellWidth));
        for (int x = 0; x < worldWidth; x++)
        {
            for (int y = 0; y < worldHeight; y++)
            {
                // y * width + x ( 2D --> 1D transformation
                int i = y * worldWidth + x;

                rect.setPosition(sf::Vector2f(x * cellWidth, y * cellWidth));

                if (nodes[i].isVisited)
                {
                    rect.setFillColor(sf::Color::Color(128, 128, 128, 100));
                    window.draw(rect);
                }

                if (nodes[i].isFinalTrack)
                {
                    rect.setFillColor(sf::Color::Yellow);
                    window.draw(rect);
                }

                if (nodes[i].isAnObstacle)
                {
                    rect.setFillColor(sf::Color::Blue);
                    window.draw(rect);
                }

                if (nodes[i].isEndNode)
                {
                    rect.setFillColor(sf::Color::Red);
                    window.draw(rect);
                }

                if (nodes[i].isStartNode)
                {
                    rect.setFillColor(sf::Color::Green);
                    window.draw(rect);
                }
            }
        }

        // Grid
        for (int i = 0; i < grid.size(); i++)
        {
            window.draw(grid[i]);
        }
    }
};


int main()
{
    std::cout << "Welcome to Our Project \n" << "How to use: \n"
        << "'S': Set a starting node,     'E': Set an ending node\n"
        << "'R': Restart,                 'Enter' : Run(only after setting starting and ending points)\n"
        << "'Left Mouse': Add obstacle,   'Right Mouse': Remove obstacle" << std::endl;

    sf::RenderWindow window(sf::VideoMode(1920, 1080), "My window", sf::Style::Fullscreen);

    World world(window);

    while (window.isOpen())
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();

            if (event.type == sf::Event::KeyReleased)
            {
                if (event.key.code == sf::Keyboard::R)
                    world = World(window);

                if (event.key.code == sf::Keyboard::Escape)
                    window.close();
            }
        }

        window.clear(sf::Color::White);
        world.update(window, event);
        world.draw(window);
        window.display();
    }

    return 0;
}

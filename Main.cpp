#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include "SelbaWard/Line.hpp"
using namespace std;

enum NodeState { Empty = 0, Visited = 1, Junction = 2, Obstacle = 3, Start = 4, End = 5, Path = 6 };

struct Graph {
    vector<vector<pair<int, int>>>adj_weighted; // First int holds the index of the neighbor, the other int holds the weight between the node and its neighbor
    vector<int>parent;
    vector<NodeState>state;
    vector<int>distance; // Distance of every node from the start node
    int found = 0;
    /**
     * @brief Resets the Graph.
     */
    void clear() {
        adj_weighted.clear();
        parent.clear();
        state.clear();
        distance.clear();
        found = 0;
    }
    /**
     * @brief This is only valid for weighted graphs
     *
     * @param Node1 an integer, must be connected to Node2
     * @param Node2 an integer, must be connected to Node1
     * @param Weight distance between Node1 and Node2
     */
    void add_edge(int Node1, int Node2, int Weight) {
        adj_weighted[Node1].push_back({ Node2, Weight });
        adj_weighted[Node2].push_back({ Node1, Weight });
    }
    // This function is written by the devil himself XD
    void update_node_weight(int Node, int Weight, int SourceNode = -1) {
        for (pair<int, int>& Nodes : adj_weighted[Node]) {
            // If we try to update with the same weight nothing will happen
            if (Nodes.second == Weight)
                continue;
            if (SourceNode == -1) {
                // Update the weight of the node
                Nodes = { Nodes.first, Weight };
                // call the function again to update the weight for the side (other neighbor/adjacent)
                update_node_weight(Nodes.first, Weight, Node);
            }
            // this will update the weight for the other neighbor/adjacent 
            if (Nodes.first == SourceNode)
                Nodes = { Nodes.first, Weight };
        }
    }
};


/**
 * @brief DFS implementation using recursion.
 *
 * @param graph Initialized graph that must not have been used.
 * @param Node Starting Node for DFS to be used.
 * @param PreviousNode Parent of the starting node, if the graph is a tree and the node is the root, the parent will be -1. That if it is acyclic, for cyclic, it will be the node itself.
 */
void DepthFirstSearch(Graph& graph, int Node, int EndNode, int Parent = -1) {
    graph.state[Node] = Visited;
    graph.parent[Node] = Parent; // Change to Node if the graph is cyclic.
    if (Node == EndNode) {
        graph.found = 1;
        return;
    }
    for (int i = 0; i < graph.adj_weighted[Node].size(); i++) {
        if (graph.found)
            break;
        int NodeState = graph.state[graph.adj_weighted[Node][i].first];
        if (NodeState == Empty || NodeState == Junction || NodeState == End)
            DepthFirstSearch(graph, graph.adj_weighted[Node][i].first, EndNode, Node);
    }
}

/**
 * @brief A BFS implementation using a queue to simulate recursion and save memory on stack frames.
 *
 * @param graph Initialized graph that must not have been used.
 * @param Source A source node to start BFS from.
 */
void BreadthFirstSearch(Graph& graph, int Source, int EndNode, int Parent = -1) {
    queue<int>que;
    que.push(Source);
    if (graph.state[Source] != Start)
        graph.state[Source] = Visited;
    graph.distance[Source] = 0;
    graph.parent[Source] = Parent;
    while (!que.empty()) {
        Parent = que.front();
        que.pop();
        if (Parent == EndNode) {
            graph.found = 1;
            break;
        }
        for (pair<int, int> NodeAndWeight : graph.adj_weighted[Parent]) {
            int Node = NodeAndWeight.first;
            int Weight = NodeAndWeight.second;
            if (graph.state[Node] != Obstacle && graph.state[Node] != Visited && graph.state[Node] != Start) { // If we haven't visited this node,
                graph.state[Node] = Visited; // Enter it and mark it as visited.
                graph.distance[Node] = graph.distance[Parent] + Weight;
                graph.parent[Node] = Parent;
                que.push(Node);
            }
        }
    }
}

void DijkstraQ(Graph& graph, int Source, int EndNode, int Parent = -1) {
    // first pair is distance second is index
    typedef pair<int, int> pairOfInt;    
    priority_queue<pairOfInt, vector<pairOfInt>, greater<pairOfInt>>que; // vector and greater parameter are for reversing the priority (min heap)
    que.push(make_pair(0, Source));
    graph.distance[Source] = 0;
    graph.parent[Source] = Parent;

    while (!que.empty()) {
        Parent = que.top().second;
        if (Parent == EndNode)
        {
            graph.found = 1;
            break;
        }
        que.pop();
        for (pair<int, int> NodeAndWeight : graph.adj_weighted[Parent]) {
            int Node = NodeAndWeight.first;
            int Weight = NodeAndWeight.second;
            if (graph.state[Node] != Obstacle && graph.state[Node] != Start) { // If we haven't visited this node,
                graph.state[Node] = Visited; // Enter it and mark it as visited.

                // check if the node needs a new distance and parent
                int NetWeight = graph.distance[Parent] + Weight;
                if (NetWeight < graph.distance[Node]) {
                    graph.distance[Node] = graph.distance[Parent] + Weight;
                    graph.parent[Node] = Parent;
                    que.push(make_pair(NetWeight, Node));
                }
            }
        }
    }
}


/**
 * @brief Get you the path whether you ran BFS, DFS or Dijkstra on the graph.
 *
 * @param graph Initialized Graph that have been used.
 * @param DestinationNode The end node of the path you want to get from the start node. The start node must have been used in the initial BFS, DFS, Dijkstra arguments.
 * @return vector<int> Path from source node to destination node, If there isn't a path, check if `path.size() == 0 || path[path.size()-1] != SourceNode` afterwards.
 */
vector<int> GetPath(Graph& graph, const int DestinationNode, const int SourceNode = -1) {
    vector<int> path;
    for (int Node = DestinationNode; Node != SourceNode; Node = graph.parent[Node], graph.state[Node] = Path)
        path.push_back(Node);
    graph.state[SourceNode] = Start;
    graph.state[DestinationNode] = End;
    reverse(path.begin(), path.end());
    return path;
}

struct World {
    int worldHeight;
    int worldWidth;
    int cellWidth;
    int startIndex = -1, endIndex = -1;
    sf::Vector2u windowSize;
    std::vector<sw::Line> grid;
    Graph graph;
    int mode = 0;

    bool once = true;

    // Text
    sf::Text text;


    World(sf::RenderWindow& window, sf::Font& arialFont) {
        // settings
        windowSize = window.getSize();
        cellWidth = 20;
        worldWidth = windowSize.x / cellWidth;
        worldHeight = windowSize.y / cellWidth;

        // Nodes
        graph.adj_weighted.resize(worldHeight * worldWidth, std::vector<pair<int, int>>());
        graph.state.resize(worldHeight * worldWidth);
        graph.parent.resize(worldHeight * worldWidth);
        graph.distance.resize(worldHeight * worldWidth, 0x7FFFFFFF);

        // Font and text settinggs
        text.setFont(arialFont); // font is a sf::Font
        text.setString("BFS");
        text.setCharacterSize(24); // in pixels, not points!
        text.setFillColor(sf::Color::Black);
        text.setStyle(sf::Text::Bold | sf::Text::Underlined);
                    

        // Set the nodes settings
        int count = 0;
        for (int y = 0; y < worldHeight; y++)
            for (int x = 0; x < worldWidth; x++) {
                // Boundery checks before adding neighbors
                int up = (y - 1) * worldWidth + x;
                int down = (y + 1) * worldWidth + x;
                int left = y * worldWidth + (x - 1);
                int right = y * worldWidth + (x + 1);

                if (y > 0)
                    graph.adj_weighted[count].push_back({ up,    1 });
                if (y < worldHeight - 1)
                    graph.adj_weighted[count].push_back({ down,  1 });
                if (x > 0)
                    graph.adj_weighted[count].push_back({ left,  1 });
                if (x < worldWidth - 1)
                    graph.adj_weighted[count].push_back({ right, 1 });
                count++;
            }

        // grid
        sw::Line line;
        line.setColor(sf::Color::Black);

        // Horizontal
        for (int y = 1; y < worldHeight; y++) {
            line.setPoint(0, sf::Vector2f(0.0f, y * cellWidth));
            line.setPoint(1, sf::Vector2f(windowSize.x, y * cellWidth));
            grid.push_back(line);
        }
        // Vertical
        for (int x = 1; x < worldWidth; x++) {
            line.setPoint(0, sf::Vector2f(x * cellWidth, 0.0f));
            line.setPoint(1, sf::Vector2f(x * cellWidth, windowSize.y));
            grid.push_back(line);
        }
    }

    void update(sf::RenderWindow& window, sf::Event& event) {
        // Checks for event that chanegs the nodes status
        updateNodes(window, event);

        // Check if mode is changed
        if (event.type == sf::Event::KeyPressed)
            if (event.key.code == sf::Keyboard::LAlt || event.key.code == sf::Keyboard::RAlt) {
                mode++;
                mode %= 3;
                switch (mode) {
                case 0:
                    text.setString("BFS");
                    break;
                case 1:
                    text.setString("Dijkstra");
                    break;
                case 2:
                    text.setString("DFS");
                    break;
                default:
                    break;
                }
            }
        // Check if Enter is pressed and run the algo for the corresponding mode
        if (event.type == sf::Event::KeyReleased)
            if (event.key.code == sf::Keyboard::Enter && startIndex != -1 && endIndex != -1) {
                if (once) {
                    switch (mode) {
                    case 0:
                        BreadthFirstSearch(graph, startIndex, endIndex);
                        break;
                    case 1:
                        DijkstraQ(graph, startIndex, endIndex);
                        break;
                    case 2:
                        DepthFirstSearch(graph, startIndex, endIndex);
                        break;
                    default:
                        break;
                    }

                    graph.state[startIndex] = Start;
                    graph.state[endIndex] = End;
                    if (graph.found)
                        GetPath(graph, endIndex, startIndex);
                }
                once = false;
            }
    }

    void updateNodes(sf::RenderWindow& window, sf::Event& event) {
        sf::Vector2i mousePos = sf::Mouse::getPosition(window);
        // y * width + x ( 2D --> 1D transformation)
        int i = (mousePos.y / cellWidth) * worldWidth + (mousePos.x / cellWidth);

        if (mousePos.x > 0 && mousePos.y > 0 && mousePos.x < windowSize.x && mousePos.y < windowSize.y) {
            if (sf::Mouse::isButtonPressed(sf::Mouse::Right)) {
                graph.update_node_weight(i, 2);
                if (graph.state[i] == Start)
                    startIndex = -1;
                else if (graph.state[i] == End)
                    endIndex = -1;
                graph.state[i] = Junction;
            }

            if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
                if (graph.state[i] == Junction)
                    graph.update_node_weight(i, 1);
                if (graph.state[i] == Start)
                    startIndex = -1;
                else if (graph.state[i] == End)
                    endIndex = -1;
                graph.state[i] = Obstacle;
            }

            if (event.type == sf::Event::KeyReleased) {
                if (event.key.code == sf::Keyboard::S) {
                    if (graph.state[i] == Junction)
                        graph.update_node_weight(i, 1);
                    // Switch (make the old startIndex empty)
                    if (startIndex >= 0)
                        graph.state[startIndex] = Empty;

                    graph.state[i] = Start;
                    startIndex = i;
                }

                if (event.key.code == sf::Keyboard::E) {
                    if (graph.state[i] == Junction)
                        graph.update_node_weight(i, 1);
                    // Switch (make the old endtIndex empty)
                    if (endIndex >= 0)
                        graph.state[endIndex] = Empty;

                    graph.state[i] = End;
                    endIndex = i;
                }
            }

            if (sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) || sf::Keyboard::isKeyPressed(sf::Keyboard::RShift)) {
                if (graph.state[i] == Junction)
                    graph.update_node_weight(i, 1);
                if (graph.state[i] == Start)
                    startIndex = -1;
                else if (graph.state[i] == End)
                    endIndex = -1;
                graph.state[i] = Empty;
            }
        }
    }

    void draw(sf::RenderWindow& window) {
        // Grid
        for (int i = 0; i < grid.size(); i++)
            window.draw(grid[i]);

        // Nodes
        sf::RectangleShape rect(sf::Vector2f(cellWidth - 1, cellWidth - 1));

        for (int x = 0; x < worldWidth; x++)
            for (int y = 0; y < worldHeight; y++) {
                // y * width + x ( 2D --> 1D transformation
                int i = y * worldWidth + x;
                rect.setPosition(sf::Vector2f(x * cellWidth, y * cellWidth));

                switch (graph.state[i]) {
                case Visited:
                    rect.setFillColor(sf::Color(128, 128, 128, 100));
                    window.draw(rect);
                    break;
                case Obstacle:
                    rect.setFillColor(sf::Color::Green);
                    window.draw(rect);
                    break;
                case Junction:
                    rect.setFillColor(sf::Color(135, 0, 0));
                    window.draw(rect);
                    break;
                case Path:
                    rect.setFillColor(sf::Color::Yellow);
                    window.draw(rect);
                    break;
                case End:
                    rect.setFillColor(sf::Color::Red);
                    window.draw(rect);
                    break;
                case Start:
                    rect.setFillColor(sf::Color::Cyan);
                    window.draw(rect);
                    break;
                default: // Not Visited (Empty) // a
                    break;
                }
            }

        // Draw text
        window.draw(text);
    }

};

int main() {
    std::cout << "Welcome to Our Project \n" << "How to use: \n"
        << "'S': Set a starting node,     'E': Set an ending node\n"
        << "'R': Restart,                 'Enter' : Run(only after setting starting and ending points)\n"
        << "'Left Mouse': Add obstacle,   'Right Mouse': Add Junction(Adds weight of 2 to all edges connected to the cell)\n"
        << "'Shift': Remove Node\n"
        << "'Alt': Switch Mode" << std::endl;
    sf::RenderWindow window(sf::VideoMode(1280, 720), "EA Project", sf::Style::Default);

    sf::Font arialFont;
    World world(window, arialFont);

    // Font and text
    if (!arialFont.loadFromFile("arial.ttf"))
        std::cout << "Error loading font" << std::endl;

    while (window.isOpen()) {
        sf::Event event;
        window.clear(sf::Color::White);
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();

            if (event.type == sf::Event::KeyReleased) {
                if (event.key.code == sf::Keyboard::R)
                    world = World(window, arialFont);

                if (event.key.code == sf::Keyboard::Escape)
                    window.close();
            }
            world.update(window, event);
        }
        world.draw(window);
        window.display();
    }
    return 0;
}
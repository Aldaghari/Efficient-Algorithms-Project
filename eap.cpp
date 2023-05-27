#include <iostream>
#include <map>
#include <vector>
#include <queue>
#include <set>
#include <algorithm>
using namespace std;

struct Graph{
    map<int, vector<int>>adj;
    map<int, vector<pair<int, int>>>adj_weighted;
    map<int, int>parent;
    map<int, char>visited;
    map<int, int>distance;
    int used = 0;
    /**
     * @brief Resets the Graph.
     */
    void clear(){
        adj.clear();
        parent.clear();
        visited.clear();
        distance.clear();
        used = 0;
    }
    /**
     * @brief This is only valid for unweighted graphs.
     * 
     * @param Node1 an integer, must be connected to Node2
     * @param Node2 an integer, must be connected to Node1
     */
    void add_edge(int Node1, int Node2){
        adj[Node1].push_back(Node2);
        adj[Node2].push_back(Node1);
    }
    /**
     * @brief This is only valid for weighted graphs
     * 
     * @param Node1 an integer, must be connected to Node2
     * @param Node2 an integer, must be connected to Node1
     * @param Weight distance between Node1 and Node2
     */
    void add_edge(int Node1, int Node2, int Weight){
        adj_weighted[Node1].push_back({Node2, Weight});
        adj_weighted[Node2].push_back({Node1, Weight});
    }
};

/**
 * @brief DFS implementation using recursion.
 * 
 * @param graph Initialized graph that must not have been used. 
 * @param Node Starting Node for DFS to be used.
 * @param PreviousNode Parent of the starting node, if the graph is a tree and the node is the root, the parent will be -1. That if it is acyclic, for cyclic, it will be the node itself.
 */
void DepthFirstSearch(Graph &graph, int Node, int Parent = -1){
    if(graph.used){
        puts("The Graph has already been used");
        exit(0);
    }
    graph.visited[Node] = 1;
    graph.parent[Node] = Parent; // Change to Node if the graph is cyclic.
    for(int i = 0; i < graph.adj[Node].size(); i++)
        if(!graph.visited[graph.adj[Node][i]])
            DepthFirstSearch(graph, graph.adj[Node][i], Node);
    graph.used = 1;
}

/**
 * @brief A BFS implementation using a queue to simulate recursion and save memory on stack frames.
 * 
 * @param graph Initialized graph that must not have been used. 
 * @param Source A source node to start BFS from.
 */
void BreadthFirstSearch(Graph &graph, int Source, int Parent = -1){
    if(graph.used){
        puts("The Graph has already been used");
        exit(0);
    }
    queue<int>que;
    que.push(Source);
    graph.visited[Source] = 1;
    graph.distance[Source] = 0;
    graph.parent[Source] = Parent;
    while(!que.empty()){
        Parent = que.front();
        que.pop();
        for(int Node: graph.adj[Parent])
            if(!graph.visited[Node]){ // If we haven't visited this node,
                graph.visited[Node] = 1; // Enter it and mark it as visited.
                graph.distance[Node] = graph.distance[Parent] + 1; // 1 is one edge distance, if a weighted graph you can change it with the weight.
                graph.parent[Node] = Parent;
                que.push(Node);
            }
    }
    graph.used = 1;
}

void Dijkstra(Graph &graph, int Source, int Parent = -1){
    set<int>NodesToProcess;
    graph.visited[Source] = 1;
    graph.distance[Source] = 0;
    graph.parent[Source] = Parent;
    while(true){
        graph.visited[Source] = 1;
        for(pair<int, int> NodeAndWeight: graph.adj_weighted[Source]){
            int Node   = NodeAndWeight.first;
            int Weight = NodeAndWeight.second;
            if(graph.visited[Node])
                continue;
            NodesToProcess.insert(Node);
            int NetWeight = graph.distance[Source] + Weight;
            if(graph.distance.count(Node) == 0)
                graph.distance[Node] = 0x7FFFFFFF;
            if(NetWeight < graph.distance[Node]){
                graph.distance[Node] = NetWeight;
                graph.parent[Node] = Source;
            }
        }
        NodesToProcess.erase(Source);
        if(NodesToProcess.empty())
            break;
        int Max = 0x7FFFFFFF;
        for(int Nodes: NodesToProcess)
            if(graph.distance[Nodes] < Max){
                Max = graph.distance[Nodes];
                Source = Nodes;
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
vector<int> GetPath(Graph &graph, const int DestinationNode, const int SourceNode = -1){
    vector<int> path;
    for (int Node = DestinationNode; Node != SourceNode; Node = graph.parent[Node])
        path.push_back(Node);
    reverse(path.begin(), path.end());
    return path;
}

int main(){
    Graph g;
    g.add_edge(1, 3, 3);
    g.add_edge(1, 2, 1);
    g.add_edge(2, 3, 1);
    /**
     *  1 - (3) - 3
     *  |       /
     *  |      /
     *  |     /
     * (1)  (1)
     *  |   /
     *  |  /
     *  | /
     *  2
     */
    cout << "Dijkstra algorithm: \n";
    Dijkstra(g, 1); // Since we start here from 1
    auto p = GetPath(g, 3); // the path will be from 1 -> 3, the shortest path.
    for(auto i: p){
        cout << i << " ";
    }
    cout << "\n";
    g.clear(); // clear
    cout << "BFS: \n";
    g.add_edge(1, 2);
    g.add_edge(1, 3);
    g.add_edge(3, 4);
    g.add_edge(3, 5);
    BreadthFirstSearch(g, 5);
    p = GetPath(g, 2);
    for(auto i: p){
        cout << i << " ";
    }
    cout << "\n";
    return 0;
}
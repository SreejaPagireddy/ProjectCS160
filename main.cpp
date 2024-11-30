
//In this project I want to first find the shortest path and then I want to use threads to make it better
//First I am using a youtube video to understand the algorithm and walk through it
//than I ma going to implement threads, tasks, and locks, semaphores, or bairriers to make the alogirth faster
//What is Dijkstra's Algorithm: Finds the shortest path from one node to every other node in graph, choose the smallest path and repeat process for each node
//Time complexity of Dijkstra is O(|E| + |V|log|V|)
//In this project I want to first find the shortest path and then I want to use threads to make it better
//First I am using a youtube video to understand the algorithm and walk through it
//than I ma going to implement threads, tasks, and locks, semaphores, or bairriers to make the alogirth faster
//What is Dijkstra's Algorithm: Finds the shortest path from one node to every other node in graph, choose the smallest path and repeat process for each node
//Time complexity of Dijkstra is O(|E| + |V|log|V|)

#include <iostream>
#include <vector>
#include <list>
#include <set>
#include <utility>
using namespace std;

#define INF 0x3f3f3f3f

class Graph {
    int V;
    list< pair<int, int> > *adj;

public:
    Graph(int V);
    void addEdge(int a, int b, int c);
    void shortestPath(int src); 
};

Graph::Graph(int V) {
    this->V = V;
    adj = new list< pair<int, int> >[V];
}

void Graph::addEdge(int a, int b, int c) {
    adj[a].push_back(make_pair(b, c));  
    adj[b].push_back(make_pair(a, c));
}

void Graph::shortestPath(int src) {
    set< pair<int, int> > setds;
    vector<int> dist(V, INF);
    dist[src] = 0;
    setds.insert(make_pair(0, src));

    while (!setds.empty()) {
        pair<int, int> tmp = *(setds.begin());  
        setds.erase(setds.begin());  
        int u = tmp.second; 

        for (auto i = adj[u].begin(); i != adj[u].end(); ++i) {
            int v = i->first;  
            int weight = i->second;  

            
            if (dist[v] > dist[u] + weight) {
                if (dist[v] != INF) {
                    setds.erase(setds.find(make_pair(dist[v], v)));  
                }
                dist[v] = dist[u] + weight;
                setds.insert(make_pair(dist[v], v));  
            }
        }
    }

    printf("Vertex   Distance from Source\n");
    for (int i = 0; i < V; ++i) {
        printf("%d \t\t %d\n", i, dist[i]);
    }
}

int main() {
    int V = 9;  
    Graph g(V);

    g.addEdge(0, 1, 4);
    g.addEdge(0, 7, 8);
    g.addEdge(1, 2, 8);
    g.addEdge(1, 7, 11);
    g.addEdge(2, 3, 7);
    g.addEdge(2, 8, 2);
    g.addEdge(2, 5, 4);
    g.addEdge(3, 4, 9);
    g.addEdge(3, 5, 14);
    g.addEdge(4, 5, 10);
    g.addEdge(5, 6, 2);
    g.addEdge(6, 7, 1);
    g.addEdge(6, 8, 6);
    g.addEdge(7, 8, 7);

    g.shortestPath(0);

    return 0;
}
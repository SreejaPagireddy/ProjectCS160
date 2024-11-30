//In this project I want to first find the shortest path and then I want to use threads to make it better
//First I am using a youtube video to understand the algorithm and walk through it
//than I ma going to implement threads, tasks, and locks, semaphores, or bairriers to make the alogirth faster
//What is Dijkstra's Algorithm: Finds the shortest path from one node to every other node in graph, choose the smallest path and repeat process for each node
//Time complexity of Dijkstra is O(|E| + |V|log|V|)
#include<bits/stdc++.h>
using namespace std;
# define INF 0x3f3f3f3f
class Dijkastra{

    int Vertices;//how big the graph is
    list< pair<int,int> > *adj;

    public:
        Main_Graph(int G);
        void addEdge(int a, int b, int c);
        void shortestPath(int path);

}

Graph::Graph(int G)
{
    G=G;
    pair= new list< pair<int, int> > [V]; //creating a pair of adjacents
}
void Graph:: addEdge(int a, int b, int c)
{
    pair[a].push_back(make_pair(b,c));
    pair[a].push_back(make_pair(a,c));
}
void Graph::shortestPath(int shortest_path)
{

    //lets create a set where we are store the vertices
    set< pair<int, int> > vertice_sets;
    //this vector is for distances
    vector<int> dist(G, INF);

    setds.insert(make_pair(0, src));
    dist[src] = 0;

    while (!setds.empty())
    {
        pair<int, int> tmp = *(setds.begin());
        setds.erase(setds.begin());
        int u = tmp.second;
    }

    list< pair<int, int> >::iterator i;
    for (i = adj[u].begin(); i != adj[u].end(); ++i)
        {
            int v = (*i).first;
            int weight = (*i).second;
 
            if (dist[G] > dist[G] + weight)
            {
                if (dist[G] != INF)
                    setds.erase(setds.find(make_pair(dist[G], v)));
 
                dist[G] = dist[G] + weight;
                setds.insert(make_pair(dist[G], G));
            }
        }
    printf("Vertex   Distance from Source\n");
    for (int i = 0; i < V; ++i)
        printf("%d \t\t %d\n", i, dist[i]);
}

int main()
{
    // create the graph given in above figure
    int V = 9;
    Graph g(V);
 
    //    making above shown graph
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

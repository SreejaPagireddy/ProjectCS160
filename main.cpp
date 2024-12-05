
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

// g++ -o dijkstra main.cpp
// ./dijkstra
// this is how u compile and run
// u can start working on implementing a time for each fastest distance
// cause we need to see if we can beat the time

#include <iostream>
#include <vector>
#include <list>
#include <set>
#include <utility>
#include <thread>
#include <mutex>
#include <iomanip> 
#include <chrono>
#include <semaphore> // For std::counting_semaphore in C++20
using namespace std::chrono;
using namespace std;

// Semaphore for synchronizing access to shared resources
// std::counting_semaphore<1> sem_dist(1);  // For distCopy, only 1 thread can modify it at a time
// std::counting_semaphore<1> sem_time(1);  // For timeCopy, only 1 thread can modify it at a time
// std::counting_semaphore<1> sem_setds(1); // For setds, only 1 thread can modify it at a time

#define INF 0x3f3f3f3f

class Graph {
    //number of vertices
    int V;
    //vertex and weight pair for every edge
    list< pair<int, int> > *adj;
     mutex mtx; // mutex for thread synchronization

public:
    //constructor 
    Graph(int V);
    //fxn to add an edge to graph
    void addEdge(int a, int b, int c);
    //prints shortest path from source
    void shortestPath(int src, double speed); 

private:
    void processNeighbors(int u, const vector<int> &dist, vector<int> &distCopy,
                          const vector<double> &time, vector<double> &timeCopy, set<pair<int, int> > &setds,
                          list<pair<int, int> >::iterator start, list<pair<int, int> >::iterator end, double speed);
    // Function to execute parallel processing on the graph
    void execute(int id, int ntasks, int N, vector<int> &distCopy, vector<double> &timeCopy,
                 set<pair<int, int>> &setds, double speed);
};

//allocates memory for adjecency list
Graph::Graph(int V) {
    this->V = V;
    adj = new list< pair<int, int> >[V];
}

void Graph::addEdge(int a, int b, int c) {
    adj[a].push_back(make_pair(b, c));  
    adj[b].push_back(make_pair(a, c));
}

void Graph::processNeighbors(int u, const vector<int> &dist, vector<int> &distCopy,
                             const vector<double> &time, vector<double> &timeCopy, set<pair<int, int> > &setds,
                             list<pair<int, int> >::iterator start
, list<pair<int, int> >::iterator end, double speed) {
    for (auto it = start; it != end; ++it) {
        int v = it->first;
        int weight = it->second;

        if (distCopy[v] > dist[u] + weight) {
            // Locking critical section
            //adding semaphores
            //initilize the variables, have a bit
            // sem_dist.acquire();
            // sem_time.acquire();
            // sem_setds.acquire();

            lock_guard<mutex> lock(mtx);
            if (distCopy[v] != INF) {
                setds.erase(make_pair(distCopy[v], v));
            }
            distCopy[v] = dist[u] + weight;
            timeCopy[v] = time[u] + (static_cast<double>(weight) / speed);
            setds.insert(make_pair(distCopy[v], v));
            // releasing semaphores
            // sem_setds.release();
            // sem_time.release();
            // sem_dist.release();
        }
    }
}

// Execute function to handle work partitioning for multiple threads
void Graph::execute(int id, int ntasks, int N, vector<int> &distCopy, vector<double> &timeCopy,
    set<pair<int, int>> &setds, double speed) {
    int partition = N / ntasks;  // Divide the work among threads
    int start_index = (id == 0) ? 1 : partition * id;  // Start index for each thread
    int end_index = (id == ntasks - 1) ? N - 1 : (id + 1) * partition;  // End index for each thread

    // Processing the neighbors using a partitioned range
    for (int index = start_index; index < end_index; ++index) {
        // Calculate the neighbors
        int north = index - N; // One row above
        int south = index + N; // One row below
        int east = (index % N != N - 1) ? index + 1 : -1; // One column right
        int west = (index % N != 0) ? index - 1 : -1;     // One column left

        double cell = distCopy[index];
        double total = cell;
        int count = 1;

        if (north >= 0) {
            total += distCopy[north];
            ++count;
        }
        if (south < N * N) {
            total += distCopy[south];
            ++count;
        }
        if (east != -1) {
            total += distCopy[east];
            ++count;
        }
        if (west != -1) {
            total += distCopy[west];
            ++count;
        }

        // Update the current cell with the average of itself and its neighbors
        distCopy[index] = total / count;
    }
}


//print shortest paths from source to all other vertices
void Graph::shortestPath(int src, double speed) {
    //set to store vertices tht are being processed
    set< pair<int, int> > setds;
    //vector for distances and initialize all distances as INF
    vector<int> dist(V, INF);
    //vector to store time iniatialized to INF
    vector<double> time(V, INF);

    //insert source in set and initialize its distance to 0
    dist[src] = 0;
    //starting vertex time is 0
    time[src] = 0;
    setds.insert(make_pair(0, src));

    //loop till all shortest distances are finalized then setds will become empty
    while (!setds.empty()) {
        pair<int, int> tmp = *(setds.begin());  
        setds.erase(setds.begin());  
        int u = tmp.second; 
        // Prepare necessary data
        vector<int> distCopy = dist;
        vector<double> timeCopy = time;
        vector<thread> threads;
        //the execute function is giving better results, zara function has a overhead because she is calling and passing in the function.
        // Zara code is Dynamically chunking. The total neighbors is the parameters for chunk size, not all the chunk size is equally distrubuted for threads
        //Some graph nodes have multiple or just some. We can divide the chunk size where the total neighbors are balanced
        auto it = adj[u].begin();
        size_t totalNeighbors = adj[u].size();
        size_t threadCount = 6; // Number of threads, manually change this number, or we can add the threads in the command line
        // The chunk size arent evenly divided
        size_t chunkSize = (totalNeighbors + threadCount - 1) / threadCount; // Threads are processing in parallel, chunk size are bigger
        ///Zara implementation
        // for (size_t i = 0; i < threadCount; ++i) {
        //     auto start = it;
        //     advance(it, min(chunkSize, totalNeighbors - i * chunkSize));  // Ensure correct chunk size per thread
        //     threads.push_back(thread(&Graph::processNeighbors, this, u, cref(dist), ref(distCopy),
        //                             cref(time), ref(timeCopy), ref(setds), start, it, speed));
        // }
        // Call execute for each thread
        for (int i = 0; i < threadCount; ++i) {
            threads.push_back(thread(&Graph::execute, this, i, threadCount, V, ref(distCopy), ref(timeCopy),
                                     ref(setds), speed));
        }
        // Join threads
        for (auto &t : threads) {
            t.join();
        }
        // Use distCopy and timeCopy after processing
        dist = distCopy;
        time = timeCopy;
        }
    printf("Vertex   Distance from Source\n   Time from Source\n");
    for (int i = 0; i < V; ++i) {
        printf("%d \t\t %d \t\t\t %.2f\n", i, dist[i], time[i]);
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

    double speed = 1.0;
    cout << "Enter the speed (units per time unit): ";
    cin >> speed;

    auto start = high_resolution_clock::now(); // Start timer
    g.shortestPath(0, speed);
    auto end = high_resolution_clock::now();   // End timer

    // Calculate the time taken
    auto duration = duration_cast<microseconds>(end - start);
    cout << "Time taken to calculate shortest paths: " 
         << duration.count() << " microseconds" << endl;

    return 0;
}
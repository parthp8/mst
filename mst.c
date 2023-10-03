#include "graphutils.h" // header for functions to load and free adjacencyList

// A program to find the minimum spanning tree of a weighted undirected graph using Prim's algorithm

int main ( int argc, char* argv[] ) {

    // READ INPUT FILE TO CREATE GRAPH ADJACENCY LIST
    AdjacencyListNode* adjacencyList;
    graphNode_t graphNodeCount = adjMatrixToList(argv[1], &adjacencyList);

    // An array that keeps track of who is the parent node of each graph node we visit
    // In Prim's algorithm, this parents array keeps track of what is the edge that connects a node to the MST.
    graphNode_t* parents = calloc( graphNodeCount, sizeof(graphNode_t) );
    bool* visited = calloc( graphNodeCount, sizeof(bool) );
    double* cost = calloc( graphNodeCount, sizeof(double) );
    for (size_t i = 0; i < graphNodeCount; i++) {
        parents[i] = -1; // -1 indicates that a nodes is not yet visited; i.e., node not yet connected to MST.
        visited[i] = false;
        if (i == 0) cost[i] = 0;
        else cost[i] = DBL_MAX;
    }

    // Prim's algorithm:
    // A greedy algorithm that builds the minimum spanning tree.
    // For a graph with N nodes, the minimum spanning tree will have N-1 edges spanning all nodes.
    // Prim's algorithm starts with all nodes unconnected.
    // At each iteration of Prim's algorithm, the minimum weight node that connects an unconnected node to the connected set of nodes is added to the MST.
    for (unsigned iter = 0; iter < graphNodeCount - 1; iter++) {

        double minWeight = DBL_MAX; // If we find an edge with weight less than this minWeight, and edge connects a new node to MST, then mark this as the minimum weight to beat.
        graphNode_t minSource = -1;

        for (unsigned i = 0; i < graphNodeCount; i++) {
            if (cost[i] < minWeight && !visited[i]) {
                minWeight = cost[i];
                minSource = i;
            }
        }
        
        visited[minSource] = true;

        for (AdjacencyListNode* e = adjacencyList[minSource].next; e; e = e->next) {
            if (!visited[e->graphNode] && e->weight < cost[e->graphNode]) {
                cost[e->graphNode] = e->weight;
                parents[e->graphNode] = minSource;
            }
        }
    }

    for (int i = 0; i < graphNodeCount; i++) {
        if (parents[i] != -1) printf("%ld %d\n", parents[i], i);
    }

    free (parents);
    free(visited);
    free(cost);
    freeAdjList ( graphNodeCount, adjacencyList );

    return EXIT_SUCCESS;
}
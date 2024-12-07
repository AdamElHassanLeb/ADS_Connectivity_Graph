#ifndef ADS_MIDTERM_GRAPH_H
#define ADS_MIDTERM_GRAPH_H

#include "sparsepp/spp.h"
#include "config.h"
#include<iostream>
#include <climits>
#include <forward_list>
#include <queue>
#include <unordered_set>
#include <omp.h>
#include <condition_variable>
#include <fstream>
#include <filesystem>

// 0.9 since the library uses separate chaining and all Ids are unique integers
const float LOAD_FACTOR = 0.9f;

/**
 * Represents a graph data structure.
 */
class Graph{

private:
    struct AdjVertex{
        int id;
        std::forward_list<int> childList;
        // Constructor to initialize id and childList
        explicit AdjVertex(int idValue){
            id = idValue;
        }
        // This is only for erase function to work
        AdjVertex(){}
    };

    int nbVerticies;
    int nbEdges;
    // HashMap used as it was the best way to mitigate the empty slots problem
    spp::sparse_hash_map<int, AdjVertex> verticies; // Updated to spp::sparse_hash_map
    typedef spp::sparse_hash_map<int, AdjVertex> vertexMap; // Updated typedef
    // Cleaner Use for iterators
    typedef spp::sparse_hash_map<int, AdjVertex>::iterator vertexIterator; // Updated typedef
    typedef std::forward_list<int>::iterator listIterator;
    // To Avoid using auto
    typedef std::pair<vertexIterator, bool> insertReturn;
    // For MultiThreaded Function Calls
    typedef spp::sparse_hash_map<int, bool> visitedMap; // Updated to spp::sparse_hash_map
    typedef spp::sparse_hash_map<int, int> parentMap; // Updated to spp::sparse_hash_map

public:
    // No-Arg
   Graph();

   /**
    * Constructs a graph with an expected number of elements.
    * @param nbElements The expected number of elements to be inserted.
    */
   Graph(int nbElements);

   /**
    * Adds a vertex to the graph.
    * @param id The ID of the vertex to add.
    * @return 0 if successful, -1 if the vertex already exists.
    */
   int addVertex(int id);

   /**
    * Adds an edge between two vertices.
    * @param id1 The ID of the first vertex.
    * @param id2 The ID of the second vertex.
    * @return 0 if successful, -1 if one of the vertices doesn't exist, 1 if the edge already exists.
    */
   int addEdge(int id1, int id2);

   /**
    * Deletes a vertex from the graph.
    * @param id The ID of the vertex to delete.
    * @return 0 if successful, 1 if the vertex doesn't exist.
    */
   int deleteVertex(int id);

   /**
    * Deletes an edge between two vertices.
    * @param id1 The ID of the first vertex.
    * @param id2 The ID of the second vertex.
    * @return 0 if successful, 1 if the edge doesn't exist.
    */
   int deleteEdge(int id1, int id2);

   /**
    * Checks if a vertex exists in the graph.
    * @param id The ID of the vertex to check.
    * @return True if the vertex exists, false otherwise.
    */
   bool vertexExist(int id);

   /**
    * Checks if an edge exists between two vertices.
    * @param id1 The ID of the first vertex.
    * @param id2 The ID of the second vertex.
    * @return True if the edge exists, false otherwise.
    */
   bool edgeExist(int id1, int id2);

   /**
    * Checks if graph is empty.
    * @return True if the graph is empty, false otherwise.
    */
    bool isEmpty();

   /**
    * Gets all vertices in the graph.
    * @return vector of vertices.
    */
    std::vector<int> getAllVertices();

   /**
    * Gets all edges in the graph.
    * @return vector of edges in the graph.
    */
    std::vector<std::pair<int, int>> getAllEdges();


    /**
 * Retrieves the user edges for the specified vertex ID.
 * The user edges represent the vertices directly
 * connected to the specified vertex.
 * @param id the ID of the vertex to retrieve the user edges for.
 * @return a vector containing the user edges for the specified vertex ID, or an empty vector if no user edges are found or if the vertex does not exist.
 */
    std::vector<int> getUserEdges(int id);

   /**
    * Gets number of edges of a vertex.
    * @param id the id of the vertex
    * @return integer representing number of edges the vertex has.
    */
    int getNumberOfUserEdges(int id);

    /**
     * Returns the number of all edges in the graph.
     *
     * @return the number of all edges in the graph.
     */
    int getNbAllEdges();
    /**
     * Returns the number of all vertices in the graph.
     *
     * @return the number of all vertices in the graph.
     */
    int getNbAllVertices();


   /**
    * Gets vertex with maximum number of edges.
    * @return integer representing id of the vertex with maximum number of edges.
    */
    int getVertexWithMaxDegree();

   /**
    * Clears the graph.
    */
    void clear();

   /**
    * Checks if two vertices are connected.
    * @param id1 The ID of the first vertex.
    * @param id2 The ID of the second vertex.
    * @return True if there is a connection, false otherwise.
    */
    bool isConnected(int id1, int id2);

   /**
    * Checks if graph is Complete (every vertex is directly connected).
    * @return True if it is complete, false otherwise.
    */
    bool isComplete();

   /**
    * Depth-First Search (DFS) traversal function.
    * @param startVertex The starting vertex for DFS traversal.
    * @return Vector containing vertices visited during DFS traversal.
    */
    std::vector<int> DFS(int startVertex);

   /**
    * Breadth-First Search (BFS) traversal function.
    * @param startVertex The starting vertex for BFS traversal.
    * @return Vector containing vertices visited during BFS traversal.
    */
    std::vector<int> BFS(int startVertex);

 /**
   * Writes graph data to a file.
   * The file will contain vertex and edge information in a specific format.
   * @param filename The name of the file to write to.
   */
   int writeToFile(const std::string& filename);

  /**
   * Reads graph data from a file and populates the graph accordingly.
   * The file should contain vertex and edge information in a specific format.
   * @param filename The name of the file to read from.
   */
   int readFromFile(const std::string& filename);

   /**
    * Prints the adjacency list of the graph.
    */
    void printList();

    /**
     * Finds the shortest path between two vertices using bidirectional BFS.
     * @param source The ID of the source vertex.
     * @param target The ID of the target vertex.
     * @return A vector containing the IDs of the vertices in the shortest path, or an empty vector if no path exists.
     */
     std::vector<int> shortestPath(int source, int target);

    /**
     * Finds the shortest path between two vertices using bidirectional BFS with multithreading.
     * @param source The ID of the source vertex.
     * @param target The ID of the target vertex.
     * @return A vector containing the IDs of the vertices in the shortest path, or an empty vector if no path exists.
     */
     std::vector<int> shortestPathThreaded(int source, int target);

    /**
     * Rehashes the graph to accommodate a new number of elements.
     * @param nbElements The new expected number of elements.
     * @return 0 if successful, 1 if the rehashing failed.
     */
     int rehash(int nbElements);


    /**
        * Checks whether the graph is regular.
        *
        * @return true if the graph is regular, false otherwise.
        */
    bool isRegular();

    /**
     * Calculates the density of the graph.
     *
     * @return The density of the graph as a double value.
     */
    double graphDensity();

    /**
     * Computes the average path length of the graph.
     *
     * @return The average path length of the graph as a double value.
     */
    double averagePathLength();

private:
    /**
     * Performs BFS traversal from the source vertex.
     * @param s_queue The queue for BFS traversal.
     * @param mtx The mutex for synchronization.
     * @param cv The condition variable for synchronization.
     * @param finished A flag indicating if traversal is finished.
     * @param s_visited Visited nodes from the source.
     * @param t_visited Visited nodes from the target.
     * @param s_parent Parent nodes from the source.
     * @param t_parent Parent nodes from the target.
     * @param verticies The map of vertices.
     * @param target The ID of the target vertex.
     * @param intersectNode The ID of the intersection node.
     */
    void bfsFromSource(std::queue<int> &s_queue, std::mutex& mtx, std::condition_variable& cv,
                       bool& finished, visitedMap &s_visited,
                       visitedMap &t_visited, parentMap &s_parent, parentMap & t_parent,
                       vertexMap &verticies, int target, int &intersectNode);

    /**
     * Performs BFS traversal from the target vertex.
     * @param t_queue The queue for BFS traversal.
     * @param mtx The mutex for synchronization.
     * @param cv The condition variable for synchronization.
     * @param finished A flag indicating if traversal is finished.
     * @param s_visited Visited nodes from the source.
     * @param t_visited Visited nodes from the target.
     * @param s_parent Parent nodes from the source.
     * @param t_parent Parent nodes from the target.
     * @param verticies The map of vertices.
     * @param source The ID of the source vertex.
     * @param intersectNode The ID of the intersection node.
     */
    void bfsFromTarget(std::queue<int>& t_queue, std::mutex& mtx, std::condition_variable& cv,
                       bool& finished, visitedMap & s_visited, visitedMap & t_visited, parentMap & s_parent,
                       parentMap & t_parent, vertexMap& verticies, int source,
                       int& intersectNode);

    /**
     * Depth-First Search (DFS) traversal starting from a given vertex.
     * @param startVertex The starting vertex for DFS traversal.
     * @param visited vector to keep track of visited vertices.
     * @param visitedVertices Vector to store visited vertices during traversal.
     */
    void DFSAux(int startVertex, std::vector<bool>& visited, std::vector<int>& visitedVertices);

};

#endif //ADS_MIDTERM_GRAPH_H

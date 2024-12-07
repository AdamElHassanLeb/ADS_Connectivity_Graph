//
// Created by adam on 4/24/24.
//
#include "Graph.h"

Graph::Graph(){
    verticies.set_deleted_key(INT_MAX);
    nbVerticies = 0;
    nbEdges = 0;
}

Graph::Graph(int nbElements) {
    verticies.set_deleted_key(INT_MAX);
    rehash(nbElements);
    nbVerticies = 0;
    nbEdges = 0;
}

int Graph::addVertex(int id){

    insertReturn result = verticies.insert({id, AdjVertex{id}});
    if(result.second){
        nbVerticies++;
        return 0;
    }
    return 1;
}

int Graph::addEdge(int id1, int id2){

    //Check if vertex is trying to connect to itself
    if(id1 == id2) return 1;

    vertexIterator it1 = verticies.find(id1);
    vertexIterator it2 = verticies.find(id2);
    //If verticies do not exist return 1
    if(it1 == verticies.end() || it2 == verticies.end())
        return 1;

    listIterator lit1 = std::find(it1->second.childList.begin(), it1->second.childList.end(), id2);
    listIterator lit2 = std::find(it2->second.childList.begin(), it2->second.childList.end(), id1);

    //If edge already exists return 1
    if(lit1 != it1->second.childList.end() || lit2!= it2->second.childList.end()){
        return 1;
    }

    //Insert Edge
    it1->second.childList.push_front(id2);
    it2->second.childList.push_front(id1);
    //TODO Optimize by inserting at position
    it1->second.childList.sort();
    it2->second.childList.sort();

    //Increment nbEdges;
    nbEdges++;
    return 0;
}

int Graph::deleteVertex(int id){

    //Get iterator to vertex
    vertexIterator vit = verticies.find(id);

    //Check if vertex exists
    if(vit == verticies.end()) return 1;

    //Deleting all related edges

    //Cannot use deleteEdge function due to Iterator(pointer) bug with forward_list
    //Delete all edges pointing to this vertex in other verticies
    for(listIterator lit = vit->second.childList.begin(); lit != vit->second.childList.end(); lit++){
        vertexIterator it1 = verticies.find(*lit);
        it1->second.childList.remove(id);
        nbEdges--;
    }
    // Erase the vertex from the map using the iterator returned by find including all its edges.
    //Decrement nbEdges and return 0
    nbVerticies--;
    verticies.erase(id);
    return 0;
}


int Graph::deleteEdge(int id1,int id2){
    //Check if vertex is trying to connect to itself
    if(id1 == id2) return 1;

    vertexIterator it1 = verticies.find(id1);
    vertexIterator it2 = verticies.find(id2);
    //If verticies do not exist return 1
    if(it1 == verticies.end() || it2 == verticies.end())
        return 1;

    listIterator lit1 = std::find(it1->second.childList.begin(), it1->second.childList.end(), id2);
    listIterator lit2 = std::find(it2->second.childList.begin(), it2->second.childList.end(), id1);

    //If edge exists return 0
    if(lit1 != it1->second.childList.end() && lit2!= it2->second.childList.end()){

        it1->second.childList.remove(id2);
        it2->second.childList.remove(id1);
        nbEdges--;
        return 0;
    }
    return 1;
}

bool Graph::vertexExist(int id){
    // Check if the vertex with the given ID exists in the map.
    return (verticies.find(id) != verticies.end());
}
bool Graph::edgeExist(int id1,int id2){
    //Check if vertex is trying to connect to itself
    if(id1 == id2) return false;

    vertexIterator it1 = verticies.find(id1);
    vertexIterator it2 = verticies.find(id2);
    //If verticies do not exist return 1
    if(it1 == verticies.end() || it2 == verticies.end())
        return false;

    listIterator lit1 = std::find(it1->second.childList.begin(), it1->second.childList.end(), id2);
    listIterator lit2 = std::find(it2->second.childList.begin(), it2->second.childList.end(), id1);

    //If edge exists return 0
    if(lit1 != it1->second.childList.end() && lit2!= it2->second.childList.end()){
        return true;
    }

    return false;
}


bool Graph::isEmpty() {
    //check if graph is empty
    return verticies.empty();
}

std::vector<int> Graph::getAllVertices() {
    //create vector of size number of vertices
    std::vector<int> allVertices(verticies.size());
    //create iterator and set at beginning of verticies
    vertexIterator vit = verticies.begin();
    //loop over vector populating it with the ids from the AdjVertex in verticies
    for(int i = 0; i < allVertices.size(); i++){
        allVertices[i] = vit->second.id;
        ++vit;
    }
    return allVertices;
}

std::vector<std::pair<int, int>> Graph::getAllEdges() {
    //create vector of pairs
    std::vector<std::pair<int, int>> allEdges;
    //create an iterator for the vertices, and a list iterator
    vertexIterator vit;
    listIterator lit;
    //Go over all the verticies
    for(vit = verticies.begin(); vit != verticies.end(); vit++) {
        for(lit = vit->second.childList.begin(); lit != vit->second.childList.end(); lit++) {
            //check if edge already added in the list
            bool alreadyAdded = false;

            for(int i = 0; i < allEdges.size(); i++) {
                if(allEdges[i].first == vit->second.id && allEdges[i].second == *lit
                    || allEdges[i].first == *lit && allEdges[i].second == vit->second.id) {
                    alreadyAdded = true;
                    break;
                }
            }
            // If the edge doesn't exist, add it to the vector
            if (!alreadyAdded) {
                //emplace_back constructs the pair directly and adds it to the vector
                //(instead of using "push_back(std::make_pair")
                allEdges.emplace_back(vit->second.id, *lit);
            }
        }
    }
    return allEdges;
}

std::vector<int> Graph::getUserEdges(int id) {

    std::vector<int> edges;

    vertexIterator vit = verticies.find(id);
    //check if vertex exists
    if (vit == verticies.end()) {
        return edges;
    }
    //set list iterator to iterate over the child list
    listIterator lit = vit->second.childList.begin();

    //loop over every edge and increment the counter
    while(lit != vit->second.childList.end()) {
        edges.emplace_back(*lit);
        lit++;
    }
    return edges;
}


int Graph::getNumberOfUserEdges(int id) {
    return (int)getUserEdges(id).size();
}


int Graph::getNbAllEdges() {
    return nbEdges;
}

int Graph::getNbAllVertices() {
    return nbVerticies;
}

int Graph::getVertexWithMaxDegree() {
    //set vertex iterator over first vertex and set it as the max
    vertexIterator vit = verticies.begin();
    int max = vit->second.id;

    //iterate over vertices until reaching the end
    while(vit != verticies.end()) {
        //check if current vertex has more edges than the max
        if(getNumberOfUserEdges(max) < getNumberOfUserEdges(vit->second.id)) {
            //set the current vertex as the max
            max = vit->second.id;
        }
        vit++;
    }
    return max;
}

void Graph::clear() {
    verticies.clear();
}

bool Graph::isConnected(int id1, int id2) {
    //check if same id
    if(id1 == id2) return false;

    return !(shortestPathThreaded(id1, id2).empty());
}

bool Graph::isComplete() {
    //Iterate over all pairs of distinct vertices
    for (vertexIterator it1 = verticies.begin(); it1 != verticies.end(); ++it1) {
        for (vertexIterator it2 = std::next(it1); it2 != verticies.end(); ++it2) {
            // Check if there is an edge between the pair of vertices
            if (!edgeExist(it1->second.id, it2->second.id)) {
                // If any pair of vertices is not connected, graph is not complete
                return false;
            }
        }
    }
    // If all pairs of distinct vertices are connected, graph is complete
    return true;
}

void Graph::DFSAux(int startVertex, std::vector<bool>& visited, std::vector<int>& visitedVertices) {
    // Mark the current vertex as visited
    visited[startVertex] = true;

    // Store the visited vertex
    visitedVertices.push_back(startVertex);

    //initialize the vertices iterator to current vertex
    vertexIterator vit = verticies.find(startVertex);
    //check if vertes exists
    if (vit == verticies.end())
        return;
    // Loop over the child list of the current vertex
    for (listIterator lit = vit->second.childList.begin(); lit != vit->second.childList.end(); ++lit) {
        int neighbor = *lit;
        if (!visited[neighbor]) {
            // Recursively visit the neighbor
            DFSAux(neighbor, visited, visitedVertices);
        }
    }
}

std::vector<int> Graph::DFS(int startVertex) {
    // Initialize the visited array
    std::vector<bool> visited(verticies.size(), false);

    // Vector to store visited vertices during traversal
    std::vector<int> visitedVertices;

    // Perform DFS traversal from the startVertex
    DFSAux(startVertex, visited, visitedVertices);

    return visitedVertices;
}

std::vector<int> Graph::BFS(int startVertex) {
    std::vector<int> traversal;
    std::unordered_map<int, bool> visited; //unordered map better time complexity than ordered map
    std::queue<int> q;
    listIterator lit;

    // Mark the start vertex as visited and enqueue it
    visited[startVertex] = true;
    q.push(startVertex);

    while (!q.empty()) {
        // Dequeue a vertex from the queue
        int currentVertex = q.front();
        q.pop();
        // Push it into the traversal result
        traversal.push_back(currentVertex);

        // Iterate over the adjacency list of the current vertex
        for (lit = verticies[currentVertex].childList.begin(); lit != verticies[currentVertex].childList.end(); ++lit) {
            int neighbor = *lit;
            // If the neighbor has not been visited, mark it as visited and enqueue it
            if (!visited[neighbor]) {
                visited[neighbor] = true;
                q.push(neighbor);
            }
        }
    }
    return traversal;
}

// Method to read graph data from a file
int Graph::readFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        //std::cerr << "Error: Unable to open file: " << filename << std::endl;
        return 1;
    }

    char type;
    while (file >> type) {
        if (type == 'V') {
            int vertexId;
            file >> vertexId;
            addVertex(vertexId);
        } else if (type == 'E') {
            int id1, id2;
            file >> id1 >> id2;
            addEdge(id1, id2);
        }
    }

    file.close();
    return 0;
}

// Method to write graph data to a file
int Graph::writeToFile(const std::string& filename) {
    // Open the file for writing, creating a new one
    std::ofstream file(filename); // Truncate mode to delete existing data and rewrite
    if (!file.is_open()) {
        // Return 1 if unable to open the file
        return 1;
    }

    // Write vertices
    std::vector<int> vertices = getAllVertices();
    for (int i = 0; i < vertices.size(); ++i) {
        file << "V " << vertices[i] << std::endl; // Write each vertex with 'V' prefix
    }

    // Write edges
    std::vector<std::pair<int, int>> edges = getAllEdges();
    for (int i = 0; i < edges.size(); ++i) {
        file << "E " << edges[i].first << " " << edges[i].second << std::endl; // Write each edge with 'E' prefix
    }

    file.close(); // Close the file after writing
    return 0;
}

void Graph::printList() {
    for (vertexIterator vit = verticies.begin(); vit != verticies.end(); ++vit) {
        std::cout << "Vertex: " << vit->second.id << " Edges: " ;

        for(listIterator lit = vit->second.childList.begin(); lit != vit->second.childList.end(); lit++){
            std::cout << static_cast<int>(*lit) << " -> ";
        }

        std::cout << std::endl;
    }
}

int Graph::rehash(int nbElements){
    //Checks if greater than 0 and
    //that the map will maintain an acceptable load factor
    if(nbElements > verticies.size()) {
        int size = (int) (nbElements / LOAD_FACTOR); // to maintain the preferred load factor
        verticies.resize(size);
        return 0;
    }
    return 1;
}


bool Graph::isRegular() {
    // Get the degree of the first vertex
    int degree = getNumberOfUserEdges(verticies.begin()->first);

    // Iterate over all vertices and check if their degrees are equal
    for (vertexIterator vit = verticies.begin(); vit != verticies.end(); ++vit) {
        if (getNumberOfUserEdges(vit->first) != degree) {
            return false;
        }
    }
    return true;
}

double Graph::graphDensity() {
    int numEdges = nbEdges;

    // For an undirected graph, divide by 2
    numEdges /= 2;

    // Calculate the maximum possible number of edges
    int maxEdges = (nbVerticies * (nbVerticies - 1)) / 2;

    // Calculate the density
    double density = static_cast<double>(numEdges) / maxEdges;

    return density;
}

double Graph::averagePathLength() {
    double totalPathLength = 0;
    int nbPaths = 0;

    for (vertexIterator it = verticies.begin(); it != verticies.end(); ++it) {
        // Sum up the distances
        for (vertexIterator mit = verticies.begin(); mit != verticies.end(); ++mit) {

            if(mit->second.id == it->second.id) continue;

            std::vector<int> v = shortestPathThreaded(it->second.id, mit->second.id);
            if(!v.empty()) {
                nbPaths++;
                totalPathLength += static_cast<double>(v.size() - 1);
            }
        }
    }

    if (nbPaths == 0) {
        return 0; // Avoid division by zero
    }

    return totalPathLength / nbPaths;
}

std::vector<int> Graph::shortestPath(int source, int target) {

    // Vector to store the shortest path
    std::vector<int> path;

    if(source == target){
        path.push_back(source);
        return path;
    }

    // Hash maps to track visited nodes from source and target
    spp::sparse_hash_map<int, bool> s_visited;
    spp::sparse_hash_map<int, bool> t_visited;

    // Arrays to store parent nodes for reconstructing the path
    spp::sparse_hash_map<int, int> s_parent;
    spp::sparse_hash_map<int, int> t_parent;

    // Queue for BFS traversal from source and target vertices
    std::queue<int> s_queue, t_queue;

    // Initialize source and target queues and mark them visited
    s_queue.push(source);
    s_visited[source] = true;
    s_parent[source] = -1;

    t_queue.push(target);
    t_visited[target] = true;
    t_parent[target] = -1;

    // Intersection point of BFS traversal from source and target
    int intersectNode = -1;

    // Perform bidirectional BFS
    while (!s_queue.empty() && !t_queue.empty()) {
        // BFS from source
        int current = s_queue.front();
        s_queue.pop();
        for (listIterator it = verticies[current].childList.begin();
        it != verticies[current].childList.end(); ++it) {
            int child = *it;
            if (!s_visited[child]) {
                s_visited[child] = true;
                s_parent[child] = current;
                s_queue.push(child);
            }
            // Check for intersection
            if (t_visited[child]) {
                intersectNode = child;
                break;
            }
        }

        // BFS from target
        current = t_queue.front();
        t_queue.pop();
        for (listIterator it = verticies[current].childList.begin();
             it != verticies[current].childList.end(); ++it) {
            int child = *it;
            if (!t_visited[child]) {
                t_visited[child] = true;
                t_parent[child] = current;
                t_queue.push(child);
            }
            // Check for intersection
            if (s_visited[child]) {
                intersectNode = child;
                break;
            }
             }

        // If intersection found, break the loop
        if (intersectNode != -1)
            break;
    }

    // If there's no intersection, there's no path
    if (intersectNode == -1)
        return path;

    // Construct the shortest path from source to intersection
    int current = intersectNode;
    while (current != source) {
        path.push_back(current);
        current = s_parent[current];
    }
    path.push_back(source);
    std::reverse(path.begin(), path.end());

    // Construct the shortest path from target to intersection
    current = intersectNode;
    while (current != target) {
        current = t_parent[current];
        path.push_back(current);
    }

    return path;
}

std::vector<int> Graph::shortestPathThreaded(int source, int target) {
    // Vector to store the shortest path
    std::vector<int> path;

    // Hash maps to track visited nodes from source and target
    spp::sparse_hash_map<int, bool> s_visited;
    spp::sparse_hash_map<int, bool> t_visited;

    // Arrays to store parent nodes for reconstructing the path
    spp::sparse_hash_map<int, int> s_parent;
    spp::sparse_hash_map<int, int> t_parent;

    // Queue for BFS traversal from source and target vertices
    std::queue<int> s_queue, t_queue;

    // Initialize source and target queues and mark them visited
    s_queue.push(source);
    s_visited[source] = true;
    s_parent[source] = -1;

    t_queue.push(target);
    t_visited[target] = true;
    t_parent[target] = -1;

    // Intersection point of BFS traversal from source and target
    int intersectNode = -1;

    // Mutex and condition variable for synchronization
    std::mutex mtx;
    std::condition_variable cv;
    bool finished = false;

    // Start two threads for BFS traversal from source and target
#pragma omp parallel sections
    {
#pragma omp section
        {
            // Perform BFS traversal from source vertex
            bfsFromSource(s_queue, mtx, cv, finished, s_visited,
                          t_visited, s_parent, t_parent, verticies,
                          target, intersectNode);
        }
#pragma omp section
        {
            // Perform BFS traversal from target vertex
            bfsFromTarget(t_queue, mtx, cv, finished, s_visited,
                          t_visited, s_parent, t_parent, verticies,
                          source, intersectNode);
        }
    }

    // If there's no intersection, there's no path
    if (intersectNode == -1)
        return path;

    // Construct the shortest path from source to intersection
    int current = intersectNode;
    while (current != source) {
        path.push_back(current);
        current = s_parent[current];
    }
    path.push_back(source);
    std::reverse(path.begin(), path.end());

    // Construct the shortest path from target to intersection
    current = intersectNode;
    while (current != target) {
        current = t_parent[current];
        path.push_back(current);
    }

    return path;
}

// Function for performing BFS from source vertex
void Graph::bfsFromSource(std::queue<int> &s_queue, std::mutex& mtx, std::condition_variable& cv,
                          bool& finished, visitedMap &s_visited,
                          visitedMap &t_visited, parentMap &s_parent, parentMap & t_parent,
                          vertexMap &verticies, int target, int &intersectNode) {
    while (!s_queue.empty()) {
        int current;
        {
            std::unique_lock<std::mutex> lock(mtx);
            // Exit if finished flag is true
            if (finished) return;
            current = s_queue.front();
            s_queue.pop();
        }
        // Iterate through child vertices
        for (listIterator it = verticies[current].childList.begin();
             it != verticies[current].childList.end(); ++it) {
            int child = *it;
            {
                std::lock_guard<std::mutex> lock(mtx);
                // Mark child as visited and update parent node
                if (!s_visited[child]) {
                    s_visited[child] = true;
                    s_parent[child] = current;
                    s_queue.push(child);
                }
                // Check if child is also visited by the other thread
                if (t_visited[child]) {
                    intersectNode = child;
                    finished = true;
                    // Notify other threads
                    cv.notify_all();
                    return;
                }
            }
        }
    }
}

// Function for performing BFS from target vertex
void Graph::bfsFromTarget(std::queue<int>& t_queue, std::mutex& mtx, std::condition_variable& cv,
                          bool& finished, visitedMap & s_visited, visitedMap & t_visited, parentMap & s_parent,
                          parentMap & t_parent, vertexMap& verticies, int source,
                          int& intersectNode) {
    while (!t_queue.empty()) {
        int current;
        {
            //Lock For Critical Section
            //Unlocked Automatically When Out Of Scope (Loop Iteration in This Case)
            std::unique_lock<std::mutex> lock(mtx);
            // Exit if finished flag is true
            if (finished) return;
            current = t_queue.front();
            t_queue.pop();
        }
        // Iterate through child vertices
        for (listIterator it = verticies[current].childList.begin();
             it != verticies[current].childList.end(); ++it) {
            int child = *it;
            {
                //Lock For Critical Section
                //Unlocked Automatically When Out Of Scope (Loop Iteration in This Case)
                std::lock_guard<std::mutex> lock(mtx);
                // Mark child as visited and update parent node
                if (!t_visited[child]) {
                    t_visited[child] = true;
                    t_parent[child] = current;
                    t_queue.push(child);
                }
                // Check if child is also visited by the other thread
                if (s_visited[child]) {
                    intersectNode = child;
                    finished = true;
                    // Notify other threads
                    cv.notify_all();
                    return;
                }
            }
        }
    }
}



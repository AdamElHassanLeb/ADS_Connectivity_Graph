#include <iostream>
#include "Graph.h"

//Used in main as no others are used
using namespace std;

Graph social_network;
string fileName = "graph.txt";

void printFirstMenu();
void printMainMenu();
void mainMenu();
void firstMenu();
int takeAndValidateNumericalUserInput();

int main() {
    firstMenu();
    mainMenu();
    return 0;
}

void printFirstMenu() {
    cout << "Welcome!\n";
    cout << "Create Network by:\n";
    cout << "1. Import from File\n";
    cout << "2. Start With Blank Graph \n";
    cout << "Enter your choice: ";
}

void printMainMenu() {
    cout << "\n\n /**************************************************/ \n\n";
    cout << "Main Menu\n";
    cout << "1.  Display Network\n";
    cout << "2.  Clear Network \n";
    cout << "3.  Add User \n";
    cout << "4.  Delete User \n";
    cout << "5.  Add Friendship\n";
    cout << "6.  Delete Friendship\n";
    cout << "7.  Get Most Popular User\n";
    cout << "8.  Check If 2 Users Are Connected\n";
    cout << "9.  Get All Users\n";
    cout << "10. Get All Friendships\n";
    cout << "11. Get All Friendships of User\n";
    cout << "12. Get Degree of Friendship between 2 users\n";
    cout << "13. Check If Network is complete\n";
    cout << "14. DFS traversal\n";
    cout << "15. BFS traversal\n";
    cout << "16. Check If User Exists\n";
    cout << "17. Number of Users\n";
    cout << "18. Number of Friendships\n";
    cout << "19. Check If Graph Is Regular\n";
    cout << "20. Get Graph Density\n";
    cout << "21. Get Average Graph Length\n";
    cout << "22. Write Current State To File\n";
    cout << "23. Check If 2 Users Are Friends\n";
    cout << "0. Exit\n";
    cout << "Enter your choice: ";
}

void mainMenu(){
    int id1, id2, misc;
    int choice;
    std::vector<int> vertices;
    std::vector<int> vertices1;
    std::vector<int> vertices2;
    std::vector<std::pair<int, int>> edges;
    do{
        printMainMenu();
        choice = takeAndValidateNumericalUserInput();
        cout << "\n\n /**************************************************/ \n\n";
        switch (choice) {
            case 0:
                cout << "Exiting Now. Thank you.\n";
                return;
            case 1:
                social_network.printList();
                break;
            case 2:
                social_network.clear();
                break;
            case 3:
                cout << "Enter First User's Id: ";
                id1 = takeAndValidateNumericalUserInput();
                cout << (social_network.addVertex(id1) == 0 ?
                "Added User Successfully\n" : "Could Not Add User \n");
                break;
            case 4:
                cout << "Enter First User's Id: ";
                id1 = takeAndValidateNumericalUserInput();
                cout << (social_network.deleteVertex(id1) == 0 ?
                "Deleted Successfully\n" : "Could Not Delete User \n");
                break;
            case 5:
                cout << "Enter First User's Id: ";
                id1 = takeAndValidateNumericalUserInput();
                cout << "\nEnter Second User's Id: ";
                id2 = takeAndValidateNumericalUserInput();
                cout << (social_network.addEdge(id1, id2) == 0 ?
                "Added Friendship Successfully\n" :
                "Could Not Add Friendship \n");
                break;
            case 6:
                cout << "Enter First User's Id: ";
                id1 = takeAndValidateNumericalUserInput();
                cout << "\nEnter Second User's Id: ";
                id2 = takeAndValidateNumericalUserInput();
                cout << (social_network.deleteEdge(id1, id2) == 0 ?
                         "Deleted Friendship Successfully\n" :
                         "Could Not Delete Friendship \n");
                break;
            case 7:
                if(social_network.isEmpty()) {
                    cout << "List is Empty\n";
                    break;
                }
                else{
                    int x = social_network.getVertexWithMaxDegree();
                    cout << x << " With a Degree of "
                    << social_network.getNumberOfUserEdges(x) << "\n";
                }
                break;
            case 8:
                cout << "Enter First User's Id: ";
                id1 = takeAndValidateNumericalUserInput();
                cout << "\nEnter Second User's Id: ";
                id2 = takeAndValidateNumericalUserInput();
                cout << (social_network.isConnected(id1, id2) ?
                         "They Are Connected\n" :
                         "They Are Not Connected \n");
                break;
            case 9:
                    vertices = social_network.getAllVertices();
                    if(vertices.empty()){
                        cout << "List is Empty \n";
                        break;
                    }

                    for(std::vector<int>::iterator lit = vertices.begin();
                    lit != vertices.end(); lit++){
                        cout << *lit <<" -> ";
                    }
                    cout << "\n";
                break;
            case 10:
                edges = social_network.getAllEdges();
                if(edges.empty()){
                    cout << "List has no edges \n";
                    break;
                }

                for(std::vector<std::pair<int, int>>::iterator lit = edges.begin();
                    lit != edges.end(); lit++){
                    cout << " ("<<lit->first <<" , "<<lit->second <<") "<<" -> ";
                }
                cout << "\n";
                break;
            case 11:
                cout << "Enter First User's Id: ";
                id1 = takeAndValidateNumericalUserInput();
                vertices = social_network.getUserEdges(id1);

                if(vertices.empty()){
                    cout << "User Does Not Exist or Has No Friends \n";
                    break;
                }

                for(std::vector<int>::iterator lit = vertices.begin();
                    lit != vertices.end(); lit++){
                    cout << *lit <<" -> ";
                }
                cout << "\n";

                break;
            case 12:
                cout << "Enter First User's Id: ";
                id1 = takeAndValidateNumericalUserInput();
                cout << "\nEnter Second User's Id: ";
                id2 = takeAndValidateNumericalUserInput();

                vertices = social_network.shortestPathThreaded(id1, id2);
                if(vertices.empty()){
                    cout << "No Path Exists \n";
                    break;
                }

                for(std::vector<int>::iterator lit = vertices.begin();
                    lit != vertices.end(); lit++){
                    cout << *lit <<" -> ";
                }
                cout << "\n";
                cout << "Friendship is of degree: " << vertices.size() - 1<< endl;
                break;
            case 13:
                cout << (social_network.isComplete() ? "Is Complete\n"
                : "Is Not Complete\n");
                break;
            case 14:
                cout << "Enter First User's Id: ";
                id1 = takeAndValidateNumericalUserInput();

                vertices = social_network.DFS(id1);
                if(vertices.empty()){
                    cout << "Not Connected \n";
                    break;
                }

                for(std::vector<int>::iterator lit = vertices.begin();
                    lit != vertices.end(); lit++){
                    cout << *lit <<" -> ";
                }
                cout << "\n";
                break;
            case 15:
                cout << "Enter First User's Id: ";
                id1 = takeAndValidateNumericalUserInput();

                vertices = social_network.BFS(id1);
                if(vertices.empty()){
                    cout << "Not Connected \n";
                    break;
                }

                for(std::vector<int>::iterator lit = vertices.begin();
                    lit != vertices.end(); lit++){
                    cout << *lit <<" -> ";
                }
                cout << "\n";
                break;
            case 16:
                cout << "Enter First User's Id: ";
                id1 = takeAndValidateNumericalUserInput();

                cout << (social_network.vertexExist(id1) ?
                "User Exists\n" : "User Doesnt Exist\n");
                break;
            case 17:
                misc = social_network.getNbAllVertices();
                cout << (misc == 0 ? "The List is Empty\n" :
                "There are " + (to_string(misc)) + " User(s)");
                break;
            case 18:
                misc = social_network.getNbAllEdges();
                cout << (misc == 0 ? "There Are No Friendships\n" :
                         "There are " + (to_string(misc)) + " Friendship(s)");
                break;
            case 19:
                    cout << (social_network.isRegular() ?
                    "Graph is Regular\n" : "Graph is Not Regular\n");
                break;
            case 20:
                cout << "Density is: "<<social_network.graphDensity() << "\n";
                break;
            case 21:
                cout << "Average Path Length is: "<<social_network.averagePathLength() << "\n";
                break;
            case 22:
                cout << (social_network.writeToFile(fileName) == 0 ?
                         "Write Successful\n" : "Error Writing\n");
                break;
            case 23:
                cout << "Enter First User's Id: ";
                id1 = takeAndValidateNumericalUserInput();
                cout << "\nEnter Second User's Id: ";
                id2 = takeAndValidateNumericalUserInput();
                cout << (social_network.edgeExist(id1, id2) ?
                "The 2 Users are friends\n" : "They Are Not Friends\n");
                break;
            default:
                cout << "Invalid choice. Please enter a number between 0 and 10.\n";
                break;
        }

    }while(true);
}

void firstMenu(){
    bool loop = true;
    int choice;
    do {
        printFirstMenu();
        choice = takeAndValidateNumericalUserInput();
        // Process user's choice for first menu
        switch (choice) {
            case 1:
                if(social_network.readFromFile(fileName) == 0)
                    cout << "Network Created!\n";
                else cout << "Error Importing File \n";

                loop = false;
                break;
            case 2:
                //Default Graph
                loop = false;
                break;
            default:
                cout << "Invalid choice. Please enter 1 or 2.\n";
                break;
        }
    }while(loop);

}

int takeAndValidateNumericalUserInput() {
    int userInput;
    // Read user input
    std::cin >> userInput;

    // Check if the input was successful and if the input is an integer
    while (std::cin.fail() || std::cin.peek() != '\n') {
        std::cin.clear(); // Clear error flag
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n'); // Discard invalid input
        std::cout << "Invalid input. Please enter an integer \n";
        std::cin >> userInput;
    }

    return userInput;
}


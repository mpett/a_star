//
//  main.cpp
//  pathfinder
//
//  Created by Martin Pettersson on 16/02/17.
//  Copyright © 2017 Martin Pettersson. All rights reserved.
//

#include <iostream>
#include <cmath>
#include <sstream>
#include <stdlib.h>
#include <unordered_map>
#include <set>
#include <functional>
#include <queue>
#include <vector>
#include <map>

using namespace std;

struct Node {
    int xPosition;
    int yPosition;
    int fScore;
    int gScore;
    
    Node(int xCoordinate, int yCoordinate) {
        this -> xPosition = xCoordinate;
        this -> yPosition = yCoordinate;
        this -> fScore = 0;
        this -> gScore = 0;
    }
    
    void setFScore(int nFScore) {
        this->fScore = nFScore;
    }
    
    void setGScore(int nGScore) {
        this->gScore = nGScore;
    }
    
    bool operator<(const Node& comparableNode) const {
        return this->fScore > comparableNode.fScore;
    }
    
    bool operator==(const Node& anotherNode) const {
        if (this -> xPosition == anotherNode.xPosition
            && this -> yPosition == anotherNode.yPosition
            && this -> fScore == anotherNode.fScore
            && this -> gScore == anotherNode.gScore) {
            return true;
        } else {
            return false;
        }
    }
    
    int inputIndex(int width) {
        return yPosition * width + xPosition;
    }
    
    bool isEqual(const Node & anotherNode) {
        if (this -> xPosition == anotherNode.xPosition
            && this -> yPosition == anotherNode.yPosition
            && this -> fScore == anotherNode.fScore
            && this -> gScore == anotherNode.gScore) {
            return true;
        } else {
            return false;
        }
    }
    
    bool hasEqualCoordinates(const Node& anotherNode) {
        if (this -> xPosition == anotherNode.xPosition
            && this -> yPosition == anotherNode.yPosition) {
            return true;
        } else {
            return false;
        }
    }
    
    int heuristicDistanceFunction(const Node& destinationNode) {
        int xDistance = destinationNode.xPosition - this -> xPosition;
        int yDistance = destinationNode.yPosition - this -> yPosition;
        int manhattanDistance = std::abs(xDistance) + std::abs(yDistance);
        return manhattanDistance;
    }
};

template<typename T> void printPriorityQueue(T& priorityQueue) {
    while (!priorityQueue.empty()) {
        cout << priorityQueue.top() << " ";
        priorityQueue.pop();
    }
    cout << '\n';
}

bool queueContains(priority_queue<Node> nodeQueue, const Node& possibleNode) {
    while (!nodeQueue.empty()) {
        auto topNode = nodeQueue.top();
        if (topNode.isEqual(possibleNode)) {
            return true;
        }
        nodeQueue.pop();
    }
    return false;
}

vector<Node> reconstructPath(const map<Node, Node>& cameFrom, Node currentNodeCopy) {
    vector<Node> totalPath;
    Node & currentNode = currentNodeCopy;
    totalPath.push_back(currentNode);
    while (cameFrom.count(currentNode)) {
        currentNode = cameFrom.at(currentNode);
        totalPath.push_back(currentNode);
    }
    return totalPath;
}

vector<Node> getNeighbors(const Node& currentNode, const unsigned char * pMap,
                          const unordered_map<int,Node>& inputMap, int width, int bufferSize) {
    vector<Node> neighbors;
    
    int mapSize = bufferSize;
    bool isNextToLeftWall = currentNode.xPosition == 0;
    bool isNextToRightWall = currentNode.xPosition == width - 1;
    int currentNodeIndex = currentNode.xPosition + currentNode.yPosition * width;
    
    if ((currentNodeIndex + 1) >= 0 && (currentNodeIndex + 1) < mapSize
        && !isNextToRightWall && static_cast<int>(pMap[currentNodeIndex + 1] != 0)) {
        neighbors.push_back(inputMap.at(currentNodeIndex + 1));
    }
    if ((currentNodeIndex - 1) >= 0 && (currentNodeIndex - 1) < mapSize
        && !isNextToLeftWall && static_cast<int>(pMap[currentNodeIndex - 1] != 0)) {
        neighbors.push_back(inputMap.at(currentNodeIndex - 1));
    }
    if ((currentNodeIndex + width) >= 0 && (currentNodeIndex + width) < mapSize
        && static_cast<int>(pMap[currentNodeIndex + width] != 0)) {
        neighbors.push_back(inputMap.at(currentNodeIndex + width));
    }
    if ((currentNodeIndex - width) >= 0 && (currentNodeIndex - width) < mapSize
        && static_cast<int>(pMap[currentNodeIndex - width] != 0)) {
        neighbors.push_back(inputMap.at(currentNodeIndex - width));
    }
    
    return neighbors;
}

bool setContainsNode(const set<Node>& nodeSet, const Node& nodeCandidate) {
    for (auto nodeElement : nodeSet) {
        if (nodeElement.hasEqualCoordinates(nodeCandidate)) {
            return true;
        }
    }
    return false;
}

int FindPath(const int nStartX, const int nStartY,
             const int nTargetX, const int nTargetY,
             const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
             int* pOutBuffer, const int nOutBufferSize) {
    
    int firstElement;
    firstElement = static_cast<int>(pMap[0]);
    
    cout << firstElement << endl;
    cout << "Testing equals..." << endl;
    
    Node n1(0,0);
    Node n2(1,0);
    bool nodeEquality = n1.isEqual(n2);
    
    cout << nodeEquality << endl;
    cout << "Done testing equals..." << endl;
    
    unordered_map<int, Node> inputMap;
    for (int yCoordinate = 0; yCoordinate < nMapHeight; yCoordinate++) {
        for (int xCoordinate = 0; xCoordinate < nMapWidth; xCoordinate++) {
            Node nodeToBeInserted(xCoordinate, yCoordinate);
            int inputKey = yCoordinate * nMapWidth + xCoordinate;
            inputMap.insert(pair<int, Node>(inputKey, nodeToBeInserted));
        }
    }
    
    Node startNode(nStartX, nStartY);
    set<Node> closedSet;
    
    cout << "-------------------------------" << endl;
    cout << "Testing priority queue" << endl;
    
    priority_queue<int> queue;
    
    for (int number : {1,8,5,6,3,4,0,9,7,2}) {
        queue.push(number);
    }
    
    printPriorityQueue(queue);
    priority_queue<int, vector<int>, greater<int>> secondQueue;
    
    for (int number : {1,8,5,6,3,4,0,9,7,2}) {
        secondQueue.push(number);
    }
    
    printPriorityQueue(secondQueue);
    priority_queue<Node> nodeQueue;
    Node firstNode(0,1);
    Node secondNode(1,0);
    Node thirdNode(1,1);
    firstNode.fScore = 23131;
    secondNode.fScore = 231;
    thirdNode.fScore = 1339837;
    nodeQueue.push(firstNode);
    nodeQueue.push(secondNode);
    nodeQueue.push(thirdNode);
    auto topNode = nodeQueue.top();
    
    cout << "nodequeue size " << nodeQueue.size() << endl;
    cout << "Top node fscore: " << topNode.fScore << endl;
    
    Node nodeNotInQueue(12,12);
    
    cout << "n1 in queue: " << queueContains(nodeQueue, firstNode) << endl;
    cout << "should not be in queue: " << queueContains(nodeQueue, nodeNotInQueue) << endl;
    cout << "I started populating a test node map." << endl;
    
    Node fourthNode(1,4);
    map<Node, Node> nodeMap;
    pair<Node, Node> firstPair(firstNode, secondNode);
    pair<Node, Node> secondPair(thirdNode, fourthNode);
    pair<Node, Node> thirdPair(firstNode, thirdNode);
    nodeMap.insert(firstPair);
    nodeMap.insert(secondPair);
    nodeMap.insert(thirdPair);
    
    cout << "I finished populating the node map." << endl;
    cout << "Counting a node in map: " << nodeMap.count(fourthNode) << endl;
    cout << "Alright, count seems to work as a keyset contains in a map..." << endl;
    cout << "Done testing priority queue..." << endl;
    
    vector<Node> nodeVector;
    nodeVector.push_back(firstNode);
    nodeVector.push_back(secondNode);
    
    cout << "I should test element contains in a node set." << endl;
    set<Node> nodeSet;
    nodeSet.insert(firstNode);
    nodeSet.insert(secondNode);
    cout <<  "The first and the second node are now both in the set, but not the third and the fourth." << endl;
    cout << "Count first node: " << nodeSet.count(firstNode) << endl;
    cout << "Count second node: " << nodeSet.count(secondNode) << endl;
    cout << "Count third node: " << nodeSet.count(thirdNode) << endl;
    cout << "Count fourth node: " << nodeSet.count(fourthNode) << endl;
    cout << "The method obviously works?" << endl;
    set<Node>::iterator it;
    it = nodeSet.find(firstNode);
    bool nodeExistsInSet = it != nodeSet.end();
    cout << "Find on first node in set: " << nodeExistsInSet << endl;
    it = nodeSet.find(fourthNode);
    nodeExistsInSet = it != nodeSet.end();
    cout << "Find on fourth node in set: " << nodeExistsInSet << endl;
    Node identicalToFirstNode(0,1);
    it = nodeSet.find(identicalToFirstNode);
    nodeExistsInSet = it != nodeSet.end();
    cout << "Find on identical node in set: " << nodeExistsInSet << endl;
    Node fifthNode(1337,12);
    nodeSet.insert(fifthNode);
    
    cout << "I'm starting to test my own contains method now." << endl;
    
    cout << setContainsNode(nodeSet, firstNode) << endl;
    cout << setContainsNode(nodeSet, secondNode) << endl;
    cout << setContainsNode(nodeSet, thirdNode) << endl;
    cout << setContainsNode(nodeSet, fourthNode) << endl;
    cout << setContainsNode(nodeSet, fifthNode) << endl;
    cout << setContainsNode(nodeSet, identicalToFirstNode) << endl;
    
    cout << "It works as intended." << endl;
    
    cout << "-------------------------------" << endl;
    
    priority_queue<Node> openSet;
    
    openSet.push(startNode);
    map<Node, Node> cameFrom;
    startNode.gScore = 0;
    Node goalNode(nTargetX, nTargetY);
    startNode.fScore = startNode.heuristicDistanceFunction(goalNode);
    
    while (!openSet.empty()) {
        Node currentNode = openSet.top();
        if (currentNode.hasEqualCoordinates(goalNode)) {
            //vector<Node> totalPath = reconstructPath(cameFrom, currentNode);
            //return static_cast<int>(totalPath.size());
            return 1;
        }
        openSet.pop();
        closedSet.insert(currentNode);
        cout << "I inserted " << currentNode.xPosition << " " << currentNode.yPosition <<  " in the closed set" << endl;
        vector<Node> currentNeighbors = getNeighbors(currentNode, pMap, inputMap, nMapWidth, nOutBufferSize);
        
        for (Node neighbor : currentNeighbors) {
            cout << "current node: " << currentNode.xPosition << " " << currentNode.yPosition << " neighbor " << neighbor.xPosition << " " << neighbor.yPosition << endl;
            if (setContainsNode(closedSet, neighbor)) {
                cout << "I decided that neighbor " << neighbor.xPosition << " " << neighbor.yPosition << " was in the closed set and continued looping." << endl;
                cout << "closed set contained neighbor" << endl;
                continue;
            }
            int tentativeGScore = currentNode.gScore + 1; // dist between?
            bool shouldPushToOpenSet = false;
            if (!queueContains(openSet, neighbor)) {
                shouldPushToOpenSet = true;
                //openSet.push(neighbor);
                cout << "I pushed a node to the open set." << endl;
            } else if (tentativeGScore >= neighbor.gScore) {
                cout << "I decided to continue." << endl;
                continue;
            }
            
            cameFrom.insert(pair<Node, Node>(neighbor, currentNode));
            neighbor.gScore = tentativeGScore;
            neighbor.fScore = neighbor.gScore + neighbor.heuristicDistanceFunction(goalNode);
            
            if (shouldPushToOpenSet) {
                openSet.push(neighbor);
            }
            
            cout << "lol" << endl;
                
            
        }
        cout << "-----------START NEIGHBOR LOOP--------------------" << endl;
        cout << "number of neighbors " << currentNeighbors.size() << endl;
        cout << "open set size " << openSet.size() << endl;
        cout << "closed set size " << closedSet.size() << endl;
        cout << "came from size " << cameFrom.size() << endl;
        cout << "-----------END NEIGHBOR LOOP--------------------" << endl;
    }
    
    return -1;
}

int main(int argc, const char * argv[]) {
    unsigned char pMap[] = {1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1};
    int pOutBuffer[12];
    int result = FindPath(0, 0, 1, 2, pMap, 4, 3, pOutBuffer, 12);
    cout << "result: " << result << endl;
    return 0;
}

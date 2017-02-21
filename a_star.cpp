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
    }
    
    bool operator<(const Node& comparableNode) const {
        return this->fScore > comparableNode.fScore;
    }
    
    int inputIndex(int width) {
        return yPosition * width + xPosition;
    }
    
    bool isEqual(const Node & anotherNode) {
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
                          const unordered_map<int,Node>& inputMap, int width) {
    vector<Node> neighbors;
    
    int mapSize = sizeof(pMap);
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
    cout << "Top node fscore: " << topNode.fScore << endl;
    cout << "Done testing priority queue..." << endl;
    
    
    priority_queue<Node> openSet;

    openSet.push(startNode);
    map<Node, Node> cameFrom;
    startNode.gScore = 0;
    Node goalNode(nTargetX, nTargetY);
    startNode.fScore = startNode.heuristicDistanceFunction(goalNode);
    
    
    while (!openSet.empty()) {
        auto currentNode = openSet.top();
        if (currentNode.isEqual(goalNode)) {
            vector<Node> totalPath = reconstructPath(cameFrom, currentNode);
            return sizeof(totalPath);
        }
        openSet.pop();
        closedSet.insert(currentNode);
        vector<Node> currentNeighbors = getNeighbors(currentNode, pMap, inputMap, nMapWidth);
        
        for (auto neighbor : currentNeighbors) {
            if (closedSet.count(neighbor)) {
                continue;
            }
            int tentativeGScore = currentNode.gScore + 1; // dist between?
            if (!queueContains(openSet, neighbor)) {
                openSet.push(neighbor);
            } else if (tentativeGScore >= neighbor.gScore) {
                continue;
            }
            cameFrom.insert(pair<Node, Node>(neighbor, currentNode));
            neighbor.gScore = tentativeGScore;
            neighbor.fScore = neighbor.gScore + neighbor.heuristicDistanceFunction(goalNode);
        }
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
